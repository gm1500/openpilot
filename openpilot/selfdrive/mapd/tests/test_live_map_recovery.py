import json
import sys
from contextlib import nullcontext
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import requests

from openpilot.selfdrive.mapd import osm_speed_limit as osm


class StopWorker(Exception):
  pass


def http_error(status, retry_after=None):
  response = requests.Response()
  response.status_code = status
  if retry_after is not None:
    response.headers['Retry-After'] = retry_after
  return requests.HTTPError(response=response)


@pytest.fixture
def worker(monkeypatch):
  clock = [100.]
  service = osm.OSMSpeedLimit()
  service._thread = Mock()
  service._offline.update = Mock(return_value=None)  # no downloaded regional maps
  service._fix = osm.GpsFix(0., 0., 0., 3., clock[0])
  cloudlog = Mock()
  monkeypatch.setitem(sys.modules, 'openpilot.common.realtime', SimpleNamespace(drop_realtime=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=cloudlog))
  monkeypatch.setattr(osm.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(osm, 'fetch_zones', Mock(return_value=()))
  monkeypatch.setattr(service._wake, 'wait', Mock(side_effect=StopWorker))
  return service, clock, cloudlog


def run(service):
  with pytest.raises(StopWorker):
    service._run()


@pytest.mark.parametrize('header_wait', [38.2, 61.])
def test_slow_response_without_offline_maps_keeps_size_and_deadline_bounds(worker, monkeypatch, header_wait):
  service, clock, cloudlog = worker
  element = {'type': 'way', 'id': 123, 'nodes': [456, 789],
             'tags': {'highway': 'primary', 'maxspeed': '70'},
             'geometry': [{'lat': -.001, 'lon': 0.}, {'lat': .02, 'lon': 0.}]}
  response = Mock()
  response.iter_content.return_value = [json.dumps({'elements': [element]}).encode()]

  def post(url, **kwargs):
    assert url == osm.OVERPASS_URL
    assert kwargs['timeout'] == (5., 45.)
    assert 'way(around:1500,' in kwargs['data']['data']
    clock[0] += header_wait
    service._fix = replace(service._fix, latitude=.003, timestamp=clock[0])
    return nullcontext(response)

  monkeypatch.setattr(osm.requests, 'post', post)
  run(service)
  sample = service.update(service._fix, clock[0])
  if header_wait < 60:
    assert sample[1] == pytest.approx(70 / 3.6)
    assert service._cache[0][0].node_ids == (456, 789)
    assert any(call.args[0] == 'mapd.road_response' and call.kwargs['header_wait'] == header_wait
               for call in cloudlog.event.call_args_list)
  else:
    assert service._cache[1] is None
    assert sample is None


def test_outage_retries_cap_at_one_minute_and_keep_previous_cache(worker, monkeypatch):
  service, clock, _ = worker
  previous_cache = ((), replace(service._fix, latitude=1.), 90.)
  service._cache = previous_cache
  calls = []

  def fetch(_):
    calls.append(clock[0])
    raise requests.ReadTimeout()

  def wait(_):
    if clock[0] >= 370:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert calls == [100., 130., 190., 250., 310., 370.]
  assert service._cache is previous_cache


def test_wifi_to_cellular_handoff_retries_after_five_seconds(worker, monkeypatch):
  service, clock, _ = worker
  calls = []
  service.update_network('wifi')

  def fetch(_):
    calls.append(clock[0])
    raise requests.ReadTimeout()

  def wait(_):
    if clock[0] >= 136:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])
    if clock[0] == 131:
      service.update_network('cell4G')

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert calls == [100., 130., 135.]


@pytest.mark.parametrize('server_delay', [None, '120'])
@pytest.mark.parametrize('service_name', ['fetch_roads', 'fetch_zones'])
def test_flapping_connections_bound_retries_and_honor_server_cooldown(worker, monkeypatch, server_delay, service_name):
  service, clock, _ = worker
  calls = []
  monkeypatch.setattr(osm, 'fetch_roads', Mock(return_value=()))

  def fetch(_):
    calls.append(clock[0])
    raise http_error(503, server_delay)

  def wait(_):
    if clock[0] >= 220:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])
    service.update_network('wifi' if int(clock[0]) % 2 else 'cell4G')

  monkeypatch.setattr(osm, service_name, fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert calls == ([100., 220.] if server_delay else list(range(100, 221, 5)))


def test_slow_download_refreshes_at_current_position_without_extra_wait(worker, monkeypatch):
  service, clock, _ = worker
  calls = []
  roads = (osm.Road(((-.001, 0.), (.02, 0.)), {'highway': 'primary', 'maxspeed': '70'}),)

  def fetch(fix):
    from openpilot.common.upload_priority import map_download_active
    assert map_download_active()
    calls.append((clock[0], fix))
    if len(calls) == 1:
      clock[0] += 38.2
      service._fix = replace(fix, latitude=.009, timestamp=clock[0])
    return roads

  def wait(_):
    from openpilot.common.upload_priority import map_download_active
    assert not map_download_active()
    assert service.update(service._fix, clock[0])[1] == pytest.approx(70 / 3.6)
    if len(calls) == 2:
      raise StopWorker

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert [t for t, _ in calls] == [100., 138.2]
  assert calls[1][1].latitude == .009
  assert osm.fetch_zones.call_args.args[0].latitude == .009


def test_gps_loss_after_slow_fetch_does_not_query_municipal_data_at_old_position(worker, monkeypatch):
  service, clock, _ = worker

  def fetch(_):
    clock[0] += 45
    raise requests.ReadTimeout()

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  run(service)
  osm.fetch_zones.assert_not_called()
  assert service.update(service._fix, clock[0]) is None


def test_connection_changes_do_not_refetch_a_fresh_successful_cache(worker, monkeypatch):
  service, clock, _ = worker
  fetch = Mock(return_value=())

  def wait(_):
    if clock[0] >= 200:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])
    service.update_network('wifi' if int(clock[0]) % 2 else 'cell4G')

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  fetch.assert_called_once()


def test_retry_after_http_date_and_bad_headers(monkeypatch):
  monkeypatch.setattr(osm.time, 'time', lambda: 0.)
  assert osm.retry_after_seconds(http_error(429, 'Thu, 01 Jan 1970 00:02:00 GMT')) == 120.
  for value in (None, 'invalid', '-1', 'NaN', 'inf'):
    assert osm.retry_after_seconds(http_error(503, value)) == 0.
