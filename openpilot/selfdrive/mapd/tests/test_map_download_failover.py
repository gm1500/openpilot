import sys
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
  service._fix = osm.GpsFix(0., 0., 0., 3., clock[0])
  cloudlog = Mock()
  monkeypatch.setitem(sys.modules, 'openpilot.common.realtime', SimpleNamespace(drop_realtime=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=cloudlog))
  monkeypatch.setattr(osm.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(osm, 'fetch_zones', lambda _: ())
  monkeypatch.setattr(service._wake, 'wait', Mock(side_effect=StopWorker))
  return service, clock, cloudlog


def run(service):
  with pytest.raises(StopWorker):
    service._run()


@pytest.mark.parametrize('error', [requests.ReadTimeout(), http_error(502)])
def test_failed_server_uses_backup_at_current_position_without_minute_backoff(worker, monkeypatch, error):
  service, clock, cloudlog = worker
  calls = []
  roads = (osm.Road(((.01, 0.), (.02, 0.)), {'highway': 'primary', 'maxspeed': '70'}),)

  def fetch(fix, endpoint):
    calls.append((clock[0], fix, endpoint))
    if len(calls) == 1:
      clock[0] += 35
      service._fix = replace(fix, latitude=.01, timestamp=clock[0])
      raise error
    return roads

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  run(service)
  assert [(t, endpoint) for t, _, endpoint in calls] == [(100., osm.OVERPASS_ENDPOINTS[0]), (135., osm.OVERPASS_ENDPOINTS[1])]
  assert calls[1][1].latitude == .01
  assert service._cache == (roads, calls[1][1], 135.)
  assert cloudlog.event.call_args_list[0].kwargs['duration'] == 35.


def test_successful_backup_remains_preferred(worker, monkeypatch):
  service, clock, _ = worker
  calls = []

  def fetch(fix, endpoint):
    calls.append(endpoint)
    if endpoint == osm.OVERPASS_ENDPOINTS[0]:
      raise http_error(502)
    return ()

  def wait(_):
    if clock[0] >= 640:
      raise StopWorker
    clock[0] = 640
    service._fix = replace(service._fix, timestamp=clock[0])

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert calls == [osm.OVERPASS_ENDPOINTS[0], osm.OVERPASS_ENDPOINTS[1], osm.OVERPASS_ENDPOINTS[1]]


@pytest.mark.parametrize('error', [http_error(400), http_error(401), http_error(403), http_error(429),
                                 requests.exceptions.SSLError(), ValueError('Incomplete OSM response')])
def test_refusals_and_invalid_data_do_not_rotate_providers(worker, monkeypatch, error):
  service, _, _ = worker
  fetch = Mock(side_effect=error)
  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  run(service)
  assert fetch.call_count == 1
  assert service._cache[1] is None


def test_both_servers_down_retains_cache_and_bounds_retries(worker, monkeypatch):
  service, clock, _ = worker
  previous_cache = ((), replace(service._fix, latitude=1.), 90.)
  service._cache = previous_cache
  calls = []

  def fetch(fix, endpoint):
    calls.append((clock[0], endpoint))
    raise http_error(502)

  def wait(_):
    if clock[0] >= 300:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert service._cache is previous_cache
  assert len(calls) <= 8
  assert sorted({t for t, _ in calls}) == [100., 130., 190., 250.]
  for endpoint in osm.OVERPASS_ENDPOINTS:
    times = [t for t, e in calls if e == endpoint]
    assert all(b - a >= 30 for a, b in zip(times, times[1:], strict=False))


def test_gps_loss_during_timeout_does_not_send_stale_backup_request(worker, monkeypatch):
  service, clock, _ = worker

  def timeout(fix, endpoint):
    clock[0] += 35
    raise requests.ReadTimeout()

  fetch = Mock(side_effect=timeout)
  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  run(service)
  assert fetch.call_count == 1
  assert service._cache[1] is None


def test_server_retry_after_survives_network_changes(worker, monkeypatch):
  service, clock, _ = worker
  calls = []

  def fetch(fix, endpoint):
    calls.append(clock[0])
    raise http_error(503, '120')

  def wait(_):
    if clock[0] >= 220:
      raise StopWorker
    clock[0] += 1
    service._fix = replace(service._fix, timestamp=clock[0])
    service.update_network('wifi' if int(clock[0]) % 2 else 'cell4G')

  monkeypatch.setattr(osm, 'fetch_roads', fetch)
  monkeypatch.setattr(service._wake, 'wait', wait)
  run(service)
  assert calls == [100., 220.]


def test_retry_after_http_date(monkeypatch):
  monkeypatch.setattr(osm.time, 'time', lambda: 0.)
  assert osm.overpass_retry_after(http_error(503, 'Thu, 01 Jan 1970 00:02:00 GMT')) == 120.
  assert osm.overpass_retry_after(http_error(503, 'invalid')) == 0.
