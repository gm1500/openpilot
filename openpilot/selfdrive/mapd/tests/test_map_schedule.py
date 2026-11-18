from datetime import datetime
from unittest.mock import Mock

import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.mapd import map_download as download
from openpilot.selfdrive.mapd.map_schedule import schedule_period
from openpilot.selfdrive.mapd.offline_maps import atomic_json, load_json
from openpilot.selfdrive.mapd.tests.test_offline_maps import NetworkSM


def stamp(value):
  return datetime.fromisoformat(value).timestamp()


@pytest.fixture
def params(tmp_path):
  p = Params(str(tmp_path/'params'))
  p.put('SentryTimeZone', 'America/Edmonton', block=True)
  p.put('OfflineMapUpdateSchedule', 2, block=True)
  p.put('OfflineMapUpdateWeekday', 0, block=True)
  p.put_bool('IsOffroad', True, block=True)
  return p


@pytest.mark.parametrize('mode,weekday,now,expected', [
  (1, 0, '2026-10-19T02:00:00+00:00', '2026-10-18T00:00:00-06:00'),
  (2, 0, '2026-10-19T02:00:00+00:00', '2026-10-12T00:00:00-06:00'),
  (2, 0, '2026-10-19T06:00:00+00:00', '2026-10-19T00:00:00-06:00'),
  (2, 4, '2026-10-19T18:00:00+00:00', '2026-10-16T00:00:00-06:00'),
  (1, 0, '2026-11-01T04:00:00-07:00', '2026-11-01T00:00:00-06:00'),
  (2, 0, '2026-11-01T04:00:00-07:00', '2026-10-26T00:00:00-06:00'),
  (1, 0, '2026-03-08T04:00:00-06:00', '2026-03-08T00:00:00-07:00'),
])
def test_local_calendar_boundary_and_dst(params, mode, weekday, now, expected):
  params.put('OfflineMapUpdateSchedule', mode, block=True)
  params.put('OfflineMapUpdateWeekday', weekday, block=True)
  assert schedule_period(params, stamp(now)) == stamp(expected)


def test_manual_invalid_clock_and_unknown_timezone(params):
  params.remove('OfflineMapUpdateSchedule')
  assert schedule_period(params, stamp('2026-10-19T12:00:00+00:00')) is None
  params.put('OfflineMapUpdateSchedule', 1, block=True)
  for value in (0., float('nan'), float('inf'), stamp('2040-01-01T00:00:00+00:00')):
    assert schedule_period(params, value) is None
  params.put('SentryTimeZone', 'unknown', block=True)
  assert schedule_period(params, stamp('2026-10-19T02:00:00+00:00')) == stamp('2026-10-19T00:00:00+00:00')


@pytest.fixture
def manager(params, tmp_path, monkeypatch):
  groups = {'nation.CA': ['0_0', '0_2'], 'nation.US': ['0_2', '0_4']}
  monkeypatch.setattr(download, 'region_groups', groups.__getitem__)
  store = download.MapStore(tmp_path/'maps')
  old = stamp('2026-10-10T12:00:00-06:00')
  store.manifest['regions'] = {r: {'groups': g, 'complete': True, 'updated_at': old} for r, g in groups.items()}
  store.manifest['groups'] = {g: {'downloaded_at': old, 'generation': 'a'*32} for g in ('0_0', '0_2', '0_4')}
  store.save()
  return download.MapDownloads(store, params)


def test_catch_up_only_installed_complete_regions_and_do_not_repeat(manager):
  now = stamp('2026-10-21T12:00:00-06:00')
  manager.store.manifest['regions']['nation.US']['complete'] = False
  manager.schedule(now)
  assert manager.job['automatic'] and manager.job['region'] == 'nation.CA'
  assert manager.job['done'] == []
  assert manager.store.manifest['regions']['nation.CA']['complete']
  for group in manager.job['groups']:
    manager.complete_group(group)
  manager.finish(now)
  manager.schedule(now+3600)
  assert manager.job is None
  assert manager.store.manifest['regions']['nation.CA']['updated_at'] == now
  manager.schedule(now+7*86400)
  assert manager.job['region'] == 'nation.CA'


def test_recent_manual_update_satisfies_schedule_and_shared_groups_are_reused(manager):
  now = stamp('2026-10-21T12:00:00-06:00')
  manager.store.manifest['regions']['nation.CA']['updated_at'] = now
  manager.store.manifest['groups']['0_2']['downloaded_at'] = now
  manager.schedule(now+1)
  assert manager.job['region'] == 'nation.US'
  assert manager.job['done'] == ['0_2']


def test_cancel_survives_restart_without_requeue_until_next_period(manager):
  now = stamp('2026-10-21T12:00:00-06:00')
  manager.schedule(now)
  manager.cancel(now)
  restarted = download.MapDownloads(manager.store, manager.params)
  restarted.schedule(now+3600)
  assert restarted.job is None
  restarted.start('nation.CA', update=True)
  assert not restarted.job['automatic']  # A deliberate update still works.


def test_cancel_after_calendar_rollover_suppresses_all_regions_in_current_period(manager):
  now = stamp('2026-10-18T23:00:00-06:00')
  manager.schedule(now)
  manager.cancel(now+7200)
  restarted = download.MapDownloads(manager.store, manager.params)
  restarted.schedule(now+7201)
  assert restarted.job is None
  restarted.schedule(now+8*86400)
  assert restarted.job is not None


def test_failure_retains_progress_and_backs_off_across_restart(manager):
  now = stamp('2026-10-21T12:00:00-06:00')
  manager.schedule(now)
  manager.complete_group('0_0')
  manager.failed(now)
  restarted = download.MapDownloads(manager.store, manager.params)
  assert restarted.job['done'] == ['0_0']
  assert restarted.job['retry_at'] == now+3600
  assert restarted.store.manifest['regions']['nation.CA']['complete']
  restarted.complete_group('0_2')
  assert 'retry_at' not in restarted.job
  restarted.finish(now+3600)
  assert not restarted.job_path.exists()


def test_old_manual_jobs_resume_and_conflicting_requests_preserve_progress(manager):
  old = {'region': 'nation.CA', 'groups': ['0_0', '0_2'], 'done': ['0_0']}
  atomic_json(manager.job_path, old)
  restarted = download.MapDownloads(manager.store, manager.params)
  for action in ('delete', 'download', 'update'):
    with pytest.raises(ValueError, match='Cancel'):
      restarted.request({'action': action, 'region': 'nation.US'})
    assert restarted.job == old and load_json(restarted.job_path, None) == old


@pytest.mark.parametrize('network', ['none', 'wifi', 'cell4G', 'ethernet'])
@pytest.mark.parametrize('metered', [False, True])
def test_automatic_refresh_requires_unmetered_wifi(params, network, metered):
  sm = NetworkSM(network, metered)
  if network == 'wifi' and not metered:
    download.check_download_allowed(params, sm, 100., wifi_only=True)
  else:
    with pytest.raises(download.DownloadPaused):
      download.check_download_allowed(params, sm, 100., wifi_only=True)


def test_wifi_to_unmetered_cellular_handoff_interrupts_automatic_body(manager):
  sm = NetworkSM('wifi')
  response = Mock(headers={'Content-Length': '6'})
  response.__enter__ = Mock(return_value=response)
  response.__exit__ = Mock(return_value=False)
  def chunks():
    yield b'abc'
    sm['deviceState'].networkType = 'cell4G'
    yield b'def'
  response.iter_content.return_value = chunks()
  def check():
    download.check_download_allowed(manager.params, sm, 100., wifi_only=True)
  with pytest.raises(download.DownloadPaused, match='Wi-Fi'):
    download.fetch_archive(manager.store, '0_0', check, lambda *args: None, Mock(get=Mock(return_value=response)))
  assert (manager.store.root/'downloads/0_0.tar.gz.part').read_bytes() == b'abc'


def test_started_device_blocks_even_before_offroad_param_catches_up(params):
  sm = NetworkSM('wifi')
  sm['deviceState'].started = True
  with pytest.raises(download.DownloadPaused, match='parked'):
    download.check_download_allowed(params, sm, 100., wifi_only=True)


def test_switching_to_manual_cancels_pending_auto_job_without_network_or_download(manager, monkeypatch):
  manager.schedule(stamp('2026-10-21T12:00:00-06:00'))
  manager.params.put('OfflineMapUpdateSchedule', 0, block=True)
  sm = NetworkSM('none')
  sm.update = lambda timeout: None
  fetch = Mock()
  class Finished(BaseException):
    pass
  def stop(seconds):
    raise Finished
  monkeypatch.setattr(download, 'Params', lambda: manager.params)
  monkeypatch.setattr(download, 'MapStore', lambda: manager.store)
  monkeypatch.setattr(download.messaging, 'SubMaster', lambda services: sm)
  monkeypatch.setattr(download, 'fetch_archive', fetch)
  monkeypatch.setattr(download.time, 'sleep', stop)
  with pytest.raises(Finished):
    download.main()
  assert not manager.job_path.exists()
  fetch.assert_not_called()
  assert manager.store.manifest['regions']['nation.CA']['complete']


def test_daemon_waits_for_wifi_then_starts_due_update(manager, monkeypatch):
  now = stamp('2026-10-21T12:00:00-06:00')
  clock = [100.]
  sm = NetworkSM('cell4G')
  sm.update = lambda timeout: sm.recv_time.update(deviceState=clock[0])
  class Finished(BaseException):
    pass
  def sleep(seconds):
    assert not manager.job_path.exists()  # No automatic job on unmetered cellular.
    sm['deviceState'].networkType = 'wifi'
    clock[0] += seconds
  def fetch(store, group, check, progress):
    check(True)
    job = load_json(manager.job_path, None)
    assert job['automatic'] and job['region'] == 'nation.CA'
    assert group == '0_0'
    raise Finished
  monkeypatch.setattr(download, 'Params', lambda: manager.params)
  monkeypatch.setattr(download, 'MapStore', lambda: manager.store)
  monkeypatch.setattr(download.messaging, 'SubMaster', lambda services: sm)
  monkeypatch.setattr(download, 'fetch_archive', fetch)
  monkeypatch.setattr(download.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(download.time, 'time', lambda: now)
  monkeypatch.setattr(download.time, 'sleep', sleep)
  with pytest.raises(Finished):
    download.main()
