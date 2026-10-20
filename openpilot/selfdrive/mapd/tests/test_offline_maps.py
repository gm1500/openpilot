import io
import tarfile
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.selfdrive.mapd import offline_maps as offline, map_download as download
from openpilot.selfdrive.mapd.osm_speed_limit import GpsFix, OSMSpeedLimit
from openpilot.selfdrive.mapd.tests.test_osm_speed_limit import road


class NetworkSM(dict):
  def __init__(self, network='cell4G', metered=False, valid=True, received_at=100.):
    state = download.log.DeviceState.new_message(networkType=network, networkMetered=metered)
    super().__init__(deviceState=state)
    self.valid = {'deviceState': valid}
    self.recv_time = {'deviceState': received_at}


@pytest.mark.parametrize('network', ['wifi', 'cell2G', 'cell3G', 'cell4G', 'cell5G', 'ethernet'])
@pytest.mark.parametrize('metered', [False, True])
def test_download_respects_active_connection_metered_policy(network, metered):
  params = Mock(get_bool=Mock(return_value=True))
  sm = NetworkSM(network, metered)
  if metered:
    with pytest.raises(download.DownloadPaused, match='unmetered connection'):
      download.check_download_allowed(params, sm, 100.)
  else:
    download.check_download_allowed(params, sm, 100.)


@pytest.mark.parametrize('network,valid,received_at', [
  ('none', True, 100.), ('cell4G', False, 100.), ('cell4G', True, 94.9), ('cell4G', True, 101.),
])
def test_unmetered_setting_does_not_bypass_missing_or_stale_connection(network, valid, received_at):
  params = Mock(get_bool=Mock(return_value=True))
  with pytest.raises(download.DownloadPaused, match='network connection'):
    download.check_download_allowed(params, NetworkSM(network, valid=valid, received_at=received_at), 100.)


def test_cellular_download_still_requires_parked_vehicle():
  params = Mock(get_bool=Mock(return_value=False))
  with pytest.raises(download.DownloadPaused, match='parked'):
    download.check_download_allowed(params, NetworkSM(), 100.)


def archive_bytes(speed=50., conditional='', one_way=True):
  result = io.BytesIO()
  with tarfile.open(fileobj=result, mode='w:gz') as tar:
    for y in range(8):
      for x in range(8):
        lat, lon = y / 4, x / 4
        tile = offline.SCHEMA.Offline.new_message(minLat=lat, minLon=lon, maxLat=lat+.25, maxLon=lon+.25, overlap=.001)
        if y == x == 0:
          ways = tile.init('ways', 1)
          w = ways[0]
          w.id, w.name, w.maxSpeed, w.oneWay = 10, 'Test road', speed / 3.6, one_way
          w.ref, w.highwayClass = '216', 'motorway'
          w.minLat, w.maxLat, w.minLon, w.maxLon = .09, .11, .1, .1
          w.maxSpeedConditional = conditional
          w.maxSpeedBackward = 30 / 3.6
          nodes = w.init('nodes', 2)
          nodes[0].latitude, nodes[0].longitude = .09, .1
          nodes[1].latitude, nodes[1].longitude = .11, .1
        else:
          tile.init('ways', 0)
        data = tile.to_bytes_packed()
        entry = tarfile.TarInfo('offline/0/0/' + offline.tile_name(lat, lon)[1])
        entry.size = len(data)
        tar.addfile(entry, io.BytesIO(data))
  return result.getvalue()


@pytest.fixture
def store(tmp_path, monkeypatch):
  monkeypatch.setattr(download, 'storage_available', lambda *args: None)
  return download.MapStore(tmp_path/'maps')


def install(store, tmp_path, **kwargs):
  archive = tmp_path/'maps.tar.gz'
  archive.write_bytes(archive_bytes(**kwargs))
  store.install(archive, '0_0')
  return archive


def test_catalog_north_america_includes_countries_and_non_contiguous_us():
  north_america = set(offline.region_groups('region.NA'))
  for region in ('nation.CA', 'nation.US', 'nation.MX', 'us_state.AK', 'us_state.HI'):
    assert set(offline.region_groups(region)) <= north_america
  for kind, entries in offline.region_catalog().items():
    for code in entries:
      assert offline.region_groups(f'{kind}.{code}')
  with pytest.raises(ValueError):
    offline.region_groups('../../etc')


def test_real_format_persists_across_restart_and_preserves_direction_and_conditions(store, tmp_path):
  install(store, tmp_path, conditional='30 @ (Mo-Fr 08:00-16:00)')
  restarted = offline.OfflineMaps(store.root)
  fix = GpsFix(.1, .1, 0., 3., 100.)
  roads = restarted.roads(fix)
  assert len(roads) == 1
  assert roads[0].way_id == 10 and roads[0].node_ids == ()
  assert roads[0].tags['maxspeed'] == '50.0'
  assert roads[0].tags['maxspeed:backward'] == '30.0'
  assert roads[0].tags['maxspeed:conditional'] == '30 @ (Mo-Fr 08:00-16:00)'
  assert roads[0].tags['oneway'] == 'yes'
  assert roads[0].tags['ref'] == '216'
  assert roads[0].tags['highway'] == 'motorway'
  # An atomic update is observed by an already-running reader.
  install(store, tmp_path, speed=80)
  assert restarted.roads(fix)[0].tags['maxspeed'] == '80.0'


def test_missing_boundary_tile_is_not_reported_as_complete_coverage(store, tmp_path):
  install(store, tmp_path)
  reader = offline.OfflineMaps(store.root)
  assert reader.roads(GpsFix(.1, .1, 0., 3., 100.))
  assert reader.roads(GpsFix(.001, .001, 0., 3., 100.)) is None
  assert reader.roads(GpsFix(89.99, 179.99, 0., 3., 100.)) is None


def test_restart_cleans_interrupted_extraction_but_keeps_committed_maps(store, tmp_path):
  install(store, tmp_path)
  orphan = store.root/'groups/0_0'/('0'*32)
  orphan.mkdir()
  (orphan/'partial').write_bytes(b'incomplete')
  restarted = download.MapStore(store.root)
  assert not orphan.exists()
  assert restarted.size() == store.size()
  assert offline.OfflineMaps(store.root).roads(GpsFix(.1, .1, 0., 3., 100.))


def test_corrupt_download_never_replaces_usable_maps(store, tmp_path):
  archive = install(store, tmp_path)
  previous = store.manifest['groups']['0_0'].copy()
  archive.write_bytes(archive_bytes(speed=80)[:-6])
  with pytest.raises((EOFError, OSError)):
    store.install(archive, '0_0')
  assert store.manifest['groups']['0_0'] == previous
  assert offline.OfflineMaps(store.root).roads(GpsFix(.1, .1, 0., 3., 100.))[0].tags['maxspeed'] == '50.0'
  assert len(list((store.root/'groups/0_0').iterdir())) == 1


@pytest.mark.parametrize('name,kind', [('../../escape', tarfile.REGTYPE), ('offline/0/0/link', tarfile.SYMTYPE)])
def test_archive_traversal_and_links_are_rejected(store, tmp_path, name, kind):
  archive = tmp_path/'bad.tar.gz'
  with tarfile.open(archive, 'w:gz') as tar:
    entry = tarfile.TarInfo(name)
    entry.type, entry.linkname = kind, '/etc/passwd'
    tar.addfile(entry)
  with pytest.raises(ValueError):
    store.install(archive, '0_0')
  assert not store.manifest['groups']


def test_deleting_one_region_retains_shared_map_groups(store, tmp_path):
  install(store, tmp_path)
  store.manifest['regions'] = {'first': {'groups': ['0_0']}, 'second': {'groups': ['0_0']}}
  store.save()
  store.remove('first')
  assert offline.OfflineMaps(store.root).roads(GpsFix(.1, .1, 0., 3., 100.))
  store.remove('second')
  assert not store.manifest['groups']
  assert not (store.root/'groups/0_0').exists()


def test_onroad_pause_during_download_leaves_installed_maps_unchanged(store, tmp_path):
  install(store, tmp_path)
  previous = store.manifest['groups']['0_0'].copy()
  response = Mock()
  response.headers = {'Content-Length': '9'}
  response.iter_content.return_value = [b'abc', b'def', b'ghi']
  response.__enter__ = Mock(return_value=response)
  response.__exit__ = Mock(return_value=False)
  calls = []
  def check():
    calls.append(1)
    if len(calls) == 2:
      raise download.DownloadPaused('Driving')
  with pytest.raises(download.DownloadPaused):
    download.fetch_archive(store, '0_0', check, lambda *args: None, NS(get=lambda *a, **kw: response))
  assert store.manifest['groups']['0_0'] == previous
  assert (store.root/'downloads/0_0.tar.gz.part').read_bytes() == b'abc'


def test_metered_toggle_pauses_transfer_and_unmetered_allows_restart(store, tmp_path):
  install(store, tmp_path)
  previous = store.manifest['groups']['0_0'].copy()
  params = Mock(get_bool=Mock(return_value=True))
  sm = NetworkSM()
  response = Mock(headers={'Content-Length': '9'})
  response.__enter__ = Mock(return_value=response)
  response.__exit__ = Mock(return_value=False)
  session = NS(get=lambda *args, **kwargs: response)

  def chunks():
    yield b'abc'
    sm['deviceState'].networkMetered = True
    yield b'def'

  response.iter_content.return_value = chunks()
  def check():
    download.check_download_allowed(params, sm, 100.)
  with pytest.raises(download.DownloadPaused, match='unmetered connection'):
    download.fetch_archive(store, '0_0', check, lambda *args: None, session)
  assert (store.root/'downloads/0_0.tar.gz.part').read_bytes() == b'abc'
  sm['deviceState'].networkMetered = False
  response.iter_content.return_value = [b'abc', b'def', b'ghi']
  check()
  path = download.fetch_archive(store, '0_0', check, lambda *args: None, session)
  assert path.read_bytes() == b'abcdefghi'
  assert store.manifest['groups']['0_0'] == previous


def test_storage_reserve_checks_both_absolute_and_percentage(tmp_path, monkeypatch):
  gb = 1024**3
  monkeypatch.setattr(download.shutil, 'disk_usage', lambda root: NS(total=128*gb, free=10*gb))
  with pytest.raises(OSError):
    download.storage_available(tmp_path)
  monkeypatch.setattr(download.shutil, 'disk_usage', lambda root: NS(total=32*gb, free=4*gb))
  with pytest.raises(OSError):
    download.storage_available(tmp_path)


def test_offline_speed_fallback_never_invents_junction_topology():
  service = OSMSpeedLimit()
  service._thread = Mock()
  service._offline = NS(update=lambda fix, now: (road('50'),))
  fix = GpsFix(0., 0., 0., 3., 100.)
  sample = service.update(fix, 100.)
  assert sample[1] == 50/3.6
  assert service.control_match is None and service.control is None
  # A fresh richer online result retains priority and its existing behavior.
  service._cache = (road('30'),), fix, 100.
  assert service.update(replace(fix, timestamp=101.), 101.)[1] == 30/3.6
  assert service.update(replace(fix, timestamp=1.), 101.) is None
