"""Offroad, unmetered regional downloads with atomic group installation."""
import gzip
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tarfile
import time
import uuid

import requests

from openpilot.cereal import messaging, log
from openpilot.common.params import Params
from openpilot.selfdrive.mapd.map_schedule import schedule_period
from openpilot.selfdrive.mapd.offline_maps import (SCHEMA, MAX_TILE_BYTES, atomic_json, load_json,
                                                   map_root, region_groups, region_name, tile_name)

URL = 'https://map-data.pfeifer.dev/offline'
MAX_ARCHIVE = 512 * 1024 * 1024
MAX_EXPANDED = 2 * 1024 * 1024 * 1024
RESERVE_BYTES = 5 * 1024 * 1024 * 1024
AUTO_RETRY_SECONDS = 3600


class DownloadPaused(Exception):
  pass


class DownloadCancelled(Exception):
  pass


def check_download_allowed(params, sm, now, *, wifi_only=False):
  if not params.get_bool('IsOffroad'):
    raise DownloadPaused('Waiting until parked')
  if (not sm.valid['deviceState'] or not 0 <= now-sm.recv_time['deviceState'] <= 5 or
      sm['deviceState'].networkType == log.DeviceState.NetworkType.none):
    raise DownloadPaused('Waiting for network connection')
  if sm['deviceState'].started:
    raise DownloadPaused('Waiting until parked')
  if wifi_only and sm['deviceState'].networkType != log.DeviceState.NetworkType.wifi:
    raise DownloadPaused('Waiting for Wi-Fi')
  # hardwared resolves the active connection's policy, including GsmMetered
  # for cellular and the selected Wi-Fi network's metered setting.
  if sm['deviceState'].networkMetered:
    raise DownloadPaused('Waiting for an unmetered connection')


def storage_available(root: Path, needed=0):
  usage = shutil.disk_usage(root)
  if usage.free - needed < max(RESERVE_BYTES, usage.total * .1):
    raise OSError('Not enough space; keep at least 5 GB and 10% free for recordings')


class MapStore:
  def __init__(self, root=None):
    self.root = Path(root) if root is not None else map_root()
    self.root.mkdir(parents=True, exist_ok=True)
    self.manifest = load_json(self.root/'installed.json', {'groups': {}, 'regions': {}})
    if not isinstance(self.manifest, dict) or not all(isinstance(self.manifest.get(k), dict) for k in ('groups', 'regions')):
      self.manifest = {'groups': {}, 'regions': {}}
    # A power loss during extraction can leave a generation that was never
    # published. Only delete owned generation directories absent from the index.
    for directory in (self.root/'groups').glob('*/*'):
      active = self.manifest['groups'].get(directory.parent.name, {}).get('generation')
      if re.fullmatch(r'[0-9a-f]{32}', directory.name) and directory.name != active:
        shutil.rmtree(directory, ignore_errors=True)

  def save(self):
    atomic_json(self.root/'installed.json', self.manifest)

  def size(self):
    return sum(group.get('bytes', 0) for group in self.manifest['groups'].values())

  def install(self, archive: Path, group: str, check=lambda: None):
    lat, lon = map(int, group.split('_'))
    generation = uuid.uuid4().hex
    stage = self.root/'groups'/group/generation
    stage.mkdir(parents=True)
    total = count = 0
    names = set()
    try:
      with tarfile.open(archive, 'r:gz') as tar:
        for entry in tar:
          check()
          parts = PurePosixPath(entry.name).parts
          if entry.isdir():
            continue
          if (not entry.isfile() or len(parts) != 4 or parts[:3] != ('offline', str(lat), str(lon)) or
              not re.fullmatch(r'-?\d+\.\d{6}_-?\d+\.\d{6}_-?\d+\.\d{6}_-?\d+\.\d{6}', parts[3])):
            raise ValueError('Unexpected map archive entry')
          if entry.size <= 0 or entry.size > MAX_TILE_BYTES or parts[3] in names:
            raise ValueError('Invalid map tile size or duplicate')
          total += entry.size
          if total > MAX_EXPANDED or count >= 64:
            raise ValueError('Map archive exceeds size limit')
          storage_available(self.root, entry.size)
          data = tar.extractfile(entry).read()
          if len(data) != entry.size:
            raise ValueError('Truncated map tile')
          tile = SCHEMA.Offline.from_bytes_packed(data, traversal_limit_in_words=16*1024*1024, nesting_limit=64)
          values = tuple(map(float, parts[3].split('_')))
          if (not all(math.isfinite(v) for v in values) or
              tile_name(values[0], values[1]) != (group, parts[3]) or
              any(abs(a-b) > 1e-7 for a,b in zip(values, (tile.minLat, tile.minLon, tile.maxLat, tile.maxLon), strict=True))):
            raise ValueError('Map tile bounds mismatch')
          with (stage/parts[3]).open('wb') as file:
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
          names.add(parts[3])
          count += 1
      # tar can finish before gzip's trailer. Verify the complete gzip CRC before
      # publishing any files, including on truncated responses with HTTP 200.
      with gzip.open(archive, 'rb') as file:
        checked = 0
        while block := file.read(1024*1024):
          check()
          checked += len(block)
          if checked > MAX_EXPANDED + 1024*1024:
            raise ValueError('Map archive exceeds expanded limit')
      if count != 64:
        raise ValueError('Incomplete map archive: expected 64 tiles')
      previous = self.manifest['groups'].get(group)
      info = {'generation': generation, 'bytes': total, 'downloaded_at': time.time()}  # noqa: TID251
      self.manifest['groups'][group] = info
      try:
        self.save()
      except Exception:
        if previous is None:
          self.manifest['groups'].pop(group, None)
        else:
          self.manifest['groups'][group] = previous
        raise
      if previous:
        shutil.rmtree(self.root/'groups'/group/previous['generation'], ignore_errors=True)
      return info
    except Exception:
      shutil.rmtree(stage, ignore_errors=True)
      raise

  def remove(self, region):
    self.manifest['regions'].pop(region, None)
    used = set().union(*(set(item['groups']) for item in self.manifest['regions'].values()))
    obsolete = set(self.manifest['groups']) - used
    for group in obsolete:
      self.manifest['groups'].pop(group)
    self.save()
    for group in obsolete:
      shutil.rmtree(self.root/'groups'/group, ignore_errors=True)


def fetch_archive(store, group, check, progress, session=requests):
  lat, lon = group.split('_')
  path = store.root/'downloads'/f'{group}.tar.gz.part'
  path.parent.mkdir(exist_ok=True)
  # Restart only the interrupted archive. Completed groups survive pauses,
  # cancellations and reboots; a partial archive is never mistaken for a map.
  with session.get(f'{URL}/{lat}/{lon}.tar.gz', stream=True, timeout=(5, 15)) as response:
    response.raise_for_status()
    length = int(response.headers.get('Content-Length', 0))
    if length < 0 or length > MAX_ARCHIVE:
      raise ValueError('Map archive too large')
    storage_available(store.root, length)
    received = 0
    with path.open('wb') as file:
      for chunk in response.iter_content(256*1024):
        check()
        received += len(chunk)
        if received > MAX_ARCHIVE:
          raise ValueError('Map archive too large')
        storage_available(store.root, len(chunk))
        file.write(chunk)
        progress(received, length)
    if length and received != length:
      raise ValueError('Incomplete map download')
  return path


class MapDownloads:
  """One resumable job shared by manual requests and scheduled refreshes."""
  def __init__(self, store, params):
    self.store, self.params = store, params
    self.job_path = store.root/'download.json'
    self.job = load_json(self.job_path, None)
    self.attempt_path = store.root/'scheduled.json'
    self.attempts = load_json(self.attempt_path, {})

  def save(self):
    atomic_json(self.job_path, self.job)

  def start(self, region, *, update=False, period=None, wall=None):
    if self.job is not None:
      raise ValueError('Cancel the active download first')
    groups = region_groups(region)
    installed = self.store.manifest['groups']
    done = [g for g in groups if g in installed and
            (not update or period is not None and installed[g].get('downloaded_at', 0) >= period)]
    self.job = {'region': region, 'groups': groups, 'done': done, 'automatic': period is not None}
    # Keep the last successful update while refreshing. Existing maps stay
    # usable, including when an update is paused, cancelled or interrupted.
    self.store.manifest['regions'].setdefault(region, {'name': region_name(region), 'groups': groups, 'complete': False})
    self.store.save()
    self.save()
    if period is not None:
      # Cancelling an automatic job suppresses it for this calendar period,
      # even across restarts. A manual Update can always start another attempt.
      self.attempts[region] = wall
      atomic_json(self.attempt_path, self.attempts)

  def schedule(self, wall):
    period = schedule_period(self.params, wall)
    if self.job is not None or period is None:
      return
    for region, info in self.store.manifest['regions'].items():
      if info.get('complete') and max(info.get('updated_at', 0), self.attempts.get(region, 0)) < period:
        self.start(region, update=True, period=period, wall=wall)
        return

  def clear(self):
    self.job_path.unlink(missing_ok=True)
    self.job = None

  def cancel(self, wall):
    if self.job and self.job.get('automatic'):
      # Cancel the refresh cycle, including other installed areas and a job
      # that has crossed into a new calendar period since it was started.
      self.attempts.update({region: wall for region, info in self.store.manifest['regions'].items() if info.get('complete')})
      atomic_json(self.attempt_path, self.attempts)
    self.clear()

  def complete_group(self, group):
    self.job['done'].append(group)
    self.job.pop('retry_at', None)
    self.save()

  def finish(self, wall):
    self.store.manifest['regions'][self.job['region']].update(complete=True, updated_at=wall)
    self.store.save()
    self.clear()

  def failed(self, wall):
    if self.job and self.job.get('automatic'):
      # Preserve completed groups and back off instead of retrying every tick.
      self.job['retry_at'] = wall + AUTO_RETRY_SECONDS
      self.save()
    else:
      self.clear()

  def request(self, request):
    action, region = request.get('action'), request.get('region', '')
    if action == 'cancel':
      raise DownloadCancelled
    if self.job is not None:
      raise ValueError('Cancel the active download first')
    if action == 'delete':
      if not self.params.get_bool('IsOffroad'):
        raise ValueError('Park before deleting maps')
      region_groups(region)
      self.store.remove(region)
      self.attempts.pop(region, None)
      atomic_json(self.attempt_path, self.attempts)
    elif action in ('download', 'update'):
      self.start(region, update=action == 'update')
    else:
      raise ValueError('Unknown map action')


def main():
  params = Params()
  sm = messaging.SubMaster(['deviceState'])
  store = MapStore()
  downloads = MapDownloads(store, params)
  last_status = last_check = 0.

  def status(phase, **extra):
    nonlocal last_status
    job = downloads.job
    last_status = time.monotonic()
    params.put('OfflineMapStatus', {'phase': phase, 'region': job['region'] if job else '',
               'active': job is not None,
               'completed': len(job['done']) if job else 0, 'total': len(job['groups']) if job else 0,
               'installed_bytes': store.size(),
               'regions': {key: {k: v for k,v in value.items() if k != 'groups'} for key,value in store.manifest['regions'].items()}, **extra})

  def check(force=False):
    nonlocal last_check
    now = time.monotonic()
    if not force and now - last_check < .2:
      return
    last_check = now
    request = params.get('OfflineMapRequest')
    if request and request.get('action') == 'cancel':
      params.remove('OfflineMapRequest')
      raise DownloadCancelled
    automatic = downloads.job is not None and downloads.job.get('automatic', False)
    if automatic and params.get('OfflineMapUpdateSchedule', return_default=True) not in (1, 2):
      raise DownloadCancelled
    sm.update(0)
    check_download_allowed(params, sm, time.monotonic(), wifi_only=automatic)
    retry_in = downloads.job.get('retry_at', 0) - time.time() if automatic else 0  # noqa: TID251
    if retry_in > 0:
      raise DownloadPaused(f'Update failed; retry in {math.ceil(retry_in/60)} min')

  def progress(received, total):
    if time.monotonic() - last_status >= 1:
      status('Downloading', file_bytes=received, file_total=total)

  while True:
    request = params.get('OfflineMapRequest')
    try:
      if request:
        params.remove('OfflineMapRequest')
        try:
          downloads.request(request)
        except ValueError as error:
          # A stale UI request must not discard an in-progress download.
          status('Request rejected', error=str(error))
          continue
        if request.get('action') == 'delete':
          status('Maps removed')
      if downloads.job is None:
        sm.update(0)
        try:
          check_download_allowed(params, sm, time.monotonic(), wifi_only=True)
        except DownloadPaused:
          pass
        else:
          downloads.schedule(time.time())  # noqa: TID251
      if downloads.job is None:
        if time.monotonic() - last_status > 10:
          status('Ready')
        time.sleep(1)
        continue
      check(True)
      job = downloads.job
      pending = [g for g in job['groups'] if g not in job['done']]
      if not pending:
        downloads.finish(time.time())  # noqa: TID251
        status('Download complete')
        continue
      group = pending[0]
      status('Downloading')
      archive = fetch_archive(store, group, check, progress)
      status('Checking maps')
      store.install(archive, group, check)
      archive.unlink(missing_ok=True)
      downloads.complete_group(group)
    except DownloadCancelled:
      downloads.cancel(time.time())  # noqa: TID251
      status('Cancelled; downloaded maps kept')
    except DownloadPaused as error:
      if time.monotonic()-last_status > 2:
        status(str(error))
      time.sleep(1)
    except Exception as error:
      downloads.failed(time.time())  # noqa: TID251
      status('Download stopped', error=str(error)[:180] if isinstance(error, (ValueError, OSError)) else type(error).__name__)
      time.sleep(1)


if __name__ == '__main__':
  main()
