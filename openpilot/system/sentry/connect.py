"""Optional owner authorization for Sentry's Connect upload requests.

The account token is entered locally, verified with comma, and kept in a 0600
file outside Params/route logs. Storage PUTs never receive this account token.
"""
import argparse
import getpass
import json
import os
from pathlib import Path
import sys
import time
import warnings
from urllib.parse import urlsplit

import requests

from openpilot.common.api import API_HOST
from openpilot.common.hardware.hw import Paths
from openpilot.common.utils import atomic_write
from openpilot.common.version import get_version

STATUS_PATH = Path('/dev/shm/openpilot_sentry_upload.json')
CONNECT_HOSTS = ('https://api.comma.ai', 'https://api.commadotai.com')


class SetupError(Exception):
  pass


def credential_path():
  return Path(Paths.persist_root())/'comma'/'sentry_connect.json'


def account_token(dongle_id):
  try:
    path = credential_path()
    if path.stat().st_mode & 0o077:
      return None
    data = json.loads(path.read_text())
    if data['dongle_id'] == dongle_id and data['api_host'] == API_HOST and API_HOST in CONNECT_HOSTS:
      token = data['token']
      if isinstance(token, str) and token and not any(c.isspace() for c in token):
        return token
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return None


def upload_url_request(dongle_id, key, token):
  return requests.post(API_HOST+'/v1/'+dongle_id+'/upload_urls/',
                       json={'paths': [key], 'expiry_days': 1}, timeout=10,
                       headers={'Authorization': 'JWT '+token, 'User-Agent': 'openpilot-'+get_version()},
                       allow_redirects=False)


def upload_url_data(response):
  data = json.loads(response.text)
  if not isinstance(data, list) or len(data) != 1 or not isinstance(data[0], dict):
    raise ValueError('Invalid Sentry batch upload URL response')
  item = data[0]
  url = item.get('url')
  if not isinstance(url, str) or urlsplit(url).scheme != 'https' or not urlsplit(url).netloc:
    raise ValueError('Missing or invalid Sentry batch upload URL')
  headers = item.get('headers', {})
  if not isinstance(headers, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in headers.items()):
    raise ValueError('Invalid Sentry batch upload headers')
  return url, {'x-ms-blob-type': 'BlockBlob', **headers}


def set_upload_status(phase):
  try:
    with atomic_write(str(STATUS_PATH), overwrite=True) as f:
      json.dump({'monoTime': time.monotonic(), 'phase': phase}, f)
  except OSError:
    pass  # Status reporting must never prevent an upload.


def upload_status():
  try:
    data = json.loads(STATUS_PATH.read_text())
    if 0 <= time.monotonic()-data['monoTime'] < 300 and isinstance(data['phase'], str):
      return data['phase']
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return 'Automatic'


def authorize_account(token, dongle_id, key):
  if API_HOST not in CONNECT_HOSTS:
    raise SetupError('Account authorization requires the official comma API host.')
  if not token or any(c.isspace() for c in token):
    raise SetupError('Enter the token alone, without spaces or an Authorization prefix.')
  headers = {'Authorization': 'JWT '+token, 'User-Agent': 'openpilot-'+get_version()}
  response = requests.get(API_HOST+'/v1.1/devices/'+dongle_id+'/', headers=headers, timeout=10, allow_redirects=False)
  if response.status_code != 200:
    raise SetupError(f'Account verification returned HTTP {response.status_code}. Authorization was not saved.')
  device = response.json()
  if not isinstance(device, dict) or device.get('dongle_id') != dongle_id or device.get('is_owner') is not True:
    raise SetupError('This account is not the owner of this comma. Authorization was not saved.')
  response = upload_url_request(dongle_id, key, token)
  if response.status_code != 200:
    raise SetupError(f'Account upload request returned HTTP {response.status_code}. Authorization was not saved.')
  upload_url_data(response)  # Verify a usable URL before persisting the token. No PUT here.
  path = credential_path()
  path.parent.mkdir(parents=True, exist_ok=True)
  with atomic_write(str(path), overwrite=True) as f:
    os.fchmod(f.fileno(), 0o600)
    json.dump({'dongle_id': dongle_id, 'api_host': API_HOST, 'token': token}, f)
    f.flush()
    os.fsync(f.fileno())
  set_upload_status('Account linked; queued')


def newest_saved_video(root):
  # Use a real, completed Sentry file for the authorization check.
  for marker in sorted(Path(root).glob('*/.sentry'), reverse=True):
    directory = marker.parent
    if any(directory.glob('*.lock')):
      continue
    for name in ('fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc'):
      try:
        if (directory/name).stat().st_size > 0:
          return directory.name+'/'+name
      except OSError:
        continue
  return None


def main():
  parser = argparse.ArgumentParser(description='Authorize automatic Sentry uploads with your own Connect account.')
  parser.add_argument('--clear', action='store_true', help='Remove the saved account token; return to device authorization')
  args = parser.parse_args()
  if args.clear:
    credential_path().unlink(missing_ok=True)
    set_upload_status('Account authorization removed')
    print('Saved account authorization removed. Uploads will use device authorization.')
    return 0
  from openpilot.common.params import Params
  dongle_id = Params().get('DongleId')
  key = newest_saved_video(Paths.log_root())
  if not dongle_id or not key:
    print('A registered comma and a completed Sentry video are required. Save an event first.')
    return 1
  print('Open https://jwt.comma.ai and sign in to the account that owns this comma.')
  print('Paste the account token at the hidden prompt below. Do not send it in chat.')
  print('After verification it is saved on this device with owner-only file permissions.')
  try:
    # Never fall back to echoing a secret when no interactive terminal exists.
    with warnings.catch_warnings():
      warnings.simplefilter('error', getpass.GetPassWarning)
      token = getpass.getpass('Connect token (hidden): ').strip()
    authorize_account(token, dongle_id, key)
  except SetupError as e:
    print(str(e))
    return 1
  except (requests.RequestException, ValueError, OSError):
    print('Could not complete verification or save authorization. Check the connection and try again.')
    return 1
  except (getpass.GetPassWarning, EOFError, KeyboardInterrupt):
    print('\nSetup cancelled. Use an interactive SSH terminal with hidden input.')
    return 1
  print('Ownership verified and video upload URL issued. Account authorization saved.')
  print('Automatic uploads use this account on the next retry (up to four minutes); no reboot is needed.')
  print('Remove authorization later with: python -m openpilot.system.sentry.connect --clear')
  return 0


if __name__ == '__main__':
  sys.exit(main())
