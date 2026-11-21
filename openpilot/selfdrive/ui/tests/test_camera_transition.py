from types import SimpleNamespace as NS

from openpilot.selfdrive.ui.onroad import cameraview as camera


def test_engagement_cancels_a_pending_wide_stream_before_any_frame_arrives():
  """Reversing the request must not briefly show an outdated wide frame."""
  view = camera.CameraView.__new__(camera.CameraView)
  narrow = camera.VisionStreamType.VISION_STREAM_NARROW_ROAD
  wide = camera.VisionStreamType.VISION_STREAM_WIDE_ROAD
  view._name = 'camerad'
  view._stream_type = narrow
  view._clients = {narrow: NS(), wide: NS()}
  view.client = view._clients[narrow]
  view.available_streams = set(view._clients)
  view.frame = None
  view._target_client = view._target_stream_type = None
  view._switching = False
  view.switch_stream(wide)
  assert view._switching and view._target_client is view._clients[wide]
  view.switch_stream(narrow)
  assert not view._switching and view._target_client is None
  assert view._target_stream_type is None and view.stream_type == narrow
