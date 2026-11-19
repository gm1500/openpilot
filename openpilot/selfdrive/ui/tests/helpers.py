class FakeSM(dict):
  def __init__(self, **messages):
    super().__init__(messages)
    self.services = list(messages)
    self.valid = dict.fromkeys(messages, True)
    self.recv_frame = dict.fromkeys(messages, 10)
    self.recv_time = dict.fromkeys(messages, 100.)
    self.logMonoTime = dict.fromkeys(messages, int(100e9))
