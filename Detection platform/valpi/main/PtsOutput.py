import io
import picamera
import time
import datetime

class PtsOutput(object):
   def __init__(self, camera, video_filename, pts_filename):
      self.camera = camera
      self.video_output = io.open(video_filename, 'wb')
      self.pts_output = io.open(pts_filename, 'wb')
      self.start_time = None

   def write(self, buf):
      self.video_output.write(buf)
      if self.camera.frame.complete and self.camera.frame.timestamp:
         self.pts_output.write(str(datetime.datetime.now())+"\n")

   def flush(self):
      self.video_output.flush()
      self.pts_output.flush()

   def close(self):
      self.video_output.close()
      self.pts_output.close()

# with picamera.PiCamera() as camera:
#    camera.resolution = (640,480)
#    camera.framerate = 90
#    camera.rotation = 180
#    camera.start_recording(PtsOutput(camera, 'test_30fps.h264', 'pts.txt'), format='h264' )
#    camera.wait_recording(30)
#    camera.stop_recording()
