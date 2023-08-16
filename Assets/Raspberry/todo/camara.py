from time import sleep
from picamera import PiCamera
from io import BytesIO
from PIL import Image
import time

camera = None
def initCamera(iso = 800):
    global camera
    camera = PiCamera(resolution=(1280, 720), framerate=30)
    # Set ISO to the desired value
    camera.iso = iso
    # Wait for the automatic gain control to settle
    sleep(2)
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    # Finally, take several photos with the fixed settings
    #camera.capture_sequence(['image%02d.jpg' % i for i in range(10)],resize=(320, 240))

def capture():
    global camera
    # Create the in-memory stream
    stream = BytesIO()
    camera.capture(stream, format='jpeg',resize=(320, 240))
    # "Rewind" the stream to the beginning so we can read its content
    stream.seek(0)
    img = Image.open(stream)
    img.save(f"{int(time.time())}.jpeg")
    #img.show()

    #content = Image.tobytes()
    #content = stream.read()
    content = stream.getvalue()
    #byte_size = stream.tell()
    #info = f"Ahi te van {byte_size} B de datos:\n\r{content}"
    stream.close()
    return content
