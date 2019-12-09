# includes
import serial, time, random

import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

import bibliopixel
from bibliopixel.drivers.SPI import APA102
from bibliopixel.drivers.SimPixel import SimPixel
from bibliopixel.layout import Strip

from colorsys import hls_to_rgb

strip_len = 50

print('Starting')

spi_rate = 16


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)


# setup
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
ser.flushInput()

rl = ReadLine(ser)


def new_pixel():
    return {'h': 0.0, 's': 0.5, 'l': 0.2}


def clamp_value(val):
    if val < 0:
        return 0
    if val > 1.0:
        return 0.999
    return val


def write_to_strip():
    for index in range(strip_len):
        rgb = [int(255 * clamp_value(v)) for v in
               hls_to_rgb(pixel_matrix[index]['h'] % 1.0, pixel_matrix[index]['l'], pixel_matrix[index]['s'])]

        pixel = index
        offset = pixel * 3
        # print pixel
        # logger.debug('%s %s %s', si, offset, pixel)
        pixel_raw_array[offset] = rgb[2]
        pixel_raw_array[offset + 1] = rgb[1]
        pixel_raw_array[offset + 2] = rgb[0]
    led_strip.setBuffer(pixel_raw_array)
    led_strip.update()


pixel_matrix = [new_pixel() for j in range(strip_len)]
pixel_raw_array = bytearray(3 * strip_len)

led_strip = Strip(APA102.APA102(strip_len, interface='PYDEV', spi_speed=spi_rate, gamma=bibliopixel.gamma.APA102), False, 255, 1)

frame = 0
lasttime = time.time()

# start serial read thread
while True:

    frame += 1
    if frame % 500 == 0:
        logger.debug('fps: {}'.format(500 / (time.time() - lasttime)))
        lasttime = time.time()
    try:
        # 		ser_bytes = ser.readline()
        ser_bytes = rl.readline()
        # 		print(ser_bytes)
        message = ser_bytes
    except:
        message = []

    if len(message) == 0:
        logger.debug('empty serial message')
        continue

    try:
        message = message.decode('ascii').split()
    except:
        continue

    if len(message) == 0 or message[0] != '2':
        continue

    try:
        message = [int(num, 16) for num in message]
    except:
        continue

    accel_data = message[2:5]

    # logger.debug(accel_data)

    # draw here
    for pi in range(strip_len):
        pixel_matrix[pi]['h'] = accel_data[0] / 255.0 + random.random()/10

    write_to_strip()

# start light pattern threads


# main loop start

# collect inputs with history

# detect motion patterns

# update light patterns

# send serial output

# main loop end
