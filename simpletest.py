# Copyright (c) 2016 John Robinson
# Author: John Robinson
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# Global Imports
import logging
import time
import Adafruit_GPIO.SPI as SPI
import RPi.GPIO as rpi_gpio

# Local Imports
from Adafruit_MAX31856 import MAX31856 as MAX31856

logging.basicConfig(filename='log.txt', level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
_logger = logging.getLogger(__name__)

# Raspberry Pi hardware SPI configuration.
SPI_PORT   = 0
SPI_DEVICE = 0
sensor = MAX31856(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE), cs=[16, 3], tc_type=MAX31856.MAX31856_J_TYPE)

# Loop printing measurements every second.
print('Press Ctrl-C to quit.')
try:
    while True:
        temp = sensor.readTempC()
        for t in temp:
            print('Thermocouple Temperature: {0:0.3F}*C'.format(t))
        time.sleep(1.0)
except:
    rpi_gpio.cleanup()
