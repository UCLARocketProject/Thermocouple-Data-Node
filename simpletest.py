#Copyright (c) 2016 John Robinson
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
from threading import Thread
import time
from multiprocessing import Queue
import sys

# Local Imports
from MAX31856_Driver import MAX31856 as MAX31856

logging.basicConfig(filename='log.txt', level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
_logger = logging.getLogger(__name__)

# Raspberry Pi hardware SPI configuration.
SPI_PORT   = 0
SPI_DEVICE = 0
sensor = MAX31856(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE), cs=[11, 7, 37, 3, 13], tc_type=MAX31856.MAX31856_J_TYPE)

q = Queue()
sleepTime1 = 0.5
sleepTime2 = 2.5
senderDie = False
counter = 0

def sender():
  global counter
  while not senderDie:
    try:
      tempRead = q.get(timeout=1)
      #Change to send data to server
      print tempRead
      counter -= 1
      #print "Counter: ", str(counter)
    except:
      print "Unexpected Sender Error: ", sys.exc_info()[0]

t = Thread(target=sender)
t.start()
backoffSize = 1000000 #SET TO BETTER VALUE

# Loop printing measurements every second.
print('Press Ctrl-C to quit.')
try:
    while True:
        temp = sensor.readTempC()
        for i, t in enumerate(temp):
           tempRead = str(i+1) + ", " + str(time.time()) + ", " + str(t)
           q.put(tempRead)
           counter += 1
        if counter > backoffSize:
          sleepTime = sleepTime2
        else:
          sleepTime = sleepTime1
        time.sleep(sleepTime)
        #print("")
        #print("==================")
        #print("")
except:
    print "Unexpected Reader Error: ", sys.exc_info()[0]
    senderDie = True
    rpi_gpio.cleanup()
