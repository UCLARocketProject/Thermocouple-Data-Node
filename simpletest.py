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
import MySQLdb

# Local Imports
from MAX31856_Driver import MAX31856 as MAX31856

logging.basicConfig(filename='log.txt', level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
_logger = logging.getLogger(__name__)

# Raspberry Pi hardware SPI configuration.
SPI_PORT   = 0
SPI_DEVICE = 0
sensor = MAX31856(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE), cs=[11, 7, 37, 3, 13], tc_type=MAX31856.MAX31856_J_TYPE)

q = Queue()
sleepTime = 0.5
senderDie = False
counter = 0

def sender():
  db = MySQLdb.connect(host="127.0.0.1", user="root", passwd="rockets", db="testing")
  cursor = db.cursor()
  sql = "CREATE TABLE IF NOT EXISTS thermocouples (abs_t DOUBLE, rel_t INT, tmp1 DOUBLE, tmp2 DOUBLE, tmp3 DOUBLE, tmp4 DOUBLE, tmp5 DOUBLE)"
  cursor.execute(sql)
  while not senderDie:
    try:
      abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5 = q.get(timeout=1)
      print abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5
      sql = "INSERT INTO thermocouples (abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5) VALUES ('%f', '%i', '%f', '%f', '%f', '%f', '%f')" % (abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5)
      try:
        cursor.execute(sql)
        db.commit()
      except:
        db.rollback()
    except:
      print "Unexpected Sender Error: ", sys.exc_info()[0]

t = Thread(target=sender)
startTime = time.time()
t.start()

try:
    while True:
        temp = sensor.readTempC()
        timeCurrent = time.time()
        #for i, t in enumerate(temp):
        #tempRead = str(i+1) + ", " + str(time.time()) + ", " + str(t)
        tempRead = (timeCurrent, int(1000*(timeCurrent - startTime)), temp[0], temp[1], temp[2], temp[3], temp[4])
        q.put(tempRead)
        time.sleep(sleepTime)
except:
    print "Unexpected Reader Error: ", sys.exc_info()[0]
    senderDie = True
    rpi_gpio.cleanup()
