#!/usr/bin/python
#The MIT License (MIT)
#
#Copyright (c) 2015 Stephen P. Smith
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import time, math
import RPi.GPIO as GPIO
import RPi.GPIO as rpi_gpio
from threading import Thread
from multiprocessing import Queue
import sys
import MySQLdb
from parameters import *

class max31856(object):
	"""Read Temperature on the Raspberry PI from the MAX31856 chip using GPIO
	   Any pins can be used for CS (chip select), MISO, MOSI and CLK
	"""

	def __init__(self, csPins, misoPin, mosiPin, clkPin):
		self.csPins = csPins
		self.misoPin = misoPin
		self.mosiPin = mosiPin
		self.clkPin = clkPin
		self.setupGPIO()
		#
		# Config Register 2
		# ------------------
		# bit 7: Reserved                                -> 0 
		# bit 6: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 5: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 4: Averaging Mode 1 Sample                 -> 0 (default)
		# bit 3: Thermocouple Type -> K Type (default)   -> 0 (default)
		# bit 2: Thermocouple Type -> K Type (default)   -> 0 (default)
		# bit 1: Thermocouple Type -> K Type (default)   -> 1 (default)
		# bit 0: Thermocouple Type -> K Type (default)   -> 1 (default)
		#
		#Uncomment one of the following to select thermocouple type
		#self.writeRegister(1, 0x00) #for B Type
		#self.writeRegister(1, 0x01) #for E Type
		#self.writeRegister(1, 0x02) #for J Type
                for i in range(5):
                        self.writeRegister(1, 0x03, i) #for K Type
		#self.writeRegister(1, 0x04) #for N Type
		#self.writeRegister(1, 0x05) #for R Type
		#self.writeRegister(1, 0x06) #for S Type
		#self.writeRegister(1, 0x07) #for T Type
		
	def setupGPIO(self):
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.csPins[0], GPIO.OUT)
		GPIO.setup(self.csPins[1], GPIO.OUT)
		GPIO.setup(self.csPins[2], GPIO.OUT)
		GPIO.setup(self.csPins[3], GPIO.OUT)
		GPIO.setup(self.csPins[4], GPIO.OUT)
		GPIO.setup(self.misoPin, GPIO.IN)
		GPIO.setup(self.mosiPin, GPIO.OUT)
		GPIO.setup(self.clkPin, GPIO.OUT)

		GPIO.output(self.csPins[0], GPIO.HIGH)
		GPIO.output(self.csPins[1], GPIO.HIGH)
		GPIO.output(self.csPins[2], GPIO.HIGH)
		GPIO.output(self.csPins[3], GPIO.HIGH)
		GPIO.output(self.csPins[4], GPIO.HIGH)
		GPIO.output(self.clkPin, GPIO.LOW)
		GPIO.output(self.mosiPin, GPIO.LOW)	
	
	def readThermocoupleTemp(self):
		self.requestTempConv()
                temp_C = []
		# read 4 registers starting with register 12
		for i in range(5):
			out = self.readRegisters(0x0c, 4, i) 
			
			[tc_highByte, tc_middleByte, tc_lowByte] = [out[0], out[1], out[2]]	
			temp = ((tc_highByte << 16) | (tc_middleByte << 8) | tc_lowByte) >> 5
			
			if (tc_highByte & 0x80):
				temp -= 0x80000
			
			temp_C.append(temp * 0.0078125)
			
			fault = out[3]
			
			if ((fault & 0x80) == 1):
				raise FaultError("Cold Junction Out-of-Range")
			if ((fault & 0x40) == 1):
				raise FaultError("Thermocouple Out-of-Range")
			if ((fault & 0x20) == 1):
				raise FaultError("Cold-Junction High Fault")
			if ((fault & 0x10) == 1):
				raise FaultError("Cold-Junction Low Fault")
			if ((fault & 0x08) == 1):
				raise FaultError("Thermocouple Temperature High Fault")
			if ((fault & 0x04) == 1):
				raise FaultError("Thermocouple Temperature Low Fault")
			if ((fault & 0x02) == 1):
				raise FaultError("Overvoltage or Undervoltage Input Fault")
			if ((fault & 0x01) == 1):
				raise FaultError("Thermocouple Open-Circuit Fault")
				
		return temp_C
				
	def readJunctionTemp(self):
		self.requestTempConv()
		
		# read 3 registers starting with register 9
		out = self.readRegisters(0x09, 3)
		
		offset = out[0]
		
		[junc_msb, junc_lsb] = [out[1], out[2]]
		
		temp = ((junc_msb << 8) | junc_lsb) >> 2
		temp = offset + temp
		
		if (junc_msb & 0x80):
			temp -= 0x4000
		
		temp_C = temp * 0.015625
		
		return temp_C
	
	def requestTempConv(self):
		#
		# Config Register 1
		# ------------------
		# bit 7: Conversion Mode                         -> 0 (Normally Off Mode)
		# bit 6: 1-shot                                  -> 1 (ON)
		# bit 5: open-circuit fault detection            -> 0 (off)
		# bit 4: open-circuit fault detection            -> 0 (off)
		# bit 3: Cold-junction temerature sensor enabled -> 0 (default)
		# bit 2: Fault Mode                              -> 0 (default)
		# bit 1: fault status clear                      -> 1 (clear any fault)
		# bit 0: 50/60 Hz filter select                  -> 0 (60Hz)
		#
		# write config register 0
                for i in range(5):
                        self.writeRegister(0, 0x42, i)
		# conversion time is less than 150ms
		time.sleep(.05) #give it 200ms for conversion

	def writeRegister(self, regNum, dataByte, csNum):
		GPIO.output(self.csPins[csNum], GPIO.LOW)
		
		# 0x8x to specify 'write register value'
		addressByte = 0x80 | regNum;
		
		# first byte is address byte
		self.sendByte(addressByte)
		# the rest are data bytes
		self.sendByte(dataByte)

		GPIO.output(self.csPins[csNum], GPIO.HIGH)

		
	def readRegisters(self, regNumStart, numRegisters, csNum):
		out = []
		GPIO.output(self.csPins[csNum], GPIO.LOW)
		
		# 0x to specify 'read register value'
		self.sendByte(regNumStart)
		
		for byte in range(numRegisters):	
			data = self.recvByte()
			out.append(data)

		GPIO.output(self.csPins[csNum], GPIO.HIGH)
		return out

	def sendByte(self,byte):
		for bit in range(8):
			GPIO.output(self.clkPin, GPIO.HIGH)
			if (byte & 0x80):
				GPIO.output(self.mosiPin, GPIO.HIGH)
			else:
				GPIO.output(self.mosiPin, GPIO.LOW)
			byte <<= 1
			GPIO.output(self.clkPin, GPIO.LOW)

	def recvByte(self):
		byte = 0x00
		for bit in range(8):
			GPIO.output(self.clkPin, GPIO.HIGH)
			byte <<= 1
			if GPIO.input(self.misoPin):
				byte |= 0x1
			GPIO.output(self.clkPin, GPIO.LOW)
		return byte	

class FaultError(Exception):
	pass


q = Queue()
senderDie = False

def sender():
        db = MySQLdb.connect(host="127.0.0.1", user="root", passwd="pinetree", db="testing")
        cursor = db.cursor()
        sql = "CREATE TABLE IF NOT EXISTS thermocouple (abs_t DOUBLE, rel_t INT, " + tc1Name + " FLOAT, " + tc2Name + " FLOAT, " + tc3Name + " FLOAT, " + tc4Name + " FLOAT, " + tc5Name + " FLOAT)"
        cursor.execute(sql)
        while not senderDie:
                try:
                        abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5 = q.get(timeout=1)
                        print "%20s %20s %20s %20s %20s %20s %20s\n" % tempRead
                        sql = "INSERT INTO thermocouple (abs_t, rel_t, %s, %s, %s, %s, %s) VALUES ('%f', '%i', '%f', '%f', '%f', '%f', '%f')" % (tc1Name, tc2Name, tc3Name, tc4Name, tc5Name, abs_t, rel_t, tmp1, tmp2, tmp3, tmp4, tmp5)
                        try:
                                cursor.execute(sql)
                                db.commit()
                        except:
                                db.rollback()
                except:
                        print "Unexpected Sender Error: ", sys.exc_info()[0]


                                                                

if __name__ == "__main__":

	import max31856
  #V5 = 2, V3.3 = 17. G = 6, 9, 14
	csPins = [11, 13, 15, 12, 16]
	misoPin = 5
	mosiPin = 7
	clkPin = 3
	max = max31856.max31856(csPins,misoPin,mosiPin,clkPin)

        t = Thread(target=sender)
        startTime = time.time()
        t.start()

        try:
                while True:
                        temp = max.readThermocoupleTemp()
                        timeCurrent = time.time()
                        tempRead = (timeCurrent, int(1000*(timeCurrent - startTime)), temp[0], temp[1], temp[2], temp[3], temp[4])
                        q.put(tempRead)
        except:
                print "Unexpected Reader Error: ", sys.exc_info()[0]
                senderDie = True
                GPIO.cleanup()


