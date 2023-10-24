#!/usr/bin/env python3

#Thomas Flayols - feb 2022
#https://github.com/thomasfla/odri-spi-rpi
#https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md
import struct
from pyftdi.spi import SpiController
from math import pi
import time

def crc32(buf):
  crc=0xffffffff
  for val in buf:
    crc ^= val << 24
    for _ in range(8):
      crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
  return crc

def checkcrc(buf):
  crc = crc32(buf[:-4])
  if (crc&0xffff == buf[-4]*256+buf[-3] and (crc&0xFFFF0000)>>16 == buf[-2]*256+buf[-1] ): #todo, clean
    return True
  return False




class SPIuDriver:
  def __init__(self, waitForInit = True, absolutePositionMode = False, offsets=[0.0,0.0]):

    #Initialise SPI
    self.ctrl = SpiController()
    #self.ctrl.configure('ftdi://ftdi:232h:bus:device/1')
    self.ctrl.configure('ftdi://ftdi:232h:01:03/1') 
    self.spi = self.ctrl.get_port(0, mode=0, freq=8000000)

    # Allocate all variables
    self.is_system_enabled = 0
    self.error_code = 0
    self.position0 = 0
    self.position1 = 0
    self.velocity0 = 0
    self.velocity1 = 0
    self.current0 = 0
    self.current1 = 0
    self.is_enabled0 = 0
    self.is_enabled1= 0
    self.is_ready0 = 0
    self.is_ready1 = 0
    self.has_index_been_detected0 = 0
    self.has_index_been_detected1 = 0
    self.index_toggle_bit0 = 0
    self.index_toggle_bit1 = 0

    self.offset0 = offsets[0]
    self.offset1 = offsets[1]

    self.EI1OC = 1 if absolutePositionMode else 0
    self.EI2OC = 1 if absolutePositionMode else 0
    self.refPosition0 = 0
    self.refPosition1= 0
    self.refVelocity0= 0
    self.refVelocity1= 0
    self.refCurrent0= 0
    self.refCurrent1= 0
    self.iSatCurrent0= 5.0
    self.iSatCurrent1= 5.0
    self.kp0= 0
    self.kp1= 0
    self.kd0= 0
    self.kd1= 0
    self.timeout = 20
    self.count_crc = 0

    self.error = -1
    #wait for system enable
    if waitForInit:
      print(">> Calibrating motor, please wait")
      while(not self.is_ready0):
        self.transfer()
        time.sleep(0.001)
    if absolutePositionMode:
      if (not self.has_index_been_detected0 or not self.has_index_been_detected1):
          print(">> Waiting for index pulse to have absolute position reference, please move the motors manualy")
          displayedIndex0 = False
          displayedIndex1 = False
          while(not self.has_index_been_detected0): # or not self.has_index_been_detected1):
            self.transfer()
            if self.has_index_been_detected0 == True and displayedIndex0 == False:
                print (" >> Index 0 detected !")
                displayedIndex0 = True
            if self.has_index_been_detected1 == True and displayedIndex1 == False:
                print (" >> Index 1 detected !")
                displayedIndex1 = True
            time.sleep(0.001)
    print ("ready!")
  def transfer(self):

    #generate command packet
    ES = 1
    EM1 = 1	
    EM2	= 1
    EPRE = 1
    EI1OC = self.EI1OC
    EI2OC	= self.EI2OC
    mode = (ES << 7) | (EM1 << 6) | (EM2<<5)|(EPRE<<4)|(EI1OC<<3)|(EI2OC<<2)
    timeout = self.timeout
    rawRefPos0 = int((self.refPosition0-self.offset0)/(2.*pi)*(1<<24) )
    rawRefPos1 = int((self.refPosition1-self.offset1)/(2.*pi)*(1<<24))
    # rawRefVel0 = int(self.refVelocity0*(1<<11)*60.0/(2000*pi))
    rawRefVel1 = int(self.refVelocity1*(1<<11)*60.0/(2000*pi))
    # rawRefIq0 = int(self.refCurrent0*(1<<10))
    rawRefIq1 = int(self.refCurrent1*(1<<10))
    rawIsat0 = int(self.iSatCurrent0*(1<<3))
    rawIsat1 = int(self.iSatCurrent1*(1<<3))
    # rawKp0 = int(self.kp0*(1<<11)*(2*pi) )


    rawRefVel0  = int(self.refVelocity0 * (1<<11))
    rawRefIq0   = int(self.refCurrent0  * (1<<11))
    rawKp0      = int(self.kp0  * (1<<11))


    rawKp1 = int(self.kp1*(1<<11)*(2*pi) )
    rawKd0 = int(self.kd0*(1<<10)*(2*pi*1000./60.0) )
    rawKd1 = int(self.kd1*(1<<10)*(2*pi*1000./60.0) )
    commandPacket = bytearray(struct.pack(">BBiihhhhHHHHBBHI",mode,timeout,rawRefPos0,rawRefPos1,rawRefVel0,rawRefVel1,rawRefIq0,rawRefIq1,rawKp0,rawKp1,rawKd0,rawKd1, rawIsat0, rawIsat1, 0,0))
    crc=crc32(commandPacket[:-4])
    commandPacket[-4]=(crc>>24)&0xff
    commandPacket[-3]=(crc>>16)&0xff
    commandPacket[-2]=(crc>>8)&0xff
    commandPacket[-1]=(crc)&0xff
    sensorPacket = bytearray(self.spi.exchange(commandPacket, 34, duplex=True))
    #print(commandPacket.hex(),checkcrc(commandPacket))
    #print(sensorPacket.hex(),checkcrc(sensorPacket))
    if checkcrc(sensorPacket):
    #decode received sensor packet
      data = struct.unpack(">H H i i h h h h h h H H xxxxxx",sensorPacket)
      self.is_system_enabled        = data[0]&0b1000000000000000 != 0
      self.is_enabled0              = data[0]&0b0100000000000000 != 0
      self.is_ready0                = data[0]&0b0010000000000000 != 0
      self.is_enabled1              = data[0]&0b0001000000000000 != 0
      self.is_ready1                = data[0]&0b0000100000000000 != 0
      self.has_index_been_detected0 = data[0]&0b0000010000000000 != 0
      self.has_index_been_detected1 = data[0]&0b0000001000000000 != 0

      self.error             = data[0]&0b0000000000001111
      self.position0 = data[2] / (1<<24) * 2.0 * pi + self.offset0
      self.position1 = data[3] / (1<<24) * 2.0 * pi + self.offset1


      self.velocity0 = data[4] / (1<<11)
      self.velocity1 = data[5] / (1<<11)
      self.current0 = data[6]  / (1<<11)
      self.current1 = data[7]  / (1<<11)
      self.tension0 = data[8]  / (1<<11)
      self.tension1 = data[9]  # / (1<<11)
      self.supply0 = data[10]  / (1<<11)
      self.supply1 = data[11]  / (1<<24)
      #print("velocity =", self.velocity0)
      #print("cur =", self.current0)
      self.count_crc = 0
    else:
      self.count_crc += 1
      if self.count_crc == 10:
        raise(Exception(f"Error: sensor frame is corrupted is uDriver powered on?"))
    if (self.error!=0):
        raise(Exception(f"Error from motor driver: Error {self.error}"))
    #print(sensorPacket.hex())
  def goto(self,p0,p1):
        Kp = 3.0
        Kd = 0.06
        Ki = 0*50
        pid0 = PID(Kp,Ki,Kd)
        pid1 = PID(Kp,Ki,Kd)
        p0_start = self.position0
        p1_start = self.position1
        eps = 0.01
        dt=0.001
        T=1000
        t = time.perf_counter()
        for i in range(T):
            goalPosition0 = (i/T) * p0 + (1 - i/T) * p0_start
            goalPosition1 = (i/T) * p1 + (1 - i/T) * p1_start
            self.transfer() #transfer
            #self.refCurrent0 = 1.0*(goalPosition0-self.position0)-0.1*self.velocity0
            #self.refCurrent1 = 1.0*(goalPosition1-self.position1)-0.1*self.velocity1
            self.refCurrent0 = pid0.compute(self.position0,self.velocity0,goalPosition0,0.0)
            self.refCurrent1 = pid1.compute(self.position1,self.velocity1,goalPosition1,0.0)
            #wait for next control cycle
            t +=dt
            while(time.perf_counter()-t<dt):
                pass
        self.refCurrent0 = 0
        self.refCurrent1 = 0
        self.transfer()
  def stop(self):
        self.EI1OC = 0
        self.EI2OC = 0
        self.refPosition0 = 0
        self.refPosition1= 0
        self.refVelocity0= 0
        self.refVelocity1= 0
        self.refCurrent0= 0
        self.refCurrent1= 0
        self.iSatCurrent0= 0
        self.iSatCurrent1= 0
        self.kp0= 0
        self.kp1= 0
        self.kd0= 0
        self.kd1= 0
        self.timeout = 0
        dt=0.001
        t = time.perf_counter()
        for i in range(100):
            self.transfer() #transfer
            #wait for next control cycle
            t +=dt
            while(time.perf_counter()-t<dt):
                pass

class PID:
    def __init__(self,Kp,Ki,Kd, sat = 2.0, dt=0.001):
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.sat = sat
        self.u = 0.0
        self.ierr = 0.0
        self.dt = dt
    def compute(self, p, v, p_ref=0.0, v_ref=0.0):
        perr = p_ref-p
        verr = v_ref-v
        self.ierr = self.ierr + perr * self.dt
        if (self.ierr > self.sat) :
            self.ierr = self.sat
        if (self.ierr < -self.sat) :
            self.ierr = -self.sat
        self.u = self.Kp * perr + self.Kd * verr + self.Ki * self.ierr
        if (self.u > self.sat) :
            self.u = self.sat
        if (self.u < -self.sat) :
            self.u = -self.sat
        return self.u