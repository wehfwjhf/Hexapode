# coding: utf-8
#Thassilo Bücker, Alexander Orzol, Frederick Mueller
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Programm zum Empfangen der Daten ueber die serielle Schnittstelle des XBees

#----------imports--------------
from time import sleep
import serial
import Adafruit_BBIO.UART as UART

UART.setup("UART1")
port = serial.Serial("/dev/ttyO1", baudrate=9600, timeout=0.1)

#----------Variablen-----------
global rightStick
global leftStick
global button
rightStick='n'
leftStick='n'
button=[0,0,0,0,0,0,0,0]



def lesen():
	reply = []
	port.flushInput()
	while True:
		if ord(port.read(1)) == 255:
			break
	sleep(0.0001)
	for j in xrange (7):		
		reply.append(ord(port.read(1)))

	#print str((reply[0])) + " " + str((reply[1])) + " " +  str((reply[2])) + " " +  str((reply[3])) + " " +  str((reply[4])) + " " +  str((reply[5])) + " " +  str((reply[6])) 
	return reply

def getStatus():
	command = 'N'
	attemptFailed = 1
	reply = []
	if port.inWaiting()>0:
		
		attemptFailed = 0
		try:
			reply.append(port.read(3))
			command= reply[0][0]
		except():
			attemptFailed = 1 
			print "getStatusFailed"
	else:
		command = ' '
	if attemptFailed == 0:
		print "getStatusSuccess"
	port.flushInput()
	

	return command

def getRightStick():
	global rightStick
	return rightStick
#lesen()
#getStatus();
#print "rightStick: "+str(rightStick)
#print "leftStick: "+str(leftStick)
#for i in xrange(8):
#	print "Button "+str(i)+": "+str( button[i]);
