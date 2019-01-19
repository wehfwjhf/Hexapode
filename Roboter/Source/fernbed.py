# coding: utf-8
#Thassilo Bücker, Alexander Orzol, Frederick Mueller
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Max Born, Ferdinand Heinemann
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2018/19

#Programm zum Empfangen der Daten ueber die serielle Schnittstelle des XBees

#----------imports--------------
import time
import serial
import Adafruit_BBIO.UART as UART

UART.setup("UART1")
port = serial.Serial("/dev/ttyO1", baudrate=38400, timeout=0.1)

#----------Variablen-----------
global rightStick
global leftStick
global button
rightStick='n'
leftStick='n'
button=[0,0,0,0,0,0,0,0]
#global RemCtrlBufferError
RemCtrlBufferError = 0

def lesen():
	reply = []
	port.flushInput()
	while True:
		if ord(port.read(1)) == 255:
			break
	#sleep(0.0001)
	for j in xrange (7):		
		reply.append(ord(port.read(1)))

	#print str((reply[0])) + " " + str((reply[1])) + " " +  str((reply[2])) + " " +  str((reply[3])) + " " +  str((reply[4])) + " " +  str((reply[5])) + " " +  str((reply[6])) 
	return reply

def getStatus():
	global RemCtrlBufferError

	StickRightSpeed = ' '
	StickRightAngle1 = ' '
	StickRightAngle2 = ' '
	StickLeftSpeed = ' '
	StickLeftAngle1 = ' '
	StickLeftAngle2 = ' '

	command = 'N'
	WaitingBytes = port.inWaiting()
	#print ("WaitBytes: " + str(WaitingBytes))

	if WaitingBytes > 72: #RemoteControl is sending 36 Bytes, get two full sequences
		timestart = time.time()
		WasteBuffer = port.readline() #read until /n, could be only parts of one sequence
		CheckBuffer = port.read(3) # read the first Bytes in Buffer

		#print ("Rest: " + str(WasteBuffer))
		#print ("Check: " + str(CheckBuffer))

		# Read all values from Buffer
		if CheckBuffer == '255': #check for intial 0xFF in sequence
			RemCtrlBuffer = port.read(33) # read the next 25 Bytes, the data, in Buffer
			print ("Buffer: " + str(RemCtrlBuffer))

			if RemCtrlBuffer[0] == 'S':
				StickRightSpeed = RemCtrlBuffer [1:4]
				StickRightSpeed = int(float(StickRightSpeed))
			else:
				StickRightSpeed = ' '
				print('Buffer corrupted')

			if 	RemCtrlBuffer[4:6] == 'A1':
				StickRightAngle1 = RemCtrlBuffer [6:9]
				StickRightAngle1 = int(float(StickRightAngle1))
			else:
				StickRightAngle1 = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[9:11] == 'A2':
				StickRightAngle2 = RemCtrlBuffer [11:14]
				StickRightAngle2 = int(float(StickRightAngle2))
			else:
				StickRightAngle2 = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[14] == 'S':
				StickLeftSpeed = RemCtrlBuffer [15:18]
				StickLeftSpeed = int(float(StickLeftSpeed))
			else:
				StickLeftSpeed = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[18:20] == 'A1':
				StickLeftAngle1 = RemCtrlBuffer[20:23]
				StickLeftAngle1 = int(float(StickLeftAngle1))
			else:
				StickLeftAngle1 = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[23:25] == 'A2':
				StickLeftAngle2 = RemCtrlBuffer[25:28]
				StickLeftAngle2 = int(float(StickLeftAngle2))
			else:
				StickLeftAngle2 = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[28] == 'B':
				Buttons = RemCtrlBuffer [29:32]
				Buttons = int(float(Buttons))
			else:
				Buttons = ' '
				print('Buffer corrupted')

		else:
			RemCtrlBuffer = ' '
			RemCtrlBufferError = RemCtrlBufferError + 1
			print ("Buffer corrupted")

		port.flushInput()

		# set command
		command_left = getStickCommand(StickLeftSpeed , StickLeftAngle1, StickLeftAngle2)
		print ('Command: ' + command_left)

		timestop = time.time()

		timediff = timestop - timestart
		#try:
			#reply.append(port.read(56))
			#command= reply[0][0] #asdasd
		#except():
			#attemptFailed = 1
			#print ("getStatusFailed")
	else:
		command_left = ' '
	#if attemptFailed == 0:
		#print ("getStatusSuccess")

	

	return command_left

def getRightStick():
	global rightStick
	return rightStick


def getStickCommand(Speed, Angle1, Angle2):
	ret_val = ' '
	Angle = Angle1 + Angle2

	if Speed > 0:
		if Angle == 0:
			ret_val = 'W'
		elif Angle == 90:
			ret_val = 'A'
		elif Angle == 180:
			ret_val = 'S'
		elif Angle == 270:
			ret_val = 'D'
		else:
			ret_val = ' '

	return ret_val

#lesen()
#getStatus();
#print "rightStick: "+str(rightStick)
#print "leftStick: "+str(leftStick)
#for i in xrange(8):
#	print "Button "+str(i)+": "+str( button[i]);
