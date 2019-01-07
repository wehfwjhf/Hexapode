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
	command = 'N'
	WaitingBytes = port.inWaiting()
	#print ("WaitBytes: " + str(WaitingBytes))

	if WaitingBytes > 50: #RemoteControl is sending 23 Bytes, get two full sequences
		timestart = time.time()
		WasteBuffer = port.readline() #read until /n, could be only parts of one sequence
		CheckBuffer = port.read(3) # read the first Bytes in Buffer

		#print ("Rest: " + str(WasteBuffer))
		#print ("Check: " + str(CheckBuffer))

		# Read all values from Buffer
		if CheckBuffer == '255': #check for intial 0xFF in sequence
			RemCtrlBuffer = port.read(20) # read the next 20 Bytes, the data, in Buffer
			print ("Buffer: " + str(RemCtrlBuffer))

			if RemCtrlBuffer[0] == 'x':
				StickRightVert = RemCtrlBuffer [1:4]
				StickRightVert = int(float(StickRightVert))
			else:
				StickRightVert = ' '
				print('Buffer corrupted')

			if 	RemCtrlBuffer[4] == 'x':
				StickRightHorz = RemCtrlBuffer [5:8]
				StickRightHorz = int(float(StickRightHorz))
			else:
				StickRightHorz = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[8] == 'x':
				StickLeftVert = RemCtrlBuffer [9:12]
				StickLeftVert = int(float(StickLeftVert))
			else:
				StickLeftVert = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[12] == 'x':
				StickLeftHorz = RemCtrlBuffer [13:16]
				StickLeftHorz = int(float(StickLeftHorz))
			else:
				StickLeftHorz = ' '
				print('Buffer corrupted')

			if RemCtrlBuffer[16] == 'x':
				Buttons = RemCtrlBuffer [17:21]
				Buttons = int(float(Buttons))
			else:
				Buttons = ' '
				print('Buffer corrupted')

		else:
			RemCtrlBuffer = ' '
			RemCtrlBufferError = RemCtrlBufferError + 1
			print ("Buffer corrupted")

		port.flushInput()

		# Rate the data
		StickLeftVert = getBoolStick(StickLeftVert)
		StickLeftHorz = getBoolStick(StickLeftHorz)
		StickRightVert = getBoolStick(StickRightVert)
		StickRightHorz = getBoolStick(StickRightHorz)

		# set command
		command_left = getStickCommand(StickLeftVert,StickLeftHorz)
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

def getBoolStick(value):
	if value > 200:
		return 1
	elif value < 50:
		return -1
	else:
		return 0

def getStickCommand(vert_value, horz_value):
	ret_val = ' '
	if horz_value == 0:
		if vert_value == 1:
			ret_val = 'W'
		elif vert_value == -1:
			ret_val = 'S'
		else:
			ret_val = ' '

	if vert_value == 0:
		if horz_value == 1:
			ret_val = 'D'
		elif horz_value == -1:
			ret_val = 'A'
		else:
			ret_val = ' '

	return ret_val

#lesen()
#getStatus();
#print "rightStick: "+str(rightStick)
#print "leftStick: "+str(leftStick)
#for i in xrange(8):
#	print "Button "+str(i)+": "+str( button[i]);
