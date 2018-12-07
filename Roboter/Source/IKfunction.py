#Thassilo Bücker, Alexander Orzol, Frederick Mueller, Moritz Kolb
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

# Dieses Programm enthaelt alle erweiterten Bewegungsfunktionen.
# Hier befinden sich alle Betriebsarten, welche auf die Basisfunktionen der IK.py zugreifen.

from tastfunktion import *
from fernbed import *
from ax12 import Ax12
import curses
from IK import *
servos = Ax12()

'''
#alt/obsolet
def Automatikbetrieb():
	global lastAngles	
	global cGaitXZ
	global cSyncSpeed
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	cSyncSpeed = False
	counterError = 0
	counterReading = 0
	for i in xrange(100):
		isPosition = lastAngles[1]	
		start = time.time(); 
		while (abs(isPosition - cGaitXZ[1])>1):
			try: 
				counterReading += 1
				isPositon = servos.readPosition(1)
			except:
				print "error"
				counterError += 1
				sleep(0.01)
		end = time.time()
		print (end - start)
		#end=time.time()
		#print(end-start)
		#start = time.time()
		Gait(walkX=False, walkZ=True, backwards=False)
		MoveIK(x,y,z,rotx,roty,rotz)
		sleep(0.01)
		i += 1;
	print counterReading
	print counterError
'''

def newAutomatikbetrieb() :
	#Automatische Vorwaertsbewegung mit Pruefung ob mindestens 10 Servos die Zielposition
	#erreicht haben, bevor ein neuer Bewegungsbefehl abgesetzt wird.
	PositionToReach = getLastAngles()	
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	j=0
	k=0
	while(getStatus()  != "1"):
		MotorsReady = 0
		currentPositions = readAllPositions();
		k += 1
		print currentPositions
		print PositionToReach
		for i in xrange(0,18):
			if abs(PositionToReach[i] - currentPositions[i]) < 5 or currentPositions[i] == 1:
				MotorsReady += 1
				#print MotorsReady
			
				 
		if(MotorsReady >= 10):
			Gait(walkX=False, walkZ=True, backwards=False)
			MoveIK(x,y,z,rotx,roty,rotz)
			PositionToReach = getLastAngles()
			print k
			k = 0
		sleep(0.01)
		j += 1


def Fernbedienungsbetrieb():
	#Entspricht dem Handbetrieb ueber Konsole, jedoch mit geringerem
	#Funktionsumfang und Ansteuerung ueber die Fernbedienung.
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	freeToMove = 0;
	oldCommand = 'N'
	stamp=time.time();
	while(True):
		stamp2 = time.time();
		if (stamp2 - stamp)*1000  >= 50:
			#stamp5 = time.time()
			stamp = time.time();
			#print "attempting to read"
			command = getStatus();
			if(command == ' '):
				command = oldCommand;  
			oldCommand = command
			freeToMove = 1;
			#print str((time.time()-stamp)*1000)
		else:
			
			if(freeToMove == 1 ):
				stamp3 = 0
				freeToMove = 0;
				if(command == "W" ):#and servos.getMovingStatus(4) == 0):
					#stamp3 = time.time()
					Gait(walkX=False, walkZ=True, backwards=False)
					MoveIK(x,y,z,rotx,roty,rotz)
					print "bewegung nach vorne"
					#sleep(1) #Beaglebone ist mit print alleine Abgestuerzt
				
				if(command == "S" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(walkX=False, walkZ=True, backwards=True)
					MoveIK(x,y,z,rotx,roty,rotz)
				if(command == "A" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(walkX=True, walkZ=False, backwards=True)
					MoveIK(x,y,z,rotx,roty,rotz)
				if(command == "D" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(walkX=True, walkZ=False, backwards=False)
					MoveIK(x,y,z,rotx,roty,rotz)
				if(command == "1" ):#and servos.getMovingStatus(4) == 0):
					break;
				if(command == "L" ):#and servos.getMovingStatus(4) == 0):
					oldCommand = "N"
					gegenstand = packen()
					if gegenstand == 1:
						werfen()
				#stamp4 = time.time()
				#if stamp3 != 0:
				#	print stamp4-stamp3

					
	#end = time.time()
	#print end-start

def servoErrorTest():
	#Liest Positionen aller Servos aus und misst so, wie viele Fehler diese verursachen.
	#Gibt die Servo-ID mit jeweiliger Fehlersumme zurueck.
	error = [0]*19
	for y in xrange(100):
		for x in xrange(1,19):
			try:
				servos.readPosition(x)
			except:
				error[x] += 1
			x += 1
		y += 1
	
	for x in xrange(1,19):
	 	print "Fehleranzahl Servo " + str(x) + " = " + str(error[x])
	
def readAllPositions(times=1):
	#Liest die Positionen aller Servos "times"-mal aus und gibt durschnittliche Zeit und Fehlersumme zurueck.
	#(Dient hauptsaechlich der statistischen Auswertung von Fehlersumme und Befehlszeit)
	i=1;
	j=0;
	k=0
	o=0
	isPositions = [None]*18
	sumup=0.0
	#servos.setReturnDelayTime(1,250)
	for x in xrange(times):
		while(i<=18):
			try:
				o += 1
				k += 1
				if k % 19 == 0:
					k = 1
				start = time.time()
				isPositions[k-1]= servos.readPosition(k)	

				delta = time.time() - start
				#print "Servo: " + str(k)
				#print "Zeit: " + str(delta)
				sumup += delta
			
			except:
				j += 1
				if isPositions[k-1] == None:
					isPositions[k-1] = (-1)
			i += 1
		i=0
	print "Anzahl an Leseversuchen" + str(o)
	print "Fehler: " + str(j)
	print "Durschnittliche Zeit: " + str(sumup/o)
	return isPositions

def ReadOnePosition(i=1):
	#Liest die Position eines Servos und gibt sie aus.
	try:
		a = servos.readPosition(i)
		print a
	except:
		print "ReadOnePosition: failed"
	

def timeTillStopped(k):
	global cSyncSpeed
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	cSyncSpeed = False
	for i in xrange(1,2):
		x = 0
		y = 0
		z = 0
		rotx = 0
		roty = 0
		rotz = 0

		zeroGait();
		MoveIK(x,y,z,rotx,roty,rotz)
		sleep(0.5)
			
		start = time.time()
		Gait(walkX=False, walkZ=True, backwards=False)
		MoveIK(x,y,z,rotx,roty,rotz)
		isMoving = 1;
		while True:
			try:
				isMoving = servos.readMovingStatus(k)
			except:
				sleep(0.01)
				continue
			if  isMoving:
				break
		delta = time.time() - start
		print "Time till " +str(k)  + "Servo stopped: " + str(delta)
	

def Handbetrieb():
	#Steuerung ueber Tastatur aus der Konsole heraus. Nur senden von Befehlen, keine Kontrolle der Ausfuehrung
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	ArrayTravelDistance = 1
	cSyncSpeed = True	
	stdscr = curses.initscr()
	curses.cbreak()
	stdscr.keypad(1)
	WalkingSpeed = 200	
	stdscr.addstr(0,10, "Quit with b")
	stdscr.addstr('W A S D Q E UP DOWN LEFT RIGHT')
	stdscr.refresh()
	add = 3

	key = ''
	while key != ord('b'):
		key = stdscr.getch()
		
		if key == ord('w'):
			z = z - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('a'):
			x = x + add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('s'):
			z = z + add	
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('d'):
			x = x - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('q'):
			y = y + add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('e'):
			y = y - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('i'):
			x = 0
			y = 0
			z = 0
			rotx = 0
			roty = 0
			rotz = 0
			zeroGait();
			MoveIK()
		elif key == ord('r'):
			rotx = rotx + add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('f'):
			rotx = rotx - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('t'):
			roty = roty + add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('g'):
			roty = roty - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('z'):
			rotz = rotz + add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == ord('h'):
			rotz = rotz - add
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
		elif key == curses.KEY_UP:# or rightStick == 'W':
			Gait(walkX=False, walkZ=True, backwards=False, cArrayTravelDistance=ArrayTravelDistance)
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_DOWN:
			Gait(walkX=False, walkZ=True, backwards=True, cArrayTravelDistance=ArrayTravelDistance)
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_RIGHT:
			Gait(walkX=True, walkZ=False, backwards=False, cArrayTravelDistance=ArrayTravelDistance)
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_LEFT:
			Gait(walkX=True, walkZ=False, backwards=True, cArrayTravelDistance=ArrayTravelDistance)
			MoveIK(x,y,z,rotx,roty,rotz,WalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == ord('y'):
			if cArrayTravelDistance < 5 :
				cArrayTravelDistance += 1
		
		elif key == ord('x'):
			if cArrayTravelDistance > 1:
				cArrayTravelDistance -= 1
	curses.endwin()

def Init2():
	#Initialisierung der Basisfunktionen sowie Bewegung in Initialposition.
	Init()
	InitPosition()
	print "Init done."

def InitPosition():
	#Bewegung in Initialposition
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	x = 0
	y = 0
	z = 0
	rotx = 0
	roty = 0
	rotz = 0
	zeroGait();
	MoveIK(x,y,z,rotx,roty,rotz)
	sleep(1)
