# coding: utf-8
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


def newAutomatikbetrieb() :
	#Automatische Vorwaertsbewegung mit Pruefung ob mindestens 10 Servos die Zielposition
	#erreicht haben, bevor ein neuer Bewegungsbefehl abgesetzt wird.
	PositionToReach = getLastAngles()	
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	j=0
	k=0
	while(getStatus()  != "1"):
		MotorsReady = 0
		currentPositions = readAllPositions()
		k += 1
		print (currentPositions)
		print (PositionToReach)
		for i in xrange(0,18):
			if abs(PositionToReach[i] - currentPositions[i]) < 5 or currentPositions[i] == 1:
				MotorsReady += 1
				#print MotorsReady
			
				 
		if(MotorsReady >= 10):
			Gait(phi=0)
			MoveIK(x,y,z,rotx,roty,rotz)
			PositionToReach = getLastAngles()
			print (k)
			k = 0
		sleep(0.01)
		j += 1


def Fernbedienungsbetrieb():
	#Entspricht dem Handbetrieb ueber Konsole, jedoch mit geringerem
	#Funktionsumfang und Ansteuerung ueber die Fernbedienung.
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	freeToMove = 0
	oldCommand = 'N'
	TimeStamp = 0
	old_TimeStamp = 0

	print ('Fernbedienung gestartet')

	while True:
		TimeStamp = time.time()
		time_dif = TimeStamp - old_TimeStamp
		if (time_dif*1000)  >= 50:
			old_TimeStamp = TimeStamp
			#print "attempting to read"
			command = getStatus()
			if command == ' ':
				command = oldCommand
			oldCommand = command
			freeToMove = 1
		else:
			
			if freeToMove == 1 :
				stamp3 = 0
				freeToMove = 0
				if command == "W" :#and servos.getMovingStatus(4) == 0):
					#stamp3 = time.time()
					Gait(phi=0)
					MoveIK(x,y,z,rotx,roty,rotz)
					print ("bewegung nach vorne")
					#sleep(1) #Beaglebone ist mit print alleine Abgestuerzt
				
				if(command == "S" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(phi=180)
					MoveIK(x,y,z,rotx,roty,rotz)
				if(command == "A" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(phi=270)
					MoveIK(x,y,z,rotx,roty,rotz)
				if(command == "D" ):#and servos.getMovingStatus(4) == 0):
					stamp3 = time.time()
					Gait(phi=90)
					MoveIK(x,y,z,rotx,roty,rotz)
				if command == "1" :#and servos.getMovingStatus(4) == 0):
					print ('beendet ')
					#break
				if command == "L" :#and servos.getMovingStatus(4) == 0):
					oldCommand = "N"
					gegenstand = packen()
					if gegenstand == 1:
						werfen()
				#stamp4 = time.time()
				#if stamp3 != 0:
				#	print stamp4-stamp3

	print ('Fernbedienung beendet')
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
		print ("Fehleranzahl Servo" + str(x) + " = " + str(error[x]))
	
def readAllPositions(times=1):
	#Liest die Positionen aller Servos "times"-mal aus und gibt durschnittliche Zeit und Fehlersumme zurueck.
	#(Dient hauptsaechlich der statistischen Auswertung von Fehlersumme und Befehlszeit)
	i=1
	j=0
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
				if isPositions[k-1] is None:
					isPositions[k-1] = (-1)
			i += 1
		i=0
	print ("Anzahl an Leseversuchen" + str(o))
	print ("Fehler: " + str(j))
	print ("Durschnittliche Zeit: " + str(sumup/o))
	return isPositions

def ReadOnePosition(i=1):
	#Liest die Position eines Servos und gibt sie aus.
	try:
		a = servos.readPosition(i)
		print (a)
	except:
		print ("ReadOnePosition: failed")
	

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

		zeroGait()
		MoveIK(x,y,z,rotx,roty,rotz)
		sleep(0.5)
			
		start = time.time()
		Gait(phi=0)
		MoveIK(x,y,z,rotx,roty,rotz)
		isMoving = 1
		while True:
			try:
				isMoving = servos.readMovingStatus(k)
			except:
				sleep(0.01)
				continue
			if  isMoving:
				break
		delta = time.time() - start
		print ("Time till " +str(k)  + "Servo stopped: " + str(delta))
	

def Handbetrieb():
	#Steuerung ueber Tastatur aus der Konsole heraus. Nur senden von Befehlen, keine Kontrolle der Ausfuehrung
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	cSyncSpeed = True	
	stdscr = curses.initscr()
	curses.cbreak()
	stdscr.keypad(1)
	stdscr.addstr(0,10, "\nQuit with b \n")
	stdscr.addstr('W A S D Q E UP DOWN LEFT RIGHT')
	stdscr.refresh()
	add = 3
	InitPosition()

	key = ''
	while key != ord('b'):
		key = stdscr.getch()
		
		if key == ord('w'):
			z = z - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('a'):
			x = x + add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('s'):
			z = z + add	
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('d'):
			x = x - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('q'):
			y = y + add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('e'):
			y = y - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('i'):
			x = 0
			y = 0
			z = 0
			rotx = 0
			roty = 0
			rotz = 0
			zeroGait()
			MoveIK()
		elif key == ord('r'):
			rotx = rotx + add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('f'):
			rotx = rotx - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('t'):
			roty = roty + add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('g'):
			roty = roty - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('z'):
			rotz = rotz + add
			MoveIK (x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('h'):
			rotz = rotz - add
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('o'):
			Gait(phi=0,move=False,turn=True)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == ord('p'):
			Gait(phi=0,move=False,turn=True, turnDirection=True)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
		elif key == curses.KEY_UP:# or rightStick == 'W':
			Gait(phi=0)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_DOWN:
			Gait(phi=180)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_RIGHT:
			Gait(phi=90)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		elif key == curses.KEY_LEFT:
			Gait(phi=270)
			MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
			#sleep(0.03)
		'''
		elif key == ord('y'):
			if cArrayTravelDistance < 5 :
				cArrayTravelDistance += 1 # TODO: Das hier funktioniert nicht mehr, da cArrayTravelDistance nach IK.py verlegt. sinnvoll?
		
		elif key == ord('x'):
			if cArrayTravelDistance > 1:
				cArrayTravelDistance -= 1 # TODO: Das hier funktioniert nicht mehr, da cArrayTravelDistance nach IK.py verlegt. sinnvoll?
		'''
	curses.endwin()

def Init2():
	#Initialisierung der Basisfunktionen sowie Bewegung in Initialposition.
	Init()
	InitPosition()
	print ("Init done.")

def InitPosition():
	#Bewegung in Initialposition
	global x,y,z,rotx,roty,rotz,GaitPosX,GaitPosY,GaitPosZ
	x = 0
	y = 0
	z = 0
	rotx = 0
	roty = 0
	rotz = 0
	zeroGait()
	MoveIK(x,y,z,rotx,roty,rotz)
	sleep(1)

def turn(phi=0):
	#Funktion für das Drehen des gesamten Hexapoden um einen vorgegebenen Winkel.
	#direction=False - Rechtsrum
	#TODO: Funktioniert bisher nur gut für große Winkel (>20°)
	#Experimentell
	#(40 = Testwert TravelLengthX/Z, 96 = Testwert cStepAmount, 250/93 = Faktor basierend auf Messung Schritte pro Grad)

	#Die Einstellungsmöglichkeiten via Variablen sorgen hier für extreme Verlangsamung.
	#phi = phi.astype('int64') #nur notwendig für Berechnung mit Variablen
	#turnStepAmount = np.round((abs(phi)*250*cStepAmount*40*40)/(93*96*cGaitTravelLengthX*cGaitTravelLengthZ)).astype("int") #TODO zu testen

	#Gekürzte Fassung für Experimentelle Werte s.o.
	if (phi < 0):
		direction = True
	else:
		direction = False

	turnStepAmount = np.round((abs(phi)*250)/93).astype("int") 

	print (turnStepAmount)
	for i in xrange(0,turnStepAmount,1):
		Gait(phi=0,move=False,turn=True, turnDirection=direction)
		MoveIK(x,y,z,rotx,roty,rotz,cWalkingSpeed,cSyncSpeed)
	InitPosition()
