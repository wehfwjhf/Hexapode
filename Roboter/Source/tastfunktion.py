#Thassilo Bücker, Alexander Orzol
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Das Programm stellt Funktionen zum Tasten zur Verfügung   

cWalkingSpeed = 150
cSyncSpeed = False

##### Imports #########################
from IKfunction import *
from ax12 import Ax12
import time
from fernbed import *

servos = Ax12()
#Init()

def Inittasten():
	#Initialposition zum Tasten / mittleren Beine vorne, vordere Beine oben 
	start= [412,612,412,612,767,255,575,449,400,624,767,255,350,674,400,624,767,255]
	for i in xrange(0,18):
		if not(i==0 or i==1):
			try:
				servos.move(i+1,start[i])
			except:
				print "Servo move failed"
				i -= 1

def Vorderbeinefuehlen():
	# Mit den Vorderbeinen wird geprüft ob sich ein Hindernis vor ihm befindet	
	start= [412,612,255,767,512,512,575,449,400,624,767,255,350,674,400,624,767,255]
	#start= [412,612,255,767,512,512,575,449,400,624,700,324,350,674,400,624,700,324]
	for i in xrange(0,18):
		try:

			servos.move(i+1,start[i])
		except:
			print "Servos move failed"	
	load = 0
	for i in reversed(xrange(525,767,4)):
		if load > 1000 or load < 60:
			load = -1
			try:
				servos.move(4,i)
				sleep(0.25)
				load = servos.readLoad(4)
			except:
				i+=1
			print "|" +str(i) + "|" + str(load) + "\n"
		else:
			print "Gegestand erkannt"
			break
	try:
		servos.move(4,767)
	except:
		print "Servos move failed"
	load = 0	
	for i in xrange(255,490,4):
		if load < 1050:
			load = -1
			try:
				servos.move(3,i)
				sleep(0.25)
				load = servos.readLoad(3)
			except:
				i-=1
			print "|" +str(i) + "|" + str(load) + "\n"
		else:
			print "Gegenstand erkannt"
			break
	try:
		servos.move(3,255)
	except:
		print "Servos move failed"

def packen():
	#mit den Vorderbeinen wird versucht ein Gegenstand zu packen
	gegenstand = 0
	Inittasten()
	sleep(1)
	start= [412,612,349,675,767,255,575,449,400,624,767,255,350,674,400,624,767,255]
	#start= [412,612,255,767,512,512,575,449,400,624,700,324,350,674,400,624,700,324]
	#start = [511, 509, 255, 767, 763, 259, 509, 514, 361, 666, 764, 259, 512, 510, 359, 659, 763, 258]
	for i in xrange(0,18):
		try:
			servos.move(i+1,start[i])
		except:
			print "Servo move failed"
			
	load = 0
	load1 = 0
	for i in xrange(512,300,-5):
		#if load >1000 or load <250 and  load1 < 1250:
		if load >1000 or load <300 and  load1 < 1325:
			load = -1
			load1 = -1
			try:
				servos.move(1,i)
				servos.move(2,512+abs(512-i))
				sleep(0.25)
				load  = servos.readLoad(1)
				load1 = servos.readLoad(2)
			except:
				i+=1
			print  "|1|" +str(i) + "|" + str(load) + " |2|" +str(512+abs(512-i)) + "|" + str(load1)
		else:
			print "Gegestand erkannt"
			gegenstand = 1
			break
	try:
		servos.move(3,255)
		servos.move(4,767)
	except:
		print "Servo move failed"
	return gegenstand

def werfen():
	#Wirft einen gepackten Gegenstand nach vorne
	start= [412,612,349,675,767,255,575,449,400,624,767,255,350,674,400,624,767,255]
	try:
		servos.moveSpeed(5,300,500)
		servos.moveSpeed(6,724,500)
		sleep(0.05)
		servos.moveSpeed(1,462,500)
		servos.moveSpeed(2,562,500)
		sleep(1)
		servos.moveSpeed(5,767,200)
		servos.moveSpeed(6,255,200)	
	except:
		print "werfen failed"
#Inittasten()
