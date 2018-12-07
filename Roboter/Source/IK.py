#Thassilo Bücker, Alexander Orzol, Frederick Mueller, Moritz Kolb
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Enthaelt alle Basisfunktionen und die Inverse Kinematik des Hexapoden

# coding: utf-8
##### Config #########################
# Debugmode
cDebug = False

# Ausschalt- bzw Wiedereinschaltspannung für die Servos
cTurnOffVoltage = 1000	#10V
cTurnOnVoltage = 1100	#11V

# Servo IDs
cRRCoxaPin = 8		#Rear Right leg Hip Horizontal
cRRFemurPin = 10   	#Rear Right leg Hip Vertical
cRRTibiaPin = 12   	#Rear Right leg Knee

cRMCoxaPin = 14  	#Middle Right leg Hip Horizontal
cRMFemurPin = 16	#Middle Right leg Hip Vertical
cRMTibiaPin = 18	#Middle Right leg Knee

cRFCoxaPin = 2		#Front Right leg Hip Horizontal
cRFFemurPin = 4		#Front Right leg Hip Vertical
cRFTibiaPin = 6		#Front Right leg Knee

cLRCoxaPin = 7		#Rear Left leg Hip Horizontal
cLRFemurPin = 9 	#Rear Left leg Hip Vertical
cLRTibiaPin = 11 	#Rear Left leg Knee

cLMCoxaPin = 13		#Middle Left leg Hip Horizontal
cLMFemurPin = 15	#Middle Left leg Hip Vertical
cLMTibiaPin = 17	#Middle Left leg Knee

cLFCoxaPin = 1		#Front Left leg Hip Horizontal
cLFFemurPin = 3		#Front Left leg Hip Vertical
cLFTibiaPin = 5		#Front Left leg Knee

# LegNumber; RR = right rear,...
cRRLegNumber = 0
cRMLegNumber = 1
cRFLegNumber = 2
cLRLegNumber = 3
cLMLegNumber = 4
CLFLegNumber = 5

# Min/Max Winkel der Servos (experimentell)
cXXCoxaMin = -700
cXXCoxaMax = 900
cXXFemurMin = -900
cXXFemurMax = 900
cXXTibiaMin = -700
cXXTibiaMax = 900

cRRCoxaMin = cXXCoxaMin
cRRCoxaMax = cXXCoxaMax
cRRFemurMin = cXXFemurMin
cRRFemurMax = cXXFemurMax
cRRTibiaMin = cXXTibiaMin
cRRTibiaMax = cXXTibiaMax

cRMCoxaMin = cXXCoxaMin
cRMCoxaMax = cXXCoxaMax
cRMFemurMin = cXXFemurMin
cRMFemurMax = cXXFemurMax
cRMTibiaMin = cXXTibiaMin
cRMTibiaMax = cXXTibiaMax

cRFCoxaMin = cXXCoxaMin
cRFCoxaMax = cXXCoxaMax
cRFFemurMin = cXXFemurMin
cRFFemurMax = cXXFemurMax
cRFTibiaMin = cXXTibiaMin
cRFTibiaMax = cXXTibiaMax

cLRCoxaMin = cXXCoxaMin
cLRCoxaMax = cXXCoxaMax
cLRFemurMin = cXXFemurMin
cLRFemurMax = cXXFemurMax
cLRTibiaMin = cXXTibiaMin
cLRTibiaMax = cXXTibiaMax

cLMCoxaMin = cXXCoxaMin
cLMCoxaMax = cXXCoxaMax
cLMFemurMin = cXXFemurMin
cLMFemurMax = cXXFemurMax
cLMTibiaMin = cXXTibiaMin
cLMTibiaMax = cXXTibiaMax

cLFCoxaMin = cXXCoxaMin
cLFCoxaMax = cXXCoxaMax
cLFFemurMin = cXXFemurMin
cLFFemurMax = cXXFemurMax
cLFTibiaMin = cXXTibiaMin
cLFTibiaMax = cXXTibiaMax

# Beinlängen in mm
cXXCoxaLength = 53    # PhantomX leg dimensions.
cXXFemurLength = 66
cXXTibiaLength = 134

cRRCoxaLength = cXXCoxaLength	    #Right Rear leg
cRRFemurLength = cXXFemurLength
cRRTibiaLength = cXXTibiaLength

cRMCoxaLength = cXXCoxaLength	    #Right middle leg
cRMFemurLength = cXXFemurLength
cRMTibiaLength = cXXTibiaLength

cRFCoxaLength = cXXCoxaLength	    #Rigth front leg
cRFFemurLength = cXXFemurLength
cRFTibiaLength = cXXTibiaLength

cLRCoxaLength = cXXCoxaLength	    #Left Rear leg
cLRFemurLength = cXXFemurLength
cLRTibiaLength = cXXTibiaLength

cLMCoxaLength = cXXCoxaLength	    #Left middle leg
cLMFemurLength = cXXFemurLength
cLMTibiaLength = cXXTibiaLength

cLFCoxaLength = cXXCoxaLength	    #Left front leg
cLFFemurLength = cXXFemurLength
cLFTibiaLength = cXXTibiaLength

# Köpermaße des Hexapods
cRRCoxaAngle = 45       #Rotationswinkel des jeweiligen Beins um die Y-Achse aus Körpergeometrie, mit RMCoxaAngle gesetzt = 0
cRMCoxaAngle = 0        #Default Coxa setup angle
cRFCoxaAngle = 315      #Default Coxa setup angle
cLRCoxaAngle = 135      #Default Coxa setup angle
cLMCoxaAngle = 180      #Default Coxa setup angle
cLFCoxaAngle = 225      #Default Coxa setup angle


cX_COXA = 60			# (unused) distance (in mm) between front and back legs /2
cY_COXA = 60			# (unused) distance (in mm) between front and back legs /2
cM_COXA = 100			# (unused) distance (in mm) between middle legs /2

cRROffsetX = 60			#Distance X from center of the body to the Right Rear coxa
cRROffsetZ = -120		#Distance Z from center of the body to the Right Rear coxa

cRMOffsetX = 100		#Distance X from center of the body to the Right Middle coxa
cRMOffsetZ = 0			#Distance Z from center of the body to the Right Middle coxa

cRFOffsetX = 60			#Distance X from center of the body to the Right Front coxa
cRFOffsetZ = 120		#Distance Z from center of the body to the Right Front coxa

cLROffsetX = -60		#Distance X from center of the body to the Left Rear coxa
cLROffsetZ = -120		#Distance Z from center of the body to the Left Rear coxa

cLMOffsetX = -100    	#Distance X from center of the body to the Left Middle coxa
cLMOffsetZ = 0      	#Distance Z from center of the body to the Left Middle coxa

cLFOffsetX = -60     	#Distance X from center of the body to the Left Front coxa
cLFOffsetZ = 120   		#Distance Z from center of the body to the Left Front coxa

# Startpositionen
cHexInitXZM = 127		#Initial XZ Position of middle legs
cHexInitXZ = 90			#Initial XZ Position of front and rear legs
cHexInitY = 100 		#Initial Y Position of Body/all legs, maximum ~150

cRRInitPosX = cHexInitXZ    #Start positions of the Right Rear leg
cRRInitPosY = cHexInitY
cRRInitPosZ = -cHexInitXZ

cRMInitPosX = cHexInitXZM      #Start positions of the Right Middle leg
cRMInitPosY = cHexInitY
cRMInitPosZ = 0

cRFInitPosX = cHexInitXZ      #Start positions of the Right Front leg
cRFInitPosY = cHexInitY
cRFInitPosZ = cHexInitXZ

cLRInitPosX = -cHexInitXZ      #Start positions of the Left Rear leg
cLRInitPosY = cHexInitY
cLRInitPosZ = -cHexInitXZ

cLMInitPosX = -cHexInitXZM      #Start positions of the Left Middle leg
cLMInitPosY = cHexInitY
cLMInitPosZ = 0

cLFInitPosX = -cHexInitXZ      #Start positions of the Left Front leg
cLFInitPosY = cHexInitY
cLFInitPosZ = cHexInitXZ

# Gait - maximum leg distance from initial position during movement
cGaitTravelLengthX = 30
cGaitTravelLengthZ = 30
cGaitTravelHeigth = -30

cStepAmount = 96 #Menge an Schritten pro Bewegungszyklus, nur vielfache von 12
cWalkingSpeed = 100 #Bewegungsgeschwindigkeit der Servos
cSyncSpeed = False #Bewegungsgeschwindigkeit gleich für alle Servos

#lastca = [512, 512, 512, 512, 512, 512]
#lastfa = [644, 656, 644, 378, 366, 378]
#lastta = [295, 257, 295, 727, 767, 727]
lastAngles = [512, 512, 377, 647, 728, 296, 512, 512, 377, 647, 728, 296, 512, 512, 365, 659, 767, 257] #Initialposition

##################################################################
##### End Config ##### Don't change anything below this line #####
##################################################################

##### Imports #########################
import numpy as np
from time import sleep
import time
import copy
from ax12 import Ax12

servos = Ax12()

##### Numpy dependant Constants #############
cGaitLeg = [2,4,0,5,1,3]
#cGaitY = [0, 0.5, 1, 1, 1, 0.5, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#cGaitXZ = [-1, -0.66, -0.33, 0, 0.33, 0.66, 1, 0.88, 0.76, 0.65, 0.53, 0.42, 0.30, 0.18, 0.07, 0, -0.07, -0.18, -0.30, -0.42, -0.53, -0.65, -0.76, -0.88]

cGaitXZ = [None]*cStepAmount
cGaitY = [None]*cStepAmount
for i in range(0,cStepAmount):

	if(i < (cStepAmount*3/4)):
		cGaitXZ[i] = np.around(np.sin(i* 1 * np.pi / (cStepAmount*3/4) + 1 * np.pi / 2), decimals=2)
	else:
		cGaitXZ[i] = np.around(np.sin((i-(cStepAmount*3/4)) * np.pi / (cStepAmount*1/4) + 3 * np.pi / 2), decimals=2)
	cGaitY[i] = 0
	if (i >= (cStepAmount*3/4)):
		cGaitY[i] = np.around(np.sin((i-(cStepAmount*3/4)) * 1 * np.pi / (cStepAmount*1/4)), decimals=2)
if cDebug:
	print cGaitY
	print cGaitXZ



##### Create Lists #########################
cCoxaMin = [cRRCoxaMin, cRMCoxaMin, cRFCoxaMin, cLRCoxaMin, cLMCoxaMin, cLFCoxaMin]
cCoxaMax = [cRRCoxaMax, cRMCoxaMax, cRFCoxaMax, cLRCoxaMax, cLMCoxaMax, cLFCoxaMax]
cFemurMin = [cRRFemurMin, cRMFemurMin, cRFFemurMin, cLRFemurMin, cLMFemurMin, cLFFemurMin]
cFemurMax = [cRRFemurMax, cRMFemurMax, cRFFemurMax, cLRFemurMax, cLMFemurMax, cLFFemurMax]
cTibiaMin = [cRRTibiaMin, cRMTibiaMin, cRFTibiaMin, cLRTibiaMin, cLMTibiaMin, cLFTibiaMin]
cTibiaMax = [cRRTibiaMax, cRMTibiaMax, cRFTibiaMax, cLRTibiaMax, cLMTibiaMax, cLFTibiaMax]

cCoxaLength = [cRRCoxaLength, cRMCoxaLength, cRFCoxaLength, cLRCoxaLength, cLMCoxaLength, cLFCoxaLength]
cFemurLength = [cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength]
cTibiaLength = [cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength]

cOffsetX = [cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX]
cOffsetZ = [cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ]
cCoxaAngle = [cRRCoxaAngle, cRMCoxaAngle, cRFCoxaAngle, cLRCoxaAngle, cLMCoxaAngle, cLFCoxaAngle]

cInitPosX = [cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX]
cInitPosY = [cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY]
cInitPosZ = [cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ]

cServoIDs = [[8, 10, 12], [14, 16, 18], [2, 4, 6], [7, 9, 11], [13, 15, 17], [1, 3, 5]]

##### Variables #########################
GaitPosX = [0, 0, 0, 0, 0, 0]
GaitPosY = [0, 0, 0, 0, 0, 0]
GaitPosZ = [0, 0, 0, 0, 0, 0]
GaitRotY = [0, 0, 0, 0, 0, 0]



GaitXZ = np.zeros((0,6))
GaitY = np.zeros((0,6))
##### Functions #########################
roll = 0 #Roll counter
def Init():
	global GaitXZ, GaitY
	GaitXZ = np.zeros((cStepAmount, 6))
	GaitY = np.zeros((cStepAmount, 6))

	for i in xrange(6):
		GaitXZ[:,i] = np.roll(cGaitXZ, cGaitLeg[i]*(cStepAmount/6), axis=0)
		GaitY[:,i] = np.roll(cGaitY, cGaitLeg[i]*(cStepAmount/6), axis=0)

def LegIK(LegIKPosX, LegIKPosY, LegIKPosZ, LegIKLeg, transformation=True):
	#[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
	#IKFeetPosX				- Input position of the Feet X
	#IKFeetPosY				- Input position of the Feet Y
	#IKFeetPosZ				- Input Position of the Feet Z
	#IKSolutionError		- Output true if the solution is NOT possible
	#IKSolutionWarning		- Output true if the soltuion is critical
	#FemurAngle				- Output Angle of Femur in degrees
	#TibiaAngle				- Output Angle of Tibia in degrees
	#CoxaAngle				- Output Angle of Coxa in degrees
	IKSolutionError = False

	if transformation:
		IKFeetPosX = (LegIKPosX * np.cos(cCoxaAngle[LegIKLeg] * np.pi/180)) - (LegIKPosZ * np.sin(cCoxaAngle[LegIKLeg] * np.pi/180))
		IKFeetPosY = LegIKPosY #+ 50
		IKFeetPosZ = (LegIKPosX * np.sin(cCoxaAngle[LegIKLeg] * np.pi/180)) + (LegIKPosZ * np.cos(cCoxaAngle[LegIKLeg] * np.pi/180))
	else:
		IKFeetPosX = LegIKPosX
		IKFeetPosY = LegIKPosY #+ 50 # Damit 0 auf Höhe des Körpers ist
		IKFeetPosZ = LegIKPosZ

	legLength = np.sqrt((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ))	
	hf = np.sqrt(np.power((legLength-cXXCoxaLength), 2) + IKFeetPosY*IKFeetPosY)
	a1 = np.arctan2((legLength-cXXCoxaLength), IKFeetPosY) * 180/np.pi

	temp = ((hf*hf)-np.power(cXXTibiaLength, 2)+np.power(cXXFemurLength, 2)) / (2*cXXFemurLength*hf)
	if temp < -1 or temp > 1:
		IKSolutionError = True
		return IKSolutionError, None, None, None
	else:
		a2 = np.arccos(temp) * 180/np.pi
	
	temp = (np.power(cXXTibiaLength, 2)+np.power(cXXFemurLength, 2)-(hf*hf)) / (2*cXXFemurLength*cXXTibiaLength)
	if temp < -1 or temp > 1:
		IKSolutionError = True
		return IKSolutionError, None, None, None
	else:
		b1 = np.arccos(temp) * 180/np.pi

	CoxaAngle  = 90 + (np.arctan2(IKFeetPosZ, IKFeetPosX) * 180/np.pi) # +90 um von -90° bis +90° ins Winkelmaß des Servos zu kommen
	FemurAngle = a1 + a2 + 13# +90 um von -90° bis +90° ins Winkelmaß des Servos zu kommen
	#TibiaAngle = 180 - b1 + 45# +90 um von -90° bis +90° ins Winkelmaß des Servos zu kommen
	TibiaAngle = b1 - 52#/(np.pi/2)#*7/11#-(b1*20/55)#- 23# +90 um von -90° bis +90° ins Winkelmaß des Servos zu kommen

	#Fehlerabfrage bei zu kleinen/ zu grossen Winkeln (nicht getestet, experimentell)
 	#if CoxaAngle > 200 or CoxaAngle < -5:
 		#print 'Coxa Out of Range ' + str(LegIKLeg) + ' ' + str(CoxaAngle)
 		#IKSolutionError = True
		#return IKSolutionError, None, None, None
 	#if FemurAngle > 200 or FemurAngle < -5:
 		#print 'Femur Out of Range ' + str(LegIKLeg) + ' ' + str(FemurAngle)
 		#IKSolutionError = True
		#return IKSolutionError, None, None, None
 	#if TibiaAngle > 200 or TibiaAngle < -5:
 		#print 'Tibia Out of Range ' + str(LegIKLeg) + ' ' + str(TibiaAngle)
 		#IKSolutionError = True
		#return IKSolutionError, None, None, None

	if LegIKLeg > 2:	# Servos sind alle andersrum montiert
		#CoxaAngle = 180 - CoxaAngle
		FemurAngle = 180 - FemurAngle
		TibiaAngle = 180 - TibiaAngle
	

	if cDebug:
		print '### LegIK() ### ' + str(LegIKLeg)
		print 'IKFeetPosX ' + str(IKFeetPosX)
		print 'IKFeetPosY ' + str(IKFeetPosY)
		print 'IKFeetPosZ ' + str(IKFeetPosZ)
		print 'legLength ' + str(legLength)
		print 'HF ' + str(hf)
		print 'A1 ' + str(a1)
		print 'A2 ' + str(a2)
		print 'B1 ' + str(b1)
		print 'CoxaAngle ' + str(CoxaAngle)
		print 'FemurAngle ' + str(FemurAngle)
		print 'TibiaAngle ' + str(TibiaAngle)
		print '#######'

	return  IKSolutionError, CoxaAngle, FemurAngle, TibiaAngle


def BodyIK (BodyIKPosX, BodyIKPosY, BodyIKPosZ, BodyIKRotX, BodyIKRotY, BodyIKRotZ, BodyIKLeg):
	#(BODY INVERSE KINEMATICS) 
	#BodyIKRotX         - Input pitch of the body 
	#BodyIKRotY         - Input rotation of the body 
	#BodyIKRotZ         - Input roll of the body 
	#BodyIKPosX       	- Input position of the feet X 
	#BodyIKPosY         - Input position of the feet Y
	#BodyIKPosZ         - Input position of the feet Z 
	#BodyFKPosX       - Output Position X of feet with Rotation 
	#BodyFKPosY       - Output Position Y of feet with Rotation 
	#BodyFKPosZ       - Output Position Z of feet with Rotation

	# Calculate Sin & Cos
	sinX = np.sin(BodyIKRotX * np.pi/180)
	cosX = np.cos(BodyIKRotX * np.pi/180)
	sinY = np.sin(BodyIKRotY * np.pi/180)
	cosY = np.cos(BodyIKRotY * np.pi/180)
	sinZ = np.sin(BodyIKRotZ * np.pi/180)
	cosZ = np.cos(BodyIKRotZ * np.pi/180)

	TotalX = cInitPosX[BodyIKLeg] + cOffsetX[BodyIKLeg] + BodyIKPosX + GaitPosX[BodyIKLeg]
	TotalY = cInitPosY[BodyIKLeg] + GaitPosY[BodyIKLeg]
	TotalZ = cInitPosZ[BodyIKLeg] + cOffsetZ[BodyIKLeg] + BodyIKPosZ + GaitPosZ[BodyIKLeg]

	BodyIKX = (TotalX * cosZ * cosY) - (TotalZ * cosZ * sinY) + (TotalY * sinZ) - TotalX
	BodyIKY = (TotalX * sinY * sinX) - (TotalX * cosY * cosX * sinZ) + (TotalZ * cosY * sinX) + (TotalZ * cosX * sinY * sinZ) + (TotalY * cosZ * cosX) - TotalY
 	BodyIKZ = (TotalX * cosX * sinY) + (TotalX * cosY * sinZ * sinX) + (TotalZ * cosY * cosX) - (TotalZ * sinY * sinZ * sinX) - (TotalY * cosZ * sinX) - TotalZ

	LegIKPosX = BodyIKX + GaitPosX[BodyIKLeg] + cInitPosX[BodyIKLeg] + BodyIKPosX
	LegIKPosY = BodyIKY + GaitPosY[BodyIKLeg] + cInitPosY[BodyIKLeg] + BodyIKPosY
	LegIKPosZ = BodyIKZ + GaitPosZ[BodyIKLeg] + cInitPosZ[BodyIKLeg] + BodyIKPosZ

	if cDebug:
		print '### BodyIK() ### ' + str(BodyIKLeg)
		print 'InPosX ' + str(BodyIKPosX)
		print 'InPosY ' + str(BodyIKPosY)
		print 'InPosZ ' + str(BodyIKPosZ)
		print 'InRotX ' + str(BodyIKRotX)
		print 'InRotY ' + str(BodyIKRotY)
		print 'InRotZ ' + str(BodyIKRotZ)
		print 'OutLegPosX ' + str(LegIKPosX)
		print 'OutLegPosY ' + str(LegIKPosY)
		print 'OutLegPosZ ' + str(LegIKPosZ)

	return LegIKPosX, LegIKPosY, LegIKPosZ

def DegreeToServo(angle):
	#Umwandlung der Winkel in Servopositionswerte 0...1024
	#value = np.empty((1), dtype=int)
	value=np.around(((614.4/180.0)*angle)+204.8)

	if cDebug:
		print 'DegreeToServo() ' + str(value)

	return int(value)

def Gait(walkX=True, walkZ=False, backwards=False, move = True, cArrayTravelDistance=1):
	#Berechnung der Gangart durch Rollen der Gait-Arrays
	global GaitXZ, GaitY, GaitPosX, GaitPosY, GaitPosZ, roll
	if move:
		if (backwards):
			GaitXZ = np.roll(GaitXZ, cArrayTravelDistance, axis=0)
			GaitY = np.roll(GaitY, cArrayTravelDistance, axis=0)
			roll -= 1*cArrayTravelDistance
		else:
			GaitXZ = np.roll(GaitXZ, -cArrayTravelDistance, axis=0)
			GaitY = np.roll(GaitY, -cArrayTravelDistance, axis=0)
			roll += 1*cArrayTravelDistance

	if(walkZ):
		GaitPosZ = GaitXZ[0]*cGaitTravelLengthZ

	else:
		GaitPosZ = [0,0,0,0,0,0]

	if(walkX):
		GaitPosX = GaitXZ[0]*cGaitTravelLengthX
	else:
		GaitPosX = [0,0,0,0,0,0]

	if(walkX or walkZ):
		GaitPosY = GaitY[0]*cGaitTravelHeigth
	else:
		GaitPosY = [0,0,0,0,0,0]

def MoveIK(x=0,y=0,z=0,rotx=0,roty=0,rotz=0,maxSpeed=cWalkingSpeed,cSyncSpeed=False):
	#Bewegung der Beine durch Verschiebung/Rotation des Koerpers
	#Diese ruft BodyIK und LegIK auf. Wenn cSyncSpeed=False, werden abhaengig
	#von der Distanz zum naechsten Punkt die Geschwindigkeit fuer jeden
	#Servo einzeln berechnet.
	global lastAngles
	Distance=[None]*18
	p = [None] * 18
	speed = [None] * 18
	for i in xrange(6):
		bx, by, bz = BodyIK(x, y, z, rotx, roty, rotz, i)
		error, ca, fa, ta = LegIK(bx, by, bz, i, transformation=True)
		if not error:
			p[cServoIDs[i][0]-1] = DegreeToServo(ca)
			p[cServoIDs[i][1]-1] = DegreeToServo(fa)
			p[cServoIDs[i][2]-1] = DegreeToServo(ta)

		else:
			print 'Error: ' + str(i)
	#Berechne Speed
	if not cSyncSpeed:
		for i in xrange (18):
			Distance[i] = abs(p[i]- lastAngles[i])
		for i in xrange(18):
			if(Distance[i] > 0):
				#speed[i] = 5*int(np.around((Distance[i] / float( max(Distance))) * (maxSpeed/5)))
				#speed[i] = int( (1-(1/np.exp(5*(Distance[i] / float( max(Distance)))))) * maxSpeed)
				speed[i] = int((np.exp(3*(Distance[i] / float( max(Distance))/np.exp(3)))*maxSpeed))
				#print speed[i]
			else:
				speed[i] = maxSpeed
	else:
		for i in xrange (18):
			speed[i] = maxSpeed
	#print "speed:",speed
	#print "pos:",p
	# Neue lastAngles
	lastAngles = copy.deepcopy(p)

	servos.moveSpeedSync(p, speed)

def getLastAngles():
	return lastAngles

def zeroGait():
	#Initialposition und Initialschritt annehmen.
	global GaitPosX, GaitPosY, GaitPosZ, GaitXZ, GaitY,roll
	GaitPosX=[0,0,0,0,0,0]
	GaitPosY=[0,0,0,0,0,0]
	GaitPosZ=[0,0,0,0,0,0]
	GaitXZ = np.roll(GaitXZ, roll, axis=0)
	GaitY = np.roll(GaitY, roll, axis=0)
	roll=0



# Error counter beim lesen
