# coding: utf-8
#Frederick Mueller
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2018/2019

# Dieses Programm enthält alle Funktionen, welche Zugriff auf die Kamera via OpenCV benötigen.

from tastfunktion import *
from fernbed import *
from IKfunction import *
from ax12 import Ax12
from IK import *
servos = Ax12()
import cv2

cXResolution = 640
cYResolution = 480
dp=2 # 0.001
p1=100 #100
p2=170 #150
mindist=20
minrad=80
maxrad=300
cFocalLength = 3.67 #mm
cRealObjectRadius = 35 #mm
cSensorHeight = 3.5 #mm

#preset Colors TODO:Rot feinjustieren.
lowerRed=[(0, 140, 40), (160, 140, 40)]
upperRed=[(10, 230, 180), (180, 230, 180)]
lowerBlue = [(100, 34, 45)]
upperBlue = [(117, 192, 179)]
lowerYellow = [(11, 120, 77)]
upperYellow = [(25, 244, 214)]

foundObjects = []

#Kamera sollte aktiv bleiben (Im Test dauert der Open/Release Zyklus ca. 0,5s! )
cap = cv2.VideoCapture(0)
cap.open(0)
ret = cap.set(cv2.CAP_PROP_FRAME_WIDTH,cXResolution)
ret = cap.set(cv2.CAP_PROP_FRAME_HEIGHT,cYResolution)

def findObjectFromColor(lowerIntervals = 0, upperIntervals = 0, preset = ''): #TODO: Bild mithilfe der Farbe (Argumente hinzufügen) zuschneiden, dann Houghcircles. IF Abfrage und Argumente für Geometrie.Testen.
    #Sucht nach einem Objekt anhand bekannter Farbe/Geometrie
    if (preset != '' or lowerIntervals == 0 or upperIntervals == 0):
        if (preset == 'yellow'):
            print 'preset yellow'
            lowerIntervals=lowerYellow
            upperIntervals=upperYellow
        elif (preset == 'blue'):
            print 'preset blue'
            lowerIntervals=lowerBlue
            upperIntervals=upperBlue
        elif (preset == 'red'):
            print 'preset red'
            lowerIntervals=lowerRed
            upperIntervals=upperRed
        else:
            raise ValueError('Unknown preset or bad Intervals')
    ret, image = cap.read()
    ret, image = cap.read()
    image = cv2.medianBlur(image,3) # Für Distanzberechnung ggf. wenig hilfreich
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (255,255,255),(255,255,255))#Generieren einer 'Nullmaske'
    for i in xrange(0,len(lowerIntervals)):
        try:
            tempmask = cv2.inRange(hsv, lowerIntervals[i], upperIntervals[i])
            mask = cv2.bitwise_or(tempmask, mask)
        except IndexError:
            print "bad Intervals"
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] #TODO: Argumente so i.o.?
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            distance = (cFocalLength*cRealObjectRadius*cYResolution)/(radius*cSensorHeight)  #TODO:Das hier scheint sich für versch. Auflösungen zu ändern!
            '''
            cv2.imshow("Frame", image)
            cv2.imshow("Mask", mask)
            key = cv2.waitKey(1) & 0xFF
            '''
            return np.round((x,y,radius,distance),decimals=1)
    else:
    	'''
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", mask)
        '''
        key = cv2.waitKey(1) & 0xFF
        print 'No Contours found.'
        return None

def faceObject(targetObject):
    #Objekt wird durch Bewegung im Zentrum des Bildes gehalten.
    #Berechnung des Winekls zum Objekt. ACHTUNG: Horizontales FOV abhängig von Auflösung und FPS!! Hier sollte man sich bald festlegen!
    #FOV wahrscheinlich HFOV=70,58° (http://therandomlab.blogspot.com/2013/03/logitech-c920-and-c910-fields-of-view.html)
    HFOV = 70.58
    phiThreshold = 20
    dx= targetObject[0] - cXResolution/2
    degPerX= HFOV/cXResolution
    dPhi = degPerX * dx
    dPhi = np.round(dPhi).astype('int')
    if (abs(dPhi) >= phiThreshold):
        if(dPhi < 0):
            turn(dPhi, direction=True)
        elif(dPhi > 0):
            turn(dPhi, direction=False)

def findObjects(): #TODO: Bild mithilfe der Farbe (Argumente hinzufügen) zuschneiden, dann Houghcircles. IF Abfrage und Argumente für Geometrie.Testen.
    #Sucht nach einem Objekt anhand bekannter Farbe/Geometrie
    cap = cv2.VideoCapture(0)
    ret = cap.set(cv2.CAP_PROP_FRAME_WIDTH,cXResolution)
    ret = cap.set(cv2.CAP_PROP_FRAME_HEIGHT,cYResolution)
    cap.open(0)
    ret, image = cap.read()
    ret, image = cap.read() #Doppeltes auslesen um Buffer zu loeschen
    cap.release()

def destroyCV():
    cap.release()