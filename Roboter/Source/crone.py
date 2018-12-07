#Thassilo Bücker, Alexander Orzol, Frederick Mueller
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Startprogramm fuer Cronjob/Systemctl. 
#Ermoeglicht starten des Steuerungsprogramms ohne Eingabe von Aussen

from IKfunction import *
Init2()

while (True):
	Fernbedienungsbetrieb()
	newAutomatikbetrieb()
