#Thassilo Bücker, Alexander Orzol, Frederick Mueller
#Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

#Startprogramm fuer Start ueber Konsole bzw. Hauptmenue.

import IKfunction
IKfunction.Init2()
import time
def ProgrammSelect():

	while(1):
		print ("Hauptmenu")
		print ("Handbetrieb          = 1")
		print ("Automatikbetrieb     = 2")
		print ("Fernbedinungsbetrieb = 3")
		print ("Programm beenden = 0")
		programmSelect = raw_input("Welches Programm wollen Sie ausfuehren") 
		if (programmSelect == "1"):
			IKfunction.Handbetrieb();
		if (programmSelect == "2"):
			IKfunction.newAutomatikbetrieb();
		if (programmSelect == "3"):
			IKfunction.Fernbedienungsbetrieb();
		if (programmSelect == "0"):
			break;

ProgrammSelect();