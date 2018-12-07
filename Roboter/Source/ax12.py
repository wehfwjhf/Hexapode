# coding: utf-8
'''
Thassilo Bücker, Alexander Orzol, Frederick Mueller, Moritz Kolb
Campus Velbert/Heiligenhaus, Hochschule Bochum, 2016/2017

Based on Jesse Merritt's script:
https://github.com/jes1510/python_dynamixels

and Josue Alejandro Savage's Arduino library:
http://savageelectronics.blogspot.it/2011/01/arduino-y-dynamixel-ax-12.html
'''

#Eine grundlegende Kommunikationsbibliothek, welche alle Befehle nach dem
#Datenblatt der AX12-A Servos umsetzt. Nicht alle Befehle wurden getestet!

from time import sleep
import time
import serial
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART

UART.setup("UART2")
UART.setup("UART4")

class Ax12:
    # important AX-12 constants
    # /////////////////////////////////////////////////////////// EEPROM AREA
    AX_MODEL_NUMBER_L = 0
    AX_MODEL_NUMBER_H = 1
    AX_VERSION = 2
    AX_ID = 3
    AX_BAUD_RATE = 4
    AX_RETURN_DELAY_TIME = 5
    AX_CW_ANGLE_LIMIT_L = 6
    AX_CW_ANGLE_LIMIT_H = 7
    AX_CCW_ANGLE_LIMIT_L = 8
    AX_CCW_ANGLE_LIMIT_H = 9
    AX_SYSTEM_DATA2 = 10
    AX_LIMIT_TEMPERATURE = 11
    AX_DOWN_LIMIT_VOLTAGE = 12
    AX_UP_LIMIT_VOLTAGE = 13
    AX_MAX_TORQUE_L = 14
    AX_MAX_TORQUE_H = 15
    AX_RETURN_LEVEL = 16
    AX_ALARM_LED = 17
    AX_ALARM_SHUTDOWN = 18
    AX_OPERATING_MODE = 19
    AX_DOWN_CALIBRATION_L = 20
    AX_DOWN_CALIBRATION_H = 21
    AX_UP_CALIBRATION_L = 22
    AX_UP_CALIBRATION_H = 23

    # ////////////////////////////////////////////////////////////// RAM AREA
    AX_TORQUE_STATUS = 24
    AX_LED_STATUS = 25
    AX_CW_COMPLIANCE_MARGIN = 26
    AX_CCW_COMPLIANCE_MARGIN = 27
    AX_CW_COMPLIANCE_SLOPE = 28
    AX_CCW_COMPLIANCE_SLOPE = 29
    AX_GOAL_POSITION_L = 30
    AX_GOAL_POSITION_H = 31
    AX_GOAL_SPEED_L = 32
    AX_GOAL_SPEED_H = 33
    AX_TORQUE_LIMIT_L = 34
    AX_TORQUE_LIMIT_H = 35
    AX_PRESENT_POSITION_L = 36
    AX_PRESENT_POSITION_H = 37
    AX_PRESENT_SPEED_L = 38
    AX_PRESENT_SPEED_H = 39
    AX_PRESENT_LOAD_L = 40
    AX_PRESENT_LOAD_H = 41
    AX_PRESENT_VOLTAGE = 42
    AX_PRESENT_TEMPERATURE = 43
    AX_REGISTERED_INSTRUCTION = 44
    AX_PAUSE_TIME = 45
    AX_MOVING = 46
    AX_LOCK = 47
    AX_PUNCH_L = 48
    AX_PUNCH_H = 49

    # /////////////////////////////////////////////////////////////// Status Return Levels
    AX_RETURN_NONE = 0
    AX_RETURN_READ = 1
    AX_RETURN_ALL = 2

    # /////////////////////////////////////////////////////////////// Instruction Set
    AX_PING = 1
    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_REG_WRITE = 4
    AX_ACTION = 5
    AX_RESET = 6
    AX_SYNC_WRITE = 131

    # /////////////////////////////////////////////////////////////// Lengths
    AX_RESET_LENGTH = 2
    AX_ACTION_LENGTH = 2
    AX_ID_LENGTH = 4
    AX_LR_LENGTH = 4
    AX_SRL_LENGTH = 4
    AX_RDT_LENGTH = 4
    AX_LEDALARM_LENGTH = 4
    AX_SHUTDOWNALARM_LENGTH = 4
    AX_TL_LENGTH = 4
    AX_VL_LENGTH = 6
    AX_AL_LENGTH = 7
    AX_CM_LENGTH = 6
    AX_CS_LENGTH = 5
    AX_COMPLIANCE_LENGTH = 7
    AX_CCW_CW_LENGTH = 8
    AX_BD_LENGTH = 4
    AX_TEM_LENGTH = 4
    AX_MOVING_LENGTH = 4
    AX_RWS_LENGTH = 4
    AX_VOLT_LENGTH = 4
    AX_LOAD_LENGTH = 4
    AX_LED_LENGTH = 4
    AX_TORQUE_LENGTH = 4
    AX_POS_LENGTH = 4
    AX_GOAL_LENGTH = 5
    AX_MT_LENGTH = 5
    AX_PUNCH_LENGTH = 5
    AX_SPEED_LENGTH = 5
    AX_GOAL_SP_LENGTH = 7

    # /////////////////////////////////////////////////////////////// Specials
    AX_BYTE_READ = 1
    AX_INT_READ = 2
    AX_ACTION_CHECKSUM = 250
    AX_BROADCAST_ID = 254
    AX_START = 255
    AX_CCW_AL_L = 255
    AX_CCW_AL_H = 3
    AX_LOCK_VALUE = 1
    LEFT = 0
    RIGTH = 1
    RX_TIME_OUT = 10
    TX_DELAY_TIME =  0.000002

    # RPi constants, diese definieren die Steuerung des ICs bzw. der Umwandlung voll- auf halbduplex.
    RPI_DIRECTION_PIN = "P8_12"
    RPI_DIRECTION_TX = GPIO.HIGH
    RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001



    ######################## Character Constants (shortened) ################
    # Entsprechen den oben stehenden Variablen in Form eines Characters
    # Diese werden genutzt, um eine erneute Umwandlung der Variablen in Character
    # bei jedem Senden zu vermeiden. Dadurch wird die Geschwindigkeit erhöht.
    cAX_BAUD_RATE = chr(4)
    cAX_RETURN_DELAY_TIME = chr(5)
    cAX_CW_ANGLE_LIMIT_L = chr(6)
    cAX_LIMIT_TEMPERATURE = chr(11)
    cAX_DOWN_LIMIT_VOLTAGE = chr(12)
    cAX_MAX_TORQUE_L = chr(14)
    cAX_RETURN_LEVEL = chr(16)
    cAX_ALARM_LED = chr(17)
    cAX_ALARM_SHUTDOWN = chr(18)
    cAX_TORQUE_STATUS = chr(24)
    cAX_LED_STATUS = chr(25)
    cAX_CW_COMPLIANCE_MARGIN = chr(26)
    cAX_GOAL_POSITION_L = chr(30)
    cAX_PRESENT_POSITION_L = chr(36)
    cAX_PRESENT_SPEED_L = chr(38)
    cAX_PRESENT_LOAD_L = chr(40)
    cAX_PRESENT_VOLTAGE = chr(42)
    cAX_PRESENT_TEMPERATURE = chr(43)
    cAX_REGISTERED_INSTRUCTION = chr(44)
    cAX_MOVING = chr(46)
    cAX_LOCK = chr(47)
    cAX_PUNCH_L = chr(48)
    cAX_PING = chr(1)
    cAX_READ_DATA = chr(2)
    cAX_WRITE_DATA = chr(3)
    cAX_REG_WRITE = chr(4)
    cAX_ACTION = chr(5)
    cAX_RESET = chr(6)
    cAX_SYNC_WRITE = chr(131)
    cAX_RESET_LENGTH = chr(2)
    cAX_ACTION_LENGTH = chr(2)
    cAX_ID_LENGTH = chr(4)
    cAX_LR_LENGTH = chr(4)
    cAX_SRL_LENGTH = chr(4)
    cAX_RDT_LENGTH = chr(4)
    cAX_LEDALARM_LENGTH = chr(4)
    cAX_SHUTDOWNALARM_LENGTH = chr(4)
    cAX_TL_LENGTH = chr(4)
    cAX_VL_LENGTH = chr(6)
    cAX_AL_LENGTH = chr(7)
    cAX_COMPLIANCE_LENGTH = chr(7)
    cAX_BD_LENGTH = chr(4)
    cAX_TEM_LENGTH = chr(4)
    cAX_MOVING_LENGTH = chr(4)
    cAX_RWS_LENGTH = chr(4)
    cAX_VOLT_LENGTH = chr(4)
    cAX_LOAD_LENGTH = chr(4)
    cAX_LED_LENGTH = chr(4)
    cAX_TORQUE_LENGTH = chr(4)
    cAX_POS_LENGTH = chr(4)
    cAX_GOAL_LENGTH = chr(5)
    cAX_MT_LENGTH = chr(5)
    cAX_PUNCH_LENGTH = chr(5)
    cAX_SPEED_LENGTH = chr(5)
    cAX_GOAL_SP_LENGTH = chr(7)
    cAX_BYTE_READ = chr(1)
    cAX_INT_READ = chr(2)
    cAX_ACTION_CHECKSUM = chr(250)
    cAX_BROADCAST_ID = chr(254)
    cAX_START = chr(255)
    cAX_LOCK_VALUE = chr(1)
    ############################################################

    # static variables
    port = None
    gpioSet = False

    def __init__(self):
        if(Ax12.port == None):
       		#Binden der seriellen Schnittstellen
            Ax12.port = serial.Serial("/dev/ttyO4", baudrate=1000000, timeout=0.003)
            Ax12.port2 = serial.Serial("/dev/ttyO2", baudrate=1000000, timeout=0.003)
        if(not Ax12.gpioSet):
            GPIO.setwarnings(False)
            #GPIO.setmode(GPIO.BCM)
            GPIO.setup(Ax12.RPI_DIRECTION_PIN, GPIO.OUT)
            Ax12.gpioSet = True
        self.direction(Ax12.RPI_DIRECTION_RX)

    connectedServos = []

    # Error lookup dictionary for bit masking
    dictErrors = {  1 : "Input Voltage",
            2 : "Angle Limit",
            4 : "Overheating",
            8 : "Range",
            16 : "Checksum",
            32 : "Overload",
            64 : "Instruction"
            }

    # Custom error class to report AX servo errors
    class axError(Exception) : pass

    # Servo timeout
    class timeoutError(Exception) : pass

    def direction(self,d):
        GPIO.output(Ax12.RPI_DIRECTION_PIN, d)
        sleep(Ax12.RPI_DIRECTION_SWITCH_DELAY)


    def readData(self,id, amount=7):
    	# readData muss die Richtige Menge an zu empfangenden Bytes uebergeben bekommen.
    	# Es wurden noch nicht alle Befehle getestet, jedoch sind 7 bytes fuer die meisten
    	# Befehle zutreffend.

        self.direction(Ax12.RPI_DIRECTION_RX)
        reply = []
        append = reply.append
        read = Ax12.port2.read
        i = 0
        for i in xrange (amount):
            append(read(1)) # [0xff, 0xff, origin, length, error] 
            i += 1
        try:
            assert ord(reply[1]) == 0xFF
        except:
            e = "Timeout on servo " + str(id)
            #Inputbuffer leeren bei error um Folgefehler zu vermeiden
            while True:
                try:
                    reply.append(Ax12.port2.read(1)) # [0xff, 0xff, origin, length, error]
                    print ("error on reply:" + reply)
                    if (reply[-1] == ''):
                        raise
                except:
                    break
            raise Ax12.timeoutError(e)

        try:
            length = ord(reply[4]) - 2
            error = ord(reply[5])
            if(error != 0):
                print ("Error from servo: " + Ax12.dictErrors[error] + ' (code  ' + hex(error) + ')')
                return -error
            elif(length == 0):
                return error
            else:
                if(length > 1):
                    returnValue = (ord(reply[7])<<8) + (ord(reply[6])<<0)
                else:
                    returnValue = ord(reply[6])
                return returnValue
        except (Exception, detail):
            raise Ax12.axError(detail)

    def ping(self,id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_READ_DATA + Ax12.AX_PING))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_READ_DATA,Ax12.cAX_PING,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        self.direction(Ax12.RPI_DIRECTION_RX)
        return self.readData(id)
        
    def factoryReset(self,id, confirm = False):
        if(confirm):
            self.direction(Ax12.RPI_DIRECTION_TX)
            Ax12.port.flushInput()
            checksum = (~(id + Ax12.AX_RESET_LENGTH + Ax12.AX_RESET))&0xff
            outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_RESET_LENGTH,Ax12.cAX_RESET,chr(checksum)])
            Ax12.port.write(outData)
            sleep(Ax12.TX_DELAY_TIME)
            return self.readData(id)
        else:
            print ("nothing done, please send confirm = True as this fuction reset to the factory default value, i.e reset the motor ID")
            return

    def setID(self, id, newId):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_ID_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_ID + newId))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_ID_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_ID,chr(newId),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setBaudRate(self, id, baudRate):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        br = ((2000000/long(baudRate))-1)
        checksum = (~(id + Ax12.AX_BD_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_BAUD_RATE + br))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_BD_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_BAUD_RATE,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setStatusReturnLevel(self, id, level):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_SRL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_RETURN_LEVEL + level))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_SRL_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_RETURN_LEVEL,chr(level),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setReturnDelayTime(self, id, delay):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_RDT_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_RETURN_DELAY_TIME + (int(delay)/2)&0xff))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_RDT_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_RETURN_DELAY_TIME,chr((int(delay)/2)&0xff),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def lockRegister(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_LR_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_LOCK + Ax12.AX_LOCK_VALUE))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_LR_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_LOCK,Ax12.cAX_LOCK_VALUE,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def move(self, id, position):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + Ax12.AX_GOAL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_GOAL_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_GOAL_POSITION_L,chr(p[0]),chr(p[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def moveSpeed(self, id, position, speed):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + Ax12.AX_GOAL_SP_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_GOAL_SP_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_GOAL_POSITION_L,chr(p[0]),chr(p[1]),chr(s[0]),chr(s[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)


    def moveSpeedSync(self, position, speed):
    	#Broadcast-Bewegung aller Servos
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        s = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        for i in xrange(18):
            p[i] = [position[i]&0xff, position[i]>>8]
            s[i] = [speed[i]&0xff, speed[i]>>8]
        L = 4
        Length = (L+1)*18+4
        checksum = Ax12.AX_BROADCAST_ID + Length + Ax12.AX_SYNC_WRITE + Ax12.AX_GOAL_POSITION_L + L
        for i in xrange(18):
        	checksum += i+1 + p[i][0] + p[i][1] + s[i][0] + s[i][1]
        checksum = (~checksum)&0xFF
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,Ax12.cAX_BROADCAST_ID,chr(Length),Ax12.cAX_SYNC_WRITE,Ax12.cAX_GOAL_POSITION_L,chr(L)])

        for i in xrange(18):
        	outData += "".join([chr(i+1),chr(p[i][0]),chr(p[i][1]),chr(s[i][0]),chr(s[i][1])])
        

        outData += chr(checksum)
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)

    def moveRW(self, id, position):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + Ax12.AX_GOAL_LENGTH + Ax12.AX_REG_WRITE + Ax12.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_GOAL_LENGTH,Ax12.cAX_REG_WRITE,Ax12.cAX_GOAL_POSITION_L,chr(p[0]),chr(p[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def moveSpeedRW(self, id, position, speed):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + Ax12.AX_GOAL_SP_LENGTH + Ax12.AX_REG_WRITE + Ax12.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_GOAL_SP_LENGTH,Ax12.cAX_REG_WRITE,Ax12.cAX_GOAL_POSITION_L,chr(p[0]),chr(p[1]),chr(s[0]),chr(s[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        #return self.readData(id)

    def action(self):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_BROADCAST_ID,Ax12.cAX_ACTION_LENGTH,Ax12.cAX_ACTION,Ax12.cAX_ACTION_CHECKSUM])
        Ax12.port.write(outData)
        #sleep(Ax12.TX_DELAY_TIME)

    def setTorqueStatus(self, id, status):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        ts = 1 if ((status == True) or (status == 1)) else 0
        checksum = (~(id + Ax12.AX_TORQUE_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_TORQUE_STATUS + ts))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_TORQUE_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_TORQUE_STATUS,chr(ts),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setLedStatus(self, id, status):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        ls = 1 if ((status == True) or (status == 1)) else 0
        checksum = (~(id + Ax12.AX_LED_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_LED_STATUS + ls))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_LED_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_LED_STATUS,chr(ls),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setTemperatureLimit(self, id, temp):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_TL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_LIMIT_TEMPERATURE + temp))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_TL_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_LIMIT_TEMPERATURE,chr(temp),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setVoltageLimit(self, id, lowVolt, highVolt):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_VL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_DOWN_LIMIT_VOLTAGE + lowVolt + highVolt))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_VL_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_DOWN_LIMIT_VOLTAGE,chr(lowVolt),chr(highVolt),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setAngleLimit(self, id, cwLimit, ccwLimit):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        cw = [cwLimit&0xff, cwLimit>>8]
        ccw = [ccwLimit&0xff, ccwLimit>>8]
        checksum = (~(id + Ax12.AX_AL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_CW_ANGLE_LIMIT_L + cw[0] + cw[1] + ccw[0] + ccw[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_AL_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_CW_ANGLE_LIMIT_L,chr(cw[0]),chr(cw[1]),chr(ccw[0]),chr(ccw[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setTorqueLimit(self, id, torque):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        mt = [torque&0xff, torque>>8]
        checksum = (~(id + Ax12.AX_MT_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_MAX_TORQUE_L + mt[0] + mt[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_MT_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_MAX_TORQUE_L,chr(mt[0]),chr(mt[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setPunchLimit(self, id, punch):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        p = [punch&0xff, punch>>8]
        checksum = (~(id + Ax12.AX_PUNCH_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_PUNCH_L + p[0] + p[1]))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_PUNCH_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_PUNCH_L,chr(p[0]),chr(p[1]),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setCompliance(self, id, cwMargin, ccwMargin, cwSlope, ccwSlope):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_COMPLIANCE_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_CW_COMPLIANCE_MARGIN + cwMargin + ccwMargin + cwSlope + ccwSlope))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_COMPLIANCE_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_CW_COMPLIANCE_MARGIN,chr(cwMargin),chr(ccwMargin),chr(cwSlope),chr(ccwSlope),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setLedAlarm(self, id, alarm):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_LEDALARM_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_ALARM_LED + alarm))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_LEDALARM_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_ALARM_LED,chr(alarm),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def setShutdownAlarm(self, id, alarm):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_SHUTDOWNALARM_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_ALARM_SHUTDOWN + alarm))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_SHUTDOWNALARM_LENGTH,Ax12.cAX_WRITE_DATA,Ax12.cAX_ALARM_SHUTDOWN,chr(alarm),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,7)

    def readTemperature(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_TEM_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_TEMPERATURE + Ax12.AX_BYTE_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_TEM_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_PRESENT_TEMPERATURE,Ax12.AX_BYTE_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readPosition(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_POS_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_POSITION_L + Ax12.AX_INT_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_POS_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_PRESENT_POSITION_L,Ax12.cAX_INT_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,9)

    def readVoltage(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_VOLT_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_VOLTAGE + Ax12.AX_BYTE_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_VOLT_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_PRESENT_VOLTAGE,Ax12.cAX_BYTE_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readSpeed(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_SPEED_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_SPEED_L + Ax12.AX_INT_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_SPEED_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_PRESENT_SPEED_L,Ax12.cAX_INT_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readLoad(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_LOAD_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_LOAD_L + Ax12.AX_INT_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_LOAD_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_PRESENT_LOAD_L,Ax12.cAX_INT_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,9)

    def readMovingStatus(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_MOVING_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_MOVING + Ax12.AX_BYTE_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_MOVING_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_MOVING,Ax12.cAX_BYTE_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,8)

    def readRWStatus(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + Ax12.AX_RWS_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_REGISTERED_INSTRUCTION + Ax12.AX_BYTE_READ))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),Ax12.cAX_RWS_LENGTH,Ax12.cAX_READ_DATA,Ax12.cAX_REGISTERED_INSTRUCTION,Ax12.cAX_BYTE_READ,chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readRegisterStatus(self, id,reg):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + 0x04 + 0x02 + reg + 0x01))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),chr(0x04),chr(0x02),chr(reg),chr(0x01),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,8)

    def readLedStatus(self, id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(id + 0x04 + 0x02 + 0x19 + 0x01))&0xff
        outData = "".join([Ax12.cAX_START,Ax12.cAX_START,chr(id),chr(0x04),chr(0x02),chr(0x19),chr(0x01),chr(checksum)])
        Ax12.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id,8)

    def learnServos(self,minValue=1, maxValue=18, verbose=False):
    	# Pingen aller Servos in dem angegebenen Wertebereich und Ausgabe welche gefunden wurden.
        servoList = []
        for i in range(minValue, maxValue + 1):
            try :
                temp = self.ping(i)
                servoList.append(i)
                if verbose: print ("Found servo #" + str(i))
                time.sleep(0.1)

            except (Exception, detail):
                if verbose : print ("Error pinging servo #" + str(i) + ': ' + str(detail))
                pass
        return servoList

#def playPose() :
#    '''
#    Open a file and move the servos to specified positions in a group move
#    '''
#    infile=open(Arguments.playpose, 'r')    # Open the file
#    poseDict = {}                           # Dictionary to hold poses and positions
#    if Arguments.verbose : print "Reading pose from", Arguments.playpose
#    for line in infile.readlines() :        # Read the file and step through it
#        servo = int(line.split(':')[0])     # Servo is first
#        position = int(line.split(':')[1])  # Position is second
#        poseDict[servo]=position            # add the servo to the Dictionary
#
#    groupMove2(poseDict)
#
#
#
#def writePose() :
#    '''
#    Read the servos and save the positions to a file
#    '''
#    of = open(Arguments.savepose, 'w')      # open the output file
#    pose = getPose2(connectedServos)        # get the positions
#    if Arguments.verbose :
#        print "Servo Positions"
#        print "---------------"
#
#    for key in  pose.keys():                # step through the keys, writing to the file
#        if Arguments.verbose : print "Servo " + str(key), pose[key]
#        of.write(str(key) + ':' + str(pose[key]) + '\n')    # Write to the file
#
#    if Arguments.verbose :
#        print "Wrote pose to " + Arguments.savepose
#        print
#
#    of.close()      # close the file
#
