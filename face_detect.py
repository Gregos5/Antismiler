import numpy as np
import cv2, os
import serial, time 
import serial.tools.list_ports

def ID_PORTS_AVAILABLE():
    devices = []
    for port in ['COM%s' % (i + 1) for i in range(256)]:
        try:
            s = serial.Serial(port)
            s.close()
            devices.append(port)
            print('port found: ', port)
        except (OSError, serial.SerialException):
            pass
    
    return devices

def CLOSE_SERIAL_PORT(arduinos):
    try:
        for arduino in arduinos:
            arduino.device.close()
            print('arduino closed ', arduino.device)
    except:
        pass

def OPEN_SERIAL_PORT(DEV):
    try:
        Dev = serial.Serial(port=DEV,baudrate=115200, timeout=.1)
        Dev.isOpen()
    except IOError:
        Dev.close()
        Dev = serial.Serial(port=DEV,baudrate=115200, timeout=.1)
        Dev.isOpen()
    except (OSError, serial.SerialException,ValueError):
        return None
    
    return Dev

def SERIAL_WRITE_LINE(DEV,COMMAND):
    try:
        DEV.write(COMMAND.encode('UTF-8'))
        return 1
    except:
        return -1

def WRITE(DEV,COMMAND):
    STATE = -1
    TRY = 0
    while(STATE == -1):
        STATE = SERIAL_WRITE_LINE(DEV,COMMAND)
        TRY = TRY + 1
        if(TRY>10):
            return -1
    return STATE

class Buffer:
    def __init__(self):
        self.buffer = []
    def IN(self, dev, command):
        self.buffer.append(command)

class Arm:
    def __init__(self, device, webcam_range = 100):
        self.device = device
        self.current_slot = 0
        self.webcam_range = webcam_range 
        self.range_low = 90 - int(self.webcam_range/2)
    def set(self, xpos, ypos):
        anglex = int(self.range_low + xpos*self.webcam_range)
        angley = int(self.range_low + ypos*self.webcam_range)
        WRITE(self.device, "[A1 X{} Y{}]".format(anglex, angley))
    def get_slot(self):
        return self.current_slot

class Valve:
    def __init__(self, device):
        self.device = device

    def set(self, state):
        """
        0 close, 1 open
        """
        WRITE(self.device, "[V1 S{}]".format(state))

class Pump:
    def __init__(self, device):
        self.device = device

    def pump(self, volume: float):
            WRITE(self.device, "[P1 m{}]".format(volume))

class Smiles: 
    def __init__(self, time_limit: float, period: float):
        """
        ##
        Keeps track of consecutive smiles detection in a list called smiles
        the size of the list is fixed and dependent on size  time limit divided by the period in seconds 
        ex : 5 seconds limit at 0.2 sec period is 25 elements
        T is the period in seconds
        """
        # smiles variable is a list that holds boolean values depending on detected smile
        self.time_limit = time_limit
        self.period = period
        self.set()

    def set(self):
        self.smiles = [1]*(int(self.time_limit/self.period))
    
    def new(self, smile):
        """
        Adds True to the start of the list if there is a smile
        False if no smile 

        returns the 
        smile is the ouput from OpenCV cascade
        if no smile is detected, openCV returns an empty tuple 
        if a smile is detected , openCV returns a list with 4 coordinates 
        """
        if type(smile) == tuple:
            # No smile detected :(
            self.smiles.insert(0,0)
        else:
            # Smile Detected !  :)
            self.smiles.insert(0,1)
        # pop the last value to keep the list the same size
        self.smiles.pop()
        self.elapsed_time()

    def elapsed_time(self):
        """
        calculate elapsed time without a smile 
        """
        time_no_smile = 0.0
        # iterate through smiles list in order from newest to later 
        for value in self.smiles:
            if value == False:
                time_no_smile+=self.period
                time_no_smile = round(time_no_smile,1)
            else:
                # one True detected 
                self.time_no_smile = time_no_smile
                time_left = round(self.time_limit-time_no_smile, 1)
                # if (time_left)<=3 and time_left%1 == 0:
                #     print("{} seconds left ".format(time_left))
                return 
            

        # if no smile was detected in the whole array, it is time to... SHOOT !!!
        print('you should smile more, SHOOT')
        # send open valve command,  close valve, pump 
        # reset the list after shooting
        self.set()  
        return

def init_module():
    global arm, pump
    Ports = ID_PORTS_AVAILABLE()
    print("\nSource: ", Ports)
    for Port in Ports:
        device = OPEN_SERIAL_PORT(Port)
        print("\nDevice: ", device)
        arm = Arm(device)
        pump = Pump(device)

init_module()

# multiple cascades: https://github.com/opencv/opencv/tree/master/data/haarcascades

#https://github.com/opencv/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
path = os.path.dirname(os.path.realpath(__file__))
face_cascade = cv2.CascadeClassifier(os.path.join(path,'haarcascade_frontalface_default.xml'))
#https://github.com/opencv/opencv/blob/master/data/haarcascades/haarcascade_eye.xml
# eye_cascade = cv2.CascadeClassifier(os.path.join(path,'haarcascade_eye.xml'))

smile_cascade = cv2.CascadeClassifier(os.path.join(path,'haarcascade_smile.xml'))
cap = cv2.VideoCapture(0)
period = 0.2
smiles = Smiles(time_limit = 4, period = period)

while 1:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    
    h, w, c = img.shape
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        # print(int(x+w/2), int(y+h/2))
        relx = 1.0-float((x+w/2)/640)
        rely = 1.0-float((y+h/2)/480)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        smile = smile_cascade.detectMultiScale(
            roi_gray,
            scaleFactor= 1.5,
            minNeighbors=15,
            minSize=(25, 25),
            )
        for (sx, sy, sw, sh) in smile:
            cv2.rectangle(roi_color,(sx,sy),(sx+sw,sy+sh),(0,255,0),2)
        
        smiles.new(smile)
        
    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    try:
        arm.set(relx, rely)
        
    except:
        pass
    time.sleep(period)

cap.release()
cv2.destroyAllWindows()
