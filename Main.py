import cv2, os
import serial, time, datetime
import serial.tools.list_ports

class MyStr(str):
    def __eq__(self, other):
        return self.__contains__(other)

class Serial:
    """
    # Serial communication 
    communicate between laptop and any amount of arduinos 
    ## FIFO buffer 
    sequential commands.\n
    call IN([device, command]) to add a specific command\n
    call OUT() method to execute commands from the list
    """
    def __init__(self):
        self.state = False
        self.buffer = []
        self.size = 200
        self.current_device = None
        self.blocked = False

    def ID_PORTS_AVAILABLE(self):
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

    def CLOSE_SERIAL_PORT(self, arduinos):
        try:
            for arduino in arduinos:
                arduino.device.close()
                print('arduino closed ', arduino.device)
        except:
            pass

    def OPEN_SERIAL_PORT(self, DEV):
        try:
            Dev = serial.Serial(port=DEV,baudrate=115200, timeout=.1)
            Dev.isOpen()
        except IOError:
            Dev.close()
            Dev = serial.Serial(port=DEV,baudrate=115200, timeout=.1)
            Dev.isOpen()
        except (OSError, serial.SerialException,ValueError):
            return None
        self.current_device = Dev
        return Dev

    def SERIAL_WRITE_LINE(self, DEV,COMMAND):
        try:
            DEV.write(COMMAND.encode('UTF-8'))
            print(">> ", COMMAND)
            return 1
        except:
            return -1
    
    def SERIAL_READ_LINE(self, DEV):
        try:
            Incoming_Data = DEV.readlines()
            Incoming_Data = self.DECODE_LINES(Incoming_Data)
            self.FLUSH_PORT(DEV) 
            return Incoming_Data
        except (NameError,IOError,ValueError):
            pass
        return [-1]

    def WRITE(self, DEV, COMMAND):
        STATE = -1
        TRY = 0
        while(STATE == -1):
            STATE = self.SERIAL_WRITE_LINE(DEV,COMMAND)
            self.current_device = DEV
            TRY = TRY + 1
            if(TRY>10):
                return -1
        return STATE

    def READ(self, DEV):
        STATE = -1
        TRY = 0
        try:
            while(STATE == -1):
                if (DEV.inWaiting() > 0):
                    STATE = self.SERIAL_READ_LINE(DEV)
                TRY = TRY + 1
                if(TRY>10):
                    return -1
            return STATE
        except:
            pass
        
    def FLUSH_PORT(self, DEV):
        try:
            for i in range(len(DEV)):
                print('flushed input')
                DEV[i].flushInput()
        except:
            pass
    
    def DECODE_LINES(self, cmd_list):
        for i in range(0,len(cmd_list)):

            cmd_list[i] = cmd_list[i].decode('UTF-8').replace('\r\n','')
            cmd_list[i] = self.DECODE_LINE(cmd_list[i])
            print(cmd_list[i])
        return cmd_list

    def DECODE_LINE(self, COMMAND):
        if COMMAND[0]!="[" or COMMAND[-1]!="]":
            return "- " + COMMAND 
        
        operator = MyStr(COMMAND[1:-1])
        out = COMMAND[1:-1]
        match operator:
            case "ERR":
                self.POP()
            case "FREE":
                # Serial is now available to receive next command
                self.state = False
                self.OUT()
            case "VALID":
                # POP buffer
                self.POP()
        return out
      
    def IN(self, DEV, COMMAND):
        '''
        add new command to the buffer list
        '''
        if len(self.buffer)<self.size:
            self.buffer.append([DEV, COMMAND])
        else: 
            print("buffer full")

    def OUT(self):
        '''
        Exectute next command in the buffer List
        3 types : Package, Block, Notification
        '''
       
        try: 
            if len(self.buffer) and self.state==False:
                if (not self.blocked):
                    if self.buffer[0][0]=='WAIT':
                        print('Blocked')
                        self.START_BLOCK()
                    else:
                        dev, com = [*self.buffer[0]]
                        self.WRITE(dev, com)
                        self.state = True
                    
                if self.blocked:
                    if datetime.datetime.now().timestamp()>self.time_to_unblock:
                        print('Unblocked')
                        self.blocked=False
                        self.POP()
        except:
            pass
            
    def POP(self):
        '''Delete executed/Validated command from list'''
        if len(self.buffer):
            print("pop")
            device, command = self.buffer.pop(0)
            return command
    
    def RESET(self):
        self.buffer = []

    def BLOCK(self, seconds: float):
        self.seconds = seconds
        self.buffer.append(['WAIT', str(seconds)])

    def START_BLOCK(self):
        start_time = datetime.datetime.now().timestamp()
        self.time_to_unblock = self.seconds + start_time
        self.blocked = True

class Arm:
    def __init__(self, SERIAL, device, webcam_range = 100):
        self.ser = SERIAL
        self.device = device
        self.current_slot = 0
        self.webcam_range = webcam_range 
        self.range_low = 90 - int(self.webcam_range/2)
    def set(self, xpos, ypos):
        anglex = int(self.range_low + xpos*self.webcam_range)
        angley = int(self.range_low + ypos*self.webcam_range)
        self.ser.IN(self.device, "[A1 X{} Y{}]".format(anglex, angley))
    def get_slot(self):
        return self.current_slot

class Valve:
    def __init__(self, SERIAL, device):
        self.device = device
        self.ser = SERIAL
    def set(self, state):
        """
        0 close, 1 open
        """
        self.ser.IN(self.device, "[V1 S{}]".format(state))

class Pump:
    def __init__(self,SERIAL, device):
        self.ser = SERIAL
        self.device = device

    def pump(self, volume: float):
        self.ser.IN(self.device, "[P1 m{}]".format(volume))

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

ser = Serial()

def init_module():
    global valve, arm, pump, ser
    Ports = ser.ID_PORTS_AVAILABLE()
    print("\nSource: ", Ports)
    for Port in Ports:
        device = ser.OPEN_SERIAL_PORT(Port)
        print("\nDevice: ", device)
        time.sleep(2)
        ser.IN(device, "[D]")
        ser.OUT()
        while(ser.state == True):
            details = ser.READ(device)
            time.sleep(0.1)

        details = details[0].split("(")
        device_desc =  details[1][:-1]
        print(device_desc)
        details = details[0].split(" ")
        for detail in details:
            print(detail)
            try:
                op = detail[0]
                match op:
                    case 'A':
                        arm = Arm(ser, device)
                    case 'P':
                        pump = Pump(ser, device)
                    case 'V':
                        valve = Valve(ser, device)
                    case _:
                        pass
            except IndexError:
                pass
        
        

init_module()

# multiple cascades: https://github.com/opencv/opencv/tree/master/data/haarcascades

#https://github.com/opencv/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
path = os.path.dirname(os.path.realpath(__file__))
face_cascade = cv2.CascadeClassifier(os.path.join(path,'face_detection_xml\\','haarcascade_frontalface_default.xml'))
smile_cascade = cv2.CascadeClassifier(os.path.join(path,'face_detection_xml\\','haarcascade_smile.xml'))
cap = cv2.VideoCapture(0)
period = 0.2
smiles = Smiles(time_limit = 5, period = period)

while 1:
    global arm, pump
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
        if len(ser.buffer)<1:
            arm.set(relx, rely)
    except:
        pass
    ser.OUT()
    time.sleep(period)
    ser.READ(ser.current_device)
    
cap.release()
cv2.destroyAllWindows()
