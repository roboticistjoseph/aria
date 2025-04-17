#Importing Libraries
import time
from time import sleep
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import RPi.GPIO as GPIO
from http.server import BaseHTTPRequestHandler, HTTPServer

#Assigning Pins- Rover
m11=29
m12=31
m21=33
m22=35

#Assigning Pins- ARM
s_base=7
s_shoulder=12
s_elbow=13
s_gripper=15

#Assigning Pins- Ultra Sonic Sensor
TRIG = 16
ECHO = 18

###BCM and GPIO Config
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#Rover Setup
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

#Arm Setup
GPIO.setup(s_base, GPIO.OUT)
GPIO.setup(s_shoulder, GPIO.OUT)
GPIO.setup(s_elbow, GPIO.OUT)
GPIO.setup(s_gripper, GPIO.OUT)

#All Off- ARM
#GPIO.output(s_base, False)
#GPIO.output(s_shoulder, False)
#GPIO.output(s_elbow, False)
#GPIO.output(s_gripper, False)

#Servo Setup
servo1 = GPIO.PWM(s_base,50)
servo2 = GPIO.PWM(s_shoulder,50) # pin 12 for servo2
servo3 = GPIO.PWM(s_elbow,50) # pin 11 for servo1
servo4 = GPIO.PWM(s_gripper,50) # pin 12 for servo2

#Ultra Sonic Sensor Setup
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

#All Off- Rover
GPIO.output(m11, False)
GPIO.output(m12, False)
GPIO.output(m21, False)
GPIO.output(m22, False)

#Ultra Sonic Sensor Trigger pin Off
GPIO.output(TRIG, False)

#WebServer Variable
Request = None

#Camera Variables
global index1
global index2

index1=0
index2=0

#Time to Settle and Rest after Initialization
time.sleep(1)

#Functions

#Function for Forward Movement of Rover
def forward():
   print("FORWARD")
   GPIO.output(m11 , True)
   GPIO.output(m12 , False)
   GPIO.output(m21 , False)
   GPIO.output(m22 , True)

#Function for BACKWARD Movement of Rover
def reverse():
   print("BACKWARD")
   GPIO.output(m11 , False)
   GPIO.output(m12 , True)
   GPIO.output(m21 , True)
   GPIO.output(m22 , False)

#Function for RIGHT(Clockwise) Movement of Rover
def right():
   print("RIGHT")
   GPIO.output(m11 , True)
   GPIO.output(m12 , False)
   GPIO.output(m21 , True)
   GPIO.output(m22 , False)

#Function for LEFT(Anti-Clockwise) Movement of Rover
def left():
   print("LEFT")
   GPIO.output(m11 , False)
   GPIO.output(m12 , True)
   GPIO.output(m21 , False)
   GPIO.output(m22 , True)

#Function for STOP the Movement of Rover
def stop():
   print("STOP")
   GPIO.output(m11 , False)
   GPIO.output(m12 , False)
   GPIO.output(m21 , False)
   GPIO.output(m22 , False)

#Mapping Function (Similar to that of Arduino)
def my_map(x, in_min, in_max, out_min, out_max):
    return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

#Manipulation Fucntion
def SetAngle(pin,angle):
    # Start PWM running on both servos, value of 0 (pulse off)
    pin.start(0)
    # Turn servo1 to 90
    pin.ChangeDutyCycle(angle)
    time.sleep(0.5)
    pin.ChangeDutyCycle(0)
    pin.stop()

#Pi Cam- Image Capture
def cami():
   #Camera Variables
   global index1
   #index1=0
   index1+=1
   with picamera.PiCamera() as camera:
       camera.resolution = (1280, 720)
       camera.capture('/home/pi/Desktop/image%s.jpg' % index1)
   camera.close()

   
#Pi Cam- Video Capture
def camv():
   #Camera Variables
   global index2
   #index2=0
   index2+=1

   with picamera.PiCamera() as camera:
      camera.start_recording('/home/pi/Desktop/video%s.h264' % index2)
      sleep(20)
      camera.stop_recording()
      camera.close()


#Function to measure Distance using Ultra Sonic Sensor
def ultra():

           global distance

           GPIO.output(TRIG, True)
           time.sleep(0.00001)
           GPIO.output(TRIG, False)

           while GPIO.input(ECHO)==0:
              pulse_start = time.time()

           while GPIO.input(ECHO)==1:
              pulse_end = time.time()

           pulse_duration = pulse_end - pulse_start
           distance = pulse_duration * 17150
           distance = round(distance+1.15, 2)
           time.sleep(2)

           return

#Main
try:

       #Webserver Reception for Manual or Automatic Control
       class RequestHandler_httpd(BaseHTTPRequestHandler):
         def do_GET(self):
           global Request
           messagetosend = bytes('Press the Button to Calculate Distance of Object Ahead',"utf")
           self.send_response(200)
           self.send_header('Content-Type', 'text/plain')
           self.send_header('Content-Length', len(messagetosend))
           self.end_headers()
           self.wfile.write(messagetosend)
           Request = self.requestline
           Request = Request[5 : int(len(Request)-9)]
           print(Request)
           
           data=[]
           data=Request.split(':')

           #Manual Control over Rover
           if data[0] == 'up':
              forward()
           if data[0] == 'down':
              reverse()
           if data[0] == 'left':
              left()
           if data[0] == 'right':
              right()
           if data[0] == 'stop':
              stop()
              
           #Pi Camera Image Mode
           if data[0] == 'cam':
              cami()
              print("Image Captured!!")
              
           #Pi Camera Video Mode
           if data[0] == 'vid':
              camv()
              print("Video Captured!!")
              
           #Distance Evaluation
           if data[0] == 'ultra':
              ultra()
              print ("Object is at a distance of :",distance,"cm")
              messagetosend = bytes(str(distance),"utf")
              self.send_response(200)
              self.send_header('Content-Type', 'text/plain')
              self.send_header('Content-Length', len(messagetosend))
              self.end_headers()
              self.wfile.write(messagetosend)

           #Manual Control over Rover
           if data[0] == "B":
              data[1]=int(data[1])
              data[1]=my_map(data[1],0,180,1,12)
              #SetAngle(servo1,data[1])
           if data[0] == "S":
              data[1]=int(data[1])
              data[1]=my_map(data[1],0,180,1,12)
              #SetAngle(servo2,data[1])
           if data[0] == "E":
              data[1]=int(data[1])
              data[1]=my_map(data[1],0,180,1,12)
              #SetAngle(servo3,data[1])
           if data[0] == "G":
              data[1]=int(data[1])
              data[1]=my_map(data[1],0,180,1,12)
              #SetAngle(servo4,data[1])

           return





       server_address_httpd = ('192.168.43.219',8080)
       httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
       print('Starting Server')
       httpd.serve_forever()


#Function for Exiting Program by cleaning all GPIO's
finally:
       servo1.stop()
       servo2.stop()
       servo3.stop()
       servo4.stop()
       GPIO.cleanup()
