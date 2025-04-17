####Importing Libraries####
#Libraries for Time and GPIO's of Raspberry Pi
import time
from time import sleep
import RPi.GPIO as GPIO
#Libraries for Web Server
import http.server
from http.server import BaseHTTPRequestHandler, HTTPServer
#Libraries for Object Detection
#Picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
#Computation
import cv2
import numpy as np


###BCM and GPIO Config

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

#Count for alignment
global FPS
FPS = 1

#Camera Variables
global index1
global index2

index1=0
index2=0

###BCM and GPIO Config
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#Rover Setup
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

#Arm Setup
#GPIO.setup(s_base, GPIO.OUT)
GPIO.setup(s_shoulder, GPIO.OUT)
#GPIO.setup(s_elbow, GPIO.OUT)
GPIO.setup(s_gripper, GPIO.OUT)

#Ultra Sonic Sensor Setup
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

#All Off- Rover
GPIO.output(m11, False)
GPIO.output(m12, False)
GPIO.output(m21, False)
GPIO.output(m22, False)

#All Off- ARM
#GPIO.output(s_base, False)
GPIO.output(s_shoulder, False)
#GPIO.output(s_elbow, False)
GPIO.output(s_gripper, False)

#Ultra Sonic Sensor Trigger pin Off
GPIO.output(TRIG, False)

#WebServer Variable
Request = None

#Time to Settle and Rest after Initialization
time.sleep(2)

####Functions########

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
           distance = round(distance, 2)

           return


#Function for Object Detection using Template Matching Technique
def objdetect():

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    #Getting the Template
    if Request == 'black%20king':
       #Reading Template
       template = cv2.imread('template.jpg', 0)
       print('Black King Template Uploaded!!')
    else:
       #Reading Template
       template = cv2.imread('template2.jpg', 0)
       print('White King Template Uploaded!!')

    # allow the camera to warmup
    time.sleep(0.1)

    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()

    FPS=1

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        # Start timer (for calculating frame rate)
        t1 = cv2.getTickCount()

        # grab the raw NumPy array representing the image
        image = frame.array
        # Saving the copy of Frame
        img_rgb = image
        # Convert to Gray
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        
        # Shape of Template
        w, h = template.shape[::-1]
        
        #Template Matching
        res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
        
        #Cordinates
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(res)
        # Starting and Ending Coordinates
        (startX, startY) = (int(maxLoc[0] ), int(maxLoc[1] ))
        (endX, endY) = (int((maxLoc[0] + w) ), int((maxLoc[1] + h) ))

        # Draw framerate in corner of frame
        cv2.putText(img_rgb,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

        # Draw Rectangle around Detected Object
        cv2.rectangle(img_rgb, (startX, startY), (endX, endY), (0, 0, 255), 2)

        # Show the frame
        cv2.imshow("Frame", img_rgb)

        # Calculate framerate
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc= 1/time1

        key = cv2.waitKey(1) & 0xFF


        #CODE FOR ALIGNING
        if FPS > 10:
           #Center of rectangle
           xcord = (startX+endX)/2
           ycord = (startY+endY)/2
	
           #Alignment
           print("Aligning the Rover")
           if xcord>330:
              right()
              print("Moving Right")
           if xcord<310:
              left()
              print("Moving Left")

           time.sleep(0.1)
           stop()
        time.sleep(1)
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        if FPS>15:
           # Exit when Centerized
           if xcord>310 and xcord<330:
               camera.close()
               print("Centerized")
               break

        FPS = FPS+1
        print ("FPS: ",FPS)
        
    cv2.destroyAllWindows()
    time.sleep(1)

    return


#Function for Picking Up the Object using Robotic ARM
def pickup():
    #Code Please

    # Set pins 11 & 12 as outputs, and define as PWM servo1 & servo2
    GPIO.setup(12,GPIO.OUT)
    servo1 = GPIO.PWM(12,50) # pin 11 for servo1
    GPIO.setup(15,GPIO.OUT)
    servo2 = GPIO.PWM(15,50) # pin 12 for servo2

    # Start PWM running on both servos, value of 0 (pulse off)
    servo2.start(0)
    # Turn servo1 to 90
    servo2.ChangeDutyCycle(7)
    time.sleep(0.5)
    servo2.ChangeDutyCycle(0)
    servo2.stop()
    # Wait for 2 seconds
    time.sleep(2)

    # Start PWM running on both servos, value of 0 (pulse off)
    servo1.start(0)
    # Turn servo1 to 90
    servo1.ChangeDutyCycle(5.5)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()
    # Wait for 2 seconds
    time.sleep(2)

    # Start PWM running on both servos, value of 0 (pulse off)
    servo2.start(0)
    # Turn servo1 to 90
    servo2.ChangeDutyCycle(1)
    time.sleep(1)
    #servo2.ChangeDutyCycle(0)
    #servo2.stop()
    # Wait for 2 seconds
    #time.sleep(2)

    # Start PWM running on both servos, value of 0 (pulse off)
    servo1.start(0)
    # Turn servo1 to 90
    servo1.ChangeDutyCycle(1)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()
    # Wait for 2 seconds
    time.sleep(2)

    #Clean things up at the end
    servo1.stop()
    servo2.stop()

    print ("The Object has been Picked")
    print ("Thankyou Staff for your time!!")
    return

#Function for Autonomous Detection and Picking of Object
def automatic():

    #Webserver Reception for Voice Command via APP
    class RequestHandler_httpd(BaseHTTPRequestHandler):
      def do_GET(self):
        global Request
        messagetosend = bytes('Project Rover',"utf")
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', len(messagetosend))
        self.end_headers()
        self.wfile.write(messagetosend)
        Request = self.requestline
        Request = Request[5 : int(len(Request)-9)]
        print(Request)
		
		
        #Checking if the Obtined Voice Command is Valid
        if Request == 'black%20king' or Request == 'white%20king':
           print("Command Accepted")
           time.sleep(1)
           print("Searching for the Requested Object.....")
           #time.sleep(0.5)
           ultra()
           print("distance:",distance)
           #Moving the Rover to distance where Object Recognition can be Done
           if distance>130:
              while distance>130:
                 forward()
                 ultra()
           else:
              print ("distance:",distance,"cm")

           stop()
           #Aligned Distance for Recognition
           print ("Object is at a distance of :",distance,"cm")
			
	   #Code for Object Recognition and Rover Alignment
           print ("Initiating Object Recognition")
           time.sleep(0.5)
	   #Object Detection
           objdetect()
           time.sleep(0.5)
           print ("Object has been Recognised and Rover has been Aligned towards the Object")
		   
	   #Moving towards the Object
           if distance>27:
              while distance>27:
                 forward()
                 #stop()
                 ultra()
           else:
              print ("distance:",distance,"cm")

           stop()
	   #Object Detection
           objdetect()
           time.sleep(0.5)
           print ("Object has been Recognised and Rover has been Aligned towards the Object")
		   
	   #Moving towards the Object
           if distance>13:
              while distance>13:
                 forward()
                 #stop()
                 ultra()
           else:
              print ("distance:",distance,"cm")

           stop()	  
           #Printing the Distance of Object	  
           if Request== 'black%20king':
              print ("The Chess Piece- Black King is at a Distance of: ",distance,"cm")
           else:
              print ("The Chess Piece- White King is at a Distance of: ",distance,"cm")
		   
           #ARM Picking Up Action
           time.sleep(1)
           pickup()

           #Exiting the Checking Loop
           #break



    server_address_httpd = ('192.168.43.219',8081)
    httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
    print('Starting Server')
    httpd.serve_forever()



    return


#Functions
try:

       #Webserver Reception for Manual or Automatic Control
       class RequestHandler_httpd(BaseHTTPRequestHandler):
         def do_GET(self):
           global Request
           messagetosend = bytes('Project Rover',"utf")
           self.send_response(200)
           self.send_header('Content-Type', 'text/plain')
           self.send_header('Content-Length', len(messagetosend))
           self.end_headers()
           self.wfile.write(messagetosend)
           Request = self.requestline
           Request = Request[5 : int(len(Request)-9)]
           print(Request)

           #Manual Control over Rover
           if Request == 'up':
              forward()
           if Request == 'down':
              reverse()
           if Request == 'left':
              left()
           if Request == 'right':
              right()
           if Request == 'stop':
              stop()

           #Pi Camera Image Mode
           if Request == 'cam':
              cami()
              print("Image Captured!!")

           #Pi Camera Video Mode
           if Request == 'vid':
              camv()
              print("Video Captured!!")
              
           #Distance Evaluation
           if Request == 'ultra':
              ultra()
              print ("Object is at a distance of :",distance,"cm")
              messagetosend = bytes(str(distance),"utf")
              self.send_response(200)
              self.send_header('Content-Type', 'text/plain')
              self.send_header('Content-Length', len(messagetosend))
              self.end_headers()
              self.wfile.write(messagetosend)



           #Autonomous Control over Rover
           if Request == 'auto':
              print("Autonomous Control over Rover is Enabled")
              automatic()
           return


       server_address_httpd = ('192.168.43.219',8080)
       httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
       print('Starting Server')
       httpd.serve_forever()


#Function for Exiting Program by cleaning all GPIO's
finally:

       GPIO.cleanup()
