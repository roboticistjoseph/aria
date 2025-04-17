'''
   This code is for a Raspberry Pi-based rover that can be controlled via a web server.
   It includes functions for movement, object detection, and robotic arm control.
   The rover can autonomously explore an area, detect objects, and pick them up using a robotic arm.
   The code uses libraries like RPi.GPIO for GPIO control, OpenCV for image processing, and picamera for camera operations.
   The rover can also communicate with a web server to receive commands and send responses.
'''
# Importing Libraries

# Libraries for Time and GPIO's of Raspberry Pi
import time
from time import sleep
import RPi.GPIO as GPIO

# Libraries for Web Server
import http.server
from http.server import BaseHTTPRequestHandler, HTTPServer

# Libraries for Object Detection
# Picamera for capturing images and videos
from picamera.array import PiRGBArray
from picamera import PiCamera

# OpenCV for image processing and template matching
import cv2
import numpy as np

# Additional Libraries
# Regular expressions for parsing commands
import re

# Random for generating random directions during exploration
import random

# Math for position and heading calculations
import math

# Deque for tracking movement history
from collections import deque

### BCM and GPIO Configuration

# Assigning GPIO pins for Rover motor control
m11 = 29  # Motor 1 forward
m12 = 31  # Motor 1 backward
m21 = 33  # Motor 2 forward
m22 = 35  # Motor 2 backward

# Assigning GPIO pins for Robotic Arm control
s_base = 7        # Servo for base rotation
s_shoulder = 12   # Servo for shoulder movement
s_elbow = 13      # Servo for elbow movement
s_gripper = 15    # Servo for gripper control

# Assigning GPIO pins for Ultrasonic Sensor
TRIG = 16  # Trigger pin for ultrasonic sensor
ECHO = 18  # Echo pin for ultrasonic sensor

# Global variable for alignment frame count
global FPS
FPS = 1  # Frame count for alignment during object detection

# Camera variables for image and video indexing
global index1  # Index for captured images
global index2  # Index for recorded videos

index1 = 0  # Initialize image index
index2 = 0  # Initialize video index

### BCM and GPIO Configuration
# Set the GPIO mode to BOARD (physical pin numbering)
GPIO.setmode(GPIO.BOARD)

# Disable GPIO warnings
GPIO.setwarnings(False)

# Rover Motor Setup
# Configure GPIO pins for motor control as output
GPIO.setup(m11, GPIO.OUT)  # Motor 1 forward
GPIO.setup(m12, GPIO.OUT)  # Motor 1 backward
GPIO.setup(m21, GPIO.OUT)  # Motor 2 forward
GPIO.setup(m22, GPIO.OUT)  # Motor 2 backward

# Robotic Arm Setup
# Configure GPIO pins for robotic arm servos as output
# Comment/Uncomment if using additional servos
GPIO.setup(s_base, GPIO.OUT)  # Servo for base rotation
GPIO.setup(s_shoulder, GPIO.OUT)  # Servo for shoulder movement
GPIO.setup(s_elbow, GPIO.OUT)  # Servo for elbow movement
GPIO.setup(s_gripper, GPIO.OUT)  # Servo for gripper control

# Ultrasonic Sensor Setup
# Configure GPIO pins for ultrasonic sensor
GPIO.setup(TRIG, GPIO.OUT)  # Trigger pin as output
GPIO.setup(ECHO, GPIO.IN)   # Echo pin as input

# Initialize Rover Motors to OFF state
GPIO.output(m11, False)
GPIO.output(m12, False)
GPIO.output(m21, False)
GPIO.output(m22, False)

# Initialize Robotic Arm Servos to OFF state
# Uncomment if using additional servos
# GPIO.output(s_base, False)
GPIO.output(s_shoulder, False)
# GPIO.output(s_elbow, False)
GPIO.output(s_gripper, False)

# Initialize Ultrasonic Sensor Trigger pin to OFF state
GPIO.output(TRIG, False)

# WebServer Variable
Request = None

# Time to Settle and Rest after Initialization
time.sleep(2)

#### Functions for Rover Movement ####

# Function for Forward Movement of Rover
# Activates the motors to move the rover forward
def forward():
   print("FORWARD")
   GPIO.output(m11, True)  # Motor 1 forward
   GPIO.output(m12, False) # Motor 1 backward (off)
   GPIO.output(m21, False) # Motor 2 backward (off)
   GPIO.output(m22, True)  # Motor 2 forward

# Function for Backward Movement of Rover
# Activates the motors to move the rover backward
def reverse():
   print("BACKWARD")
   GPIO.output(m11, False) # Motor 1 forward (off)
   GPIO.output(m12, True)  # Motor 1 backward
   GPIO.output(m21, True)  # Motor 2 backward
   GPIO.output(m22, False) # Motor 2 forward (off)

# Function for Right (Clockwise) Movement of Rover
# Activates the motors to rotate the rover to the right
def right():
   print("RIGHT")
   GPIO.output(m11, True)  # Motor 1 forward
   GPIO.output(m12, False) # Motor 1 backward (off)
   GPIO.output(m21, True)  # Motor 2 backward
   GPIO.output(m22, False) # Motor 2 forward (off)

# Function for Left (Anti-Clockwise) Movement of Rover
# Activates the motors to rotate the rover to the left
def left():
   print("LEFT")
   GPIO.output(m11, False) # Motor 1 forward (off)
   GPIO.output(m12, True)  # Motor 1 backward
   GPIO.output(m21, False) # Motor 2 backward (off)
   GPIO.output(m22, True)  # Motor 2 forward

# Function to Stop the Movement of Rover
# Deactivates all motors to stop the rover
def stop():
   print("STOP")
   GPIO.output(m11, False) # Motor 1 forward (off)
   GPIO.output(m12, False) # Motor 1 backward (off)
   GPIO.output(m21, False) # Motor 2 backward (off)
   GPIO.output(m22, False) # Motor 2 forward (off)


# Function for capturing an image using the Pi Camera
def cami():
   # Camera Variables
   global index1  # Global variable to keep track of image index
   index1 += 1  # Increment the image index for unique filenames
   
   # Initialize the Pi Camera
   with picamera.PiCamera() as camera:
      # Set the resolution for the captured image
      camera.resolution = (1280, 720)
      # Capture the image and save it to the specified path with the current index
      camera.capture('/home/pi/Desktop/image%s.jpg' % index1)
      # Close the camera after capturing the image
      camera.close()

# Function for recording a video using the Pi Camera
def camv():
   # Camera Variables
   global index2  # Global variable to keep track of video index
   index2 += 1  # Increment the video index for unique filenames

   # Initialize the Pi Camera
   with picamera.PiCamera() as camera:
      # Start recording a video and save it to the specified path with the current index
      camera.start_recording('/home/pi/Desktop/video%s.h264' % index2)
      # Record the video for 20 seconds
      sleep(20)
      # Stop recording the video
      camera.stop_recording()
      # Close the camera after recording the video
      camera.close()


# Function to measure distance using an ultrasonic sensor
def ultra():
   # Declare the global variable 'distance' to store the measured distance
   global distance

   # Send a 10-microsecond pulse to the TRIG pin to trigger the ultrasonic sensor
   GPIO.output(TRIG, True)
   time.sleep(0.00001)  # Wait for 10 microseconds
   GPIO.output(TRIG, False)  # Turn off the TRIG pin

   # Wait for the ECHO pin to go HIGH (start of the echo pulse)
   while GPIO.input(ECHO) == 0:
      pulse_start = time.time()  # Record the start time of the pulse

   # Wait for the ECHO pin to go LOW (end of the echo pulse)
   while GPIO.input(ECHO) == 1:
      pulse_end = time.time()  # Record the end time of the pulse

   # Calculate the duration of the pulse
   pulse_duration = pulse_end - pulse_start

   # Calculate the distance in centimeters
   # Speed of sound is approximately 34300 cm/s, so divide by 2 for round trip
   distance = pulse_duration * 17150
   distance = round(distance, 2)  # Round the distance to 2 decimal places

   # Return the measured distance
   return

# A deque to track the movement history of the rover for the return journey
movement_history = deque()

# Dictionary to store the current position of the rover
# 'x' and 'y' represent the coordinates, and 'heading' represents the direction in degrees
current_position = {"x": 0, "y": 0, "heading": 0}

# Dictionary to store the starting position of the rover
# This is used as a reference point for returning home
start_position = {"x": 0, "y": 0, "heading": 0}

# A flag to indicate whether the rover is in return mode
# This helps in distinguishing between exploration and returning home
returning_home = False

# Define a dictionary of recognizable objects and their corresponding template image filenames
# This is used for object detection via template matching
RECOGNIZABLE_OBJECTS = {
   "water bottle": "water_bottle_template.jpg",  # Template for detecting a water bottle
   "coffee mug": "coffee_mug_template.jpg",      # Template for detecting a coffee mug
   "smartphone": "smartphone_template.jpg",      # Template for detecting a smartphone
   "remote": "remote_template.jpg",              # Template for detecting a remote control
   "black king": "template.jpg",                 # Template for detecting a black king chess piece
   "white king": "template2.jpg",                # Template for detecting a white king chess piece
   "notebook": "notebook_template.jpg",          # Template for detecting a notebook
   "pen": "pen_template.jpg",                    # Template for detecting a pen
   "headphones": "headphones_template.jpg",      # Template for detecting headphones
   "glasses": "glasses_template.jpg"             # Template for detecting glasses
}

# Function to extract object name from a natural language command
def extract_object_from_command(command):
   # Replace URL-encoded spaces ("%20") with actual spaces and convert the command to lowercase
   command = command.replace("%20", " ").lower()
   
   # Iterate through the list of recognizable object names
   for object_name in RECOGNIZABLE_OBJECTS.keys():
      # Check if the object name is mentioned in the command
      if object_name in command:
         # Print the recognized object for debugging purposes
         print(f"Recognized object: {object_name}")
         # Return the recognized object name
         return object_name
         
   # If no recognizable object is found, return None
   return None

# Update movement functions to track position

# Function to move the rover forward
def forward(duration=None, record=True):
   print("FORWARD")
   # Activate motors for forward movement
   GPIO.output(m11, True)
   GPIO.output(m12, False)
   GPIO.output(m21, False)
   GPIO.output(m22, True)
   
   # If a duration is specified, move for that duration and stop
   if duration:
      time.sleep(duration)
      stop()
      if record:
         # Record the movement for the return journey
         movement_history.append(("forward", duration))
         # Update the rover's position estimate based on heading and duration
         current_position["x"] += math.cos(math.radians(current_position["heading"])) * duration
         current_position["y"] += math.sin(math.radians(current_position["heading"])) * duration
         print(f"Position estimate: ({current_position['x']:.1f}, {current_position['y']:.1f})")

# Function to move the rover backward
def reverse(duration=None, record=True):
   print("BACKWARD")
   # Activate motors for backward movement
   GPIO.output(m11, False)
   GPIO.output(m12, True)
   GPIO.output(m21, True)
   GPIO.output(m22, False)
   
   # If a duration is specified, move for that duration and stop
   if duration:
      time.sleep(duration)
      stop()
      if record:
         # Record the movement for the return journey
         movement_history.append(("reverse", duration))
         # Update the rover's position estimate based on heading and duration
         current_position["x"] -= math.cos(math.radians(current_position["heading"])) * duration
         current_position["y"] -= math.sin(math.radians(current_position["heading"])) * duration
         print(f"Position estimate: ({current_position['x']:.1f}, {current_position['y']:.1f})")

# Function to rotate the rover to the right (clockwise)
def right(duration=None, record=True):
   print("RIGHT")
   # Activate motors for right rotation
   GPIO.output(m11, True)
   GPIO.output(m12, False)
   GPIO.output(m21, True)
   GPIO.output(m22, False)
   
   # If a duration is specified, rotate for that duration and stop
   if duration:
      time.sleep(duration)
      stop()
      if record:
         # Record the movement for the return journey
         movement_history.append(("right", duration))
         # Update the rover's heading estimate (approximate 60 degrees per second rotation)
         current_position["heading"] = (current_position["heading"] - duration * 60) % 360
         print(f"Heading estimate: {current_position['heading']:.1f}°")

# Function to rotate the rover to the left (anti-clockwise)
def left(duration=None, record=True):
   print("LEFT")
   # Activate motors for left rotation
   GPIO.output(m11, False)
   GPIO.output(m12, True)
   GPIO.output(m21, False)
   GPIO.output(m22, True)
   
   # If a duration is specified, rotate for that duration and stop
   if duration:
      time.sleep(duration)
      stop()
      if record:
         # Record the movement for the return journey
         movement_history.append(("left", duration))
         # Update the rover's heading estimate (approximate 60 degrees per second rotation)
         current_position["heading"] = (current_position["heading"] + duration * 60) % 360
         print(f"Heading estimate: {current_position['heading']:.1f}°")

# Function for quick object detection during exploration
def quick_scan_for_object(target_object):
    """Quick scan to check if object is in view - returns True if found"""
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.1)  # Let camera warm up
    
    # Get the correct template
    if target_object in RECOGNIZABLE_OBJECTS:
        template_file = RECOGNIZABLE_OBJECTS[target_object]
        template = cv2.imread(template_file, 0)
    elif target_object == 'black%20king' or target_object == 'black king':
        template = cv2.imread('template.jpg', 0)
    else:
        template = cv2.imread('template2.jpg', 0)
    
    # Capture a single frame
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        w, h = template.shape[::-1]
        
        # Template matching
        res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
        (_, maxVal, _, _) = cv2.minMaxLoc(res)
        
        # If we found a match with high confidence
        if maxVal > 0.6:  # Threshold for detection
            camera.close()
            return True
        
        # Only process one frame
        break
        
    camera.close()
    return False

# Function to explore area until target object is found
def explore_and_find(target_object):
   """Explore the area while avoiding obstacles until the target object is found"""
   global movement_history, current_position
   
   # Reset position tracking at the start of exploration
   movement_history.clear()
   current_position = {"x": 0, "y": 0, "heading": 0}
   
   print(f"Starting exploration mode to find {target_object}")
   object_found = False  # Flag to indicate if the target object has been found
   
   # Exploration parameters
   turn_duration = 0.5  # Duration for turning (in seconds)
   forward_duration = 1.0  # Duration for moving forward (in seconds)
   scan_interval = 3  # Number of moves before scanning for the object
   move_count = 0  # Counter to track the number of moves
   random_direction = 1  # Direction for turning: 1 for right, -1 for left
   
   # Continue exploring until the target object is found
   while not object_found:
      # Check for obstacles using the ultrasonic sensor
      ultra()
      
      # If an obstacle is detected, turn to avoid it
      if distance < 30:
         print(f"Obstacle detected at {distance}cm")
         # Choose a random direction to turn (right or left)
         random_direction = random.choice([-1, 1])
         if random_direction > 0:
            print("Turning right to avoid obstacle")
            right(turn_duration * 1.5)  # Turn right and record the movement
         else:
            print("Turning left to avoid obstacle")
            left(turn_duration * 1.5)  # Turn left and record the movement
      else:
         # If no obstacle, move forward to explore
         print("Moving forward to explore")
         forward(forward_duration)  # Move forward and record the movement
         
         # Increment the move counter
         move_count += 1
         
         # After a certain number of moves, change direction to expand the search area
         if move_count % 4 == 0:
            print("Changing direction to expand search area")
            if random_direction > 0:
               right(turn_duration)  # Turn right and record the movement
            else:
               left(turn_duration)  # Turn left and record the movement
            random_direction *= -1  # Alternate the direction for the next change
      
      # Periodically scan for the target object
      if move_count % scan_interval == 0:
         print("Scanning for object...")
         # Perform a 360-degree scan by rotating in increments
         for i in range(4):  # 4 quarter turns to complete a full rotation
            right(turn_duration)  # Turn right and record the movement
            
            # Check if the target object is visible
            if quick_scan_for_object(target_object):
               print(f"Object found! {target_object} is in view!")
               object_found = True  # Set the flag to indicate the object is found
               break
   
   print("Exploration complete - target found!")
   return True  # Return success when the target object is found



# Pickup function to to grab object and call return_to_user
def pickup():
   # Set pins 12 & 15 as outputs, and define as PWM for servo1 & servo2
   GPIO.setup(12, GPIO.OUT)
   servo1 = GPIO.PWM(12, 50)  # pin 12 for servo1
   GPIO.setup(15, GPIO.OUT)
   servo2 = GPIO.PWM(15, 50)  # pin 15 for servo2

   # Start PWM for servo2 and move it to position (e.g., 90 degrees)
   servo2.start(0)
   servo2.ChangeDutyCycle(7)  # Move servo2 to position
   time.sleep(0.5)  # Allow time for movement
   servo2.ChangeDutyCycle(0)  # Stop PWM signal
   servo2.stop()  # Stop servo2
   time.sleep(2)  # Wait for 2 seconds

   # Start PWM for servo1 and move it to position (e.g., 90 degrees)
   servo1.start(0)
   servo1.ChangeDutyCycle(5.5)  # Move servo1 to position
   time.sleep(0.5)  # Allow time for movement
   servo1.ChangeDutyCycle(0)  # Stop PWM signal
   servo1.stop()  # Stop servo1
   time.sleep(2)  # Wait for 2 seconds

   # Start PWM for servo2 and move it to another position (e.g., close gripper)
   servo2.start(0)
   servo2.ChangeDutyCycle(1)  # Move servo2 to position
   time.sleep(1)  # Allow time for movement

   # Start PWM for servo1 and move it to another position (e.g., lift arm)
   servo1.start(0)
   servo1.ChangeDutyCycle(1)  # Move servo1 to position
   time.sleep(0.5)  # Allow time for movement
   servo1.ChangeDutyCycle(0)  # Stop PWM signal
   servo1.stop()  # Stop servo1
   time.sleep(2)  # Wait for 2 seconds

   # Clean up PWM signals for both servos
   servo1.stop()
   servo2.stop()

   # Print confirmation that the object has been picked up
   print("The Object has been Picked")

   # Call the return_to_user function to navigate back to the user
   return_to_user()

   # Print confirmation that the mission is complete
   print("Mission complete! Item delivered to user!")
   return

# Function to return to user after pickup
def return_to_user():
   global movement_history
   
   print("Now returning to user with the object...")
   
   # First approach: Calculate direct path back to starting point
   # Calculate the angle to the starting point (home) based on current position
   angle_to_home = math.degrees(math.atan2(-current_position["y"], -current_position["x"]))
   # Calculate the angle the rover needs to turn to face home
   turn_angle = (angle_to_home - current_position["heading"]) % 360
   
   # Turn the rover towards the home position
   if turn_angle < 180:
      # Turn right if the angle is less than 180 degrees
      right(turn_angle / 60)  # Assuming the rover rotates at 60° per second
   else:
      # Turn left if the angle is greater than 180 degrees
      left((360 - turn_angle) / 60)
   
   # Calculate the straight-line distance to the home position
   distance_to_home = math.sqrt(current_position["x"]**2 + current_position["y"]**2)
   
   # Return home in segments while avoiding obstacles
   remaining_distance = distance_to_home
   step_size = 1.0  # Move in 1-second increments
   
   while remaining_distance > 0:
      # Check for obstacles using the ultrasonic sensor
      ultra()
      if distance < 30:
         # If an obstacle is detected, turn right to avoid it
         print(f"Obstacle detected at {distance}cm while returning")
         right(0.5)
         continue
      
      # Move forward for a step towards home
      forward_duration = min(step_size, remaining_distance)
      forward(forward_duration, record=False)  # Move without recording the movement
      remaining_distance -= forward_duration
      
      # Check if the rover is near the user
      ultra()
      if distance < 100 and distance > 20:  # Detect potential human-sized object
         print(f"Potential user detected at {distance}cm")
         break
   
   # Second approach: Look for the user by scanning the surroundings
   found_user = False
   print("Scanning for user...")
   
   # Perform a full 360° scan in 45° increments
   for _ in range(8):  # 8 steps of 45° each
      ultra()
      if distance < 150 and distance > 30:  # Detect human-sized object
         found_user = True
         print(f"User detected at {distance}cm")
         break
      right(0.5, record=False)  # Rotate right without recording the movement
   
   if found_user:
      # Approach the user carefully
      while distance > 60:  # Stop at a comfortable distance from the user
         forward(0.5, record=False)
         ultra()
   
   # Final announcement to indicate the task is complete
   print("I've returned with your requested item!")
   stop()

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

def automatic():
   # Define a class to handle HTTP requests for the autonomous mode
   class RequestHandler_httpd(BaseHTTPRequestHandler):
      def do_GET(self):
         global Request
         # Send a response to the client indicating the server is active
         messagetosend = bytes('Project Rover', "utf")
         self.send_response(200)
         self.send_header('Content-Type', 'text/plain')
         self.send_header('Content-Length', len(messagetosend))
         self.end_headers()
         self.wfile.write(messagetosend)

         # Extract the command from the HTTP request
         Request = self.requestline
         Request = Request[5: int(len(Request) - 9)]
         print(f"Received command: {Request}")

         # Extract the target object from the command
         target_object = extract_object_from_command(Request)
         # Check if the command is valid (either a recognized object or specific keywords)
         valid_command = target_object is not None or Request == 'black%20king' or Request == 'white%20king'

         # If the command is valid, proceed with the autonomous operation
         if valid_command:
            print("Command Accepted")
            time.sleep(1)
            print("Searching for the Requested Object.....")

            # Start exploring the environment to locate the target object
            if target_object:
               explore_and_find(target_object)  # Use the extracted object name
            else:
               explore_and_find(Request)  # Use the original request format if no object extracted

            # Measure the distance to the object using the ultrasonic sensor
            ultra()
            print("distance:", distance)

            # Move the rover closer to the object if it is far away
            if distance > 130:
               while distance > 130:
                  forward()
                  ultra()
            else:
               print("distance:", distance, "cm")

            # Stop the rover once it is within range
            stop()
            print("Object is at a distance of:", distance, "cm")

            # Perform object recognition and align the rover towards the object
            print("Initiating Object Recognition")
            time.sleep(0.5)
            objdetect()  # Detect the object
            time.sleep(0.5)
            print("Object has been Recognised and Rover has been Aligned towards the Object")

            # Move closer to the object if necessary
            if distance > 27:
               while distance > 27:
                  forward()
                  ultra()
            else:
               print("distance:", distance, "cm")

            # Stop the rover and perform another round of object detection
            stop()
            objdetect()
            time.sleep(0.5)
            print("Object has been Recognised and Rover has been Aligned towards the Object")

            # Move even closer to the object for pickup
            if distance > 13:
               while distance > 13:
                  forward()
                  ultra()
            else:
               print("distance:", distance, "cm")

            # Stop the rover and print the final distance to the object
            stop()
            print(f"The requested object is at a Distance of: {distance} cm")

            # Use the robotic arm to pick up the object
            time.sleep(1)
            pickup()

   # Set up the HTTP server to listen for incoming commands
   # The server will run on the specified IP address and port (192.168.43.219:8081)
   server_address_httpd = ('192.168.43.219', 8081)

   # Create an instance of the HTTP server with the specified address and request handler
   httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)

   # Print a message indicating that the server is starting
   print('Starting Server')

   # Start the server and keep it running to handle incoming requests indefinitely
   httpd.serve_forever()
   
   return

# Main function to start the server and handle requests
try:
   # Webserver Reception for Manual or Automatic Control
   class RequestHandler_httpd(BaseHTTPRequestHandler):
      def do_GET(self):
         global Request
         # Send a response to the client indicating the server is active
         messagetosend = bytes('Project Rover', "utf")
         self.send_response(200)
         self.send_header('Content-Type', 'text/plain')
         self.send_header('Content-Length', len(messagetosend))
         self.end_headers()
         self.wfile.write(messagetosend)

         # Extract the command from the HTTP request
         Request = self.requestline
         Request = Request[5: int(len(Request) - 9)]
         print(Request)

         # Manual Control over Rover
         if Request == 'up':  # Move forward
            forward()
         if Request == 'down':  # Move backward
            reverse()
         if Request == 'left':  # Turn left
            left()
         if Request == 'right':  # Turn right
            right()
         if Request == 'stop':  # Stop movement
            stop()

         # Pi Camera Image Mode
         if Request == 'cam':  # Capture an image
            cami()
            print("Image Captured!!")

         # Pi Camera Video Mode
         if Request == 'vid':  # Record a video
            camv()
            print("Video Captured!!")

         # Distance Evaluation
         if Request == 'ultra':  # Measure distance using ultrasonic sensor
            ultra()
            print("Object is at a distance of:", distance, "cm")
            # Send the measured distance back to the client
            messagetosend = bytes(str(distance), "utf")
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Content-Length', len(messagetosend))
            self.end_headers()
            self.wfile.write(messagetosend)

         # Autonomous Control over Rover
         if Request == 'auto':  # Enable autonomous mode
            print("Autonomous Control over Rover is Enabled")
            automatic()
         return

   # Set up the HTTP server to listen for incoming commands
   server_address_httpd = ('192.168.43.219', 8080)  # IP address and port
   httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
   print('Starting Server')
   # Start the server and keep it running to handle incoming requests indefinitely
   httpd.serve_forever()

# Function for Exiting Program by cleaning all GPIOs
finally:
   # Clean up all GPIO pins to ensure a safe exit
   GPIO.cleanup()
