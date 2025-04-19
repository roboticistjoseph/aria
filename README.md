# ARIA: Autonomous Rover with Interactive Assistance
A smart robotic rover built with a Raspberry Pi 4 that combines computer vision, distance sensing, and a 4-DOF robotic arm for autonomous object retrieval. Inspired by Mars rover designs with a practical application - fetching stuff so you don't have to get up!

Key Features
- Dual Control Modes: Manual control via web interface or autonomous operation through voice commands
- Computer Vision: Template matching-based object recognition with 87% accuracy in controlled environments
- Mechanical Design: Rocker-bogie suspension system for traversing varied surfaces
- Autonomous Operation: Complete fetch-and-retrieve missions with obstacle avoidance
- Remote Control: HTTP server interface (ports 8080/8081) for extended range control

Hardware Components
- Raspberry Pi (brain)
- Pi Camera (vision)
- Ultrasonic sensor (obstacle detection)
- DC motors with rocker-bogie suspension
- 4-DOF robotic arm with servo motors
- Power management system

Software Stack
- Python for core functionality
- OpenCV for computer vision
- HTTP servers for remote communication
- MIT App Inventor for mobile interface creation

Documentation
For complete documentation of the project, check out my project website: [ARIA Project Info](https://josephkatakam.vercel.app/projects/robotics_aria)

Future Development
Working on improved position tracking, soft gripper mechanics, and MobileNet-V2 integration for better object recognition.
