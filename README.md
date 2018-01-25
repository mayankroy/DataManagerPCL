# DataManagerPCL
Sample Work in Bin Picking at IIT Delhi for BARC


1.1 Setup
• Robot manipulator – KUKA KR5
• Basler Cameras.
• Micro Epsilon ScanCONTROL.(Laser Scanner)
• Suction Gripper.
• High-end computational platform – Intel Xenon 3.7 GHz. for controlling cameras and laser range finders, and interfacing to the actual robotic arm for the tasks.
1.2 Problem Statement
This project address picking and manipulating pellets in a cluttered environment using a robotic manipulator with the help of cameras. Vision is an important requirement in such a task: this is the most important sensing mechanism for us humans as well. The inputs from cameras guide the manipulator and the gripper to perform the task at hand. The motivation is to have an automatic system to perform the task. This is not an easy job, since it involves the synergistic combination of the two important but practical fields of computer vision and robotics. Computer vision is used to first identify the object of interest, and then estimate its position and orientation. After picking it, the pellet will be placed into a hollow cylindrical tube whose exact location may have not be known prior. In this case too, identification of the target location, its orientation, and then finally, using visual guidance to enable the manipulator (which has picked up the pellet with the gripper) to place the pellet into the hollow cylindrical tube. This task requires great precision, and accuracy. Vision is a natural aid in such a synergistic task. In this work, we present a solution to automate the whole process of bin picking. Following problems are addressed in this work:
 Segmentation and estimation of orientation of isolated objects placed in front of a robot using both geometric information from 3D map and intensity data form image of cluttered (multi layered) objects.
 Pickup and placement of segmented object in desired orientation while assuring collision free operation.
6
1.3 Methodology
These are the various scientific and technical challenges that were resolved in the course of this work –
Step 1 : TCP IP protocol for intercommunication b/w KUKA, Camera, Laser Scanner.
Step 2 : Multi-Threaded process initiation and handling.
Implemented for communication in background and parallel processing.
Step 3 : KUKA Position detection and command(KRL) communication from PC.
Step 4 : Camera Calibration.
Step 5 : Laser - Scanning speed
Step 6 : Camera, Laser and KUKA to World Coordinates – frame calibration.
Step 7 : Sensor Fusion
Step 8 : 3D segmentation - Image segmentation and cylinder detection.
Step 9 : Collision avoidance
Step 10 : Disturbance Monitoring
Step 11 : Pick up
Step 12 : Complete Clearing of the Bin
For a brief overview user can should go through the presentation referred (APPENDIX A.3). For details on the scientific accomplishments and results, the research submission should be consulted (APPENDIX A.1). For visual presentation please see APPENDIX A. 3.
The following section discuss the code structure and documentation of the various functions.
