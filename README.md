# single-hue-multi-object
Object tracking algorithm for multiple single-hue objects

**Project**: Autonomous Vehicle Model

**Supervisor**: Dr Tim French

**Organisation**: The Univeristy of Western Australia

**Students**: Aaron Hurst, Ray Barker & Ridge Shrubsall

This software is designed to track one red and one orange ZenWheels car within the testbed for the above project.
This is achieved by locating matching hues in images of the area and analysing masks obtained from this.

**Dependencies**:
* OpenCV - tested using version 2.4.13.2 (http://opencv.org/releases.html)
* RaspiCam - tested using version 0.1.6 (http://www.uco.es/investiga/grupos/ava/node/40), used for interfacing the Raspberry Pi camera
* mmal (installed with Raspian by default)

**Compile** using the provided makefile. Note the linked directory - you might need to change this if working on a different device. In future (after I learn how to use it) the build process will be moved to CMake. This will hopefully check for the presence and version of the above dependencies.

**Run** by specifying two command line arguments, the number of frames to run for and the desired output mode:

./shmo [frames] [output_mode]

Available output modes are:
* 0: none
* 1: console
* 2: csv log file
* 3: console & csv
* 4: debug - console, csv and relevant images

A JSON string/object with relevant car data is always sent to the master controller via a socket, except in debug mode.
In this case the JSON string is only printed to the command line.

The configuration file (config.txt) must also be in the same directory as shmo.cpp. This contains global and car-related parameters.