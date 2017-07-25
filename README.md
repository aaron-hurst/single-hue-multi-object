# single-hue-multi-object
Object tracking algorithm for multiple single-hue objects

**Project**: Autonomous Vehicle Model

**Supervisor**: Dr Tim French

**Organisation**: The Univeristy of Western Australia

**Students**: Aaron Hurst, Ray Barker & Ridge Shrubsall

This software is designed to track one red and one orange ZenWheels car within the testbed for the above project.
This is achieved my locating matching hues in images of the area and analysing masks obtained from this.

**Dependencies**:
* OpenCV - tested using version 2.4.13.2 (http://opencv.org/releases.html)
* RaspiCam - tested using version 0.1.6 (http://www.uco.es/investiga/grupos/ava/node/40), used for interfacing the Raspberry Pi camera
* mmal (installed with Raspian by default)
* RapidJson - tested using version 1.1.0 (http://rapidjson.org/index.html, used for exporting data to a file as a JSON object)

**Compile** using the provided makefile. Note the linked directory - you might need to change this if working on a different device. In future (after I learn how to use it) the build process will be moved to CMake. This will hopefully check for the presence and version of the above dependencies.

**Run** by specifying two command line arguments, the number of frames to run for and the desired output mode:

./shmo frames output_mode

Available output modes are:
* 0: console & JSON
* 1: log file & JSON
* 2: console, JSON and log file
* 3: console, JSON and source + centroids images
* 4: console, JSON, source + centroids images and mask images
* 5: all - console, JSON, log file, source + centroids and masks

JSON object is saved to a file called output.json. Log file is called data.log.