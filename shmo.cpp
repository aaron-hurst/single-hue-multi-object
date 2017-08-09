// General includes
#include <iostream>		// cout
#include <cstdlib>		// ?
#include <fstream>		// ?
#include <sstream>		// ?
#include <unistd.h>		// sleep
#include <math.h>		// atan2

// Algorithm-specific includes
#include "shmo.hpp"		// specific to this algorithm
#include "common.hpp"	// common definitions

// OpenCV and camera interfacing includes
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"	// camera
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Socket/comms related includes
#include<sys/socket.h>	// socket
#include<arpa/inet.h>	// inet_addr

#define PI 3.14159265

// Namespaces
using namespace std;
using namespace cv;

int main(int argc,char **argv)
{
	int n_frames = atoi(argv[1]);		// number of frames to process
	int output_mode = atoi(argv[2]);
	
	// Camera setup
	raspicam::RaspiCam_Cv Camera;
	cam_setup(Camera);
	if (!Camera.open()) {
		// open camera
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
	sleep(2);	// sleep required to wait for camera to "warm up"
	
	// Grab a test image to allocate hsv Mat
	Mat src;
	Camera.grab();
	Camera.retrieve(src);										// source image
	Mat src_hsv = Mat::zeros(src.rows, src.cols, CV_8UC3);		// HSV version (only one copy, overwritten for each car)
	
	// Configure global parameters and Car structs
	int crop;
	float origin[2];
	float scale;
	int min_speed;
	vector<Car> cars_all;
	do_config(cars_all, crop, origin, scale, min_speed);	// read config file
	
	// Configure vector for masks
	vector<Mat> masks_all;
	for (int i = 0; i < cars_all.size(); i++) {
		Mat mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
		masks_all.push_back(mask);
	}
	
	// Output mode
	output_mode = state_output_mode(output_mode);
	if (output_mode > 1) {
		// Set up csv log file (data.csv)
		FILE * log_csv;
		log_csv = fopen("log.csv","w");	// clear log file
		fprintf(log_csv,"time(s)");
		for (int i = 0; i < cars_all.size(); i++) {
			// Print one lot of headers for each car
			fprintf(log_csv,",area,x,y,v_x,v_y,theta");
		}
		fprintf(log_csv,"\n");
		fclose(log_csv);
	}
	
	// Create socket
	int sock;
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1) {
		cout<< "Could not create socket" <<endl;
		return 1;
	}
	cout<< "Socket created" <<endl;
	
	// Set up server connection if not in debug mode
	if (output_mode != 4) {
		struct sockaddr_in server;
		server.sin_addr.s_addr = inet_addr("127.0.0.1");
		server.sin_family = AF_INET;
		server.sin_port = htons(1520);
	
		// Connect to remote server
		if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0) {
			cout<< "Connect failed. Error" << endl;
			return 1;
		}
		cout<< "Connected" << endl;
	}
	
	// Run tracking
	double time_new, time_old;
	double time_start = cv::getTickCount();
	for (int ii = 0; ii < n_frames; ii++) {
		// Get HSV image	
		Camera.grab();
		time_new = cv::getTickCount();			// time image collected
		Camera.retrieve(src);
		cvtColor(src, src_hsv, COLOR_BGR2HSV);	// convert to HSV
		
		for (int jj = 0; jj < cars_all.size(); jj++) {
			// Generate masks of matching hues
			do_mask(src_hsv, masks_all[jj], cars_all[jj].hue, cars_all[jj].delta, crop, cars_all[jj].name);
			
			// Detect cars
			find_car(masks_all[jj], cars_all[jj]);
			
			// Convert measurements to mm
			cars_all[jj].px_to_mm(scale, origin);
			
			// Calculate velocity
			do_velocity(cars_all[jj], time_new, time_old);
			
			// Determine orientation
			//cars_all[jj].orientation_new = 0;
			
			cout<< "speed: " << cars_all[jj].speed() <<endl;
			cout<< "min speed: " << min_speed <<endl;
			
			if (cars_all[jj].speed() > min_speed) {
				cars_all[jj].orientation_new = (int)(90 - 180/PI*atan2(cars_all[jj].velocity_new[1], cars_all[jj].velocity_new[0]));
				if (cars_all[jj].orientation_new < 0) {
					cars_all[jj].orientation_new = 360 + cars_all[jj].orientation_new;
				}
			} else {
				cars_all[jj].orientation_new = 0;
			}
		}
		
		// Update JSON output with new data
		do_json(cars_all, sock, output_mode);
		
		// Other outputs (console, csv and/or images)
		if (output_mode == 4) {
			do_debug(cars_all, src, masks_all, ii, output_mode, time_new, time_start);
		} else {
			do_outputs(cars_all, ii, output_mode, time_new, time_start);
		}
		
		// Update "old" data values
		time_old = time_new;
		for (int jj = 0; jj < cars_all.size(); jj++) {
			cars_all[jj].new_to_old();
		}
		
		// Small delay to ensure comms can keep up
		//usleep(100000);
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	cout << endl;
	cout << "Total time: " << time_total <<" seconds"<<endl;
	cout << "Total frames: " << n_frames <<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" <<endl;
	
	Camera.release();
	
	return 0;
}
