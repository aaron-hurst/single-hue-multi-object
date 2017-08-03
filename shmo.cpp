// General includes
#include <iostream>		// cout
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>		// sleep

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

// Namespaces
using namespace std;
using namespace cv;


int main(int argc,char **argv)
{
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
	
	
	
	
	
	Car car_1;
	car_1.name		= "red";
	car_1.mac_add	= "00:06:66:61:A3:48";
	car_1.hue 		= 123;
	car_1.delta 	= 4;
	car_1.size_min	= 300;
	car_1.size_max	= 650;
	car_1.area_old  = 0.0;
	
	Car car_2;
	car_2.name		= "orange";
	car_2.mac_add	= "00:06:66:61:A9:59";
	car_2.hue 		= 115;
	car_2.delta 	= 4;
	car_2.size_min	= 300;
	car_2.size_max	= 650;
	car_2.area_old  = 0.0;
	
	vector<Car> cars_all;
	cars_all.push_back(car_1);
	cars_all.push_back(car_2);
	
	int crop = 15;				// number of pixels to crop off each side (remove physical model border from analysis)
	int origin[2] = {0, 0};		// pixel location coordinate system origin
	float alpha = 1.9355;		// conversion factor between pixels and mm (i.e. length of each pixel in mm) averaged over whole frame
	
	
	
	// different mask for each car so that they are not overwritten and can be saved at end if desired
	Mat mask_1 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	Mat mask_2 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	vector<Mat> masks_all;
	masks_all.push_back(mask_1);
	masks_all.push_back(mask_2);
	
	
	
	
	
	// Output mode
	int output_mode = atoi(argv[2]);	// specify 1 to save tracking images
	output_mode = state_output_mode(output_mode);
	if (output_mode > 1) {
		// Set up csv log file (data.csv)
		FILE * log_csv;
		log_csv = fopen("log.csv","w");	// clear output data log file
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
	if (sock == -1)
	{
		cout<< "Could not create socket" <<endl;
	}
	cout<< "Socket created" <<endl;
	
	// Set up server connection if not in debug mode
	if (output_mode != 4) {
		struct sockaddr_in server;
		server.sin_addr.s_addr = inet_addr("127.0.0.1");
		server.sin_family = AF_INET;
		server.sin_port = htons(1520);
	
		// Connect to remote server
		if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
		{
			cout<< "Connect failed. Error" << endl;
			return 1;
		}
		cout<< "Connected" << endl;
	}
	
	
	// Run tracking
	int n_frames = atoi(argv[1]);	// number of frames to process
	double time_new, time_old;
	double time_start = cv::getTickCount();
	
	for (int ii = 0; ii < n_frames; ii++) {
		// Get HSV image	
		Camera.grab();
		time_new = cv::getTickCount();			// time image collected
		Camera.retrieve(src);
		cvtColor(src, src_hsv, COLOR_BGR2HSV);	// convert to HSV
		
		for (int jj = 0; jj < cars_all.size(); jj++) {
			// Generate masks of matching hues. (Note: cars are red, but by doing a BGR2HSV conversion (rather than RGB2HSV) they appear blue.)
			do_mask(src_hsv, masks_all[jj], cars_all[jj].hue, cars_all[jj].delta, crop, cars_all[jj].name);
			
			// Locate cars
			find_car(masks_all[jj], cars_all[jj]);
			
			// Conversion to mm
			cars_all[jj].px_to_mm(alpha, origin);
			
			// Calculate velocity
			do_velocity(cars_all[jj], time_new, time_old);
			
			// Determine orientation
			cars_all[jj].orientation_new = 0;
		}
		
		// Update JSON object with new data
		do_json(cars_all, sock, output_mode);
		
		// Complete other outputs (console, csv and/or images)
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
		
		// Provide small delay to ensure comms can keep up
		//usleep(100000);
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	cout << endl;
	cout << "Total time: " << time_total<<" seconds"<<endl;
	cout << "Total frames: " << n_frames<<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" << std::endl;
	
	Camera.release();
	
	return 0;
}
