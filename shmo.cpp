#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>
#include <unistd.h>
#include <math.h>
#include "shmo.hpp"

// OpenCV and camera interfacing libraries
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Namespaces
using namespace std;
using namespace cv;
using namespace rapidjson;


int main(int argc,char **argv) {
	Mat src;
	int n_frames = atoi(argv[1]);		// specify number of frames to process
	double time_new, time_old;
	
	// Output Modes
	int output_mode = atoi(argv[2]);	// specify 1 to save tracking images
	state_out_mode(output_mode);
	// Set up log file (data.log)
	if (output_mode == 1 || output_mode == 2 || output_mode == 5) {
		FILE * log_file;
		log_file = fopen("data.log","w");	// clear output data log file
		fprintf(log_file,"Data formatted as: Time | x  y  v_x  v_y  theta | ...\n\n");
		fclose(log_file);
	}
	
	// Camera setup
	raspicam::RaspiCam_Cv Camera;
	cam_setup(argc, argv, Camera);
	if (!Camera.open()) {
		// open camera
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
	sleep(2);	// sleep required to wait for camera to "warm up"
	// Grab a test image to allocate hsv Mat
	Camera.grab();
	Camera.retrieve(src);										// source image
	Mat src_hsv = Mat::zeros(src.rows, src.cols, CV_8UC3);		// HSV version (only one copy, overwritten for each car)
	
	
	
	
	
	
	Car car_1;
	car_1.name		= "red";
	car_1.mac_add	= "ABCDEFGHIJK1";
	car_1.hue 		= 123;
	car_1.delta 	= 4;
	car_1.size_min	= 300;
	car_1.size_max	= 650;
	car_1.area_old  = 0.0;
	
	Car car_2;
	car_2.name		= "orange";
	car_2.mac_add	= "ABCDEFGHIJK2";
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
	
	
	
	/// Run tracking
	double time_start = cv::getTickCount();
	for (int ii = 0; ii < n_frames; ii++) {
		// Get image	
		Camera.grab();
		Camera.retrieve(src);
		time_new = cv::getTickCount();
		
		// Convert to HSV
		cvtColor(src, src_hsv, COLOR_BGR2HSV);
		
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
		
		// Save desired logs - images, data log files
		do_outputs(src, masks_all, cars_all, ii, output_mode, time_new, time_start);	
		
		// Update JSON object with new data
		do_JSON(cars_all);
		
		// Update "old" data values
		time_old = time_new;
		for (int jj = 0; jj < cars_all.size(); jj++) {
			new_2_old(cars_all[jj]);
		}
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	cout << endl;
	cout << "Total time: " << time_total<<" seconds"<<endl;
	cout << "Total frames: " << n_frames<<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" << std::endl;
	
	Camera.release();
	
	return 0;
}
