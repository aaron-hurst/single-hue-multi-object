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
	int area_min, area_low, area_good, area_max, car_length, min_sat, min_val;
	vector<Car> cars_all;
	do_config(cars_all, crop, origin, scale, min_speed,
		area_min, area_low, area_good, area_max,
		car_length, min_sat, min_val);	// read config file
	
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
	Mat local_mask;
	int significant_hues[25];
	Point bl, tr;
	float p_position[2];
	double time_new, time_old, time_inc;
	double time_start = cv::getTickCount();
	
	for (int ii = 0; ii < n_frames; ii++) {
		// Get HSV image	
		Camera.grab();
		time_new = cv::getTickCount();			// time image collected
		Camera.retrieve(src);
		cvtColor(src, src_hsv, COLOR_BGR2HSV);	// convert to HSV
		
		for (int jj = 0; jj < cars_all.size(); jj++) {
			// Generate masks of matching hues
			do_mask(src_hsv, masks_all[jj], cars_all[jj].hue, cars_all[jj].delta, crop, cars_all[jj].name, min_sat, min_val);
			
			// Detect cars
			find_car(masks_all[jj], cars_all[jj], area_min, area_max);
			
			
			
			
			
			
			
			// Adaptive hue update
			if (cars_all[jj].area_new < area_low && cars_all[jj].found()) {
				// Car found and is low area threshold => generate mask around known location
				
				// Rectangle around known car location
				bl.x = cars_all[jj].position_new[0] - car_length/2;
				bl.y = cars_all[jj].position_new[1] - car_length/2;
				tr.x = cars_all[jj].position_new[0] + car_length/2;
				tr.y = cars_all[jj].position_new[1] + car_length/2;
				local_mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
				rectangle(local_mask, bl, tr, 255, CV_FILLED);
				
				
				
				// ALL OF THE BELOW CODE needs to be executed in either case of the above if/else if statement.
				// How to do this? Make it into another function and call it from within the if statements?
				// Note that the if statements themselves will be in another function eventually (so I will be calling a function from a function... that's not a problem)
				
				// Global "significant hue" mask
				Mat global_mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
				inRange(src_hsv, Scalar(0, min_sat, min_val), Scalar(180, 255, 255), global_mask);
				
				
				// imshow("local mask", local_mask);
				// imshow("global mask", global_mask);
				// waitKey(0);
				
				// Combine masks
				local_mask = local_mask & global_mask;
				
				// imshow("mask", local_mask);
				// waitKey(0);
				
				// Calculate histogram (use OpenCV function)
				int max_hue = 180;
				int hue_bins = max_hue;
				int hist_size[] = {hue_bins, 1};
				float bin_w = max_hue/hue_bins;
				float hue_range[] = {0, (float)max_hue};
				const float* range[] = {hue_range};
				MatND hist;
				int channel[] = {0};
				calcHist(&src_hsv, 1, channel, local_mask, hist, 1, hist_size, range, true, false);
				
				// Get maximum histogram value
				double max;
				Point max_loc;
				cv::minMaxLoc(hist, NULL, &max, NULL, &max_loc);
				
				int bin_val;
				//printf("\n");
				int idx = 0;
				for (int i = 0; i < hue_bins; i++)
				{
					bin_val = hist.at<float>(i,0);//*(hue_bins*scale)/max;
					if (bin_val > max/2) {
						//printf("Hue: %4.1f	value: %3d\n", i*bin_w, bin_val);
						significant_hues[idx] = i;
						idx++;
					}
				}
				
				
				
				
				
				
				//figure out best hue to use for tracking
				//median of hues that are above 50% of the peak hue? (use median rather than average in case there is an outlier)
				//only consider values within +/-10 of original value AND +/- 10 from 5 from measured peak? (again, dealing with outliers)
				int best_idx = (int)round(idx/2);
				float best_hue = significant_hues[idx];
				
				printf("best_hue: %f\n", best_hue);
				
				//select learning rate
				// 0.2 if found
				// 0.5 if not found? Or just keep it the same?
				// put both in config filebuf
				
				// do learning
				if (best_hue > cars_all[jj].hue - 15 && best_hue < cars_all[jj].hue + 15)//hue histogram indicates a car is actually present
				{
					// car.hue = 0.2*hue_best + 0.8*car.hue
					cars_all[jj].hue = 0.2*best_hue + 0.8*cars_all[jj].hue;
				}
				
				
				
				
			}
			else if (cars_all[jj].lost()) {
				// Car not found => generate mask around predicted location
				
				// Predict location
				time_inc = double (time_new - time_old) / double (cv::getTickFrequency());
				p_position[0] = cars_all[jj].position_old[0] + cars_all[jj].velocity_old[0]*time_inc;
				p_position[1] = cars_all[jj].position_old[1] + cars_all[jj].velocity_old[1]*time_inc;
				
				// Rectangle around predicted car location
				bl.x = p_position[0] - car_length/2;
				bl.y = p_position[1] - car_length/2;
				tr.x = p_position[0] + car_length/2;
				tr.y = p_position[1] + car_length/2;
				rectangle(local_mask, bl, tr, 255, CV_FILLED);
				
				
				
				// ALL OF THE BELOW CODE needs to be executed in either case of the above if/else if statement.
				// How to do this? Make it into another function and call it from within the if statements?
				// Note that the if statements themselves will be in another function eventually (so I will be calling a function from a function... that's not a problem)
				
				// Global "significant hue" mask
				Mat global_mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
				inRange(src_hsv, Scalar(0, min_sat, min_val), Scalar(180, 255, 255), global_mask);
				
				
				// imshow("local mask", local_mask);
				// imshow("global mask", global_mask);
				// waitKey(0);
				
				// Combine masks
				local_mask = local_mask & global_mask;
				
				// imshow("mask", local_mask);
				// waitKey(0);
				
				// Calculate histogram (use OpenCV function)
				int max_hue = 180;
				int hue_bins = max_hue;
				int hist_size[] = {hue_bins, 1};
				float bin_w = max_hue/hue_bins;
				float hue_range[] = {0, (float)max_hue};
				const float* range[] = {hue_range};
				MatND hist;
				int channel[] = {0};
				calcHist(&src_hsv, 1, channel, local_mask, hist, 1, hist_size, range, true, false);
				
				// Get maximum histogram value
				double max;
				Point max_loc;
				cv::minMaxLoc(hist, NULL, &max, NULL, &max_loc);
				
				int bin_val;
				//printf("\n");
				int idx = 0;
				for (int i = 0; i < hue_bins; i++)
				{
					bin_val = hist.at<float>(i,0);//*(hue_bins*scale)/max;
					if (bin_val > max/2) {
						//printf("Hue: %4.1f	value: %3d\n", i*bin_w, bin_val);
						significant_hues[idx] = i;
						idx++;
					}
				}
				
				
				
				
				
				
				//figure out best hue to use for tracking
				//median of hues that are above 50% of the peak hue? (use median rather than average in case there is an outlier)
				//only consider values within +/-10 of original value AND +/- 10 from 5 from measured peak? (again, dealing with outliers)
				int best_idx = (int)round(idx/2);
				float best_hue = significant_hues[idx];
				
				printf("best_hue: %f\n", best_hue);
				
				//select learning rate
				// 0.2 if found
				// 0.5 if not found? Or just keep it the same?
				// put both in config filebuf
				
				// do learning
				if (best_hue > cars_all[jj].hue - 15 && best_hue < cars_all[jj].hue + 15)//hue histogram indicates a car is actually present
				{
					// car.hue = 0.2*hue_best + 0.8*car.hue
					cars_all[jj].hue = 0.2*best_hue + 0.8*cars_all[jj].hue;
				}
				
				
				
				
			}
			
			
			
			
			
			
			
			
			
			
			
			
			// Calculate velocity
			do_velocity(cars_all[jj], time_new, time_old);
			
			// Convert measurements to mm
			cars_all[jj].px_to_mm(scale, origin);
			
			// Determine orientation
			if (cars_all[jj].speed() > min_speed) {
				cars_all[jj].orientation_new = (int)(90 - 180/PI*atan2(cars_all[jj].velocity_new[1], cars_all[jj].velocity_new[0]));
				if (cars_all[jj].orientation_new < 0) {
					cars_all[jj].orientation_new = 360 + cars_all[jj].orientation_new;
				}
			} else {
				cars_all[jj].orientation_new = 0;
			}
		}
		
		// Do outputs
		if (output_mode == 4) {
			do_debug(cars_all, src, masks_all, ii, output_mode, time_new, time_start);
		} else {
			do_outputs(cars_all, ii, output_mode, time_new, time_start);
		}
		
		// Update JSON with new data
		do_json(cars_all, sock, output_mode, time_new);
		
		// Update "old" data values
		time_old = time_new;
		for (int jj = 0; jj < cars_all.size(); jj++) {
			cars_all[jj].new_to_old();
		}
		
		// Small delay to ensure comms can keep up
		usleep(150000);
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	cout << endl;
	cout << "Total time: " << time_total <<" seconds"<<endl;
	cout << "Total frames: " << n_frames <<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" <<endl;
	
	Camera.release();
	
	return 0;
}
