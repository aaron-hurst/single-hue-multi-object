#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>
#include <unistd.h>
#include <math.h>		// for "round"
#include "shmo.hpp"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"

#include <cstdio>	// for fopen?


// OpenCV and camera interfacing libraries
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Namespaces
using namespace std;
using namespace cv;
using namespace rapidjson;


// Definitions


// Function declarations





int main(int argc,char **argv)
{
	Mat src;
	int n_frames = atoi(argv[1]);		// specify number of frames to process
	int save_images = atoi(argv[2]);	// specify 1 to save tracking images
	int time_new, time_old;
	
	if (save_images == 0) { cout<<"No images will be saved"<<endl; }	
	
	/// Camera setup
	raspicam::RaspiCam_Cv Camera;
	cam_setup(argc, argv, Camera);
	if (!Camera.open())		// open camera
	{
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
	sleep(2);	// sleep required to wait for camera to "warm up"
	
	
	
	
	
	
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
	
	
	
	// Grab a test image and allocate image Mats
	Camera.grab();
	Camera.retrieve(src);										// source image
	Mat src_hsv = Mat::zeros(src.rows, src.cols, CV_8UC3);		// HSV version (only one copy, overwritten for each car)
	// different mask for each car so that they are not overwritten and can be saved at end if desired
	Mat mask_1 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	Mat mask_2 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	vector<Mat> masks_all;
	masks_all.push_back(mask_1);
	masks_all.push_back(mask_2);
	
	
	
	/// Run tracking
	double time_start = cv::getTickCount();	
	for (int ii = 0; ii < n_frames; ii++)
	{
		cout << endl; // write a newline before each frame's outputs
		cout<<"~~~~~~~~~~~ FRAME "<< ii + 1<<" ~~~~~~~~~~~"<<endl;
		
		/// Get image	
		Camera.grab();
		Camera.retrieve(src);
		time_new = cv::getTickCount();
		
		/// Convert to HSV
		cvtColor(src, src_hsv, COLOR_BGR2HSV);
		
		/// Generate masks of matching hues.
		// Note: cars are red, but by doing a BGR2HSV conversion (rather than RGB2HSV) they appear blue		
		do_mask(src_hsv, masks_all[0], cars_all[0].hue, cars_all[0].delta, crop, cars_all[0].name);
		do_mask(src_hsv, masks_all[1], cars_all[1].hue, cars_all[1].delta, crop, cars_all[1].name);
		
		/// Locate cars
		find_car(masks_all[0], cars_all[0]);
		find_car(masks_all[1], cars_all[1]);
		
		/// Conversion to mm
		cars_all[0].px_to_mm(alpha, origin);
		cars_all[1].px_to_mm(alpha, origin);
		
		/// Calculate velocity, report outputs
		do_outputs(cars_all[0], time_new, time_old);
		do_outputs(cars_all[1], time_new, time_old);
		
		/// Save desired logs - images, data log files
		do_logs(src, masks_all, cars_all, ii, save_images);	
		
		
		
		
		
		
		
		
		
		// Declare RapidJSON document and define the document as an object rather than an array
		rapidjson::Document document;
		document.SetObject();
		
		// Get an "alloctor" - must pass an allocator when the object may need to allocate memory
		rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
		
		// Create a rapidjson array for storing data (this type has similar syntax to std::vector)
		//rapidjson::Value data(rapidjson::kArrayType);
		
		// Iterate through each car, populating data and MAC address and sending this to the rapidjson document
		for (int i = 0; i < cars_all.size(); i++)		
		{
			// Declare rapidjson array for storing data (this type has similar syntax to std::vector)
			rapidjson::Value data(rapidjson::kArrayType);
			
			// Populate data rapidjson array
			data.PushBack(1, allocator);										// physical object type
			data.PushBack(round(cars_all[i].position_new[0]), allocator);	// car position, x-component
			data.PushBack(round(cars_all[i].position_new[1]), allocator);	// car position, y-component
			data.PushBack(round(cars_all[i].velocity_new[0]), allocator);	// car velocity, x-component
			data.PushBack(round(cars_all[i].velocity_new[1]), allocator);	// car velocity, y-component
			data.PushBack(cars_all[i].orientation_new, allocator);				// car orientation
			data.PushBack(0, allocator);										// spare
			data.PushBack(0, allocator);										// spare
			
			// Create MAC Address string
			Value MAC_Address;
			char buffer[13];
			int len = sprintf(buffer, "%s", cars_all[i].mac_add.c_str());
			MAC_Address.SetString(buffer, len, allocator);
			
			// Add information to rapidjson document (an object)
			document.AddMember(MAC_Address, data, allocator);
		}
		
		// Write to output file
		FILE* fp = fopen("output.json", "w");		// open in write mode
		char writeBuffer[65536];
		FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
		Writer<FileWriteStream> fwriter(os);
		document.Accept(fwriter);
		fclose(fp);
		
		
		
		
		
		
		// Update data
		time_old = time_new;
		new_2_old(cars_all[0]);
		new_2_old(cars_all[1]);
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	
	cout << endl;
	cout << "Total time: " << time_total<<" seconds"<<endl;
	cout << "Total frames: " << n_frames<<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" << std::endl;
	
	
	Camera.release();
	
	return 0;
}





