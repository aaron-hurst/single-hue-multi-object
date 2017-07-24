#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>
#include <unistd.h>
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
	
	if (save_images == 1)
		cout<<"Output images will be saved with vehicle centroids shown"<<endl;
	else
		cout<<"No images will be saved"<<endl;
	
	
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
	
	int crop = 15;		// number of pixels to crop off each side (remove physical model border from analysis)
	
	
	/// Camera setup
	raspicam::RaspiCam_Cv Camera;
	cam_set_up(argc, argv, Camera);
	if (!Camera.open())	{
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
	sleep(2);	// sleep required to wait for camera to "warm up"
	
		
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
		//imshow("Source", src);
		
		/// Convert to HSV
		Mat src_hsv = Mat::zeros(src.rows, src.cols, CV_8UC3);
		cvtColor(src, src_hsv, COLOR_BGR2HSV);
		
		/// Generate masks of matching hues.
		// Note: cars are red, but by doing a BGR2HSV conversion (rather than RGB2HSV) they appear blue
		Mat mask_1 = Mat::zeros(src.rows, src.cols, CV_8UC1);
		Mat mask_2 = Mat::zeros(src.rows, src.cols, CV_8UC1);		
		do_mask(src_hsv, mask_1, car_1.hue, car_1.delta, crop, car_1.name);
		do_mask(src_hsv, mask_2, car_2.hue, car_2.delta, crop, car_2.name);
		
		/// Locate cars
		Mat src_ctrs = Mat::zeros(src.rows, src.cols, CV_8UC3);		// duplicate source image for printing
		src.copyTo(src_ctrs);
		
		find_car(mask_1, car_1);
		find_car(mask_2, car_2);
		
		/// Calculate velocity, report outputs
		do_outputs(car_1, time_new, time_old);
		do_outputs(car_2, time_new, time_old);
				
		if (save_images == 1)
		{
			char filename [25];
			sprintf(filename, "img_centroids_%03i.png", ii);
			imwrite(filename, src_ctrs);
		}
		
		
		
		
		
		rapidjson::Document document;
		document.SetObject();	// define the document as an object rather than an array
		
		// create a rapidjson array type with similar syntax to std::vector
		rapidjson::Value position(rapidjson::kArrayType);
		rapidjson::Value velocity(rapidjson::kArrayType);
		
		// must pass an allocator when the object may need to allocate memory
		rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
		
		// chain methods as rapidjson provides a fluent interface when modifying its objects
		position.PushBack((int)(car_1.position_new[0]+0.5), allocator).PushBack((int)(car_1.position_new[1]+0.5), allocator);
		velocity.PushBack((int)(car_1.velocity_new[0]+0.5), allocator).PushBack((int)(car_1.velocity_new[1]+0.5), allocator);
		
		// Create MAC Address string
		Value MAC_Address;
		char buffer[13];
		int len = sprintf(buffer, "%s", car_1.mac_add.c_str());
		MAC_Address.SetString(buffer, len, allocator);
		
		// Write information to JSON document object
		document.AddMember("MAC_Address", MAC_Address, allocator);
		document.AddMember("Type", "Car", allocator);
		document.AddMember("Position", position, allocator);
		document.AddMember("Velocity", velocity, allocator);
		document.AddMember("Orientation", car_1.orientation_new, allocator);
		//other veriables?
		
		// Write to output file
		FILE* fp = fopen("output.json", "w");		// open in write mode
		char writeBuffer[65536];
		FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
		Writer<FileWriteStream> fwriter(os);
		document.Accept(fwriter);
		fclose(fp);
		
		
		
		
		
		
		
		
		
		
		
		// Update data
		time_old = time_new;
		new_2_old(car_1);
		new_2_old(car_2);
	}
	
	double time_total = double ( cv::getTickCount() - time_start ) / double ( cv::getTickFrequency() ); // total time in seconds
	
	cout << endl;
	cout << "Total time: " << time_total<<" seconds"<<endl;
	cout << "Total frames: " << n_frames<<endl;
    cout << "Average processing speed: " << time_total/n_frames*1000 << " ms/frame (" << n_frames/time_total<< " fps)" << std::endl;
	
	
	Camera.release();
	
	return 0;
}





