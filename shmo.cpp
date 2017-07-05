#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <ctime>
#include <unistd.h>

// OpenCV and camera interfacing libraries
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Namespaces
using namespace std;
using namespace cv;

// Definitions
// Camera inputs:
#define IMG_BRIGHTNESS		50
#define IMG_CONTRAST		50
#define IMG_SATURATION		50
#define IMG_GAIN			50
#define IMG_SHUTTER_SPEED	3.03	// corresponds to approximately 10 ms (1/100 sec shutter speed) which mitigates banding from flourescent light source


// Function declarations
void cam_set_up(int argc, char **argv, raspicam::RaspiCam_Cv &Camera);
float get_param_val(string param, int argc, char **argv, float default_value);
void do_mask(Mat hsv, Mat mask, int mid_hue, int delta, int crop, string name);
void do_contours(Mat src, const Mat mask, int n_cars, int area_low, int area_high, string name, int frame_number);


int main(int argc,char **argv)
{
	/// Camera setup
	raspicam::RaspiCam_Cv Camera;
	cam_set_up(argc, argv, Camera);
	
	//cout<<"Connecting to camera"<<endl;
    if (!Camera.open())
	{
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
	sleep(2);	// sleep required to wait for camera to "warm up"
	
    Mat src;
	int n_frames = atoi(argv[1]);
	double time0 = cv::getTickCount();
	
	for (int ii = 0; ii < n_frames; ii++)
	{
		cout<<"FRAME "<< ii + 1<< endl;
		
		/// Get image	
		Camera.grab();
		Camera.retrieve(src);
		
		/// Convert to HSV
		Mat src_hsv = Mat::zeros(src.rows, src.cols, CV_8UC3);
		cvtColor(src, src_hsv, COLOR_BGR2HSV);
		
		/// Generate masks of matching hues.
		// Note: cars are red, but by doing a BGR2HSV conversion (rather than RGB2HSV) they appear blue.
		Mat mask_r = Mat::zeros(src.rows, src.cols, CV_8UC1);
		Mat mask_o = Mat::zeros(src.rows, src.cols, CV_8UC1);
		int mid_r = 123;	// peak hue for red car
		int delta_r = 4;
		int mid_o = 115;	// peak hue for orange car
		int delta_o = 4;
		int crop = 15;		// number of pixels to crop off each side (remove physical model border from analysis)
		
		do_mask(src_hsv, mask_r, mid_r, delta_r, crop, "red");
		do_mask(src_hsv, mask_o, mid_o, delta_o, crop, "orange");
		
		/// Determine contours
		int area_low = 350;			// smallest (pixel) area that a vehicle may appear as
		int area_high = 650;		// largest (pixel) area that a vehicle may appear as
		int n_cars = 8;				// maximum number of vehicles
		
		do_contours(src, mask_r, n_cars, area_low, area_high, "red", ii);
		do_contours(src, mask_o, n_cars, area_low, area_high, "orange", ii);
	}
	
	double secondsElapsed = double ( cv::getTickCount() - time0 ) / double ( cv::getTickFrequency() ); // program running time in seconds
	cout<< secondsElapsed<<" seconds runtime"<<endl;
	cout<< secondsElapsed/n_frames << " seconds per frame"<<endl;
	cout<< n_frames/secondsElapsed << "fps"<<endl;
	
	Camera.release();
	
	return 0;
}







float get_param_val(string param, int argc, char **argv, float default_value)
// Search command line for desired parameter string
// If found, return the associated value
{
    int index = -1;
    for ( int i = 0; i < argc && index == -1; i++ )
        if ( string ( argv[i] ) == param ) index = i;
    
	if ( index == -1 ) return default_value;
    else return atof( argv[ index + 1 ] );
}


void cam_set_up(int argc, char **argv, raspicam::RaspiCam_Cv &Camera)
// Read desired image width if provided and perform camera set up operations
{
	int image_width, image_height;
	image_width = get_param_val("-w", argc, argv, 640);
	switch ( image_width ) {
	case 2592:
		image_height = 1944;
		break;
	case 1296:
		image_height = 972;
		break;
	case 640:
		image_height = 480;
		break;
	default:
		image_height = 480;
		image_width = 640;
		break;
	}
	
	// Set frame width and height, brightness (?), contrast, saturation and gain (ISO)
	// White balance left on auto and colour image capture used
	Camera.set ( CV_CAP_PROP_FRAME_WIDTH, image_width );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, image_height );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS, IMG_BRIGHTNESS );
    Camera.set ( CV_CAP_PROP_CONTRAST, IMG_CONTRAST );
    Camera.set ( CV_CAP_PROP_SATURATION, IMG_SATURATION );
    Camera.set ( CV_CAP_PROP_GAIN, IMG_GAIN );
	Camera.set ( CV_CAP_PROP_EXPOSURE, IMG_SHUTTER_SPEED );
	
	return;
}





void do_mask(Mat hsv, Mat mask, int mid_hue, int delta, int crop, string name)
// This function derives a hue-based mask from a given HSV image
// hsv and mask must have the same dimensions
// mid_hue 	is the middle (expected peak) hue value associated with the desired object
// delta 	is the expected range either side of the peak value that should be included
// crop		is number of pixels that should be removed from each edge (to ignore the wooden frame border)
{
	// Initial hue matching operation
	inRange(hsv, Scalar(mid_hue - delta, 40, 40), Scalar(mid_hue + delta, 255, 255), mask);
	
	// Cropping mask
	Mat mask_crop = Mat::zeros(hsv.rows, hsv.cols, CV_8UC1);	// declare mask used to eliminate table borders
	rectangle(mask_crop, Point(crop, crop), Point(hsv.cols - crop, hsv.rows - crop), 255, CV_FILLED);
	
	// Merge masks
	mask = mask & mask_crop;
	
	// Apply dilation to remove holes and smooth out edges
	int dilation_iterations = 1;	// number of iterations to compute
	int dilation_size = 3;			// size of rectangular kernel
	Mat element = getStructuringElement(MORPH_RECT, Size( dilation_size, dilation_size ), Point( -1, -1));	// dilation kernel element
	
	dilate(mask, mask, element, Point(-1, -1), dilation_iterations);
	
	// char filename [25];
	// sprintf(filename, "img_mask_%s.png", name.c_str());
	// imwrite(filename, mask);
	
	return;
}


void do_contours(Mat src, const Mat mask, int n_cars, int area_low, int area_high, string name, int frame_number)
// This function identifies wehicles in a given mask and determines their centroids
// n_cars		maximum number of cars expected in the image
// area_low		smallest number of pixels that a vehicle may appear as
// area_high	largest number of pixels that a vehicle may appear as
{
	// Find contours
	vector<vector<Point> > contours;		// vector for storing contours
	findContours(mask, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);	// note that contours is modified in this step
	int n_contours = contours.size();	// number of contours
	
	// Contour areas
	float contour_areas [n_contours];
	for (int i = 0; i < n_contours; i++)
	{
		contour_areas[i] = contourArea(contours[i]);
	}
	
	// Check areas against known vehicle size and hence locate vehicles
	int contour_idx [n_cars];	// array for storing indices of contours that correspond to vehicles
	int idx = 0;				// counter
	for (int i = 0; i < n_contours; i++)		// scan through contour areas
	{
		if (contour_areas[i] > area_low && contour_areas[i] < area_high) 	// compare area to low and high thresholds
		{
			contour_idx[idx] = i;				// if within thresholds, record contour index
			cout<<"Car "<< idx + 1 << " (" << name << ") area: " << contour_areas[i] <<endl;
			idx++;								// increment index of contour_idx array
		}
	}
	
	// Determine vehicle centroids
	vector<Moments> mu(idx);			// vector for storing moments of each car-representing contour
	for (int i = 0; i < idx; i++)
	{
		mu[i] = moments(contours[contour_idx[i]], true);	// moment of i-th car-representing contour
	}
	
	Mat src2 = Mat::zeros(src.rows, src.cols, CV_8UC3);		// duplicate source image for printing
	src.copyTo(src2);
	
	vector<Point2f> mc(idx);	// vector for storing centre of mass of each car's contour
	for (int i = 0; i < idx; i++)
	{
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		cout<<"Car "<< i + 1 << " (" << name << ") location: "<< endl;
		cout<< Mat(mc[i]) << endl;
		circle(src2, mc[i], 3, Scalar(0, 0, 255), -1); 	// draw centroids on source image
	}
	
	char filename [25];
	sprintf(filename, "img_centroids_%s_%i.png", name.c_str(), frame_number);
	imwrite(filename, src2);
	
	return;
}
