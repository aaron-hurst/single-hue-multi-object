// OpenCV and camera interfacing libraries
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Definitions
// Camera inputs:
#define IMG_WIDTH			640
#define IMG_HEIGHT			480
#define IMG_BRIGHTNESS		50
#define IMG_CONTRAST		50
#define IMG_SATURATION		50
#define IMG_GAIN			50
#define IMG_SHUTTER_SPEED	3.03	// corresponds to approximately 10 ms (1/100 sec shutter speed) which mitigates banding from flourescent light source

// Namespaces
using namespace std;
using namespace cv;



// Car object structure
struct Car {
	string name;				// car name, usually its colour
	string mac_add;				// 12 character string for MAC Address
	
	// Physical characteristics
	int hue;					// colour of car
	int delta;					// half-width of car hue histogram
	int size_min;				// minimum (pixel) area car may appear as
	int size_max;				// maximum (pixel) area car may appear as
	
	// Measured parameters
	float area_old;				// measured area
	float area_new;
	float position_old[2];		// measures position
	float position_new[2];
	float velocity_old[2];		// measured velocity
	float velocity_new[2];
	int orientation_old;		// measured orientation
	int orientation_new;
};



void cam_set_up(int argc, char **argv, raspicam::RaspiCam_Cv &Camera)
// Read desired image width if provided and perform camera set up operations
{
	// Set frame width and height, brightness (?), contrast, saturation and gain (ISO)
	// White balance left on auto and colour image capture used
	Camera.set (CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
    Camera.set (CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    Camera.set (CV_CAP_PROP_BRIGHTNESS, IMG_BRIGHTNESS);
    Camera.set (CV_CAP_PROP_CONTRAST, IMG_CONTRAST);
    Camera.set (CV_CAP_PROP_SATURATION, IMG_SATURATION);
    Camera.set (CV_CAP_PROP_GAIN, IMG_GAIN);
	Camera.set (CV_CAP_PROP_EXPOSURE, IMG_SHUTTER_SPEED);
	
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
	//imshow("Initial mask", mask);
	
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
	//imshow("Final mask", mask);
	
	// char filename [25];
	// sprintf(filename, "img_mask_%s.png", name.c_str());
	// imwrite(filename, mask);
	
	return;
}


void find_car(const Mat mask, Car &car)
// This function locates a desired car in a given mask and determines its centroid.
// The centroid is then stored in the car's associated structure.
//
// mask		binary image showing hues matching to car of interest
// car		structure for car of interest
{	
	// Find contours
	vector<vector<Point> > contours;		// vector for storing contours
	findContours(mask, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);	// note that contours is modified in this step
	int n_contours = contours.size();	// number of contours
	
	if (n_contours < 1)
	{
		car.position_new[0] = -1;
		car.position_new[1] = -1;
		car.area_new = -1;
		return;
	}
	
	// Contour areas
	float contour_areas [n_contours];
	for (int i = 0; i < n_contours; i++)
	{
		contour_areas[i] = contourArea(contours[i]);
	}
	
	// Check areas against known vehicle size and hence locate vehicles
	int contour_idx;	// indices of contour that correspond to desired vehicle
	int idx = 0;		// counter
	for (int i = 0; i < n_contours; i++)		// scan through contour areas
	{
		if (contour_areas[i] > car.size_min && contour_areas[i] < car.size_max) 	// compare area to low and high thresholds
		{
			contour_idx = i;	// if area within thresholds, record contour index
			idx++;				// increment index - used later to check only one car was found
		}
	}
	
	if (idx > 1)
	{
		cout<<"WARNING: more than one object resembling the "<< car.name <<" car was found."<<endl;
		cout<<"Using the last object found for calculations. Check masks to verify this is correct."<<endl;
	}

	// Determine vehicle centroids
	Moments mu;
	mu = moments(contours[contour_idx], true);		// moment of car's contour		
	
	car.position_new[0] = 640 - mu.m10 / mu.m00;	// number of pixels along x-axis from origin
	car.position_new[1] = 480 - mu.m01 / mu.m00;	// number of pixels along y-axis from origin
	
	car.area_new = contour_areas[contour_idx];
	// cout<<"Car: "<< car.name <<endl;
	// printf("  area (pixels):	%5.1f\n", contour_areas[contour_idx]);
	// printf("  location (x, y):	(%4.1f, %4.1f)\n", car.position_new[0], car.position_new[1]);
	
	// Draw centroids on source image
	//circle(src_ctrs, Point(car_data[1], car_data[2]), 3, Scalar(0, 0, 255), -1);
	
	return;
}


void do_outputs(Car &car, double time_new, double time_old)
// This function calculates the car's velocity based on two consecutive frames and
// prints area, position and velocity outputs.
{
	if (car.area_new < 1) {
		// Car not found in current frame
		cout<<"WARNING: "<< car.name <<" car not detected"<<endl;
		// set all new values to zero? Make a "reset" function for this
		return;
	} else if (car.area_old < 1) {
		// No old data, report zero velocity
		cout<<"WARNING: previous instant has no data ("<< car.name <<" car)"<<endl;
		car.velocity_new[0] = 0.0;
		car.velocity_new[1] = 0.0;
	} else {
		double time_inc = double (time_new - time_old) / double (cv::getTickFrequency());
		car.velocity_new[0] = (car.position_new[0] - car.position_old[0])/time_inc;
		car.velocity_new[1] = (car.position_new[1] - car.position_old[1])/time_inc;
	}
	
	cout<<"Car: "<< car.name <<endl;
	printf("  area:		%5.1f		pixels\n", car.area_new);
	printf("  location:	(%4.1f, %4.1f)	pixels\n", car.position_new[0], car.position_new[1]);
	printf("  velocity:	(%4.1f, %4.1f)	pixels/s\n", car.velocity_new[0], car.velocity_new[1]);
	//printf("  orientation (degrees):	%i\n", car.orientation_new);
}


void new_2_old(Car &car)
// Update old variables with new data
{
	car.area_old = car.area_new;
	car.position_old[0] = car.position_new[0];
	car.position_old[1] = car.position_new[1];
	car.velocity_old[0] = car.velocity_new[0];
	car.velocity_old[1] = car.velocity_new[1];
	car.orientation_old = car.orientation_new;
}









