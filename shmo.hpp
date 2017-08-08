// OpenCV and camera interfacing includes
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

#include "common.hpp"

// Namespaces
using namespace std;
using namespace cv;


void do_mask(Mat hsv, Mat mask, int mid_hue, int delta, int crop, string name)
// This function derives a hue-based mask from a given HSV image
// hsv and mask must have the same dimensions
// mid_hue 	is the middle (expected peak) hue value associated with the desired object
// delta 	is the expected range either side of the peak value that should be included
// crop		is number of pixels that should be removed from each edge (to ignore the wooden frame border)
// name		chosen identifier for each car
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
	Mat element = getStructuringElement(MORPH_RECT, Size(dilation_size, dilation_size), Point(-1, -1));		// dilation kernel element
	dilate(mask, mask, element, Point(-1, -1), dilation_iterations);
	
	return;
}


void find_car(Mat mask, Car &car)
// This function locates a desired car in a given mask and determines its centroid.
// The centroid is then stored in the car's associated structure.
// mask		binary image showing hues matching to car of interest
// car		structure for car of interest
{	
	// Create mask that can be modified without affecting original
	Mat mask_use = Mat::zeros(mask.rows, mask.cols, CV_8UC1);
	mask.copyTo(mask_use);
	
	// Find contours
	vector<vector<Point> > contours;								// vector for storing contours
	findContours(mask_use, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);	// note that contours is modified in this step
	int n_contours = contours.size();								// number of contours
	
	// Return an error state if no contours, and hence no cars, are found
	if (n_contours < 1)
	{
		car.position_new[0] = -1000;
		car.position_new[1] = -1000;
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
	int contour_idx;	// index of contour that correspond to desired vehicle
	int idx = 0;		// counter
	for (int i = 0; i < n_contours; i++)	// scan through contour areas
	{
		if (contour_areas[i] > car.size_min && contour_areas[i] < car.size_max) 	// compare area to low and high thresholds
		{
			contour_idx = i;	// if area within thresholds, record contour index
			idx++;				// increment index - used later to check only one car was found
		}
	}
	
	// Display a warning if more than one object fitting the size and colour requirements is found.
	if (idx > 1)
	{
		cout<<"WARNING: more than one object resembling the "<< car.name <<" car was found."<<endl;
		cout<<"Using the last object found for calculations. Check masks to verify this is correct."<<endl;
	}
	
	// Return an error state if no contours match area requirements and hence no car is found
	if (idx == 0)
	{
		car.position_new[0] = -1;
		car.position_new[1] = -1;
		car.area_new = -1;
		return;
	}

	// Determine vehicle centroids, car position and object area
	Moments mu;
	mu = moments(contours[contour_idx], true);		// moment of car's contour		
	car.position_new[0] = mu.m10 / mu.m00;	// x-position of car in pixels along x-axis from origin
	car.position_new[1] = mu.m01 / mu.m00;	// y-position of car in pixels along y-axis from origin
	car.area_new = contour_areas[contour_idx];
	
	return;
}


void do_debug (const vector<Car> cars_all, const Mat src, const vector<Mat> masks_all, int frame, int output_mode, double time_new, double time_start)
// Save image outputs in addition to all other outputs
// Note that the debug mode is algorithm-specific, and therefore not in common.hpp
{
	// Call normal outputs function in mode 3 (console + csv)
	do_outputs(cars_all, frame, output_mode, time_new, time_start);
	
	// Source + centroids image
	// char filename [50];
	// sprintf(filename, "%03i_centroids.png", frame);
	// imwrite(filename, src);
	
	// Mask images
	// for (int i = 0; i < masks_all.size(); i++)
	// {
		// sprintf(filename, "%03i_mask_%s.png", frame, cars_all[i].name.c_str());
		// imwrite(filename, masks_all[i]);
	// }
	
	return;
}

