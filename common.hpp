// Header include guard
#ifndef COMMON_H	// if common.h has not been included, include it, otherwise do not
#define COMMON_H	// see end of file cor corresponding #endif

// OpenCV and camera interfacing libraries
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// RapidJSON
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filewritestream.h"

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
using namespace rapidjson;


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
	
	// Member function declarations
	void px_to_mm(float alpha, const int origin[]);
	void new_to_old(void);
};

// Member function definitions
void Car::px_to_mm(float alpha, const int origin[])
// Convert position and velocity measurements from pixel values to mm from the coordinate system origin
{
	position_new[0] = alpha*(position_new[0] - origin[0]);
	position_new[1] = alpha*(position_new[1] - origin[1]);
	velocity_new[0] = alpha*velocity_new[0];
	velocity_new[1] = alpha*velocity_new[1];
	return;
}

void Car::new_to_old(void)
// Update old variables with new data
{
	area_old = area_new;
	position_old[0] = position_new[0];
	position_old[1] = position_new[1];
	velocity_old[0] = velocity_new[0];
	velocity_old[1] = velocity_new[1];
	orientation_old = orientation_new;
	return;
}



void cam_setup(int argc, char **argv, raspicam::RaspiCam_Cv &Camera)
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


void state_out_mode (int output_mode)
// Display the selected output mode to the user on the console
{
	if (output_mode == 0) {
		cout<<"Output mode: console & JSON"<<endl;
	} else if (output_mode == 1) {
		cout<<"Output mode: csv & JSON"<<endl;
	} else if (output_mode == 2) {
		cout<<"Output mode: console, csv & JSON"<<endl;
	} else if (output_mode == 3) {
		cout<<"Output mode: console, source + centroids image & JSON"<<endl;
	} else if (output_mode == 4) {
		cout<<"Output mode: console, source + centroids image, masks & JSON"<<endl;
	} else if (output_mode == 5) {
		cout<<"Output mode: ALL (console, csv, source + centroids image, masks & JSON)"<<endl;
	} else {
		cout<<"ERROR: Invalid output mode. Using default: Mode 0: console + JSON."<<endl;
	}
	return;
}


void do_velocity(Car &car, double time_new, double time_old)
// This function calculates the car's velocity based on two consecutive frames and
// prints area, position and velocity outputs.
{
	if (car.area_new < 0) {
		// Car not found in current frame, report zero velocity
		car.velocity_new[0] = 0.0;
		car.velocity_new[1] = 0.0;
	} else if (car.area_old < 1) {
		// No old data (but current data is acceptable), report zero velocity
		car.velocity_new[0] = 0.0;
		car.velocity_new[1] = 0.0;
	} else {
		double time_inc = double (time_new - time_old) / double (cv::getTickFrequency());
		car.velocity_new[0] = (car.position_new[0] - car.position_old[0])/time_inc;
		car.velocity_new[1] = (car.position_new[1] - car.position_old[1])/time_inc;
	}
	return;
}


void do_outputs(const Mat src, const vector<Mat> masks_all, const vector<Car> cars_all, int frame, int output_mode, double time_new, double time_start)
// Save desired log items. At present only has the capability to save images - the source image and optionally the masks as well.
{
	// Print console output
	if (output_mode != 1) {	
		cout << endl; // write a newline before each frame's outputs
		cout<<"~~~~~~~~~~~~~~ FRAME "<< frame + 1 <<" ~~~~~~~~~~~~~~"<<endl;
		
		for (int i = 0; i < cars_all.size(); i++) {
			if (cars_all[i].area_new < 0) {
				// Car not found in current frame
				cout<<"WARNING: "<< cars_all[i].name <<" car not detected"<<endl;
				continue;
			} else if (cars_all[i].area_old < 1) {
				// No old data (but current data is acceptable)
				cout<<"WARNING: previous instant has no data ("<< cars_all[i].name <<" car)"<<endl;
			}	
			cout<<"Car: "<< cars_all[i].name <<endl;
			printf("  area:		%5.1f		pixels\n", cars_all[i].area_new);
			printf("  location:	(%5.1f, %5.1f)	mm\n", cars_all[i].position_new[0], cars_all[i].position_new[1]);
			printf("  velocity:	(%5.1f, %5.1f)	mm/s\n", cars_all[i].velocity_new[0], cars_all[i].velocity_new[1]);
			//printf("  orientation (degrees):	%i\n", cars_all[i].orientation_new);
		}
	}
	
	// Save csv file (data.csv)
	if (output_mode == 1 || output_mode == 2 || output_mode == 5) {
		// Open file
		FILE * log_csv;
		log_csv = fopen("data.csv","a");	// append mode
		fprintf(log_csv, "%7.3f,", (time_new - time_start)/(cv::getTickFrequency()));	// time (since program start)
		
		// Add data for each car
		for (int i = 0; i < cars_all.size(); i++) {
			fprintf(log_csv, "%6.1f,", cars_all[i].area_new);
			fprintf(log_csv, "%6.1f,", cars_all[i].position_new[0]);
			fprintf(log_csv, "%6.1f,", cars_all[i].position_new[1]);
			fprintf(log_csv, "%6.1f,", cars_all[i].velocity_new[0]);
			fprintf(log_csv, "%6.1f,", cars_all[i].velocity_new[1]);
			fprintf(log_csv, "%i,", cars_all[i].orientation_new);
		}
		fprintf(log_csv, "\n");
		fclose(log_csv);
	}
	
	// Save image outputs
	if (output_mode > 2) {
		// Source + centroids image
		char filename [50];
		sprintf(filename, "%03i_centroids.png", frame);
		imwrite(filename, src);
		
		if (output_mode > 3) {
			// Mask images
			for (int i = 0; i < masks_all.size(); i++)
			{
				sprintf(filename, "%03i_mask_%s.png", frame, cars_all[i].name.c_str());
				imwrite(filename, masks_all[i]);
			}
		}
	}
	
	return;
}


void do_JSON (vector<Car> cars_all)
// This function performs the task of saving the most recent data to the JSON output file
{
	// Declare RapidJSON document and define the document as an object rather than an array
	rapidjson::Document document;
	document.SetObject();
	
	// Get an "alloctor" - must pass an allocator when the object may need to allocate memory
	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
	
	// Iterate through each car, populating data and MAC address and sending this to the rapidjson document
	for (int i = 0; i < cars_all.size(); i++) {
		// Declare rapidjson array for storing data (this type has similar syntax to std::vector)
		rapidjson::Value data(rapidjson::kArrayType);
		
		// Populate data
		data.PushBack(1, allocator);										// physical object type
		data.PushBack((int)round(cars_all[i].position_new[0]), allocator);	// car position, x-component
		data.PushBack((int)round(cars_all[i].position_new[1]), allocator);	// car position, y-component
		data.PushBack((int)round(cars_all[i].velocity_new[0]), allocator);	// car velocity, x-component
		data.PushBack((int)round(cars_all[i].velocity_new[1]), allocator);	// car velocity, y-component
		data.PushBack(cars_all[i].orientation_new, allocator);				// car orientation
		data.PushBack(0, allocator);										// spare
		data.PushBack(0, allocator);										// spare
		
		// Create MAC Address string
		rapidjson::Value MAC_Address;
		char buffer[13];
		int len = sprintf(buffer, "%s", cars_all[i].mac_add.c_str());
		MAC_Address.SetString(buffer, len, allocator);
		
		// Add information to rapidjson document (an object)
		document.AddMember(MAC_Address, data, allocator);
	}
	
	// Write to output file
	FILE* fp = fopen("output.json", "w");	// open in write mode (deletes previous contents)
	char writeBuffer[65536];
	FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
	Writer<FileWriteStream> fwriter(os);
	document.Accept(fwriter);
	fclose(fp);
	
	return;
}



#endif