// Header include guard
#ifndef COMMON_H	// if common.h has not been included, include it, otherwise do not
#define COMMON_H	// see end of file cor corresponding #endif

// General includes
#include<string.h>		// to_string

// OpenCV and camera includes
#include "/home/pi/raspicam-0.1.6/src/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/highgui.h"

// Socket/comms related includes
#include<sys/socket.h>	// socket

// Camera definitions:
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
	
	// Member function declarations
	void px_to_mm(float scale, const float origin[]);
	void new_to_old(void);
};

// Member function definitions
void Car::px_to_mm(float scale, const float origin[])
// Convert position and velocity measurements from pixel values to mm from the coordinate system origin
{
	position_new[0] = scale*(position_new[0] - origin[0]);
	position_new[1] = scale*(position_new[1] - origin[1]);
	velocity_new[0] = scale*velocity_new[0];
	velocity_new[1] = scale*velocity_new[1];
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



void cam_setup(raspicam::RaspiCam_Cv &Camera)
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


void do_config(vector<Car> cars_all, int &crop, float origin[], int &scale)
// Configures algorithm data from config.txt file (which must be in the same directory as the main file)
// Creates and populates Car and Obstacle structs
// Reads and stores crop, origin and scale parameters
{
	// Open and check config file
	ifstream f("config.txt");
	if (!f) {
		cout << "Error: could not load config file: config.txt" << endl;
		return;
	}
	
	string line;
	Car car_dummy;
	//Obstacle obst_dummy;
	
	while (getline(f, line)) {		// get the next line of the file
		istringstream iss(line);	// send line to an istringstream
		iss >> name >> tmp;			// extract information
		
		// Skip invalid lines and comments
		if (iss.fail() || tmp != "=" || name[0] == '#') continue;
		
		// Global parameters
		if (name == "crop")		iss >> crop;
		if (name == "origin_x")	iss >> origin[0];
		if (name == "origin_y")	iss >> origin[1];
		if (name == "scale")	iss >> scale;
		
		// Cars
		// If dealing with a car, enter a second while loop to populate a dummy struct which is then pushed to the cars_all vector
		if (name == "Car") {
			while (getline(f, line)) {
				istringstream iss(line);
				iss >> name >> tmp >> val;
				
				if (iss.fail() || tmp != "=" || name[0] == '#') continue;	// invalid lines
				
				if (name == "name")		val >> car_dummy.name;
				if (name == "MAC_add")	val >> car_dummy.mac_add;
				if (name == "hue")		val >> car_dummy.hue;
				if (name == "delta")	val >> car_dummy.delta;
				if (name == "size_min")	val >> car_dummy.size_min;
				if (name == "size_max")	val >> car_dummy.size_max;
				
				if (val == "end") {					// signifies end of car config parameters
					cars_all.push_back(car_dummy);	// store cnewly configured car in cars_all vector
					break;							// exit car config while loop
				}
			}
		}
		
		// Obstacles
		// if (name == "Obstacles") {
			// while (getline(f, line)) {
				// istringstream iss(line);
				// iss >> name >> tmp >> val;
				
				// if (iss.fail() || tmp != "=" || name[0] == '#') continue;	// invalid lines
				
				// if (name == "name")		val >> car_dummy.name;
				// if (name == "MAC_add")	val >> car_dummy.mac_add;
				// if (name == "hue")		val >> car_dummy.hue;
				// if (name == "delta")	val >> car_dummy.delta;
				// if (name == "size_min")	val >> car_dummy.size_min;
				// if (name == "size_max")	val >> car_dummy.size_max;
				
				// if (val == "end")	break;	// signifies end of car config parameters
			// }
		// }
	}
}


int state_output_mode (int output_mode)
// Display the selected output mode to the user on the console
{
	if (output_mode == 0) {
		cout<<"Output mode: none (except JSON)"<<endl;
	} else if (output_mode == 1) {
		cout<<"Output mode: console (plus JSON)"<<endl;
	} else if (output_mode == 2) {
		cout<<"Output mode: csv log file (plus JSON)"<<endl;
	} else if (output_mode == 3) {
		cout<<"Output mode: csv and console (plus JSON)"<<endl;
	} else if (output_mode == 4) {
		cout<<"Output mode: debug - console, csv and relevant images (NO JSON - do not use this mode with controller)"<<endl;
	} else {
		cout<<"ERROR: Invalid output mode. Using default: Mode 1: console (plus JSON)."<<endl;
		output_mode = 1;
	}
	return output_mode;
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


void do_outputs(const vector<Car> cars_all, int frame, int output_mode, double time_new, double time_start)
// Save desired log items. At present only has the capability to save images - the source image and optionally the masks as well.
{
	if (output_mode == 1 || output_mode > 2) {
		// Print console output
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
	
	// Save csv file (log.csv)
	if (output_mode > 1) {
		// Open file
		FILE * log_csv;
		log_csv = fopen("log.csv","a");	// append mode
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
	
	return;
}


void do_json (vector<Car> cars_all, int sock, int output_mode)
// This function performs the task of saving the most recent data to the JSON output file
{
	// Create JSON string
	string json = "{";
	json.reserve(cars_all.size()*100);		// reserve memory for the json string (increases efficiency)
	int not_first = 0;
	for (int i = 0; i < cars_all.size(); i++) {
		// If car was found, append its data to the json string
		if (cars_all[i].area_new > 0) {
			if (not_first) {
				// Pre-pend car data with a comma (the key:value delimiter) except for first car
				json.append(",");
			}
			json.append("\"");					// leading quote for MAC address						
			json.append(cars_all[i].mac_add);	// MAC address
			json.append("\":[");				// end quote for MAC address, colon for key:value pairs, opening bracket for data array
			json.append("1");					// object type: 1 for cars
			json.append(",");
			json.append(std::to_string((int)round(cars_all[i].position_new[0])));	// x-position
			json.append(",");
			json.append(std::to_string((int)round(cars_all[i].position_new[1])));	// y-position
			json.append(",");
			json.append(std::to_string((int)round(cars_all[i].velocity_new[0])));	// x-velocity
			json.append(",");
			json.append(std::to_string((int)round(cars_all[i].velocity_new[1])));	// y-velocity
			json.append(",");
			json.append(std::to_string((int)round(cars_all[i].orientation_new)));	// orientation
			json.append(",");
			json.append("0");		// spare
			json.append(",");
			json.append("0");		// spare
			json.append("]");		// closing array bracket and key:value comma delimiter
			
			if (!not_first) {
				// Set not_first to 1 for subsequent loops
				not_first = 1;
			}
		}
	}
	json.append("}");
	
	if (output_mode == 4) {
		// Debug mode - do not send output, do print JSON to console
		cout<< json <<endl;
	} else {
		// Send JSON string to server via socket, do not print to console
		if(send(sock, json.c_str(), json.size(), 0) < 0) {
			cout<< "Send failed" <<endl;
			return;
		}
	}
	
	return;
}



#endif