#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <wiringPiI2C.h>
#include "servo_control.h"
#include "ina219_control.h"
#include <ctime>
#include <sys/stat.h>

using namespace cv;
using namespace std;

 ///////////////////////////////////////////////////////////////////////////////////////////////////
// detected moving object features //
class DetectedArea {
public:
	vector<Point> contour;
	Rect bounding_rectangle;
	float bounding_rect_aspect_ratio;  

	DetectedArea(vector<Point> object_contour)
	{
		contour = object_contour;
		bounding_rectangle = boundingRect(object_contour);
		bounding_rect_aspect_ratio = (float)bounding_rectangle.width / bounding_rectangle.height;
	}
};
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// creating path and folder for images //
string folder_path_creator()
{
	time_t current_time;
	struct tm * time_data;
	char time_buffer[50];
	time(&current_time);
	time_data = localtime(&current_time);
	strftime(time_buffer,sizeof(time_buffer),"%d-%m-%Y_%H:%M:%S",time_data);
	string folder_name(time_buffer);
	string path = "/home/pi/Documents/object_tracking/" + folder_name;
	const char *folder_path_char = path.c_str();
	int status = mkdir(folder_path_char,S_IRWXU | S_IRWXG | S_IRWXO);
	return path;
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// define gpio settings //
void gpio_define(int pin, int mode, int pull_up_down)
{
	pinMode(pin, mode);
	if(mode == INPUT)
		pullUpDnControl(pin, pull_up_down);
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// define gpio settings //
void IR_filter_control()
{
	if(digitalRead(1) == HIGH)
		digitalWrite(7,LOW);
	else
		digitalWrite(7,HIGH);
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// changing servo position to find moving objects //
void move_servo_to_check(int &servo_controller_id, int servo_1_position, int servo_2_position, bool &servo_1_motion, bool &servo_2_motion, 
			 float &servo_1_stop_delay, float &servo_2_stop_delay, int &servo_1_destination, int &servo_2_destination, bool &stop)
{
	move_servos(servo_controller_id,1,servo_1_position,60);
	move_servos(servo_controller_id,2,servo_2_position,60);
	servo_1_motion = true;
	servo_2_motion = true;
	servo_1_stop_delay = 0;
	servo_2_stop_delay = 0;
	servo_1_destination = servo_1_position;
	servo_2_destination = servo_2_position;
	stop = false;
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// creating image name and wiritng //
void create_image_name(int &frame_counter, Mat &captured_frame, string &folder_path)
{
	stringstream image_number_stream;
	image_number_stream << setw(7) << setfill('0') << ++frame_counter;
	string image_number = image_number_stream.str();
	string full_image_name = folder_path + "/img" + image_number + ".jpg";
	vector<int> compression_params;
	compression_params.push_back(IMWRITE_JPEG_QUALITY);
	compression_params.push_back(95);
	imwrite(full_image_name, captured_frame,compression_params);
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// moving servo following object //
void follow_object(int &servo_destination, int &servo_position, int servo_change_value, int &servo_controller_id, int servo_number, int velocity, bool &servo_motion, float &servo_stop_delay, Rect2d &tracking_rectangle, Mat &captured_frame_1_copy, bool &stop)
{
	servo_destination = servo_position + servo_change_value;
	move_servos(servo_controller_id, servo_number,servo_destination, velocity);
	servo_motion = true;
	servo_stop_delay = 0;
	stop = false;
	tracking_rectangle = Rect2d(captured_frame_1_copy.size().width/2-1,captured_frame_1_copy.size().height/2-1,2,2);
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// moving servo following object //
bool check_supply_voltage(int &device_I2C_id)
{
	if(supply_voltage_calc(device_I2C_id) < 8)  // change value if other than 3x3,7V Li-ion power supply use
		return false;
	else
		return true;
}
 ///////////////////////////////////////////////////////////////////////////////////////////////////
// moving servo following object //
void end_program(int &servo_controller_id, VideoCapture &camera, int &frame_counter, float &total_program_time, string &folder_path)
{
	servo_on_off(servo_controller_id, 0, 1);
	servo_on_off(servo_controller_id, 0, 2);
	float average_fps = float(frame_counter)/total_program_time;
	int rounded_fps = round(float(frame_counter)/total_program_time);
	string file_path = folder_path + "/Log.txt";
	ofstream infofile(file_path);
	infofile << "Average film speed: " << average_fps << endl;
	infofile << "To make video from images type in terminal: \n\n";
	infofile << "avconv -r " << rounded_fps << " -f image2 -i " << folder_path << "/img%07d.jpg -r " << rounded_fps << " -vcodec libx264 -crf 20 " << folder_path << "/video.mp4";
	infofile.close();
	camera.release();
	system("sudo sleep 2; sudo shutdown -h now");
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
int main()
{	
	int device_I2C_id = open_I2C();
	adress_I2C(0x40, device_I2C_id);
	ina_219_calibration(device_I2C_id);
	ina_219_configuration(device_I2C_id);
	cout << supply_voltage_calc(device_I2C_id) << endl;
	if(wiringPiSetup() == -1) 
	{ 
		cout << "Error: Cannot start wiringPi!" << endl; 
		return -1; 
	} 
		
	gpio_define(1,INPUT,PUD_DOWN);
	gpio_define(2,INPUT,PUD_DOWN);
	gpio_define(7,OUTPUT,0);
	IR_filter_control();
	string folder_path = folder_path_creator();
	int servo_controller_id = serial_open();
	if(servo_controller_id == -1) 
	{ 
		cout << "Error: Cannot start serial connection!"  << endl; 
		return -1; 
	} 
	float total_program_time = 0;
	int frame_counter = 0;
	VideoCapture camera(0);
	
	if(!check_supply_voltage(device_I2C_id))
		end_program(servo_controller_id, camera, frame_counter, total_program_time, folder_path);
		
	servo_on_off(servo_controller_id, 1, 1);
	servo_on_off(servo_controller_id, 1, 2);
	initial_position(servo_controller_id, 50);
	
	Ptr<Tracker> tracker;
	
	if (!camera.isOpened()) {
		cout << "Error: Cannot start camera" << endl;
		return -1;
	}
	camera.set(CAP_PROP_BUFFERSIZE, 1);
	
	Mat captured_frame_1;
	Mat captured_frame_2;
	bool tracker_success = false;
	bool servo_1_motion = false;
	bool servo_2_motion = false;
	bool stop = true;
	int servo_1_destination = -1;
	int servo_2_destination = -1;
	float servo_1_stop_delay = 0;
	float servo_2_stop_delay = 0;
	float blocked_tracker_time = 0;
	float object_search_time = 0;
	for(int i=0; i<2;i++)
	{
		camera >> captured_frame_1;
		create_image_name(frame_counter, captured_frame_1, folder_path);
		camera >> captured_frame_2;
		create_image_name(frame_counter, captured_frame_2, folder_path);
	}
	Rect2d tracking_rectangle(captured_frame_1.size().width/2-1,captured_frame_1.size().height/2-1,2,2);
	Rect2d previous_tracking_rectangle = tracking_rectangle;
	
	while(1) 
	{	
		  IR_filter_control();
		  int frame_time_start = getTickCount();
		  int frame_time_stop = 0;
		  float time_per_frame = 0;
		  if(!tracker_success)
		  {
				int search_object_start = getTickCount();
				float movement_detection_time = 0;	
				
				tracker.release(); 
				tracker = TrackerMOSSE::create();

				Mat captured_frame_1_copy, captured_frame_2_copy;
				
				float t0;
				float t1;
				float czas;
				
				cvtColor(captured_frame_1, captured_frame_1_copy, COLOR_BGR2GRAY);
				cvtColor(captured_frame_2, captured_frame_2_copy, COLOR_BGR2GRAY);

				int detected_biggest_object = -1;

				if(!servo_1_motion && !servo_2_motion)
				{
					vector<DetectedArea> detected_objects;
					Mat difference_image;
					Mat thresholded_image;
					
					absdiff(captured_frame_1_copy, captured_frame_2_copy, difference_image);
					blur(difference_image, difference_image, Size(3,3));
					threshold(difference_image, thresholded_image, 20, 255.0, THRESH_BINARY);
					dilate(thresholded_image, thresholded_image, getStructuringElement(MORPH_RECT, Size(3, 3)));
					
					vector<vector<Point> > contours;
					findContours(thresholded_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
					int biggest_object_counter = 0;
					for (auto &contour_to_check : contours) 
					{
						DetectedArea object_to_check(contour_to_check);
						if (object_to_check.bounding_rect_aspect_ratio >= 0.1 &&
							object_to_check.bounding_rect_aspect_ratio <= 10 &&
							object_to_check.bounding_rectangle.width > 15 &&
							object_to_check.bounding_rectangle.height > 15) 
						{
							detected_objects.push_back(object_to_check);
							if (biggest_object_counter == 0) detected_biggest_object = biggest_object_counter;
							if(biggest_object_counter > 0 && object_to_check.bounding_rectangle.area() > detected_objects[detected_biggest_object].bounding_rectangle.area()) detected_biggest_object = biggest_object_counter;
							biggest_object_counter++;
						}
					}

					if (detected_biggest_object != -1) 
						tracking_rectangle = detected_objects[detected_biggest_object].bounding_rectangle;
				}

				int servo_1_position = check_position(servo_controller_id,1), servo_2_position = check_position(servo_controller_id,2);
				if(!servo_1_motion && tracking_rectangle.x < 70 && (tracking_rectangle.x + tracking_rectangle.width) < captured_frame_2_copy.size().width-250 && servo_1_position < 5700)
					follow_object(servo_1_destination, servo_1_position, 900, servo_controller_id, 1, 80, servo_1_motion, servo_1_stop_delay, tracking_rectangle, captured_frame_1_copy,stop);
				else
					if(!servo_1_motion && tracking_rectangle.x > 250 && (tracking_rectangle.x + tracking_rectangle.width) > captured_frame_2_copy.size().width-70 && servo_1_position > 2300)
						follow_object(servo_1_destination, servo_1_position, -900, servo_controller_id, 1, 80, servo_1_motion, servo_1_stop_delay, tracking_rectangle, captured_frame_1_copy,stop);
				if(!servo_2_motion && tracking_rectangle.y < 50 && (tracking_rectangle.y + tracking_rectangle.height) < captured_frame_2_copy.size().height-190 && servo_2_position < 5500)
					follow_object(servo_2_destination, servo_2_position, 600, servo_controller_id, 2, 80, servo_2_motion, servo_2_stop_delay, tracking_rectangle, captured_frame_1_copy,stop);
				else
					if(!servo_2_motion && tracking_rectangle.y > 190 && (tracking_rectangle.y + tracking_rectangle.height) > captured_frame_2_copy.size().height-50 && servo_2_position > 3000)
						follow_object(servo_2_destination, servo_2_position, -600, servo_controller_id, 2, 80, servo_2_motion, servo_2_stop_delay, tracking_rectangle, captured_frame_1_copy,stop);
						
				if(detected_biggest_object == -1 && !servo_1_motion && !servo_2_motion)
				{
					if(object_search_time > 2 && object_search_time <= 6)
						move_servo_to_check(servo_controller_id, 4000, 4000, servo_1_motion, servo_2_motion, servo_1_stop_delay, servo_2_stop_delay, servo_1_destination, servo_2_destination, stop);
					else
						if(object_search_time > 6 && object_search_time <= 10)
							move_servo_to_check(servo_controller_id, 6000, 5500, servo_1_motion, servo_2_motion, servo_1_stop_delay, servo_2_stop_delay, servo_1_destination, servo_2_destination, stop);
						else
							if(object_search_time > 10 && object_search_time <= 14)
								move_servo_to_check(servo_controller_id, 2000, 2500, servo_1_motion, servo_2_motion, servo_1_stop_delay, servo_2_stop_delay, servo_1_destination, servo_2_destination, stop);
							else
								if(object_search_time > 14 && object_search_time <= 18)
									move_servo_to_check(servo_controller_id, 6000, 2500, servo_1_motion, servo_2_motion, servo_1_stop_delay, servo_2_stop_delay, servo_1_destination, servo_2_destination, stop);
								else
									if(object_search_time > 18 && object_search_time <= 22)
										move_servo_to_check(servo_controller_id, 2000, 5500, servo_1_motion, servo_2_motion, servo_1_stop_delay, servo_2_stop_delay, servo_1_destination, servo_2_destination, stop);
									else
										if(object_search_time > 22)
											object_search_time = 2;
										
				}
				else
					if(detected_biggest_object != -1)
						object_search_time = 0;
							
				if((servo_1_motion && servo_1_position == servo_1_destination) || (servo_2_motion && servo_2_position == servo_2_destination))
				{
					if(!stop)
					{
						stop = true;
						servo_1_stop_delay = 0;
						servo_2_stop_delay = 0;
					}
					if(servo_1_motion && servo_2_motion)
					{
						if(servo_1_stop_delay > 0.2 && servo_2_stop_delay > 0.2)
						{
							servo_1_motion = false;
							servo_2_motion = false;
							camera >> captured_frame_2;
							servo_1_stop_delay = 0;
							servo_2_stop_delay = 0;
						}
					}
					else
						if(servo_1_motion && !servo_2_motion)
						{
							if(servo_1_stop_delay > 0.2)
							{
								servo_1_motion = false;
								camera >> captured_frame_2;
								servo_1_stop_delay = 0;
							}
						}
						else
							if(!servo_1_motion && servo_2_motion)
								if(servo_2_stop_delay > 0.2)
								{
									servo_2_motion = false;
									camera >> captured_frame_2;
									servo_2_stop_delay = 0;
								}
				}
				create_image_name(frame_counter, captured_frame_2, folder_path);
								
				if(detected_biggest_object != -1 && !servo_1_motion && !servo_2_motion)
				{
					Mat image_to_track;
					int rectangle_area = tracking_rectangle.area();
					int max_tracking_area = 15000;
					
					if(rectangle_area > max_tracking_area*4)
					{
						float multiplier = float(rectangle_area)/max_tracking_area;  // describe how many times tracking_rectangle is bigger than max_tracking_area to rescale
						tracking_rectangle.x = tracking_rectangle.x/2 + tracking_rectangle.width/4 - tracking_rectangle.width/(4*sqrt(multiplier));
						tracking_rectangle.y = tracking_rectangle.y/2 + tracking_rectangle.height/4 - tracking_rectangle.height/(4*sqrt(multiplier));
						tracking_rectangle.width = tracking_rectangle.width/(2*sqrt(multiplier));
						tracking_rectangle.height = tracking_rectangle.height/(2*sqrt(multiplier));
					}
					else
					{
						tracking_rectangle.x = tracking_rectangle.x/2;
						tracking_rectangle.y = tracking_rectangle.y/2;
						tracking_rectangle.width = tracking_rectangle.width/2;
						tracking_rectangle.height = tracking_rectangle.height/2;
					}
					resize(captured_frame_2, image_to_track, Size(), 0.5, 0.5,INTER_NEAREST);
					tracker->init(image_to_track, tracking_rectangle);
					tracker_success = true;
				}
				
				captured_frame_1 = captured_frame_2.clone();
				camera >> captured_frame_2;
				
				int search_object_stop = getTickCount();
				movement_detection_time = float(search_object_stop-search_object_start)/getTickFrequency();
				servo_1_stop_delay = servo_1_stop_delay + movement_detection_time;
				servo_2_stop_delay = servo_2_stop_delay + movement_detection_time;
				object_search_time = object_search_time + movement_detection_time;
			}
		else
		{
			object_search_time = 0;
			int tracker_time_start = getTickCount();
			int tracker_time_stop = 0;
			previous_tracking_rectangle = tracking_rectangle;
			Mat captured_frame_2_copy;
			resize(captured_frame_2, captured_frame_2_copy, Size(), 0.5, 0.5, INTER_NEAREST);
			tracker_success = tracker->update(captured_frame_2_copy, tracking_rectangle);
	        int tracking_rectangle_width = tracking_rectangle.x + tracking_rectangle.width;
	        int tracking_rectangle_height = tracking_rectangle.y + tracking_rectangle.height;
	        int servo_1_position = check_position(servo_controller_id,1);
	        if(tracker_success && ((tracking_rectangle.x < 50 && tracking_rectangle_width < (captured_frame_2_copy.size().width-150) && servo_1_position < 6500) || (tracking_rectangle_width > (captured_frame_2_copy.size().width-50) && tracking_rectangle.x > 150 && servo_1_position > 1500)))
	        {
				if(tracking_rectangle.x < 50)
					move_servos(servo_controller_id,1,6500,30);
				else
					if(tracking_rectangle_width > (captured_frame_2_copy.size().width - 50))
						move_servos(servo_controller_id,1,1500,30);
			}
			else
				move_servos(servo_controller_id,1,servo_1_position,0);
			int servo_2_position = check_position(servo_controller_id,2);
			if(tracker_success && ((tracking_rectangle.y < 50 && tracking_rectangle_height < (captured_frame_2_copy.size().height-150) && servo_2_position < 6000) || (tracking_rectangle_height > (captured_frame_2_copy.size().height-50) && tracking_rectangle.y > 150 && servo_2_position > 2000)))
	        {
				if(tracking_rectangle.y < 50)
					move_servos(servo_controller_id,2,6000,20);
				else
					if(tracking_rectangle_height > (captured_frame_2_copy.size().height - 50))
						move_servos(servo_controller_id,2,2500,20);
			}
			else
				move_servos(servo_controller_id,2,servo_2_position,0);
						
			if (!tracker_success)
			{
				int position = check_position(servo_controller_id,1);
				move_servos(servo_controller_id,1,position,0);
				servo_1_destination = position;
				servo_1_motion = false;
				position = check_position(servo_controller_id,2);
				move_servos(servo_controller_id,2,position,0);
				servo_2_destination = position;
				servo_2_motion = false;
			}
			
			create_image_name(frame_counter, captured_frame_2, folder_path);

			captured_frame_1 = captured_frame_2.clone();
			camera >> captured_frame_2;

			tracker_time_stop = getTickCount();
			blocked_tracker_time = blocked_tracker_time + float(tracker_time_stop-tracker_time_start)/getTickFrequency();
		    
			if(previous_tracking_rectangle != tracking_rectangle && tracker_success)
				blocked_tracker_time = 0;
			else
				if(blocked_tracker_time > 1 && tracker_success)
				{
					tracker_success = false;
					blocked_tracker_time = 0;
				}
			
		}
			frame_time_stop = getTickCount();
			time_per_frame = float(frame_time_stop-frame_time_start)/getTickFrequency();
			total_program_time = total_program_time + time_per_frame;
			frame_time_stop = 0;
			frame_time_start = 0;
			cout << "Time per frame:  "<< time_per_frame << " sec" << endl;
			waitKey(1);
			if(!check_supply_voltage(device_I2C_id))
				end_program(servo_controller_id, camera, frame_counter, total_program_time, folder_path);
			if(digitalRead(2) == HIGH || total_program_time > 110)
				end_program(servo_controller_id, camera, frame_counter, total_program_time, folder_path);

			
			
	}
	end_program(servo_controller_id, camera, frame_counter, total_program_time, folder_path);
	return(0);
}
