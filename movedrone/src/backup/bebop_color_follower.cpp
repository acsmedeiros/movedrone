#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <string>
#include "BoundingBox.h"
#include "BoundingBoxes.h"
#include "pid.h"
#include <stdio.h>

static const std::string OPENCV_ORIGINAL = "Bebop camera window";		//Original image


const double max_obj_area = 100000; 						//Maximum area reference of the object
const double min_obj_area = 690;						//Minimum area reference of the object
const double bebop_velocity = 0.07;						//Bebop general velocity
const double bebop_turn_velocity = 0.42;					//Bebop general turn velocity
const double area_distance = 90000;						//The area determines the proximity of the camera to the object/noise
//boundinBoxArea								//Holds the general area of the bounding box of the recognized object

cv::Point2f drone_center;							//Represents the center of the image, also the Bebop's nose

cv_bridge::CvImagePtr cv_original;						//Original image container


int boundingBoxMiddleX, boundingBoxMiddleY, boundinBoxArea, boundingBoxMinY = 0; //Target Position && Area
bool keyk = false;

bool no_object = true;								//No Tracked object present

bool exit_from_cv = false;							//Variable to indicate to exit from OpenCV to ROS
int tracking_system_armed = 0;							//0 - Drone hovers, 1 - Drone move to the target

//AutoPicker HSV
bool auto_detect_hsv = false;
int pX,pY = 0;

std_msgs::Empty take_off,land;							//Variable to take_off and land
geometry_msgs::Twist cmd_vel_bebop;						//Variable that stores the linear velocity commands of Bebop

ros::Publisher takeoff_bebop;							//Sends the message to make the drone to take off
ros::Publisher land_bebop;							//Sends the message to make the drone to land
ros::Publisher cmd_vel_pub_bebop;						//Sends the message to move the drone


bool displayTargetAcquiredMsg = false;




int Area_id = 640 * 480 * 0.20; //# 20% of pixel area (for x)
int K_x = 1; //#vel_x parameter
int sum_error_x = 0;
int sum_error_y = 0;
int sum_error_z = 0;
int vel_x_past = 0;
int vel_y_past = 0;
int vel_z_past = 0;

//#parameters to tune
int Kp_fb = 0.2; //#proportional gain FORWARD/BACKWARD
int Kd_fb = 1.5; //#derivative gain FORWARD/BACKWARD
int Ki_fb = 1; //#integral gain FORWARD/BACKWARD
int Kp_lr = 0.2; //#proportional gain LEFT/RIGHT
int Kd_lr = 1.5; //#derivative gain LEFT/RIGHT
int Ki_lr = 1; //#integral gain LEFT/RIGHT
int Kp_ud= 0.2 ;//#proportional gain UP/DOWN
int Kd_ud = 1.5 ;//#derivative gain UP/DOWN
int Ki_ud = 1; //#integral gain UP/DOWN

int aproximation = 0.50; // aproximation to the window (percentage of the image area taken by the bounding box)

double error_x=0;
double error_y=0;
double error_z=0;

double vel_x=0;
double vel_y=0;
double vel_z=0;

double last_error_x=0;
double output_x=0;

double last_error_y=0;
double output_y=0;

double last_error_z=10;
double output_z=0;

void turn()
{
	cmd_vel_bebop.linear.x = 0;						//Turns the drone without translation
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
}
void hover()
{
	cmd_vel_bebop.linear.x = 0;						//Puts the drone in HOVER
	cmd_vel_bebop.linear.y = 0;
	cmd_vel_bebop.linear.z = 0;
	cmd_vel_bebop.angular.z = 0;

	cmd_vel_pub_bebop.publish(cmd_vel_bebop);				//Load the order to hover
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
	auto_detect_hsv = true;
	pX = x;
	pY = y;
     }
}


void c_drone(cv::Mat& image)
{
	cv::Point2f center;
	center.x = image.cols / 2;
	center.y = image.rows / 2;
	Area_id =image.cols*image.rows* aproximation;//50% of the image
	cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);  //cor da bola da mira do drone
	drone_center = center;
}

//############################################################################################################################################

void move_drone_wPID(cv::Mat& image){  //PID PID PID PID PID PID PID PID PID PID PID PID PID PID PID

			if (no_object){
				boundingBoxMiddleX, boundingBoxMiddleY, boundinBoxArea, boundingBoxMinY = 0;
			}

			if (boundingBoxMiddleX >= 0 && boundingBoxMiddleY >= 0)
			{
						error_y = drone_center.x - boundingBoxMiddleX; //# LEFT/RIGHT ##error is the difference between drone center and bounding box center #the y of the drone is the x of the image
						//#error_z = Cz_center - Cz //# UP/DOWN
						error_z = drone_center.y - boundingBoxMinY; //#so drone will fly higher  #the z of the drone is the y of the image
						error_x = Area_id - boundinBoxArea; //#FORWARD/BACKWARD #the x of the drone.

						vel_y = error_y/320;
						if (vel_y >= 1){ vel_y = 1;}
						else if (vel_y <= -1){ vel_y = -1 ;}


						vel_z = error_z/320;
						if (vel_z >= 1){ vel_z = 1; }
						else if(vel_z <= -1){ vel_z = -1; }

						vel_x = (error_x*K_x)/Area_id;
						if (vel_x >= 1){ vel_x = 1; }
						else if (vel_x <= -1){ vel_x = -1; }

						//#PID X AXES
						sum_error_x = sum_error_x + vel_x_past;
						last_error_x = vel_x_past;
						output_x = Kp_fb * vel_x + Kd_fb * (vel_x - last_error_x) + Ki_fb * sum_error_x;
						vel_x_past=vel_x;

						//#PID Y AXES
						sum_error_y = sum_error_y + vel_y_past;
						last_error_y = vel_y_past;
						output_y = Kp_lr * vel_y + Kd_lr * (vel_y - last_error_y) + Ki_lr * sum_error_y;
						vel_y_past=vel_y;

						//#PID Z AXES
						sum_error_z = sum_error_z + vel_z_past;
						last_error_z = vel_z_past;
						output_z = Kp_lr * vel_z + Kd_lr * (vel_z - last_error_z) + Ki_lr * sum_error_z;
						vel_z_past=vel_z;

						//#SEND COMMANDS

						cmd_vel_bebop.linear.x = 0.5*output_x;
						cmd_vel_bebop.linear.y = 0.01*output_y;
						cmd_vel_bebop.linear.z = 0.5*output_z;

						cmd_vel_bebop.angular.x = 0;
						cmd_vel_bebop.angular.y = 0;
						cmd_vel_bebop.angular.z = 0;

						std::cout << "bcmd_vel_bebop.linear.x: "<< std::endl;
						std::cout << cmd_vel_bebop.linear.x << std::endl;
						std::cout << "---------------------------"<< std::endl;
						std::cout << "bcmd_vel_bebop.linear.y: "<< std::endl;
						std::cout << cmd_vel_bebop.linear.y << std::endl;
						std::cout << "---------------------------"<< std::endl;
						std::cout << "bcmd_vel_bebop.linear.z: "<< std::endl;
						std::cout << cmd_vel_bebop.linear.z << std::endl;
						//std::cout << "\033[2J\033[1;1H"<< std::endl;     // clear terminal

						if(tracking_system_armed)	{
								if (!no_object){
								     cv::circle(image, cv::Point(boundingBoxMiddleX, boundingBoxMiddleY), 8, cv::Scalar(0, 255, 0), -1); // mudando aqui pra pegar o ponto medio do bounding box
								}
						}
						if(!tracking_system_armed){
								if (!no_object){
								     cv::circle(image, cv::Point(boundingBoxMiddleX, boundingBoxMiddleY), 8, cv::Scalar(0, 255, 255), -1); // mudando aqui pra pegar o ponto medio do bounding box
							  }
					  }

			}//if (boundingBoxMiddleX >= 0 && boundingBoxMiddleY >= 0)

			if(tracking_system_armed){
				int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
				double dblFontScale = 1;
				int intFontThickness = 2;
				cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
				if(boundinBoxArea < area_distance && !no_object)
				{
					cmd_vel_pub_bebop.publish(cmd_vel_bebop);
					cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
				}
				else if(!(boundinBoxArea < area_distance) && !no_object){turn();} // ja ta perto suficiente, agr fica so calibrando a mira
				else if(!(boundinBoxArea < area_distance) && no_object){hover();}
			}
			else{	hover();}
}

//############################################################################################################################################







void move_drone(cv::Mat& image)
{
  if (no_object){
		boundingBoxMiddleX, boundingBoxMiddleY, boundinBoxArea = 0;
	}

	if (boundingBoxMiddleX >= 0 && boundingBoxMiddleY >= 0)
	{
		// std::cout << "###############################################"<< std::endl;
    // std::cout << "baloonX - drone_centerX: "<< std::endl;
		// std::cout << boundingBoxMiddleX-(drone_center.x)<< std::endl;
		// std::cout << "###############################################"<< std::endl;
		// std::cout << "ballonY - drone_centerY: "<< std::endl;
		// std::cout << boundingBoxMiddleY- (drone_center.y)<< std::endl;
		// std::cout << "###############################################"<< std::endl;

		if (boundingBoxMiddleX < drone_center.x)
		{
			if (drone_center.x-boundingBoxMiddleX>100){
				cmd_vel_bebop.angular.z = 0.22;
				//std::cout << "Move Left with vel 0.42" << std::endl;
			}else if (drone_center.x-boundingBoxMiddleX>50){
				cmd_vel_bebop.angular.z = 0.15;
				//std::cout << "Move Left with vel 0.22" << std::endl;
			}else if (drone_center.x-boundingBoxMiddleX>25){
				cmd_vel_bebop.angular.z = 0.09;
				//std::cout << "Move Left with vel 0.09" << std::endl;
			}
		}
		if (boundingBoxMiddleX > drone_center.x)
		{
			if (boundingBoxMiddleX-drone_center.x>100){
				cmd_vel_bebop.angular.z = -0.22;
				//std::cout << "Move Right with vel 0.42" << std::endl;
			}else if (boundingBoxMiddleX-drone_center.x>50){
				cmd_vel_bebop.angular.z = -0.15;
				//std::cout << "Move Right with vel 0.22" << std::endl;
			}else if (boundingBoxMiddleX-drone_center.x>25){
				cmd_vel_bebop.angular.z = -0.09;
				//std::cout << "Move Right with vel 0.09" << std::endl;
			}
		}
		if (boundingBoxMiddleY > drone_center.y)
		{
			if (boundingBoxMiddleY-drone_center.y>50){
				cmd_vel_bebop.linear.z = -0.20;
				//std::cout << "Move Downwards with vel 0.20" << std::endl;
			}else if (boundingBoxMiddleY-drone_center.y>25){
				cmd_vel_bebop.linear.z = -0.15;
				//std::cout << "Move Downwards with vel 0.15" << std::endl;
			}else if (boundingBoxMiddleY-drone_center.y>10){
				cmd_vel_bebop.linear.z = -0.09;
				//std::cout << "Move Downwards with vel 0.09" << std::endl;
			}
		}
		if (boundingBoxMiddleY+200 < drone_center.y)
		{
			if (drone_center.y-boundingBoxMiddleY>50){
				cmd_vel_bebop.linear.z = 0.10;
				//std::cout << "Move Upwards with vel 0.20" << std::endl;
			}else if (drone_center.y-boundingBoxMiddleY>25){
				cmd_vel_bebop.linear.z = 0.09;
				//std::cout << "Move Upwards with vel 0.15" << std::endl;
			}else if (drone_center.y-boundingBoxMiddleY>10){
				cmd_vel_bebop.linear.z = 0.05;
				//std::cout << "Move Upwards with vel 0.09" << std::endl;
			}

		}
		if(tracking_system_armed)
		{
			if (!no_object){
			     cv::circle(image, cv::Point(boundingBoxMiddleX, boundingBoxMiddleY), 8, cv::Scalar(0, 255, 0), -1); // mudando aqui pra pegar o ponto medio do bounding box
      }
		}
		if(!tracking_system_armed)
		{
			if (!no_object){
			     cv::circle(image, cv::Point(boundingBoxMiddleX, boundingBoxMiddleY), 8, cv::Scalar(0, 255, 255), -1); // mudando aqui pra pegar o ponto medio do bounding box
		  }
		}

	}
	if(tracking_system_armed)
	{
		int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
		//double dblFontScale = 0.75;
		double dblFontScale = 1;
		int intFontThickness = 2;
		//cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,0), intFontThickness);
		if(boundinBoxArea < area_distance && !no_object)
		{
			cmd_vel_bebop.linear.x = bebop_velocity;
			cmd_vel_pub_bebop.publish(cmd_vel_bebop);
			//cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols-175), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		  cv::putText(image, "<Following>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
		}
		else if(!(boundinBoxArea < area_distance) && !no_object){turn();} // ja ta perto suficiente, agr fica so calibrando a mira
		else if(!(boundinBoxArea < area_distance) && no_object){hover();}
	}
	else
	{
		hover();
	}

} ////move_drone

void printToScreen(cv::Mat& image){
	if (displayTargetAcquiredMsg ==true and !no_object and !keyk){
		cv::putText(image, "Target Detected, press K to lock Target", cv::Point(0,image.cols/2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
	}
	if (boundinBoxArea!=0){
		std::string str = std::to_string(boundinBoxArea);
		//cv::putText(image, str, cv::Point(0,image.cols/2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
	}
  //std::string str2 = std::to_string(boundingBoxMiddleX);
	//cv::putText(image, str2, cv::Point(0,image.cols/2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	//Copying the image and encoding it into BGR according to opencv default settings
		cv::Mat final_img = cv_original->image.clone();			//Clone the original image
		c_drone(final_img);						//Figures out bebop's center and draws a circle
		//move_drone(final_img);						//Moves Drone
		move_drone_wPID(final_img);
		printToScreen(final_img); // check if any message need displaying
		cv::imshow(OPENCV_ORIGINAL,final_img);				//Show the original image
		int key = cv::waitKey(30);					//Contains the key value
		if(key == 27)							//Press ESC to exit
		{
			exit_from_cv = true;
			land_bebop.publish(land);
		}
		if(key == 'k')
		{
			keyk = !keyk;
			tracking_system_armed = 1 - tracking_system_armed;

			if(tracking_system_armed)
			{
				ROS_INFO("TRACKING SYSTEM ARMED!!!");
			}
			else
			{
				ROS_INFO("TRACKING SYSTEM DISARMED!!!");
			}
		}
		if(key == ' ')
		{
			ROS_INFO("TAKE-OFF!!!");
			takeoff_bebop.publish(take_off);
		}
		if(key == 'b')
		{
			ROS_INFO("LAND!!!");
			land_bebop.publish(land);
		}


	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());			//Handleling the Exception
		return;
	}
}

void boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	  displayTargetAcquiredMsg = true;
    boundingBoxMiddleX = (((msg->bounding_boxes[0].xmax)-(msg->bounding_boxes[0].xmin))/2)+(msg->bounding_boxes[0].xmin);
		boundingBoxMiddleY = (((msg->bounding_boxes[0].ymax)-(msg->bounding_boxes[0].ymin))/2)+(msg->bounding_boxes[0].ymin);
		boundinBoxArea = ((msg->bounding_boxes[0].xmax)-(msg->bounding_boxes[0].xmin))*((msg->bounding_boxes[0].ymax)-(msg->bounding_boxes[0].ymin));
		boundingBoxMinY = (msg->bounding_boxes[0].ymin);

		//std::cout<<"Bouding Boxes (header):" << msg->header <<std::endl;
    //std::cout<<"Bouding Boxes (image_header):" << msg->image_header <<std::endl;
    //std::cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[0].Class <<std::endl;
    //std::cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[0].xmin <<std::endl;
    //std::cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[0].xmax <<std::endl;
    //std::cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[0].ymin <<std::endl;
    //std::cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[0].ymax <<std::endl;
    //std::cout << "\033[2J\033[1;1H";     // clear terminal
}




void count_objCallback(const std_msgs::Int8::ConstPtr& msg){
	  //ROS_INFO("I heard: [%i]", msg->data);
    if (msg->data==0){no_object = true;}
		if (msg->data>0){no_object = false;}
		std::cout << no_object <<std::endl;
		std::cout << "\033[2J\033[1;1H";     // clear terminal
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"bebop_color_follower");				//Initialize the ROS node
	ros::NodeHandle nh_;							//Create the Node Handler
	image_transport::ImageTransport it_(nh_);				//Special message to contain the image
	image_transport::Subscriber image_sub_;					//Special subscriber to obtain the image
	//image_sub_= it_.subscribe("/usb_cam/image_raw",1,imageCallback);	//Subscribe to the Bebop image topic
	image_sub_= it_.subscribe("/bebop/image_raw",1,imageCallback);
	ros::Subscriber cood_sub = nh_.subscribe("/darknet_ros/bounding_boxes",1,boxCallback);
	ros::Subscriber count_obj = nh_.subscribe("/darknet_ros/found_object",1,count_objCallback);
	takeoff_bebop = nh_.advertise<std_msgs::Empty>("/bebop/takeoff",1000);		//Publish data to the take-off topic
	land_bebop = nh_.advertise<std_msgs::Empty>("/bebop/land",1000);		//Publish data to the land topic
	cmd_vel_pub_bebop = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Publish data to the movement topic
	cv::namedWindow(OPENCV_ORIGINAL);					//Create window to visualize the original image
	cv::setMouseCallback(OPENCV_ORIGINAL, CallBackFunc, NULL);		//Receive info from the mouse
	//TODO Resolve Segmentation Fault issue due to publishers
	while(nh_.ok())								//Asks if the node still alive
	{
		ros::spinOnce();						//Refresh ROS's topics once
		if(exit_from_cv){break;}					//Exit using while focus any opencv window (ESC key)
	}
	ROS_INFO("EXITING...");
	cv::destroyWindow(OPENCV_ORIGINAL);					//Destroy Original Window

	return 0;
}
