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
#include <fstream>
#include <string>
#include "BoundingBox.h"
#include "BoundingBoxes.h"
#include "Points34.h"
#include "pid.h"
#include "pid2.h"
#include <stdio.h>

static const std::string OPENCV_ORIGINAL = "Bebop camera";

const double max_obj_area = 100000; 						//Maximum area reference of the object
const double min_obj_area = 690;						//Minimum area reference of the object
const double bebop_velocity = 0.1;						//Bebop general velocity
const double bebop_turn_velocity = 0.42;					//Bebop general turn velocity
const double area_distance = 90000;						//The area determines the proximity of the camera to the object/noise
//boundinBoxArea								//Holds the general area of the bounding box of the recognized object

int image_total_area;

cv::Point2f drone_center;							//Represents the center of the image, also the Bebop's nose

cv_bridge::CvImagePtr cv_original;						//Original image container

double targetAboveMiddleX, targetAboveMiddleY, targetAboveArea=0;
double targetBellowMiddleX, targetBellowMiddleY, targetBellowArea =0;

double targetAbove_Xmax, targetAbove_Ymax, targetAbove_Xmin, targetAbove_Ymin=0;
double targetBellow_Xmax, targetBellow_Ymax, targetBellow_Xmin, targetBellow_Ymin=0;

int numberofObjectsDetected =0;

std::string bellowObjectClass = "orange";
std::string aboveObjectClass = "fire";


bool keyk = false;


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
bool displayOrangeAcquiredMsg = false;



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


double val=100;
double inc=0;

float mypoint3x=-3;
float mypoint3y=-3;
float mypoint3c=-3;

float mypoint4x=-3;
float mypoint4y=-3;
float mypoint4c=-3;


float mypoint6x=-3;
float mypoint6y=-3;
float mypoint6c=-3;

float mypoint7x=-3;
float mypoint7y=-3;
float mypoint7c=-3;


int myTarget = 0;
cv::Point forearm34extension_p, forearm34extension_q;



//############################################################################################################################################ target 1 is the fire extinguisher plate

void move_drone_wPID_Target1(cv::Mat& image){  //PID PID PID PID PID PID PID PID PID PID PID PID PID PID PID


						// Kp -  proportional gain
		        // Ki -  Integral gain
		        // Kd -  derivative gain
		        // dt -  loop interval time
		        // max - maximum value of manipulated variable
		        // min - minimum value of manipulated variable
		        // PID( dt, max, min, Kp, Kd, Ki );

						PID pid_forward =  PID(0.001, 0.07, -0.03, 0.003, 0, 0);
						PID pid_sideways = PID(0.01, 0.07, -0.07, 0.003, 0, 0);
						PID pid_updown = PID(0.01, 0.07, -0.07, 0.01, 0, 0);

						vel_x = pid_forward.calculate(9000, targetAboveArea);

						cmd_vel_bebop.linear.x = vel_x;


						vel_z = pid_updown.calculate(drone_center.y,targetAboveMiddleY);  //# up/down ++60

						cmd_vel_bebop.linear.z = vel_z;

						vel_y = pid_sideways.calculate(drone_center.x,targetAboveMiddleX );  //# sideways
						cmd_vel_bebop.linear.y = vel_y ;

						if(tracking_system_armed)	{
							cmd_vel_pub_bebop.publish(cmd_vel_bebop);
						}


						if(tracking_system_armed)	{
							     cv::circle(image, cv::Point(targetAboveMiddleX, targetAboveMiddleY), 8, cv::Scalar(0, 255, 255), -1); // mudando aqui pra pegar o ponto medio do bounding box
									 int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
									 double dblFontScale = 1;
									 int intFontThickness = 2;
									 cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(50,255,0), intFontThickness);
									 if(targetAboveArea < 9000)
									 {
												 cv::putText(image, "<Target: One>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(0,255,255), intFontThickness);
									 }


						}else{
			  			     cv::circle(image, cv::Point(targetAboveMiddleX, targetAboveMiddleY), 8, cv::Scalar(0, 255, 0), -1); // mudando aqui pra pegar o ponto medio do bounding box

					  }


}

//############################################################################################################################################ Target 2 is the orange one, in th paper

void move_drone_wPID_Target2(cv::Mat& image){  //PID PID PID PID PID PID PID PID PID PID PID PID PID PID PID

				PID pid_forward = PID(0.001, 0.07, -0.03, 0.003, 0, 0);
				PID pid_sideways = PID(0.01, 0.07, -0.07, 0.003, 0, 0);
				PID pid_updown = PID(0.01, 0.07, -0.07, 0.01, 0, 0);

				vel_x = pid_forward.calculate(10000, targetBellowArea);

				cmd_vel_bebop.linear.x = vel_x;

				vel_z = pid_updown.calculate(drone_center.y,targetBellowMiddleY);  //# up/down ++17

				cmd_vel_bebop.linear.z = vel_z;


				vel_y = pid_sideways.calculate(drone_center.x,targetBellowMiddleX);  //# sideways
				cmd_vel_bebop.linear.y = vel_y ;


				if(tracking_system_armed)	{
					cmd_vel_pub_bebop.publish(cmd_vel_bebop);
				}



				if(tracking_system_armed)	{
						cv::circle(image, cv::Point(targetBellowMiddleX, targetBellowMiddleY), 8, cv::Scalar(0, 0, 255), -1); // mudando aqui pra pegar o ponto medio do bounding box
						int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
						double dblFontScale = 1;
						int intFontThickness = 2;
						cv::putText(image, "Tracking System ARMED", cv::Point(0,image.cols/2), intFontFace, dblFontScale, cv::Scalar(50,255,0), intFontThickness);
						if(targetBellowArea < 10000)
						{
									cv::putText(image, "<Target: Two>", cv::Point(image.rows-20,image.cols/2), intFontFace, dblFontScale, cv::Scalar(255,20,55), intFontThickness);
						}

					}else{
				 		cv::circle(image, cv::Point(targetBellowMiddleX, targetBellowMiddleY), 8, cv::Scalar(0, 255, 0), -1); // mudando aqui pra pegar o ponto medio do bounding box


					}


}

//############################################################################################################################################







void printToScreen(cv::Mat& image){
	if (myTarget!=0 and !keyk){
		cv::putText(image, ("Target Detected, press K to lock Target"), cv::Point(0,(image.cols/2)-50), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
	}
	if (targetAboveArea!=0){
		std::string str = std::to_string(targetAboveArea);
		//cv::putText(image, str, cv::Point(0,image.cols/2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
	}

}



void drawUserPointingDirectionLine(cv::Mat& img, const std::pair<cv::Point, cv::Point>& points, cv::Scalar color)
{
      //points of line segment
      cv::Point p1 = points.first;
      cv::Point p2 = points.second;

      //points of line segment which extend the segment P1-P2 to
      //the image borders.
      cv::Point p,q;

      //test if line is vertical, otherwise computes line equation
      //y = ax + b
      if (p2.x == p1.x)
      {
          p = p1;//cv::Point(p1.x, 0);
          if(p2.y<p1.y){
            q=cv::Point(p1.x,0);
          }else{
            q = cv::Point(p1.x, img.rows);
          }
      }
      else
      {
          double a = (double)(p2.y - p1.y) / (double) (p2.x - p1.x);
          double b =  p1.y - a*p1.x;

          if (p1.x > p2.x){
            p = cv::Point(0, b);
            q = p1;
          }else{
            p = p1;
            q = cv::Point(img.cols, a*img.cols + b);
          }
          //clipline to the image borders. It prevents a known bug on OpenCV
          //versions 2.4.X when drawing
          //cv::clipLine(cv::Size(img.rows, img.cols), p, q);
      }
      cv::line(img, p, q, color, 1);
			forearm34extension_p=p;
			forearm34extension_q=q;
}




		// Determines if the lines AB and CD intersect.
			bool LinesIntersect(cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D)		{
				cv::Point2f CmP = cv::Point2f(C.x - A.x, C.y - A.y);
				cv::Point2f r = cv::Point2f(B.x - A.x, B.y - A.y);
				cv::Point2f s = cv::Point2f(D.x - C.x, D.y - C.y);

				float CmPxr = CmP.x * r.y - CmP.y * r.x;
				float CmPxs = CmP.x * s.y - CmP.y * s.x;
				float rxs = r.x * s.y - r.y * s.x;

				if (CmPxr == 0.0f)
				{
					// Lines are collinear, and so intersect if they have any overlap

					return ((C.x - A.x < 0.0f) != (C.x - B.x < 0.0f))	or ((C.y - A.y < 0.0f) != (C.y - B.y < 0.0f));
				}

				if (rxs == 0.0f)
					return false; // Lines are parallel.

				float rxsr = 1.0f / rxs;
				float t = CmPxs * rxsr;
				float u = CmPxr * rxsr;

				return (t >= 0.0f) and (t <= 1.0f) and (u >= 0.0f) and (u <= 1.0f);
			}

bool userPointingLine_isIntersectingTarget (cv::Mat& img, float x1, float y1, float x2, float y2, float minX, float minY, float maxX, float maxY) {



    // // Completely outside.
    if ((forearm34extension_p.x <= minX && forearm34extension_q.x <= minX) || (forearm34extension_p.y <= minY && forearm34extension_q.y <= minY) || (forearm34extension_p.x >= maxX && forearm34extension_q.x >= maxX) || (forearm34extension_p.y >= maxY && forearm34extension_q.y >= maxY))
        return false;

		bool test1, test2, test3, test4;


		test1=LinesIntersect(cv::Point2f(minX,minY),cv::Point2f(maxX,minY),forearm34extension_q,forearm34extension_p);
		test2=LinesIntersect(cv::Point2f(minX,minY),cv::Point2f(minX,maxY),forearm34extension_q,forearm34extension_p);
		test3=LinesIntersect(cv::Point2f(maxX,maxY),cv::Point2f(maxX,minY),forearm34extension_q,forearm34extension_p);
		test4=LinesIntersect(cv::Point2f(maxX,maxY),cv::Point2f(minX,maxY),forearm34extension_q,forearm34extension_p);


		if (test1 or test2 or test3 or test4) return true;
		else  return false;
}

void draw_gesture_pointing_line(cv::Mat& image){

    cv::Point2f centro3, centro4, centro6, centro7;


    // float p3x, p3y, p3c, p4x, p4y, p4c;
    // std::ifstream myfile34 ("/home/anna/.ros/filepoints34.txt");
    // if (myfile34.is_open())
    // {
    //   myfile34 >>  p3x >> p3y >> p3c >> p4x >> p4y>> p4c;
    //
    //   ROS_INFO("%f, %f, %f\n%f, %f, %f\n\n",p3x, p3y, p3c, p4x, p4y, p4c);
    //
    //   myfile34.close();
    // } else ROS_INFO("file didnot open");
    //
    // float p6x, p6y, p6c, p7x, p7y, p7c;
    // std::ifstream myfile67 ("/home/anna/.ros/filepoints67.txt");
    // if (myfile67.is_open())
    // {
    //   myfile67 >>  p6x >> p6y >> p6c >> p7x >> p7y>> p7c;
    //
    //   //ROS_INFO("%f, %f, %f\n%f, %f, %f\n\n",p6x, p6y, p6c, p7x, p7y, p7c);
    //
    //   myfile67.close();
    // } else ROS_INFO("file didnot open");
    //
    //
    // centro3.x = p3x;
    // centro3.y = p3y;
    // centro4.x = p4x;
    // centro4.y = p4y;

    centro3.x = mypoint3x;
    centro3.y = mypoint3y;

    centro4.x = mypoint4x;
    centro4.y = mypoint4y;

    centro6.x = mypoint6x;
    centro6.y = mypoint6y;

    centro7.x = mypoint7x;
    centro7.y = mypoint7y;

    cv::circle(image, centro3, 3, cv::Scalar(255, 255, 0), 1);  //ponto 3

    cv::circle(image, centro4, 3, cv::Scalar(255, 255, 0), 1);  //ponto 4

    cv::circle(image, centro6, 3, cv::Scalar(100, 255, 100), 1);  //ponto 6

    cv::circle(image, centro7, 3, cv::Scalar(100, 255, 100), 1);  //ponto 7

    line(image, centro3,centro4,cv::Scalar(200,200,10), 1,1,0);
    line(image, centro6,centro7,cv::Scalar(100,200,50), 1,1,0);


    drawUserPointingDirectionLine(image, std::pair<cv::Point, cv::Point>(centro3,centro4), cv::Scalar(100, 255, 100));

    bool is_inside_T1 = userPointingLine_isIntersectingTarget(image,centro3.x, centro3.y, centro4.x, centro4.y,targetAbove_Xmin, targetAbove_Ymin, targetAbove_Xmax,targetAbove_Ymax);
    bool is_inside_T2 = userPointingLine_isIntersectingTarget(image,centro3.x, centro3.y, centro4.x, centro4.y,targetBellow_Xmin, targetBellow_Ymin, targetBellow_Xmax,targetBellow_Ymax);

		//ROS_INFO("--");
		//ROS_INFO("%f", targetBellow_Xmin);
		if (is_inside_T1){//is_inside_T1
      //ROS_INFO("----------------------------------------=== 111111 ===--------------------------");
      cv::Point2f pt1,pt2;
			pt1.x=targetAbove_Xmin;
      pt1.y=targetAbove_Ymin;
      pt2.x=targetAbove_Xmax;
      pt2.y=targetAbove_Ymax;
      cv::rectangle(image, pt1, pt2, cv::Scalar(0, 255, 0), 2, 8, 0);
      myTarget=1;
    }
    if (is_inside_T2){//is_inside_T2
      //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$     2222222    $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
      myTarget=2;
      cv::Point2f pt1,pt2;
      pt1.x=targetBellow_Xmin;
      pt1.y=targetBellow_Ymin;
      pt2.x=targetBellow_Xmax;
      pt2.y=targetBellow_Ymax;
      cv::rectangle(image, pt1, pt2, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    // if (!is_inside_T1 && !is_inside_T2){
    //   //ROS_INFO("    (                          -Not Pointing to Any Target-                        )   ");
    //   myTarget=0;
    // }




} // end of draw_gesture_pointing_line

void c_drone(cv::Mat& image)
{
	cv::Point2f center;
	center.x = image.cols / 2;
	center.y = image.rows / 2;
	Area_id =image.cols*image.rows* aproximation;//50% of the image
	image_total_area=image.cols*image.rows;
	//cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);  //cor da bola da mira do drone
	drone_center = center;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) //main stuff
{
	try
	{
		cv_original = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	//Copying the image and encoding it into BGR according to opencv default settings
		cv::Mat final_img = cv_original->image.clone();			//Clone the original image
		c_drone(final_img);						//Figures out bebop's center and maybe draws a circle (optional, default: not drawing)
    draw_gesture_pointing_line(final_img);
    if(myTarget==0) cv::putText(final_img, "Please Choose a Target", cv::Point(0,final_img.cols/2), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
    //move_drone(final_img);						//Moves Drone

		ROS_INFO("my Target: %d", myTarget);
		if (myTarget==2) move_drone_wPID_Target2(final_img);
    if (myTarget==1) move_drone_wPID_Target1(final_img);

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


	  //displayTargetAcquiredMsg = true;
  for (int i=0;i<numberofObjectsDetected;i++){


				if (msg->bounding_boxes[i].Class==aboveObjectClass){


								targetAboveMiddleX = (((msg->bounding_boxes[i].xmax)-(msg->bounding_boxes[i].xmin))/2)+(msg->bounding_boxes[i].xmin);
								targetAboveMiddleY = (((msg->bounding_boxes[i].ymax)-(msg->bounding_boxes[i].ymin))/2)+(msg->bounding_boxes[i].ymin);
								targetAboveArea = ((msg->bounding_boxes[i].xmax)-(msg->bounding_boxes[i].xmin))*((msg->bounding_boxes[i].ymax)-(msg->bounding_boxes[i].ymin));

								targetAbove_Xmax = (msg->bounding_boxes[i].xmax);
								targetAbove_Ymax = (msg->bounding_boxes[i].ymax);
								targetAbove_Xmin = (msg->bounding_boxes[i].xmin);
								targetAbove_Ymin = (msg->bounding_boxes[i].ymin);


								//found the fire extinguisher
				}

			  if (msg->bounding_boxes[i].Class==bellowObjectClass){

								targetBellowMiddleX = (((msg->bounding_boxes[i].xmax)-(msg->bounding_boxes[i].xmin))/2)+(msg->bounding_boxes[i].xmin);
							  targetBellowMiddleY = (((msg->bounding_boxes[i].ymax)-(msg->bounding_boxes[i].ymin))/2)+(msg->bounding_boxes[i].ymin);
							  targetBellowArea = ((msg->bounding_boxes[i].xmax)-(msg->bounding_boxes[i].xmin))*((msg->bounding_boxes[i].ymax)-(msg->bounding_boxes[i].ymin));

							  targetBellow_Xmax = (msg->bounding_boxes[i].xmax);
							  targetBellow_Ymax = (msg->bounding_boxes[i].ymax);
							  targetBellow_Xmin = (msg->bounding_boxes[i].xmin);
							  targetBellow_Ymin = (msg->bounding_boxes[i].ymin);


								 // ROS_INFO("%f bellow inside", targetBellow_Xmin);
				}
		}


}



void count_objCallback(const std_msgs::Int8::ConstPtr& msg){

		//ROS_INFO("%d",msg->data);
			numberofObjectsDetected = msg->data;
}




void pointsCallback(const openpose_ros_msgs::Points34& msg){
      if (msg.x3!=0)mypoint3x=msg.x3;
      if (msg.y3!=0)mypoint3y=msg.y3;
      mypoint3c=msg.c3;

      if (msg.x4!=0)mypoint4x=msg.x4;
      if (msg.y4!=0)mypoint4y=msg.y4;
      mypoint4c=msg.c4;

      if (msg.x6!=0)mypoint6x=msg.x6;
      if (msg.y6!=0)mypoint6y=msg.y6;
      mypoint6c=msg.c6;

      if (msg.x7!=0)mypoint7x=msg.x7;
      if (msg.y7!=0)mypoint7y=msg.y7;
      mypoint7c=msg.c7;

      //if ((msg.x3==0))
      //ROS_INFO("Can you read this?");
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_drone");				//Initialize the ROS node
	ros::NodeHandle nh_;							//Create the Node Handler
	image_transport::ImageTransport it_(nh_);				//Special message to contain the image
	image_transport::Subscriber image_sub_;					//Special subscriber to obtain the image
  image_transport::Publisher pub;


  //image_sub_= it_.subscribe("/usb_cam/image_raw",1,imageCallback);	//Subscribe to the Bebop image topic
	image_sub_= it_.subscribe("/bebop/image_raw",1,imageCallback);
  pub=it_.advertise("/bebop/image", 1);
  ros::Subscriber cood_sub = nh_.subscribe("/darknet_ros/bounding_boxes",1,boxCallback);
  ros::Subscriber points = nh_.subscribe("/openpose_ros/points34", 1,pointsCallback);
	ros::Subscriber count_obj = nh_.subscribe("/darknet_ros/found_object",1,count_objCallback);
  // ros::Subscriber cood_sub2 = nh_.subscribe("/darknet_ros2/bounding_boxes",1,boxCallback2);
	// ros::Subscriber count_obj2 = nh_.subscribe("/darknet_ros2/found_object",1,count_objCallback2);
	takeoff_bebop = nh_.advertise<std_msgs::Empty>("/bebop/takeoff",1000);		//Publish data to the take-off topic
	land_bebop = nh_.advertise<std_msgs::Empty>("/bebop/land",1000);		//Publish data to the land topic
	cmd_vel_pub_bebop = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);	//Publish data to the movement topic
	cv::namedWindow(OPENCV_ORIGINAL);					//Create window to visualize the original image
	//cv::setMouseCallback(OPENCV_ORIGINAL, CallBackFunc, NULL);		//Receive info from the mouse


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
