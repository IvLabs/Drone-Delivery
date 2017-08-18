/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */


#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/WaypointList.h>
#define PIE 3.14159265
std_msgs::Float64 value;
mavros_msgs::WaypointList wpn;
std::vector<mavros_msgs::Waypoint> waypoints;
nav_msgs::Odometry po;

double x, y, z, dist;

static const std::string OPENCV_WINDOW = "Image window";
sensor_msgs::Image img;
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
img=*msg;
}

void cmp(const std_msgs::Float64::ConstPtr& ab){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	value= *ab;
}



using namespace std;
mavros_msgs::State current_state;
sensor_msgs::NavSatFix pt;
geometry_msgs::PoseStamped pos;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void locationCallback(const sensor_msgs::NavSatFix::ConstPtr& loc){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	pt= *loc;
}

void locp(const geometry_msgs::PoseStamped::ConstPtr& lp){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	pos= *lp;
}
void waypointfinder(const mavros_msgs::WaypointList::ConstPtr& wpts){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	wpn= *wpts;
	waypoints=wpn.waypoints;
		
	/* for(int i = 0; i < waypoints.size(); i++)
  {
    ROS_INFO("Waypoint loaded @ %f, %f", waypoints[i].x_lat, waypoints[i].y_long);
  }*/
}
void ang(const nav_msgs::Odometry::ConstPtr& angles){
    //ROS_INFO("locationCallback: %f", loc->latitude);
	po= *angles;
}



const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2014 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"


// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"


const char* windowName = "apriltags_demo";


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

int flag=0;

class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;
  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  int rate = 10;
      ros::Rate r;

  Serial m_serial;

public:

  // default constructor
  Demo() :
    r(rate),
    it_(nh_),
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),
    m_timing(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0)
  {}

  // changing the tag family
  void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 't':
        m_timing = true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case 'D':
        m_deviceId = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
    }

    if (argc > optind) {
      for (int i=0; i<argc-optind; i++) {
        m_imgNames.push_back(argv[optind+i]);
      }
    }
  }

  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(windowName, 1);
    }

    // optional: prepare serial port for communication with Arduino
    if (m_arduino) {
      m_serial.open("/dev/ttyACM0");
    }
  }

  void setupVideo() {

  image_sub_ = it_.subscribe("/rrbot/camera1/image_raw", 1,&imageCb);
  for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("width:%d height:%d ",img.width,img.height);
ros::spinOnce();
        r.sleep();
    }
    

  }

  void print_detection(AprilTags::TagDetection& detection) const {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, z=" << translation(0)
         << ", y=" <<-translation(1)
         << ", x=" <<-translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;
x=-translation(2)*1.25;
y=-translation(1)*1.25;
z=translation(0)*1.25;
dist= translation.norm();

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }

  void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
#if OPENCV3
	cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
#else
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
#endif
	
    double t0;
    if (m_timing) {
      t0 = tic();
    }
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    if (m_timing) {
      double dt = tic()-t0;
      cout << "Extracting tags took " << dt << " seconds." << endl;
    }

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
	if(detections.size()>0)
 	flag=1;

    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
      imshow(windowName, image); // OpenCV call
    }

    // optionally send tag information to serial port (e.g. to Arduino)
    if (m_arduino) {
      if (detections.size() > 0) {
        // only the first detected tag is sent out for now
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                                     translation, rotation);
        m_serial.print(detections[0].id);
        m_serial.print(",");
        m_serial.print(translation(0));
        m_serial.print(",");
        m_serial.print(translation(1));
        m_serial.print(",");
        m_serial.print(translation(2));
        m_serial.print("\n");
      } else {
        // no tag detected: tag ID = -1
        m_serial.print("-1,0.0,0.0,0.0\n");
      }
    }
  }

  // Load and process a single image
  void loadImages() {
    cv::Mat image;
    cv::Mat image_gray;

    for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
      image = cv::imread(*it); // load image with opencv
      processImage(image, image_gray);
      while (cv::waitKey(100) == -1) {}
    }
  }

  // Video or image processing?
  bool isVideo() {
    return m_imgNames.empty();
  }

  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void loop() {

    cv::Mat image;
    cv::Mat image_gray;

    int frame = 0;
    double last_t = tic();

while(1)
{
cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img 	, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      //return;
    }
    image=cv_ptr->image;
    processImage(image, image_gray);
     // Update GUI Window
   // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
if (cv::waitKey(3) == 27){cv::destroyAllWindows(); break; }
if(flag==1) {cv::destroyAllWindows(); 
break; 
}
ros::spinOnce();
      r.sleep();
 }
}

}; // Demo


// here is were everything begins
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "image_converter");

    Demo demo;

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub1 = nh.subscribe<std_msgs::Float64>
            ("/mavros/global_position/compass_hdg", 1, &cmp);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber location_sub=nh.subscribe("/mavros/global_position/global",1,&locationCallback);  
    ros::Subscriber local_pos=nh.subscribe("/mavros/local_position/pose",1,&locp);  
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    	ros::ServiceClient client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	ros::Subscriber waypt=nh.subscribe("/mavros/mission/waypoints",1,&waypointfinder); 
	 ros::Subscriber op=nh.subscribe("/mavros/global_position/local",1,&ang);
double x2,y2,z2,w2;
	x2=po.pose.pose.orientation.x;
	y2=po.pose.pose.orientation.y;
	z2=po.pose.pose.orientation.z;
	w2=po.pose.pose.orientation.w;
			ROS_INFO("x:%f y:%F z:%f w:%f ",x2,y2,z2,w2);
	double roll, pitch, yaw;

	roll    = std::atan2(2*(w2*x2 + y2*y2), 1 - 2*(x2*x2 + y2 + y2));
pitch = std::asin(2*(w2*y2 - y2*x2));
yaw  = std::atan2(2*(w2*y2 + x2*y2), 1 - 2*(y2*y2 + y2 + y2));

std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    //the setpoint publishing rate MUST be faster than 2Hz
	
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	//ROS_INFO("locationCallback: %f", wpn.waypoints[0].x_lat);
	mavros_msgs::WaypointPush srv2;
	mavros_msgs::Waypoint wp;

mavros_msgs::SetMode offb_set_mode;


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
	ros::Time count = ros::Time::now();
/*
std::cout << "X: " << pos.pose.position.x << ", Y: " << pos.pose.position.y<< ", Z: " <<pos.pose.position.z<< std::endl;
system("rosrun mavros mavwp load /home/anish/mission.txt");
   offb_set_mode.request.custom_mode = "AUTO.MISSION";
if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Auto enabled");
            }
sleep(5);



if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
	
sleep(35);
*/
for(int i=0; i<25; i++)
{std::cout << "X: " << pos.pose.position.x << ", Y: " << pos.pose.position.y<< ", Z: " <<pos.pose.position.z<< std::endl;
ros::spinOnce();
        rate.sleep();}
for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }

// mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

  //  mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
/*    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

 if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }*/

yaw=value.data*3.14156/180;
////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
  /*  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 2.5;
    srv_takeoff.request.latitude = pt.latitude;
    srv_takeoff.request.longitude = pt.longitude;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw =yaw;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    sleep(10);*/

ROS_INFO("Angle:%f",value.data);
  // process command line options
  demo.parseOptions(argc, argv);

  demo.setup();
 // sleep(25);

  if (demo.isVideo()) {
    cout << "Processing video" << endl;

    // setup image source, window for drawing, serial port...
    demo.setupVideo();

    // the actual processing loop where tags are detected and visualized
    demo.loop();

  } else {
    cout << "Processing image" << endl;

    // process single image
    demo.loadImages();

  }
double x1,y1,z1;

x1=pos.pose.position.x;
y1=pos.pose.position.y;
z1=pos.pose.position.z;
geometry_msgs::PoseStamped pose;
 //print local position before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        std::cout << "X: " << pos.pose.position.x << ", Y: " << pos.pose.position.y<< ", Z: " <<pos.pose.position.z<< std::endl;
        ros::spinOnce();
        rate.sleep();
    }



cout<<"\n  x="<<x<<"  y="<<y<<"  z="<<z;
ROS_INFO("Angle:%f",value.data);
double X, Y, Z, D;
int f1,f2;
double result;
  result = atan2(x,y);

if(result<0)
result=result+(3.14159265*2);
cout<<"sides = "<<y/x<<"   "<<result*180/PIE;	



D=(x*x)+(y*y);
double angactual;
angactual=result+(value.data*PIE/180);
cout<<"\n A="<<angactual;
Y=sqrt(D)*cos(angactual);
X=sqrt(D)*sin(angactual);
Z=z;
cout<<"\n  x="<<x1+X<<"  y="<<Y+y1<<"  z="<<z1;
    pose.pose.position.x =x1+X;
    pose.pose.position.y =y1+Y;
    pose.pose.position.z =z1;


    last_request = ros::Time::now();
    count = ros::Time::now();
    for(int i = 250; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();  
        }
	if(ros::Time::now() - count > ros::Duration(20.0))
	break;
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }


for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }

yaw=value.data*3.14156/180;
mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = pt.altitude;
    srv_land.request.latitude = pt.latitude;
    srv_land.request.longitude = pt.longitude;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = yaw;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
sleep(10);
    offb_set_mode.request.custom_mode = "STABILIZED";
if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("STABILIZED enabled");
            }

  return 0;
}
