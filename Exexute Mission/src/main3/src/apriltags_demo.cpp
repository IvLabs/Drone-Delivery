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
void foo()
{
system("python /home/odroid/track.py");
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
#include<thread>

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"


//const char* windowName = "apriltags_demo";


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

int flag;
int yflag=0;
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
  

  Serial m_serial;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),
    m_timing(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.275),
    m_fx(703),
    m_fy(706),
    m_px(305),
    m_py(245),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0)
  {flag=0;}

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
  /*  if (m_draw) {
      cv::namedWindow(windowName, 1);
    }*/

    // optional: prepare serial port for communication with Arduino
    if (m_arduino) {
      m_serial.open("/dev/ttyACM0");
    }
  }

  void setupVideo() {

#ifdef EXPOSURE_CONTROL
    // manually setting camera exposure settings; OpenCV/v4l1 doesn't
    // support exposure control; so here we manually use v4l2 before
    // opening the device via OpenCV; confirmed to work with Logitech
    // C270; try exposure=20, gain=100, brightness=150
for(int i=0; i<10; i++){
    m_deviceId=i;
    string video_str = "/dev/video0";
    video_str[10] = '0' + m_deviceId;
    int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

    if (m_exposure >= 0) {
      // not sure why, but v4l2_set_control() does not work for
      // V4L2_CID_EXPOSURE_AUTO...
      struct v4l2_control c;
      c.id = V4L2_CID_EXPOSURE_AUTO;
      c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
      if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
        cout << "Failed to set... " << strerror(errno) << endl;
      }
      cout << "exposure: " << m_exposure << endl;
      v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
    }
    if (m_gain >= 0) {
      cout << "gain: " << m_gain << endl;
      v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
    }
    if (m_brightness >= 0) {
      cout << "brightness: " << m_brightness << endl;
      v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
    }
    v4l2_close(device);
#endif 

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      continue;
    }
    
#if OPENCV3
	m_cap.set(cv::CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;
#else
	m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
#endif
    break;

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
         << "m, x=" << translation(2)
         << ", y=" << translation(1)
         << ", z=" << translation(0)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;
x=translation(2)*1.25;
y=translation(1)*1.25;
z=translation(0)*1.25;
if((x*100)<=100&&(x*100)>=(-100)&&(y*100)<=100&&(y*100)>=(-100))
flag=1;

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

    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
     // imshow(windowName, image); // OpenCV call
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
    ros::Rate rate(20.0);
    while (true) {

      // capture frame
      m_cap >> image;

      processImage(image, image_gray);
      cout<<endl<<"SIZE="<<waypoints.size()<<endl;
for(int i = 0; i < waypoints.size(); i++)
  {
    cout<<"\nWaypoint loaded "<<i<<" ";
	printf(waypoints[i].is_current ? "true" : "false");
	if(waypoints[3].is_current)
	{flag=1;
	yflag=1;}	
	
  }

ros::spinOnce();
rate.sleep();

      // print out the frame rate at which image frames are being processed
      frame++;
      if (frame % 10 == 0) {
        double t = tic();
        cout << "  " << 10./(t-last_t) << " fps" << endl;
        last_t = t;
      }

      // exit if any key is pressed
      if (cv::waitKey(3) == 27){cv::destroyAllWindows(); break; }
if(flag==1) {cv::destroyAllWindows(); break; }
    }
  }

}; // Demo


// here is were everything begins
int main(int argc, char* argv[]) {
  thread th1(foo);
  th1.detach();
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
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::Subscriber location_sub=nh.subscribe("/mavros/global_position/global",1,&locationCallback);  
    ros::Subscriber local_pos=nh.subscribe("/mavros/local_position/pose",1,&locp);  
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    	ros::ServiceClient client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	ros::Subscriber waypt=nh.subscribe("/mavros/mission/waypoints",1,&waypointfinder); 
	 ros::Subscriber op=nh.subscribe("/mavros/global_position/local",1,&ang);
	
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
for(int i = 100; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }
double old=pt.altitude;
float yaw;

   offb_set_mode.request.custom_mode = "AUTO.MISSION";
if( set_mode_client.call(offb_set_mode)){
                ROS_INFO("Auto enabled");
            }
sleep(5);

if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
	
yaw=value.data*3.14156/180;

int x=0;
ROS_INFO("Angle:%f",value.data);
  // process command line options
while(ros::ok()){
cout<<endl<<"SIZE="<<waypoints.size()<<endl;
for(int i = 0; i < waypoints.size(); i++)
  {
    cout<<"\nWaypoint loaded "<<i<<" ";
	printf(waypoints[i].is_current ? "true" : "false");
	if(waypoints[2].is_current)
	{x=1;	
	break;}
  }
if(x==1)
break;
ros::spinOnce();
rate.sleep();
}

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



if(yflag==0)
{    offb_set_mode.request.custom_mode = "AUTO.LOITER";
if( set_mode_client.call(offb_set_mode)){
                ROS_INFO("L enabled");
            }
sleep(10);
for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }

if(yflag==0)

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
}
else
for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("LAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }

cout<<endl<<(pt.altitude-old);
sleep(7);
cout<<"\n DElivered";
system("echo odroid | sudo -S ./test/s2");
for(int i = 30; ros::ok() && i > 0; --i){
		ROS_INFO("\nLAT:%f LON:%f ALT:%f",pt.latitude,pt.longitude,pt.altitude);
        ros::spinOnce();
        rate.sleep();
    }


 ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 3.5;
    srv_takeoff.request.latitude = pt.latitude;
    srv_takeoff.request.longitude = pt.longitude;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw =yaw;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    sleep(7);


    offb_set_mode.request.custom_mode = "AUTO.RTL";
if( set_mode_client.call(offb_set_mode)){
                ROS_INFO("RTL enabled");
            }

  return 0;
}
