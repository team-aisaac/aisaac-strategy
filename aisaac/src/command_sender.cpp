/**
 * @file usbserial-xbee.cpp
 *
 * @brief This program is for Xbee wireless communication
 * @author K.Terae
 * @date 2019.7.9
 *
**/

// include
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>       // For measuring processing time
#include <sys/time.h>   // For measuring processing time
#include <random>       // For generating random number
#include <vector>
#include <signal.h>
#include <errno.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <consai_msgs/robot_commands.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

const uint8_t HEAD_BYTE   = 0x7D;
const uint8_t ESCAPE_BYTE = 0x7E;
const uint8_t ESCAPE_MASK = 0x20;
#define SERIAL_PORT "/dev/ttyUSB0" // SDevice file corrensponding to serial interface
#define MAX_DATA_TYPE 3

#define VEL_MAX 1500

#define ENABLE_DBG             // Toggle when using printf() function
#ifndef ENABLE_DBG
#define DBG(...)
#else
#define DBG(...) printf("" __VA_ARGS__)
#endif

//#define DEBUG
#ifndef DEBUG
    #define debug( fmt, ... ) ((void)0)
#else
    #define debug( fmt, ... ) \
        fprintf( stderr, \
                  "[%s] %s:%u # " fmt "\n", \
                  __DATE__, __FILE__, \
                  __LINE__, ##__VA_ARGS__ \
        )
#endif

volatile sig_atomic_t errorFlag;

class Sender
{
public:
  typedef std::vector<uint8_t> vu8;
  vu8 send_buffer;
  int fd;
  int checksum = 0;
  uint8_t u8_checksum = 0;
  uint8_t datatype = 0;
  int16_t x_vector = 0, y_vector = 0, omega = 0;
  uint16_t th_vector = 0, calib_data = 0, command = 0;
  double current_orientation[3] = {0, 0, 0};

  Sender(){

    // initialize serial communication
    struct termios oldtio, newtio;         // Serial communication settings
    debug("openDevice begin\n");
    fd = open(SERIAL_PORT, O_RDWR);      // open device
    debug("openDevice end\n");

    ioctl(fd, TCGETS, &oldtio);             // Evacuate current serial port settings
    newtio = oldtio;                        // Copy current serial port settings
    newtio.c_cflag = B115200|CREAD|CS8;         // Configure port settings. See man page termios(3)
    ioctl(fd, TCSETS, &newtio);             // Enable port settings

    struct timeval tv_start, tv_end; // for realtime sequence


  }

  ~Sender(){}

  void command_callback(const consai_msgs::robot_commandsConstPtr& msg) {
    for(int i = 0; i < MAX_DATA_TYPE; i++){
      // get sending data from ROS bus
 
//      float x = -1.5;
//      float y = -1.5;
//      float th = 1.57;
//      float calib = 3.14;

      x_vector = int16_t(msg->vel_surge * 1000);
      y_vector = int16_t(msg->vel_sway * 1000);
      omega = int16_t(-msg->omega * (180.0 / M_PI) * 10);
      float vector_sum = pow(x_vector,2) + pow(y_vector,2); 
      if(sqrt(vector_sum) > VEL_MAX){
          float k = sqrt(vector_sum) / VEL_MAX;
          x_vector = x_vector / k;
          y_vector = y_vector / k;
      }

        int16_t tmp = y_vector;
        y_vector = x_vector;
        x_vector = -tmp;

      float theta_deg = msg->theta * (180 / M_PI);
      while(!(0 <= theta_deg && theta_deg < 360)){
          if(theta_deg < 0){
              theta_deg += 360;
          }else{
              theta_deg -= 360;
          }
      }

/*
      x_vector = int16_t(x * 1000);
      y_vector = int16_t(y * 1000);

      float theta_deg = th * (180 / M_PI);
      while(!(0 <= theta_deg && theta_deg < 360)){
          if(theta_deg < 0){
              theta_deg += 360;
          }else{
              theta_deg -= 360;
          }
      }
  */

      double current_orientation_deg = current_orientation[2] * (180 / M_PI);
      while(!(0 <= current_orientation_deg && current_orientation_deg < 360)){
          if(current_orientation_deg < 0){
              current_orientation_deg += 360;
          }else{
              current_orientation_deg -= 360;
          }
      }


      th_vector = uint16_t(theta_deg);
      calib_data = uint16_t(current_orientation_deg);

      // make sending byte-data buffer from integer-data
      setSendDataFromROSBus((uint8_t)i, &send_buffer);

    // send HEAD_BYTE as head-byte
      write(fd, &HEAD_BYTE, 1);
      debug("write(fd, &HEAD_BYTE,1);");
      checksum = HEAD_BYTE;
      DBG("%4d", HEAD_BYTE);
    // while buffer isn't empty, sends byte-data from buffer
      for(auto itr = send_buffer.begin(); itr != send_buffer.end(); ++itr)
      {
        debug("for loop begin");
    // if data compete with HEAD_BYTE or ESCAPE_BYTE, run escape sequence
        if(*itr == HEAD_BYTE||*itr == ESCAPE_BYTE)
        {
          write(fd, &ESCAPE_BYTE, 1); // send escape byte
          DBG("%4d", ESCAPE_BYTE);
          *itr ^= ESCAPE_MASK;        // escape mask
        }
    // send byte-data
        write(fd, (uint8_t*)&*itr, 1);
        DBG("%4d", *itr);
        checksum += *itr;
        debug("for loop end");
      }
      u8_checksum = checksum & 0xFF;
      if(u8_checksum == HEAD_BYTE || u8_checksum == ESCAPE_BYTE)
      {
        write(fd, &ESCAPE_BYTE, 1);
        DBG("%4d", ESCAPE_BYTE);
        u8_checksum ^= ESCAPE_MASK;
      }
      write(fd, &u8_checksum, 1);
      DBG("%4d\n", u8_checksum);

      send_buffer.erase(send_buffer.begin(), send_buffer.end());  //delete all elements of vector
    }


    int16_t tmp = x_vector;
    x_vector = y_vector;
    y_vector = -tmp;
      DBG(" %4d ", x_vector);
      DBG(" %4d ", y_vector);
      DBG(" %4d ", th_vector);
      DBG(" %4d ", calib_data);
      DBG(" %4d ", omega);
      DBG("\n\n\n");


  }

  void odom_callback(const nav_msgs::Odometry& msg) {
      tf::Quaternion quat(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3(quat).getRPY(current_orientation[0], current_orientation[1], current_orientation[2]);
  }

  void setSendDataFromROSBus(uint8_t datatype, vu8 *buf){
    uint8_t tmp = 0;
    switch(datatype)
    {

      case 0:
        tmp = ((0x7 & 0x7) << 5) | (x_vector >> 11);
        buf->push_back(tmp);
        tmp = (x_vector >> 3) & 0xFF;
        buf->push_back(tmp);
        tmp = (x_vector & 0x7) << 5 | (y_vector >> 11);
        buf->push_back(tmp);
        tmp = (y_vector >> 3) & 0xFF;
        buf->push_back(tmp);
        tmp = (y_vector & 0x7) << 5 | (th_vector >> 7);
        buf->push_back(tmp);
        tmp = (th_vector & 0x7F ) << 1;
        buf->push_back(tmp);
        tmp = (omega >> 8);
        buf->push_back(tmp);
        tmp = (omega & 0xFF);
        buf->push_back(tmp);
        break;
      case 1:
        tmp = ((datatype & 0x7) << 5) | (calib_data >> 8);
        buf->push_back(tmp);
        tmp = (calib_data & 0xFF);
        buf->push_back(tmp);
        break;
      case 2:
        tmp = ((datatype & 0x7) << 5) | command;
        buf->push_back(tmp);
        break;
      default:
        break;
    }
  //  datatype = (datatype+1) % MAX_DATA_TYPE;
  }

};

void signalHandler(int signo){
    switch(signo)
    {
      case SIGQUIT:
      case SIGINT:
      case SIGKILL:
      case SIGILL:
      case SIGTERM:
        errorFlag = 1;
        break;
      default:
        break;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_sender");
  ros::NodeHandle nh;
  ros::Rate r(60);
  ros::Subscriber command_sub[1], odom_sub[1];
  Sender senders[1];
  command_sub[0] = nh.subscribe("/blue/robot_1/robot_commands", 100, &Sender::command_callback, &senders[0]);
  odom_sub[0] = nh.subscribe("/blue/robot_1/odom", 100, &Sender::odom_callback, &senders[0]);

  errorFlag = 0;

  if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
      printf("\ncan't catch SIGINT\n");
    }
    if (signal(SIGQUIT, signalHandler) == SIG_ERR)
    {
      printf("\ncan't catch SIGQUIT\n");
    }
    if (signal(SIGILL, signalHandler) == SIG_ERR)
    {
      printf("\ncan't catch SIGILL\n");
    }
    if (signal(SIGTERM, signalHandler) == SIG_ERR)
    {
      printf("\ncan't catch SIGTERM\n");
    }
  while (ros::ok() && errorFlag == 0) {
//         ROS_INFO("Loop");

        ros::spinOnce();
        r.sleep();
    }
  return 0;
}
