#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>


#include "inu_codec.h"

#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

double get_time_now()
{
  auto x = std::chrono::system_clock::now().time_since_epoch();
  auto cnt = std::chrono::duration_cast<std::chrono::nanoseconds>(x).count();
  return cnt * 1.0 / 1e9;
}

class GnssBuffer
{
  public:
    sensor_msgs::NavSatFix msg;

  private:
    double alt_ts;
    double latlon_ts;
    double delay;
  public:
    GnssBuffer(double delay=0.01):delay(delay) {
      alt_ts = 0;
      latlon_ts = 0;
    }

    void set_altitude(double alt) {
      msg.altitude = alt;
      alt_ts = get_time_now();
    }

    void set_latlon(LatLon ll) {
      msg.latitude = ll.lat;
      msg.longitude = ll.lon;
      latlon_ts = get_time_now();
    }

    bool is_valid() {
      auto now = get_time_now();
      return (now - alt_ts) < delay && (now - latlon_ts) < delay;
    }
};


class ImuBuffer 
{
  public:
    sensor_msgs::Imu msg;
  private:
    double accel_ts;
    double ang_rate_ts;
    double ang_ts;
    double delay;
  public:
    ImuBuffer(double delay=0.01) :delay(delay){
      accel_ts = 0;
      ang_rate_ts = 0;
      ang_ts = 0;
    }

    void set_accel(double x, double y, double z) {
      msg.linear_acceleration.x = x;
      msg.linear_acceleration.y = y;
      msg.linear_acceleration.z = z;
      accel_ts = get_time_now();
    }

    void set_ang_rate(double x, double y, double z) {
      msg.angular_velocity.x = x;
      msg.angular_velocity.y = y;
      msg.angular_velocity.z = z;
      ang_rate_ts = get_time_now();
    }

    void set_ang(double x, double y, double z) {
      //euler to quaternian, given x and y are zero
      // carla sends us only z angle (heading)
      double cy = cos(z * 0.5);
      double sy = sin(z * 0.5);
      msg.orientation.x = 0;
      msg.orientation.y = 0;
      msg.orientation.z = sy;
      msg.orientation.w = cy;

      ang_ts = get_time_now();      
    }

    bool is_valid() {
      auto now = get_time_now();
      return (now - accel_ts) < delay && (now - ang_rate_ts) < delay && (now - ang_ts) < delay;
    }
};

class InuRx
{
public:
  InuRx(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("can_if", can_if, "can0");
    pnh.param<bool>("debug", debug, false);
    pnh.param<std::string>("imu_frame_id", imu_frame_id, "imu");

    gnss_publisher_ = nh.advertise<sensor_msgs::NavSatFix>("output/gnss", 10);
    imu_publisher_ = nh.advertise<sensor_msgs::Imu>("output/imu", 10);

    read_can();
  }

  int build_can_socket()
  {
    int skt = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    struct ifreq ifr;

    
    strcpy(ifr.ifr_name, can_if.c_str());
    ioctl(skt, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(skt, (struct sockaddr *)&addr, sizeof(addr));

    ROS_INFO("listening on %s.", can_if.c_str());
    return skt;
  }

  int gnss_check_and_send() {
    if (gnss_buffer.is_valid()) {
        
        //frame id, timestamp

        gnss_publisher_.publish(gnss_buffer.msg);

        if (debug) {
          ROS_INFO("sent NavSatFix lat: %f long: %f alt: %f\n",
            gnss_buffer.msg.latitude, gnss_buffer.msg.longitude, gnss_buffer.msg.altitude);
        }        
      }

      return 0;
  }

  int imu_check_and_send() {
    if (imu_buffer.is_valid()) {
        
        //frame id, timestamp
        imu_buffer.msg.header.stamp = ros::Time::now();
        imu_buffer.msg.header.frame_id = imu_frame_id;

        imu_publisher_.publish(imu_buffer.msg);

        if (debug) {
          ROS_INFO("sent Imu accel: %f %f %f ang_rate: %f %f %f ang: %f %f %f %f\n",
            imu_buffer.msg.linear_acceleration.x, 
            imu_buffer.msg.linear_acceleration.y, 
            imu_buffer.msg.linear_acceleration.z,
            imu_buffer.msg.angular_velocity.x, 
            imu_buffer.msg.angular_velocity.y, 
            imu_buffer.msg.angular_velocity.z,
            imu_buffer.msg.orientation.x, 
            imu_buffer.msg.orientation.y, 
            imu_buffer.msg.orientation.z, 
            imu_buffer.msg.orientation.w);
        }        
      }

      return 0;
  }

  
  int read_can()
  {
    

    int skt = build_can_socket();
    
    can_frame frame;
    
    while (ros::ok())
    {
      int bytes_read = read(skt, &frame, sizeof(frame));

      if (bytes_read < 0)
      {
        ROS_ERROR("Error reading CAN bus.\n");
        continue;
      }

      int can_id = frame.can_id & 0x0fffffff;

      if (can_id == INU_LATITUDE_LONGITUDE_FRAME_ID)
      {
        LatLon ll;
        decode_latlon(frame.data, frame.can_dlc, ll);

        
        gnss_buffer.set_latlon(ll);
        
        gnss_check_and_send();
      }
      else if (can_id == INU_ALTITUDE_FRAME_ID) {
        double alt;
        decode_alt(frame.data, frame.can_dlc, alt);
        
        gnss_buffer.set_altitude(alt);
        gnss_check_and_send();

      }
      else if (can_id == INU_ACCEL_IMU_RAW_FRAME_ID) {
        vector3d accel;
        decode_accel(frame.data, frame.can_dlc, accel);
        
        imu_buffer.set_accel(accel.x, accel.y, accel.z);
        imu_check_and_send();
      }
      else if (can_id == INU_ANG_RATE_RAW_IMU_FRAME_ID) {
        vector3d ang_rate;
        decode_angle_rate(frame.data, frame.can_dlc, ang_rate);
        
        imu_buffer.set_ang_rate(ang_rate.x, ang_rate.y, ang_rate.z);
        imu_check_and_send();
      }
      else if (can_id == INU_HEADING_PITCH_ROLL_FRAME_ID) {
        vector3d ang;
        decode_angle(frame.data, frame.can_dlc, ang);
        
        imu_buffer.set_ang(ang.x, ang.y, ang.z);

        imu_check_and_send();
      }
      else if (can_id == INU_HEADING_PITCH_ROLL_SIGMA_FRAME_ID){

      }
      else if (can_id == INU_LONGITUDE_FRAME_ID){

      }
      else
      {
        ROS_ERROR("Unknown CAN ID: %d\n", frame.can_id);
      }
    }

    close(skt);

    return 0;
  }

private:
  ros::Publisher gnss_publisher_;
  ros::Publisher imu_publisher_;

  std::string can_if;
  std::string imu_frame_id;
  bool debug;


  GnssBuffer gnss_buffer;
  ImuBuffer imu_buffer;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "inu_can");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  InuRx inu_rx(nh, pnh);
  
  return 0;
}