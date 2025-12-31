#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"
#include <XmlRpcValue.h>
// #include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono;
using std::string;

namespace miivii_gmsl
{

  MiiviiGmslCamera::MiiviiGmslCamera(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh)
  {
    this->active_camera_num = 0;
    this->last_time = ros::Time(0);
    DeclareParameters();
    GetParameters();
    CreateMvGmslCamera();
    // StartTimer();
  }

  MiiviiGmslCamera::~MiiviiGmslCamera()
  {
  }

  void MiiviiGmslCamera::SpiltResolution(std::string camera_resolution, uint *width, uint *height)
  {
    std::size_t found = camera_resolution.find("x");

    if (found != std::string::npos)
    {
      std::istringstream iss(camera_resolution);
      std::vector<std::string> parts;
      std::string part;

      while (std::getline(iss, part, 'x'))
      {
        parts.push_back(part);
      }

      *width = std::stoul(parts[0]);
      *height = std::stoul(parts[1]);
    }
    else
    {
      ROS_ERROR("Invalid Camera resolution format.");
      ros::shutdown();
    }
  }

  void MiiviiGmslCamera::DeclareParameters()
  {
    pnh_.param<double>("publish_rate", this->publish_rate, 20.0);
    pnh_.param<int>("sync_camera_number", this->sync_camera_num, 8);
    pnh_.param<int>("async_camera_number", this->async_camera_num, 0);
    pnh_.param<int>("sync_freq", this->sync_camera_freq, 30);
    pnh_.param<int>("async_freq", this->async_camera_freq, 0);
    
    int sync_trigger_temp = 0xff;
    int async_trigger_temp = 0;
    pnh_.param<int>("sync_trigger", sync_trigger_temp, 0xff);
    pnh_.param<int>("async_trigger", async_trigger_temp, 0);
    this->sync_camera_trigger = static_cast<uint>(sync_trigger_temp);
    this->async_camera_trigger = static_cast<uint>(async_trigger_temp);
    
    // Get async_angle vector parameter
    XmlRpc::XmlRpcValue async_angle_list;
    if (pnh_.getParam("async_angle", async_angle_list))
    {
      this->async_angle.clear();
      for (int i = 0; i < async_angle_list.size(); i++)
      {
        this->async_angle.push_back(static_cast<int64_t>(static_cast<int>(async_angle_list[i])));
      }
    }

    this->camera_num = 8;
  }

  void MiiviiGmslCamera::GetParameters()
  {
    pnh_.getParam("publish_rate", this->publish_rate);
    pnh_.getParam("sync_camera_number", this->sync_camera_num);
    pnh_.getParam("async_camera_number", this->async_camera_num);
    pnh_.getParam("sync_freq", this->sync_camera_freq);
    pnh_.getParam("async_freq", this->async_camera_freq);
    
    int sync_trigger_temp = 0xFF;
    int async_trigger_temp = 0;
    pnh_.getParam("sync_trigger", sync_trigger_temp);
    pnh_.getParam("async_trigger", async_trigger_temp);
    this->sync_camera_trigger = static_cast<uint>(sync_trigger_temp);
    this->async_camera_trigger = static_cast<uint>(async_trigger_temp);

    // Get async_angle vector parameter
    XmlRpc::XmlRpcValue async_angle_list;
    if (pnh_.getParam("async_angle", async_angle_list))
    {
      this->async_angle.clear();
      for (int i = 0; i < async_angle_list.size(); i++)
      {
        this->async_angle.push_back(static_cast<int64_t>(static_cast<int>(async_angle_list[i])));
      }
    }

    ROS_INFO("Publish Rate(publish_rate) is: %f", this->publish_rate);
    ROS_INFO("Sync Camera Number(sync_camera_number) is: %d", this->sync_camera_num);
    ROS_INFO("ASync Camera Number(async_camera_number) is: %d", this->async_camera_num);
    ROS_INFO("Sync Freq(sync_freq) : %d", this->sync_camera_freq);
    ROS_INFO("ASync Freq(async_freq) : %d", this->async_camera_freq);
    ROS_INFO("Sync Trigger(sync_trigger) : %d", this->sync_camera_trigger);
    ROS_INFO("ASync Trigger(async_trigger) : %d", this->async_camera_trigger);
  }

  bool MiiviiGmslCamera::fileExists(const std::string &filename)
  {
    std::ifstream file(filename);
    return file.good();
  }

  sensor_msgs::CameraInfo MiiviiGmslCamera::createCameraInfoFromYAML(const std::string &filepath)
  {
    if (filepath.empty() || !fileExists(filepath))
    {
      ROS_INFO("Params file is empty or not exist!");
      return sensor_msgs::CameraInfo();
    }
    else
    {
      ROS_INFO("Params file is : %s", filepath.c_str());
    }

    sensor_msgs::CameraInfo camera_info;
    try
    {
      YAML::Node config = YAML::LoadFile(filepath);

      camera_info.height = config["image_height"].as<int>();
      camera_info.width = config["image_width"].as<int>();
      // camera_info.header.frame_id = config["camera_name"].as<std::string>();
      camera_info.distortion_model = config["distortion_model"].as<std::string>();

      // 读取相机矩阵
      auto camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
      std::copy(camera_matrix_data.begin(), camera_matrix_data.end(), camera_info.K.begin());

      // 读取畸变系数
      auto distortion_coeffs = config["distortion_coefficients"]["data"].as<std::vector<double>>();
      camera_info.D = distortion_coeffs;

      // 读取投影矩阵
      auto projection_matrix_data = config["projection_matrix"]["data"].as<std::vector<double>>();
      std::copy(projection_matrix_data.begin(), projection_matrix_data.end(), camera_info.P.begin());

      // 读取矫正矩阵
      auto rectification_matrix_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
      std::copy(rectification_matrix_data.begin(), rectification_matrix_data.end(), camera_info.R.begin());
    }
    catch (const YAML::Exception &e)
    {
      ROS_INFO("YAML Exception! %s", e.what());
    }

    return camera_info;
  }

  void MiiviiGmslCamera::CreateCtx()
  {
    for (int i = 0; i < camera_num; i++)
    {
      std::string dev_node = "video" + std::to_string(i);
      bool active = false;
      pnh_.param<bool>(dev_node + "_active", active, false);
      if (active)
      {
        image_publisher_[active_camera_num] = nh_.advertise<sensor_msgs::Image>("miivii_gmsl/image" + std::to_string(i), 10);
        std::string camera_info_topic = "miivii_gmsl/camera_info" + std::to_string(i);
        camera_info_publisher_[active_camera_num] = nh_.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 10);
        
        std::string node_name;
        pnh_.param<std::string>(dev_node + "_node_name", node_name, "/dev/" + dev_node);
        ctx[active_camera_num].dev_node = node_name;
        
        pnh_.param<std::string>(dev_node + "_camera_fmt", ctx[active_camera_num].camera_fmt_str, DEFAULT_CAMERA_FORMAT);
        pnh_.param<std::string>(dev_node + "_output_fmt", ctx[active_camera_num].output_fmt_str, DEFAULT_OUTPUT_FORMAT);
        
        std::string camera_resolution;
        std::string output_resolution;
        pnh_.param<std::string>(dev_node + "_camera_res", camera_resolution, DEFAULR_CAMERA_RESOLUTION);
        pnh_.param<std::string>(dev_node + "_output_res", output_resolution, DEFAULR_OUTPUT_RESOLUTION);
        
        uint cam_w, cam_h, out_w, out_h;
        SpiltResolution(camera_resolution, &(cam_w), &(cam_h));
        SpiltResolution(output_resolution, &(out_w), &(out_h));
        ctx[active_camera_num].cam_w = cam_w;
        ctx[active_camera_num].cam_h = cam_h;
        ctx[active_camera_num].out_w = out_w;
        ctx[active_camera_num].out_h = out_h;

        std::string params_file_name;
        pnh_.param<std::string>(dev_node + "_params_file", params_file_name, "");

        ROS_INFO("===============================%s=====================", dev_node.c_str());
        ROS_INFO("Camera Format(camera_fmt) is : %s", ctx[active_camera_num].camera_fmt_str.c_str());
        ROS_INFO("Camera Resolution(camera_resolution) is : %s", camera_resolution.c_str());
        ROS_INFO("Output Format(output_fmt) is: %s", ctx[active_camera_num].output_fmt_str.c_str());
        ROS_INFO("Output Resolution(output_resolution) is : %s", output_resolution.c_str());

        camera_info_msg_.push_back(createCameraInfoFromYAML(params_file_name));

        this->active_camera_num++;
      }
    }
  }

  void MiiviiGmslCamera::CreateMvGmslCamera()
  {
    stCameraCfgSend.async_camera_num = this->async_camera_num;
    stCameraCfgSend.async_freq = this->async_camera_freq;
    stCameraCfgSend.async_camera_bit_draw = this->async_camera_trigger;
    stCameraCfgSend.sync_camera_num = this->sync_camera_num;
    stCameraCfgSend.sync_freq = this->sync_camera_freq;
    stCameraCfgSend.sync_camera_bit_draw = this->sync_camera_trigger;

    int async_angle_size = async_angle.size();
    if (async_angle_size == 8)
    {
      std::vector<uint>::iterator it;
      for (int i = 0; i < async_angle_size; i++)
      {
        stCameraCfgSend.async_camera_pos[i] = async_angle.at(i);
      }
    }

    CreateCtx();

    if (this->active_camera_num == 0)
    {
      pnh_.setParam("video0_active", true);
      CreateCtx();
    }

    std::cout<<"cam activate: "<<this->active_camera_num << std::endl;
    std::cout<<"sync_camera_bit_draw: " << int(stCameraCfgSend.sync_camera_bit_draw) << std::endl;
    std::cout<<"sync_camera_num: " << int(stCameraCfgSend.sync_camera_num) << std::endl;

    mvcam = new miivii::MvGmslCamera(ctx, this->active_camera_num, stCameraCfgSend);

    time_publisher_ = nh_.advertise<miivii_gmsl_camera::MiiviiTime>("miivii_gmsl/time", 10);
  }

  void MiiviiGmslCamera::StartTimer()
  {
    // Set Timer default to 33ms
    if (std::abs(this->publish_rate) < std::numeric_limits<double>::epsilon())
    {
      ROS_WARN("Invalid publish_rate = 0. Use default value 20 instead");
      this->publish_rate = 30.0;
    }
    if (this->publish_rate > 0)
    {
      image_publisher_timer_ = nh_.createTimer(ros::Duration(1.0 / this->publish_rate), 
        [this](const ros::TimerEvent&) { this->timer_callback(); });
    }
  }
  void MiiviiGmslCamera::timer_callback()
  {
    std::vector<uint8_t*> outbuf(this->active_camera_num);
    uint64_t timestamp;
    uint8_t camera_no = ctx[this->active_camera_num - 1].dev_node[10] - 0x30;

    bool res = this->mvcam->GetImagePtr(outbuf.data(), timestamp, camera_no, this->g_camera_dev);

    if (!res)
    {
      ROS_ERROR("Get Image Error!!");
    }

    ros::Time now = ros::Time::now();
    ros::Time time;
    time.fromNSec(timestamp);

    if (this->last_time == ros::Time(0))
    {
      this->last_time = time;
    }
    else
    {
      ros::Duration latency = now - time;
      ros::Duration jitter = time - this->last_time;
      this->last_time = time;

      miivii_gmsl_camera::MiiviiTime miiviimsg;
      miiviimsg.header.stamp = time;
      std::vector<std::string> nodes;
      std::vector<ros::Duration> latencys;
      std::vector<ros::Duration> jitters;
      for (int i = 0; i < this->active_camera_num; i++)
      {
        nodes.push_back(ctx[i].dev_node);
        latencys.push_back(latency);
        jitters.push_back(jitter);
      }
      miiviimsg.nodes = nodes;
      miiviimsg.jitter = jitters;
      miiviimsg.latency = latencys;
      time_publisher_.publish(miiviimsg);
    }

    for (int i = 0; i < this->active_camera_num; i++)
    {
      std::string encoding;
      int step_factor;
      if (ctx[i].camera_fmt_str == "UYVY" && ctx[i].output_fmt_str == "UYVY")
      {
        encoding = "yuv422";
        step_factor = 2;
      }
      else if (ctx[i].camera_fmt_str == "UYVY" && ctx[i].output_fmt_str == "BGRA32")
      {
        encoding = "bgra8";
        step_factor = 4;
      }

      sensor_msgs::Image image;

      image.header.stamp = time;
      image.header.frame_id = "camera" + std::to_string(ctx[i].dev_node[10] - 0x30);
      image.height = ctx[i].out_h;
      image.width = ctx[i].out_w;
      image.encoding = encoding;
      image.is_bigendian = false;
      image.step = step_factor * image.width;
      std::vector<uint8_t> image_data(outbuf[i], outbuf[i] + image.height * image.step);
      image.data = std::move(image_data);

      image_publisher_[i].publish(image);

      sensor_msgs::CameraInfo camera_info = camera_info_msg_.at(i);
      camera_info.header.stamp = image.header.stamp;
      camera_info.header.frame_id = image.header.frame_id;
      camera_info.height = ctx[i].out_h;
      camera_info.width = ctx[i].out_w;

      camera_info_publisher_[i].publish(camera_info);
    }
  }

}

// RCLCPP_COMPONENTS_REGISTER_NODE(miivii_gmsl::MiiviiGmslCamera)
