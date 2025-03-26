/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class DepthRegistration;

class Kinect2Bridge
{
public:
  Kinect2Bridge(std::shared_ptr<rclcpp::Node> node);

  bool start();
  void stop();

private:
  enum Image
  {
    IR_SD = 0,
    IR_SD_RECT,

    DEPTH_SD,
    DEPTH_SD_RECT,
    DEPTH_HD,
    DEPTH_QHD,

    COLOR_SD_RECT,
    COLOR_HD,
    COLOR_HD_RECT,
    COLOR_QHD,
    COLOR_QHD_RECT,

    MONO_HD,
    MONO_HD_RECT,
    MONO_QHD,
    MONO_QHD_RECT,

    COUNT
  };

  enum Status
  {
    UNSUBCRIBED = 0,
    RAW,
    COMPRESSED,
    BOTH
  };

  bool initialize();
  bool initRegistration(const std::string &method, const int32_t device, const double maxDepth);
  bool initPipeline(const std::string &method, const int32_t device);
  void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);
  void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png);
  void initTopics(const int32_t queueSize, const std::string &base_name);
  bool initDevice(std::string &sensor);
  void initCalibration(const std::string &calib_path, const std::string &sensor);
  bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;
  bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;
  bool loadCalibrationDepthFile(const std::string &filename, std::chrono::nanoseconds &depthShift) const;
  void createCameraInfo();
  void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::msg::CameraInfo &cameraInfo) const;
  void callbackStatus();
  bool updateStatus(bool &isSubscribedColor, bool &isSubscribedDepth);
  void setupMainThread();
  void main();
  void threadDispatcher(const size_t id);
  void receiveIrDepth();
  void receiveColor();
  bool receiveFrames(libfreenect2::SyncMultiFrameListener *listener, libfreenect2::FrameMap &frames);
  std_msgs::msg::Header createHeader(std::chrono::steady_clock::time_point &last, std::chrono::steady_clock::time_point &other);
  void processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images, const std::vector<Status> &status);
  void processColor(std::vector<cv::Mat> &images, const std::vector<Status> &status);
  void publishImages(const std::vector<cv::Mat> &images, const std_msgs::msg::Header &header, const std::vector<Status> &status, const size_t frame, size_t &pubFrame, const size_t begin, const size_t end);
  void createImage(const cv::Mat &image, const std_msgs::msg::Header &header, const Image type, sensor_msgs::msg::Image &msgImage) const;
  void createCompressed(const cv::Mat &image, const std_msgs::msg::Header &header, const Image type, sensor_msgs::msg::CompressedImage &msgImage) const;
  void publishStaticTF();
  static void setThreadName(const std::string &name);

  // Main loop state
  std::chrono::steady_clock::time_point fpsTime{};
  std::chrono::steady_clock::time_point nextFrame{};
  size_t oldFrameIrDepth = 0;
  size_t oldFrameColor = 0;

  std::vector<int> compressionParams;
  std::string compression16BitExt, compression16BitString, baseNameTF;

  cv::Size sizeColor;
  cv::Size sizeIr;
  cv::Size sizeLowRes;
  libfreenect2::Frame color;
  cv::Mat cameraMatrixColor;
  cv::Mat distortionColor;
  cv::Mat cameraMatrixLowRes;
  cv::Mat cameraMatrixIr;
  cv::Mat distortionIr;
  cv::Mat cameraMatrixDepth;
  cv::Mat distortionDepth;
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat map1Color;
  cv::Mat map2Color;
  cv::Mat map1Ir;
  cv::Mat map2Ir;
  cv::Mat map1LowRes;
  cv::Mat map2LowRes;

  std::vector<std::thread> threads;
  std::mutex lockIrDepth;
  std::mutex lockColor;
  std::mutex lockSync;
  std::mutex lockPub;
  std::mutex lockTime;
  std::mutex lockStatus;
  std::mutex lockRegLowRes;
  std::mutex lockRegHighRes;
  std::mutex lockRegSD;

  bool publishTF = false;
  std::thread tfPublisher;

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *device = nullptr;
  libfreenect2::SyncMultiFrameListener *listenerColor = nullptr;
  libfreenect2::SyncMultiFrameListener *listenerIrDepth = nullptr;
  libfreenect2::PacketPipeline *packetPipeline = nullptr;
  libfreenect2::Registration *registration = nullptr;
  libfreenect2::Freenect2Device::ColorCameraParams colorParams;
  libfreenect2::Freenect2Device::IrCameraParams irParams;

  std::shared_ptr<rclcpp::Node> node;

  DepthRegistration *depthRegLowRes = nullptr;
  DepthRegistration *depthRegHighRes = nullptr;

  size_t frameColor = 0;
  size_t frameIrDepth = 0;
  size_t pubFrameColor = 0;
  size_t pubFrameIrDepth = 0;
  std::chrono::steady_clock::time_point lastColor;
  std::chrono::steady_clock::time_point lastDepth;

  bool nextColor = false;
  bool nextIrDepth = false;
  std::chrono::nanoseconds deltaT{};
  std::chrono::nanoseconds depthShift{};
  std::chrono::nanoseconds elapsedTimeColor{};
  std::chrono::nanoseconds elapsedTimeIrDepth{};
  bool running = false;
  bool deviceActive = false;
  bool clientConnected = false;
  bool isSubscribedColor = false;
  bool isSubscribedDepth = false;

  std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> imagePubs;
  std::vector<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>> compressedPubs;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoHDPub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoQHDPub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> infoIRPub;
  std::shared_ptr<rclcpp::TimerBase> checkTimer;
  std::shared_ptr<rclcpp::TimerBase> mainTimer;
  sensor_msgs::msg::CameraInfo infoHD, infoQHD, infoIR;
  std::vector<Status> status;
};
