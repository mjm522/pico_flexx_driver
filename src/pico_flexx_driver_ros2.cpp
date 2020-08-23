#include <royale.hpp>

#include <mutex>
#include <chrono>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct PicoFlexxConfig
{
    uint32_t exposureTime;
    uint16_t grayDivisor;
    float minFilter;
    float maxFilter;
    std::string baseName;
    std::string sensor;
    int64_t filterLevel;
    int64_t useCase;
    double maxNoise;
    double rangeFactor;
    int queueSize;
};

struct PicoStream{
    int64_t exposureMode;
    bool  autoExposure;
    uint32_t exposureTime;
    std::thread  fpsProcess;
    uint64_t nFrames;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  pubCameraInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pubPointCloud;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  pubDepth;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  pubGray;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  pubNoise;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pubFps;
};


namespace pico_flex_ros2
{

using std::placeholders::_1;

class RoyaleInRos2Node :
    public rclcpp::Node, public royale::IDepthDataListener, public royale::IExposureListener
{
public:
    RoyaleInRos2Node() 
    : Node("PicoFlexNode",
    rclcpp::NodeOptions().allow_undeclared_parameters(true)
                         .automatically_declare_parameters_from_overrides(true)),
    IDepthDataListener(),
    IExposureListener(),
    _ros_clock(RCL_ROS_TIME),
    cameraDevice (nullptr)
{
    config.exposureTime = 0;
    config.grayDivisor = 100;
    config.minFilter = 0.0f;
    config.maxFilter = 7.5f;

    stream1->nFrames = 0;
    stream2->nFrames = 0;

    rclcpp::Parameter paramVal;
    this->get_parameter_or("base_name", paramVal, rclcpp::Parameter("base_name", "pico_flexx_camera_optical_frame"));
    config.baseName = paramVal.as_string();
    this->get_parameter_or("sensor", paramVal, rclcpp::Parameter("sensor", "pico_flexx_camera"));
    config.sensor = paramVal.as_string();
    this->get_parameter_or("use_case", paramVal, rclcpp::Parameter("use_case", 0));
    config.useCase = paramVal.as_int();
    
    this->get_parameter_or("automatic_exposure", paramVal, rclcpp::Parameter("automatic_exposure", true));
    stream1->autoExposure = paramVal.as_bool();
    this->get_parameter_or("exposure_time", paramVal, rclcpp::Parameter("exposure_time", 1000));
    stream1->exposureTime = (uint32_t)paramVal.as_int();
    this->get_parameter_or("exposure_mode", paramVal, rclcpp::Parameter("exposure_mode", 1));
    stream1->exposureMode = paramVal.as_int();
    //stream1 specific
    this->get_parameter_or("automatic_exposure2", paramVal, rclcpp::Parameter("automatic_exposure2", true));
    stream2->autoExposure = paramVal.as_bool();
    this->get_parameter_or("exposure_time_stream2", paramVal, rclcpp::Parameter("exposure_time_stream2", 1000));
    stream2->exposureTime = (uint32_t)paramVal.as_int();
    this->get_parameter_or("exposure_mode_stream2", paramVal, rclcpp::Parameter("exposure_mode_stream2", 1));
    stream2->exposureMode = paramVal.as_int();
    //stream2 specific
    this->get_parameter_or("max_noise", paramVal, rclcpp::Parameter("max_noise", 0.07));
    config.maxNoise = paramVal.as_double();
    this->get_parameter_or("filter_level", paramVal, rclcpp::Parameter("filter_level", 200));
    config.filterLevel = paramVal.as_int();
    this->get_parameter_or("range_factor", paramVal, rclcpp::Parameter("range_factor", 2));
    config.rangeFactor = paramVal.as_double();
    
    this->get_parameter_or("queue_size", paramVal, rclcpp::Parameter("queue_size", 5));
    config.queueSize = (int)paramVal.as_int();
}

virtual ~RoyaleInRos2Node()
{}

virtual void onInit()
{
    stream1->pubCameraInfo = this->create_publisher<sensor_msgs::msg::CameraInfo> ("pico_flexx_camera_info_stream1", 1);
    stream1->pubPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2> ("pico_flexx_point_cloud_stream1", 1);
    stream1->pubDepth = this->create_publisher<sensor_msgs::msg::Image> ("pico_flexx_depth_image_stream1", 1);
    stream1->pubGray = this->create_publisher<sensor_msgs::msg::Image> ("pico_flexx_gray_image_stream1", 1);
    stream1->pubFps = this->create_publisher<std_msgs::msg::String> ("pico_flexx_update_fps_stream1", 1);

    stream2->pubCameraInfo = this->create_publisher<sensor_msgs::msg::CameraInfo> ("pico_flexx_camera_info_stream2", 1);
    stream2->pubPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2> ("pico_flexx_point_cloud_stream2", 1);
    stream2->pubDepth = this->create_publisher<sensor_msgs::msg::Image> ("pico_flexx_depth_image_stream2", 1);
    stream2->pubGray = this->create_publisher<sensor_msgs::msg::Image> ("pico_flexx_gray_image_stream2", 1);
    stream2->pubFps = this->create_publisher<std_msgs::msg::String> ("pico_flexx_update_fps_stream2", 1);

    start();
}

void start()
{

    stream1->fpsProcess = std::thread(&RoyaleInRos2Node::fpsUpdate, this, RoyaleInRos2Node::stream1);
    stream2->fpsProcess = std::thread(&RoyaleInRos2Node::fpsUpdate, this, RoyaleInRos2Node::stream2);

    // Create a camera manager and query available cameras
    royale::CameraManager manager;
    royale::Vector<royale::String>cameraList(manager.getConnectedCameraList());

    if (cameraList.empty())
    {
        RCLCPP_ERROR(logger_, "No suitable cameras found!");
        return;
    }

    // Create the first camera that was found, register a data listener
    // and start the capturing
    cameraDevice = manager.createCamera (cameraList[0]);

    royale::String cameraName;
    std::string info = "Opened camera : ";
    cameraDevice->getCameraName(cameraName);
    RCLCPP_INFO(logger_, info.append(cameraName.c_str()));
   
    if (cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Error initializing the camera!");
        return;
    }
    else
    {
        RCLCPP_INFO(logger_, "Camera got initialized successfully :)");
    }

    if(!setUseCase( (size_t)RoyaleInRos2Node::config.useCase ))
    {
         RCLCPP_INFO(logger_, "Failed to set the use case");
         return; 
    }
    else
    {
        RCLCPP_INFO(logger_, "Use case got set :)");
    }  

    if (!setExposureModeAllStreams(RoyaleInRos2Node::stream1->autoExposure, 
                                   RoyaleInRos2Node::stream2->autoExposure))
    {
       RCLCPP_ERROR(logger_, "Failed to set exposure modes");
       return; 
    }
    else
    {
        RCLCPP_INFO(logger_, "Exposure mode got set :)");
    }

    royale::LensParameters params;

    if(!getCameraSettings(params))
    {
        RCLCPP_WARN(logger_, "Unable to retrieve camera settings");
    }

    if (!setCameraInfo())
    {
        RCLCPP_ERROR(logger_, "Couldn't create camera info!");
        return;
    }
    else
    {
        RCLCPP_INFO(logger_, "Camera info got set :)");
    }

    if (cameraDevice->registerExposureListener(this) != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Couldn't register exposure listener!");
        return;
    }    
    else
    {
        RCLCPP_INFO(logger_, "Registered exposure listener :)");
    }

    if (cameraDevice->registerDataListener(this) != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Couldn't register data listener!");
        return;
    }
    else
    {
        RCLCPP_INFO(logger_, "Registered data listener :)");
    }

    if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Error starting camera capture!");
        return;
    }
    else
    {
        RCLCPP_INFO(logger_, "Starting camera capture :)");
    }

}

void stop()
{
    // Close the camera
    if (cameraDevice &&
            cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_ERROR(logger_, "Error stopping camera capture!");
        return;
    }
    delete stream1;
    delete stream2;
    stream1->fpsProcess.join();
    stream2->fpsProcess.join();
}

private:

bool findStreamIndex(const royale::StreamId streamId, size_t &streamIndex)
{
    royale::Vector<royale::StreamId> streams;
    cameraDevice->getStreams(streams);
    auto it = std::find(streams.begin(), streams.end(), streamId);
    if (it == streams.end())
    {
      RCLCPP_ERROR(logger_, "invalid stream ID!");
      return false;
    }
    streamIndex = std::distance(streams.begin(), it);
    return true;
}

void publishStream(const royale::DepthData *data, PicoStream *stream)
{
    
    stream->nFrames++;
    // Create a standard header
    std_msgs::msg::Header header;
    header.frame_id = "pico_flexx_camera_optical_frame";
    header.stamp = _ros_clock.now();

    // Create camera info message
    sensor_msgs::msg::CameraInfo msgCameraInfo = sensor_msgs::msg::CameraInfo();

    msgCameraInfo.header = header;
    msgCameraInfo.height = data->height;
    msgCameraInfo.width = data->width;

    // Create point cloud message ...
    sensor_msgs::msg::PointCloud2 msgPointCloud = sensor_msgs::msg::PointCloud2();

    // ... where we want to save x,y,z
    msgPointCloud.fields.resize (3);

    msgPointCloud.fields[0].name = "x";
    msgPointCloud.fields[0].offset = 0;
    msgPointCloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msgPointCloud.fields[0].count = 1;

    msgPointCloud.fields[1].name = "y";
    msgPointCloud.fields[1].offset = static_cast<uint32_t> (sizeof (float));
    msgPointCloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msgPointCloud.fields[1].count = 1;

    msgPointCloud.fields[2].name = "z";
    msgPointCloud.fields[2].offset = 2u * static_cast<uint32_t> (sizeof (float));
    msgPointCloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msgPointCloud.fields[2].count = 1;

    msgPointCloud.header = header;
    msgPointCloud.width = data->width;
    msgPointCloud.height = data->height;
    msgPointCloud.is_bigendian = false;
    msgPointCloud.is_dense = false;
    msgPointCloud.point_step = static_cast<uint32_t> (3 * sizeof (float));
    msgPointCloud.row_step = static_cast<uint32_t> (3 * sizeof (float) * data->width);

    // Reserve space for the actual data
    msgPointCloud.data.resize (3 * sizeof (float) * data->points.size());

    // Create a point cloud modifier
    sensor_msgs::PointCloud2Modifier modifier (msgPointCloud);
    modifier.setPointCloud2FieldsByString (1, "xyz");

    // Create iterators for the three fields in our point cloud
    sensor_msgs::PointCloud2Iterator<float> iterX (msgPointCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY (msgPointCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ (msgPointCloud, "z");

    // Create Depth Image message
    sensor_msgs::msg::Image msgDepthImage = sensor_msgs::msg::Image();
    msgDepthImage.header = header;
    msgDepthImage.width = data->width;
    msgDepthImage.height = data->height;
    msgDepthImage.is_bigendian = false;
    msgDepthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    msgDepthImage.step = static_cast<uint32_t> (sizeof (float) * data->width);
    msgDepthImage.data.resize (sizeof (float) * data->points.size());

    float *iterDepth = (float *) &msgDepthImage.data[0];

    // Create Depth Image message
    sensor_msgs::msg::Image msgGrayImage = sensor_msgs::msg::Image();
    msgGrayImage.header = header;
    msgGrayImage.width = data->width;
    msgGrayImage.height = data->height;
    msgGrayImage.is_bigendian = false;
    msgGrayImage.encoding = sensor_msgs::image_encodings::MONO16;
    msgGrayImage.step = static_cast<uint32_t> (sizeof (uint16_t) * data->width);
    msgGrayImage.data.resize (sizeof (uint16_t) * data->points.size());

    uint16_t *iterGray = (uint16_t *) &msgGrayImage.data[0];
    
    // Iterate over all the points we received in the callback
    for (auto currentPoint : data->points)
    {
        if (currentPoint.depthConfidence > 0 
         && currentPoint.z >= config.minFilter 
         && currentPoint.z <= config.maxFilter)
        {
            *iterX = currentPoint.x;
            *iterY = currentPoint.y;
            *iterZ = currentPoint.z;
            *iterDepth = currentPoint.z;
        }
        else
        {
            // If the confidence is 0 set this point to NaN
            // (according to http://www.ros.org/reps/rep-0117.html)
            *iterX = *iterY = *iterZ = std::numeric_limits<float>::quiet_NaN();
            *iterDepth = 0.0f;
        }

        // Set divisor of gray image to adjust the brightness
        uint16_t clampedVal = std::min (config.grayDivisor, currentPoint.grayValue);
        int newGrayValue = std::min (254, static_cast<int> (254 * 1.f * (float) clampedVal / (float) config.grayDivisor));

        if (newGrayValue < 0)
        {
            newGrayValue = 0;
        }

        *iterGray = static_cast<uint16_t> (newGrayValue);

        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterDepth;
        ++iterGray;
    }

    // Publish the messages
    stream->pubCameraInfo->publish(msgCameraInfo);
    stream->pubPointCloud->publish(msgPointCloud);
    stream->pubDepth->publish(msgDepthImage);
    stream->pubGray->publish(msgGrayImage);
}

void onNewData(const royale::DepthData *data)
{
    size_t streamIndex;
    findStreamIndex(data->streamId, streamIndex);
    lockStatus.lock();
    switch(streamIndex){
        case 0:
        publishStream(data, RoyaleInRos2Node::stream1);
        break;
        case 1:
        publishStream(data, RoyaleInRos2Node::stream2);
        break;
    }
    lockStatus.unlock();

}

void onNewExposure (const uint32_t newExposureTime)
{
    if (config.exposureTime == newExposureTime)
    {
        return;
    }
    config.exposureTime = newExposureTime;
}

bool setCameraInfo()
{
    royale::LensParameters lensParams;
    if ((cameraDevice->getLensParameters (lensParams) == royale::CameraStatus::SUCCESS))
    {
        if (lensParams.distortionRadial.size() != 3)
        {
            RCLCPP_ERROR(logger_, "Unknown distortion model!");
            return false;
        }
        else
        {
            cameraInfo.distortion_model = "plumb_bob";
            cameraInfo.d.resize (5);
            cameraInfo.d[0] = lensParams.distortionRadial[0];
            cameraInfo.d[1] = lensParams.distortionRadial[1];
            cameraInfo.d[2] = lensParams.distortionTangential.first;
            cameraInfo.d[3] = lensParams.distortionTangential.second;
            cameraInfo.d[4] = lensParams.distortionRadial[2];
        }

        cameraInfo.k[0] = lensParams.focalLength.first;
        cameraInfo.k[1] = 0;
        cameraInfo.k[2] = lensParams.principalPoint.first;
        cameraInfo.k[3] = 0;
        cameraInfo.k[4] = lensParams.focalLength.second;
        cameraInfo.k[5] = lensParams.principalPoint.second;
        cameraInfo.k[6] = 0;
        cameraInfo.k[7] = 0;
        cameraInfo.k[8] = 1;

        cameraInfo.r[0] = 1;
        cameraInfo.r[1] = 0;
        cameraInfo.r[2] = 0;
        cameraInfo.r[3] = 0;
        cameraInfo.r[4] = 1;
        cameraInfo.r[5] = 0;
        cameraInfo.r[6] = 0;
        cameraInfo.r[7] = 0;
        cameraInfo.r[8] = 1;

        cameraInfo.p[0] = lensParams.focalLength.first;
        cameraInfo.p[1] = 0;
        cameraInfo.p[2] = lensParams.principalPoint.first;
        cameraInfo.p[3] = 0;
        cameraInfo.p[4] = 0;
        cameraInfo.p[5] = lensParams.focalLength.second;
        cameraInfo.p[6] = lensParams.principalPoint.second;
        cameraInfo.p[7] = 0;
        cameraInfo.p[8] = 0;
        cameraInfo.p[9] = 0;
        cameraInfo.p[10] = 1;
        cameraInfo.p[11] = 0;

        return true;
    }
    else
    {
        RCLCPP_ERROR(logger_, "Couldn't get lens parameters!");
        return false;
    }
}

void fpsUpdate(PicoStream* stream)
{
    while (rclcpp::ok())
    { 
        std::this_thread::sleep_for (std::chrono::seconds (1));
        std_msgs::msg::String msg;
        msg.data = std::to_string(stream->nFrames);
        stream->pubFps->publish(msg);
        lockStatus.lock();
        stream->nFrames = 0;
        lockStatus.unlock();
    }
}

bool getCameraSettings(royale::LensParameters &params)
{
    bool ret = true;
    royale::Vector<royale::String> useCases;
    cameraDevice->getUseCases(useCases);
    royale::String useCase;
    cameraDevice->getCurrentUseCase(useCase);
    royale::ExposureMode expMode;
    cameraDevice->getExposureMode(expMode);
    royale::Pair<uint32_t, uint32_t> limits;
    cameraDevice->getExposureLimits(limits);
    royale::Vector<royale::Pair<royale::String,royale::String>> info;
    cameraDevice->getCameraInfo(info);

    royale::String cameraProperty;
    cameraDevice->getCameraName(cameraProperty);
    RCLCPP_INFO(logger_, "camera name: %s", cameraProperty);
    cameraDevice->getId(cameraProperty);
    RCLCPP_INFO(logger_, "camera id:%s", cameraProperty);
    royale::CameraAccessLevel accessLevel;
    cameraDevice->getAccessLevel(accessLevel);
    RCLCPP_INFO(logger_, "access level: %d", (int)accessLevel+ 1);
    if(expMode == royale::ExposureMode::AUTOMATIC)
        RCLCPP_INFO(logger_, "exposure mode: automatic");
    else
        RCLCPP_INFO(logger_, "exposure mode: manual");
    RCLCPP_INFO(logger_, "exposure limits: %d / %d", limits.first, limits.second);

    RCLCPP_INFO(logger_, "camera info:");
    if(info.empty())
    {
      RCLCPP_INFO(logger_, "  no camera info available!");
    }
    else
    {
      for(size_t i = 0; i < info.size(); ++i)
      {
        RCLCPP_INFO(logger_, "info first: %f info second: %f", info[i].first, info[i].second);
      }
    }

    RCLCPP_INFO(logger_, "use cases:");
    if(useCases.empty())
    {
      RCLCPP_ERROR(logger_, "  no use cases available!");
      ret = false;
    }
    else
    {
      for(size_t i = 0; i < useCases.size(); ++i)
      {
        if(useCases[i] == useCase)
            RCLCPP_INFO(logger_, "%d : %s (selected)", i, useCases[i]);
        else
            RCLCPP_INFO(logger_, "%d : %s ()", i, useCases[i]);
      }
    }

    if(cameraDevice->getLensParameters(params) == royale::CameraStatus::SUCCESS)
    {
      RCLCPP_INFO(logger_, "camera intrinsics:");
      uint16_t maxSensorWidth;
      cameraDevice->getMaxSensorWidth(maxSensorWidth);
      RCLCPP_INFO(logger_, "width: %d", maxSensorWidth);

      uint16_t maxSensorHeight;
      cameraDevice->getMaxSensorHeight(maxSensorHeight);
      RCLCPP_INFO(logger_, "height: %d", maxSensorHeight);
      RCLCPP_INFO(logger_, "fx: %f \n fy: %f \n cx: %f \n cy: %f", 
                            params.focalLength.first, 
                            params.focalLength.second,
                            params.principalPoint.first,
                            params.principalPoint.second);
      if(params.distortionRadial.size() == 3)
      {
        RCLCPP_INFO(logger_, "k1: %f \n p1: %f \n p2: %f \n k3: %f \n",
                              params.distortionRadial[0],
                              params.distortionRadial[1],
                              params.distortionTangential.first,
                              params.distortionTangential.second,
                              params.distortionRadial[2]);
      }
      else
      {
        RCLCPP_ERROR(logger_, "distortion model unknown!");
        ret = false;
      }
    }
    else
    {
      RCLCPP_ERROR(logger_, "could not get lens parameter!");
      ret = false;
    }
    return ret;
}

bool setUseCase(const size_t idx)
  {
    royale::Vector<royale::String> useCases;
    cameraDevice->getUseCases(useCases);
    royale::String useCase;
    cameraDevice->getCurrentUseCase(useCase);

    if(useCases.empty())
    {
      RCLCPP_ERROR(logger_, "no use cases available!");
      return false;
    }

    if(idx >= useCases.size())
    {
      RCLCPP_ERROR(logger_, "use case invalid!");
      return false;
    }

    if(useCases[idx] == useCase)
    {
      RCLCPP_INFO(logger_,  "use case not changed!");
      return true;
    }

    if(cameraDevice->setUseCase(useCases[idx]) != royale::CameraStatus::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "could not set use case!");
      return false;
    }
    RCLCPP_INFO(logger_, "use case changed to: %s", useCases[idx]);

    std::string name = royale::String::toStdString(useCases[idx]);

    if (name == "Low_Noise_Extended")
    {
      lockTiming.lock();
      framesPerTiming = 5;
      lockTiming.unlock();
      return true;
    }
    if (name == "Fast_Acquisition") {
      lockTiming.lock();
      framesPerTiming = 45;
      lockTiming.unlock();
      return true;
    }

    size_t start, end;

    // handle MODE_9_5FPS_2000 etc.
    end = name.find("FPS");
    start = name.rfind('_', end);
    if (start != std::string::npos)
      start += 1;

    if(end == std::string::npos || start == std::string::npos)
    {
      // handle MODE_MIXED_30_5, MODE_MIXED_50_5
      start = name.find("MIXED_");
      if (start != std::string::npos)
        start += 6;
      end = name.find('_', start);
    }

    if(end == std::string::npos || start == std::string::npos)
    {
      RCLCPP_WARN(logger_, "could not extract frames per second from operation mode.");
      lockTiming.lock();
      framesPerTiming = 100;
      lockTiming.unlock();
      return true;
    }

    std::string fpsString = name.substr(start, end - start);
    if(fpsString.find_first_not_of("0123456789") != std::string::npos)
    {
      RCLCPP_WARN(logger_, "could not extract frames per second from operation mode.");
      lockTiming.lock();
      framesPerTiming = 100;
      lockTiming.unlock();
      return true;
    }

    lockTiming.lock();
    framesPerTiming = std::stoi(fpsString) * 5;
    lockTiming.unlock();
    return true;
  }

bool setFilterLevel(const int level, const royale::StreamId streamId = 0)
  {
    const royale::FilterLevel desiredLevel = (royale::FilterLevel) level;
    royale::FilterLevel currentLevel;
    if (cameraDevice->getFilterLevel(currentLevel) != royale::CameraStatus::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "could not get filter level!");
      return false;
    }
    RCLCPP_INFO(logger_, "current filter level: %d", (int) currentLevel);
    this->config.filterLevel = (int) currentLevel;

    if (desiredLevel == currentLevel)
    {
      RCLCPP_INFO(logger_, "filter level unchanged");
      return false;
    }
    if (desiredLevel == royale::FilterLevel::Custom)
    {
      RCLCPP_INFO(logger_, "filter level 'Custom' can only be read, not set");
      return false;
    }
    if (currentLevel == royale::FilterLevel::Custom)
    {
      RCLCPP_INFO(logger_, "current filter level is 'Custom', will not overwrite");
      return false;
    }
    if (cameraDevice->setFilterLevel(desiredLevel, streamId) != royale::CameraStatus::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "could not set filter level!");
      return false;
    }
    RCLCPP_INFO(logger_, "filter level changed to: %d", (int) desiredLevel);
    return true;
  }

bool setExposureTimeAllStreams(const uint32_t exposure, const uint32_t exposure2)
{
    bool success = true;
    royale::Vector<royale::StreamId> streams;
    cameraDevice->getStreams(streams);
    if (streams.size() >= 1)
      success &= setExposure(exposure, streams[0]);
    if (streams.size() >= 2)
      success &= setExposure(exposure2, streams[1]);
    return success;
}

bool setExposure(const uint32_t exposure, const royale::StreamId streamId = 0)
{
    int tries = 5;
    royale::CameraStatus ret;
    royale::Pair<uint32_t, uint32_t> limits;
    cameraDevice->getExposureLimits(limits, streamId);
    if(exposure < limits.first || exposure > limits.second)
    {
      RCLCPP_ERROR(logger_, "exposure outside of limits!");
      return false;
    }
    do
    {
        ret = cameraDevice->setExposureTime (config.exposureTime);
        if (ret == royale::CameraStatus::DEVICE_IS_BUSY)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds (200));
            tries--;
        }
    }
    while (tries > 0 && ret == royale::CameraStatus::DEVICE_IS_BUSY);

    if (ret != royale::CameraStatus::SUCCESS)
    {
        RCLCPP_WARN(logger_, "Couldn't set exposure time!");
        return false;
    }
}

bool setExposureModeAllStreams(const bool automaticStream1, const bool automaticStream2)
{
    bool success = true;
    royale::Vector<royale::StreamId> streams;
    cameraDevice->getStreams(streams);
    if (streams.size() >= 1)
      success &= setExposureMode(automaticStream1, streams[0]);
    if (streams.size() >= 2)
      success &= setExposureMode(automaticStream2, streams[1]);
    return success;
}

bool setExposureMode(const bool automatic, const royale::StreamId streamId = 0)
  {
    royale::ExposureMode newMode = automatic ? royale::ExposureMode::AUTOMATIC : royale::ExposureMode::MANUAL;
    royale::ExposureMode exposureMode;
    cameraDevice->getExposureMode(exposureMode, streamId);
    if(newMode == exposureMode)
    {
      RCLCPP_INFO(logger_, "exposure mode not changed!");
      return true;
    }
    if(cameraDevice->setExposureMode(newMode, streamId) != royale::CameraStatus::SUCCESS)
    {
      RCLCPP_ERROR(logger_, "could not set operation mode!");
      return false;
    }
    if(automatic)
        RCLCPP_INFO(logger_, "exposure mode changed to: automatic") ;
    else
        RCLCPP_INFO(logger_, "exposure mode changed to: manual") ;    
    return true;
  }

rclcpp::Clock _ros_clock;
rclcpp::Logger logger_ = rclcpp::get_logger("PicoFlexxRos2Node");
std::unique_ptr<royale::ICameraDevice> cameraDevice;
//boost::recursive_mutex lockServer;
sensor_msgs::msg::CameraInfo cameraInfo;
std::mutex lockStatus, lockData, lockTiming;
int framesPerTiming;
std::string baseNameTF;
struct PicoFlexxConfig config;
PicoStream* stream1 = new PicoStream();
PicoStream* stream2 = new PicoStream();

};
}

//PLUGINLIB_EXPORT_CLASS(pico_flex_ros2::RoyaleInRos2Node, rclcpp::Node)

int main (int argc, char *argv[])
{
    rclcpp::init (argc, argv);
    auto node = std::make_shared<pico_flex_ros2::RoyaleInRos2Node>();
    node->onInit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}