#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "fcu_sim_plugins/gazebo_step_camera.h"
#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <unistd.h>
#include <math.h>

#include <sensor_msgs/Illuminance.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosStepCamera)

GazeboRosStepCamera::GazeboRosStepCamera(){
}

GazeboRosStepCamera::~GazeboRosStepCamera(){
    event::Events::DisconnectWorldUpdateBegin(_updateConnection);
}

void GazeboRosStepCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
    }

    CameraPlugin::Load(_parent, _sdf);

    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);

    float worldRate = physics::get_world()->GetPhysicsEngine()->GetMaxStepSize();
    float updateRate = 1.0 / this->parentSensor_->UpdateRate();

    if(std::ceil(updateRate / worldRate) != updateRate / worldRate){
        gzwarn << "The update rate of sensor " << this->parentSensor_->Name() << " does not evenly divide into the "
               << "MaxStepSize of the world. This will result in an actual framerate that is slower than requested. "
               << "Consider decreasing the MaxStepSize, or changing the FrameRate or UpdateRate" << "\n";
    }

    // I think putting this at the end helps improve the consistency of "render-then-update-function", but I don't have
    // any super good evidence for that, only trial and error
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosStepCamera::OnUpdate, this, _1));
}

void GazeboRosStepCamera::OnUpdate(const common::UpdateInfo&){
  static common::Time timeout_start;

  timeout_start = common::Time::GetWallTime();
  while(this->parentSensor->IsActive() && (this->world_->GetSimTime() - this->last_update_time_) >= (1.0 / this->parentSensor_->UpdateRate()) ){
    if(common::Time::GetWallTime() - timeout_start > 0.10){ // Timeout in seconds
        gzerr << "Update loop timed out waiting for the renderer." << "\n";
        break;
    }
  }
}

void GazeboRosStepCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
    this->sensor_update_time_ = this->world_->GetSimTime();

    this->PutCameraData(_image);
    this->PublishCameraInfo();

    this->last_update_time_ = this->world_->GetSimTime();
}