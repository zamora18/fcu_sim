/*
 * Copyright 2015 James Jackson MAGICC Lab, BYU, Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "fcu_sim_plugins/gazebo_gimbal_plugin.h"

namespace gazebo {

GazeboGimbalPlugin::GazeboGimbalPlugin() : ModelPlugin() {}

GazeboGimbalPlugin::~GazeboGimbalPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void GazeboGimbalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Configure Gazebo Integration
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  std::string command_topic, pose_topic;

  // Load SDF parameters
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  } else {
    gzerr << "[gazeboGimbalPlugin] Please specify a namespace";
  }

  if (_sdf->HasElement("commandTopic")) {
    command_topic = _sdf->GetElement("commandTopic")->Get<std::string>();
  } else {
    gzerr << "[gazeboGimbalPlugin] Please specify a commandTopic";
  }

  if (_sdf->HasElement("poseTopic")) {
    pose_topic = _sdf->GetElement("poseTopic")->Get<std::string>();
  } else {
    gzerr << "[gazeboGimbalPlugin] Please specify a poseTopic";
  }

  if (_sdf->HasElement("yawJoint")) {
    std::string yaw_joint_name = _sdf->GetElement("yawJoint")->Get<std::string>();
    yaw_joint_ = model_->GetJoint(yaw_joint_name);
  } else{
    gzerr << "[gazeboGimbalPlugin] Please specify a yawJoint";
  }

  if (_sdf->HasElement("pitchJoint")) {
    std::string pitch_joint_name = _sdf->GetElement("pitchJoint")->Get<std::string>();
    pitch_joint_ = model_->GetJoint(pitch_joint_name);
  } else{
    gzerr << "[gazeboGimbalPlugin] Please specify a pitchJoint";
  }

  if (_sdf->HasElement("rollJoint")) {
    std::string roll_joint_name = _sdf->GetElement("rollJoint")->Get<std::string>();
    roll_joint_ = model_->GetJoint(roll_joint_name);
  } else{
    gzerr << "[gazeboGimbalPlugin] Please specify a rollJoint";
  }

  if (_sdf->HasElement("timeConstant")) {
    time_constant_ = _sdf->GetElement("timeConstant")->Get<double>();
  } else{
    gzerr << "[gazeboGimbalPlugin] Please specify a timeConstant";
  }

  if (_sdf->HasElement("useSlipring")) {
    use_slipring_ = _sdf->GetElement("useSlipring")->Get<bool>();
  } else{
    gzerr << "[gazeboGimbalPlugin] Please specify whether to use a slipring";
  }

  // Connect Gazebo Update
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGimbalPlugin::OnUpdate, this, _1));

  // Connect ROS
  nh_ = new ros::NodeHandle();
  command_sub_ = nh_->subscribe(command_topic, 1, &GazeboGimbalPlugin::commandCallback, this);
  pose_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(pose_topic, 10);

  // Initialize Commands
  yaw_desired_ = 0.0;
  pitch_desired_ = 0.0;
  roll_desired_ = 0.0;

  // Initialize Filtered Values
  yaw_actual_ = 0.0;
  pitch_actual_ = 0.0;
  roll_actual_ = 0.0;

  // Create the first order filters.
  yaw_filter_.reset(new FirstOrderFilter<double>(time_constant_, time_constant_, yaw_actual_));
  pitch_filter_.reset(new FirstOrderFilter<double>(time_constant_, time_constant_, pitch_actual_));
  roll_filter_.reset(new FirstOrderFilter<double>(time_constant_, time_constant_, roll_actual_));

  // Set the max force allowed to set the angle, they are really big because we are using the filter instead
  // of controlling actual forces
#if GAZEBO_MAJOR_VERSION > 5
  pitch_joint_->SetParam("max_force", 0, 10000);
  yaw_joint_->SetParam("max_force", 0, 10000);
  roll_joint_->SetParam("max_force", 0, 10000);
#else
  pitch_joint_->SetMaxForce(0, 10000);
  yaw_joint_->SetMaxForce(0, 10000);
  roll_joint_->SetMaxForce(0, 10000);
#endif



  // Set the axes of the gimbal
  yaw_joint_->SetAxis(0, math::Vector3(0, 0, 1));
  pitch_joint_->SetAxis(0, math::Vector3(0, 1, 0));
  roll_joint_->SetAxis(0, math::Vector3(1, 0, 0));

  // Initialize Time
  previous_time_ = 0.0;
}
// Return the Sign of the argument
void GazeboGimbalPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  // Update time
  double dt = _info.simTime.Double() - previous_time_;
  previous_time_ = _info.simTime.Double();

  // Use the Filters to figure out the actual angles
  yaw_actual_ = yaw_filter_->updateFilter(yaw_desired_, dt);
  pitch_actual_ = pitch_filter_->updateFilter(pitch_desired_, dt);
  roll_actual_ = roll_filter_->updateFilter(roll_desired_, dt);

  // Set the Joint Angles to the Filtered angles
#if GAZEBO_MAJOR_VERSION > 5
  yaw_joint_->SetPosition(0, yaw_actual_);
  pitch_joint_->SetPosition(0, pitch_actual_);
  roll_joint_->SetPosition(0, roll_actual_);
#else
  yaw_joint_->SetAngle(0, math::Angle(yaw_actual_));
  pitch_joint_->SetAngle(0, math::Angle(pitch_actual_));
  roll_joint_->SetAngle(0, math::Angle(roll_actual_));
#endif

  // Publish ROS message of actual angles
  geometry_msgs::Vector3Stamped angles_msg;
  angles_msg.header.stamp.sec = world_->GetSimTime().sec;
  angles_msg.header.stamp.nsec = world_->GetSimTime().nsec;
  angles_msg.vector.x = roll_actual_;
  angles_msg.vector.y = pitch_actual_;
  angles_msg.vector.z = yaw_actual_;
  pose_pub_.publish(angles_msg);
}

void GazeboGimbalPlugin::commandCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  // Pull in command from message, convert to NED
  yaw_desired_ = -1.0*msg->vector.z;
  pitch_desired_ = -1.0*msg->vector.y;
  roll_desired_ = msg->vector.x;

  if (!use_slipring_) {
    // Wrap Commands between -PI and PI if a slipring isn't being simulated
    while (fabs(yaw_desired_) > PI) {
      yaw_desired_ -= sign(yaw_desired_)*2.0*PI;
    }
    while (fabs(pitch_desired_) > PI){
      pitch_desired_ -= sign(pitch_desired_)*2.0*PI;
    }
    while (fabs(roll_desired_) > PI) {
      roll_desired_ -= sign(roll_desired_)*2.0*PI;
    }
  }
}

int GazeboGimbalPlugin::sign(double x){
  return (0 < x) - (x < 0);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGimbalPlugin);

} // namespace
