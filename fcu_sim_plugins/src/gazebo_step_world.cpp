/*
 * Copyright 2016 Robert Pottorff PCC Lab - BYU - Provo, UT
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

#include "fcu_sim_plugins/gazebo_step_world.h"
#include "gazebo/physics/World.hh"

namespace gazebo {

GazeboStepWorldPlugin::GazeboStepWorldPlugin() : WorldPlugin() {}

GazeboStepWorldPlugin::~GazeboStepWorldPlugin() {
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void GazeboStepWorldPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // Connect ROS
  nh_ = new ros::NodeHandle();
  command_sub_ = nh_->subscribe("/world/step", 1, &GazeboStepWorldPlugin::commandCallback, this);

  this->world_ = _parent;
}

void GazeboStepWorldPlugin::commandCallback(const std_msgs::Int16 &msg)
{
    this->world_->SetPaused(true);
    this->world_->Step(msg.data);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboStepWorldPlugin)

} // namespace

