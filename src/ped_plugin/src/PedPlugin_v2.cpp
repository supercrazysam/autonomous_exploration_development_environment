/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

#include <ped_plugin/PedPlugin.h>

#include <ros/ros.h>

//include "plugins/PedPlugin.hh"

#include <tuple>
#include <ignition/msgs.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedPlugin::PedPlugin()
{
}

/////////////////////////////////////////////////
void PedPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&PedPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void PedPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

 this->actor->SetWorldPose(
            ignition::math::Pose3d(
            this->target,
            ignition::math::Quaterniond(0, 0, 0)), false, false );

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}


std::tuple<double, double> calculate( double x_min, double x_max, double y_min, double y_max )
{
   

    double angle_in_rad = (90-70.70995330810547) * (3.14159265359 / 180);

    double origin_x = x_min;
    double origin_y = y_min;

    double pos_x = ignition::math::Rand::DblUniform(x_min,x_max);
    double pos_y = ignition::math::Rand::DblUniform(y_min,y_max);

    double processed_x = origin_x + cos(angle_in_rad) * (pos_x - origin_x) - sin(angle_in_rad) * (pos_y - origin_y);
    double processed_y = origin_y + sin(angle_in_rad) * (pos_x - origin_x) + cos(angle_in_rad) * (pos_y - origin_y);

    return  std::make_tuple( processed_x, processed_y);
}

std::tuple<double, double> check_bound( double pos_x, double pos_y  )
{
   

    double angle_in_rad = (90-70.70995330810547) * (3.14159265359 / 180);  //rotate this angle to become parking lot frame
    double angle_in_rad_to_map = -angle_in_rad;  //rotate this angle to turn parking lot frame into map frame

    //-13.0, 8.0, -67.0, -52.0
    
    double x_min = -13.0;
    double x_max =   8.0;
    double y_min =  -67.0;
    double y_max =  -52.0;

    double origin_x = x_min; //x_min;
    double origin_y = y_min; //y_min;

    double processed_x = origin_x + cos(angle_in_rad_to_map) * (pos_x - origin_x) - sin(angle_in_rad_to_map) * (pos_y - origin_y);
    double processed_y = origin_y + sin(angle_in_rad_to_map) * (pos_x - origin_x) + cos(angle_in_rad_to_map) * (pos_y - origin_y);

    if (processed_x > x_max)
       { processed_x = x_max; }
    else if (processed_x < x_min)
       { processed_x = x_min; }

    if (processed_y > y_max)
       { processed_y = y_max; }
    else if (processed_y < y_min)
       { processed_y = y_min; }    

    return  std::make_tuple( processed_x, processed_y);
}

/////////////////////////////////////////////////
void PedPlugin::ChooseNewTarget()
{

   ROS_INFO("Start enter ChooseNewTarget()");
    
  ignition::math::Vector3d newTarget(this->target);
    
   
   
  while ((newTarget - this->target).Length() < 2.0)
  {

    ROS_INFO("PICKING");
    
    double x,y;
    //std::tie(x,y) = calculate( -13.0, 8.0, -67.0, -52.0 );
    std::tie(x,y) = calculate( -12.5, 7.5, -66.5, -51.5 );
    newTarget.X(x);
    newTarget.Y(y);


      ROS_INFO("Hello World!");
      ROS_INFO("X:");
      ROS_INFO("%f", x);
      ROS_INFO("Y:");
      ROS_INFO("%f", y);
    

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;

        ROS_INFO("PICKING DONE!!!!!!!!!!!!");
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
void PedPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void PedPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  //IGN_PROFILE("PedPlugin::UpdateImpl");
  //IGN_PROFILE_BEGIN("Update");

  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();

      //ROS_INFO("On update!");



  // Choose a new target position if the actor has reached its current
  // target.
  ROS_INFO("distance");
  ROS_INFO("%f", distance);
  if (distance < 0.3)
  {
    ROS_INFO("distance < 0.3!");
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  //pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  //pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  //pose.Pos().Z(1.2138);


    //ROS_INFO("Check within bound!");

      ROS_INFO("target_X:");
      ROS_INFO("%f", this->target.X());
      ROS_INFO("target_Y:");
      ROS_INFO("%f", this->target.Y());

      ROS_INFO("X:");
      ROS_INFO("%f", pose.Pos().X());
      ROS_INFO("Y:");
      ROS_INFO("%f", pose.Pos().Y());

  // Make sure the actor stays within bounds
  //double x,y;
  //std::tie(x,y) = check_bound( pose.Pos().X(), pose.Pos().Y() );
  //pose.Pos().X(x);
  //pose.Pos().Y(y);

  pose.Pos().Z(1.2138);

  
  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
  //IGN_PROFILE_END();
}

