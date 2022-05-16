/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "SingleSupport.h"
#include <iostream>
#include <mc_mujoco/devices/RangeSensor.h>

#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <variable_stiffness/connectionFile.h>
#include <cstdlib>

namespace lipm_walking
{ 

void states::SingleSupport::start()
{
  client_ = mc_rtc::ROSBridge::get_node_handle()->serviceClient<variable_stiffness::connectionFile>("connectionFile");
  
  auto & ctl = controller();
  auto & supportContact = ctl.supportContact();
  auto & targetContact = ctl.targetContact();
  isStiffnessUpdated_ = false;

  duration_ = ctl.singleSupportDuration();
  hasUpdatedMPCOnce_ = false;
  remTime_ = ctl.singleSupportDuration();
  stateTime_ = 0.;
  altitudeGround_.clear();
  timeSinceLastPreviewUpdate_ = 0.; // don't update at transition

  if(supportContact.surfaceName == "LeftFootCenter")
  {
    ctl.leftFootRatio(1.);
    ctl.setContacts({{ContactState::Left, supportContact.pose}});
    swingFootTask = ctl.swingFootTaskRight_;    
  }
  else
  {
    ctl.leftFootRatio(0.);
    ctl.setContacts({{ContactState::Right, supportContact.pose}});
    swingFootTask = ctl.swingFootTaskLeft_;
  }
  swingFootTask->reset();
  ctl.solver().addTask(swingFootTask);

  swingFoot_.landingDuration(ctl.plan.landingDuration());
  swingFoot_.landingPitch(ctl.plan.landingPitch());
  swingFoot_.takeoffDuration(ctl.plan.takeoffDuration());
  swingFoot_.takeoffOffset(ctl.plan.takeoffOffset());
  swingFoot_.takeoffPitch(ctl.plan.takeoffPitch());
  swingFoot_.reset(swingFootTask->surfacePose(), targetContact.pose, duration_, ctl.plan.swingHeight());

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  // logger().addLogEntry("walking_phase", []() { return 1.; });

  logger().addLogEntry("MyMeasuresInSSP_range_"+targetContact.surfaceName, [this]() { return range_; });
  logger().addLogEntry("MyMeasuresInSSP_xpointGround_"+targetContact.surfaceName, [this]() { return x_pointGround_; });
  logger().addLogEntry("MyMeasuresInSSP_ypointGround_"+targetContact.surfaceName, [this]() { return y_pointGround_; });
  logger().addLogEntry("MyMeasuresInSSP_zpointGround_"+targetContact.surfaceName, [this]() { return z_pointGround_; });
  logger().addLogEntry("MyMeasuresInSSP_altitudeGround_"+targetContact.surfaceName, [this]() { return altitudeGround_; });
  
  gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Label("k", [this]() { return this->k_; }));
  logger().addLogEntry("MyMeasuresInSSP_k_"+targetContact.surfaceName, [this]() { return k_; });

  // logger().addLogEntry("k", [this]() { return k_; });

  swingFoot_.addLogEntries(logger());

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::SingleSupport::teardown()
{
  controller().solver().removeTask(swingFootTask);

  logger().removeLogEntry("contact_impulse");
  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("walking_phase");
  
  auto & targetContact = controller().targetContact();
  logger().removeLogEntry("MyMeasuresInSSP_range_"+targetContact.surfaceName);
  logger().removeLogEntry("MyMeasuresInSSP_xpointGround_"+targetContact.surfaceName);
  logger().removeLogEntry("MyMeasuresInSSP_ypointGround_"+targetContact.surfaceName);
  logger().removeLogEntry("MyMeasuresInSSP_zpointGround_"+targetContact.surfaceName);
  logger().removeLogEntry("MyMeasuresInSSP_altitudeGround_"+targetContact.surfaceName);

  logger().removeLogEntry("MyMeasuresInSSP_k_"+targetContact.surfaceName);  
  gui()->removeElement({"Walking", "Main"}, "k");
  
  swingFoot_.removeLogEntries(logger());

}

bool states::SingleSupport::checkTransitions()
{
  if(remTime_ < 0.)
  {
    output("DoubleSupport");
    controller().nrFootsteps_ ++;
    return true;
  }
  return false;
}

void states::SingleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  updateSwingFoot();
  if(timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD)
  {
    updatePreview();
  }

  ctl.preview->integrate(pendulum(), dt);
  if(hasUpdatedMPCOnce_)
  {
    pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.supportContact().p(), ctl.supportContact().normal());
    pendulum().completeIPM(ctl.supportContact().p(), ctl.supportContact().normal());
  }
  else // still in DSP of preview
  {
    pendulum().completeIPM(ctl.prevContact().p(), ctl.prevContact().normal());
  }

  stabilizer()->target(pendulum().com(), pendulum().comd(), pendulum().comdd(), pendulum().zmp());

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;

  std::string swingFootSurfaceName;

  {
    // SoftFoot: calculate ground roughness with range sensors 

    std::string SensorName;
    std::string jointName;
    if(ctl.targetContact().surfaceName == "LeftFootCenter") // This is the name "LeftFootCenter" or "RightFootCenter"
    {
      SensorName = "LeftFootRangeSensor";
      jointName = "L_VARSTIFF";
    }
    else
    {
      SensorName = "RightFootRangeSensor";
      jointName = "R_VARSTIFF";
    }

    // Return the parent body of the sensor (phalanx)
    const std::string& BodyOfSensor = ctl.robot().device<mc_mujoco::RangeSensor>(SensorName).parent(); 
    // Access the position of body name in world coordinates (phalanx position)
    sva::PTransformd X_0_ph = ctl.realRobot().bodyPosW(BodyOfSensor); 
    // Returns the transformation from the parent body to the sensor
    const sva::PTransformd& X_ph_s = ctl.robot().device<mc_mujoco::RangeSensor>(SensorName).X_p_s();
    // Sensor position in global frame Z coordinate
    range_ = ctl.robot().device<mc_mujoco::RangeSensor>(SensorName).data(); 
    const sva::PTransformd X_s_m = sva::PTransformd(Eigen::Vector3d(0,0,range_));
    sva::PTransformd X_0_m = X_s_m*X_ph_s*X_0_ph;
    x_pointGround_ = X_0_m.translation().x();
    y_pointGround_ = X_0_m.translation().y();
    z_pointGround_ = X_0_m.translation().z();

    if(remTime_ > 0.25*duration_) // Phase 1
    {
      altitudeGround_.push_back(z_pointGround_);
      k_ = 10;
    }
    else if(!isStiffnessUpdated_) // Phase 2
    {
      isStiffnessUpdated_ = true;
      variable_stiffness::connectionFile srv;
      srv.request.profile = altitudeGround_;

      if (client_.call(srv))
      {
        ROS_INFO("Stiffness: %f", (double)srv.response.stiffness);
        k_ = srv.response.stiffness;
        // Solution to modify the variable stiffness
        auto stiffnessToAngle = [this](double VarStiff) 
          {
            double angle_low = 0;
            double angle_high = 1;
            double stiffness_low = 0;
            double stiffness_high = 100;
            return angle_low+(VarStiff-stiffness_low)*(angle_high-angle_low)/(stiffness_high-stiffness_low);
          };
        ctl.robot().q()[ctl.robot().jointIndexByName(jointName)][0] = stiffnessToAngle(k_);
      }
      else
      {
        ROS_ERROR("Failed to call service connectionFile");
      }
      
    }  
  }



}

void states::SingleSupport::updateSwingFoot()
{
  auto & ctl = controller();
  auto & targetContact = ctl.targetContact();
  auto & supportContact = ctl.supportContact();
  double dt = ctl.timeStep;

  if(!stabilizer()->inDoubleSupport())
  {
    bool liftPhase = (remTime_ > duration_ / 3.);
    bool touchdownDetected = detectTouchdown(swingFootTask, targetContact.pose);
    if(liftPhase || !touchdownDetected)
    {
      swingFoot_.integrate(dt);
      swingFootTask->target(swingFoot_.pose());
      // T_0_s transforms a MotionVecd variable from world to surface frame
      sva::PTransformd T_0_s(swingFootTask->surfacePose().rotation());
      swingFootTask->refVelB(T_0_s * swingFoot_.vel());
      swingFootTask->refAccel(T_0_s * swingFoot_.accel());
    }
    else
    {
      if(supportContact.surfaceName == "LeftFootCenter")
      {
        stabilizer()->setContacts(
            {{ContactState::Left, supportContact.pose}, {ContactState::Right, targetContact.pose}});
      }
      else
      {
        stabilizer()->setContacts(
            {{ContactState::Left, targetContact.pose}, {ContactState::Right, supportContact.pose}});
      }
    }
  }
}

bool states::SingleSupport::detectTouchdown(const std::shared_ptr<mc_tasks::SurfaceTransformTask> footTask,
                                            const sva::PTransformd & contactPose)
{
  const sva::PTransformd X_0_s = footTask->surfacePose();
  const sva::PTransformd & X_0_c = contactPose;
  sva::PTransformd X_c_s = X_0_s * X_0_c.inv();
  double xDist = std::abs(X_c_s.translation().x());
  double yDist = std::abs(X_c_s.translation().y());
  double zDist = std::abs(X_c_s.translation().z());
  double Fz = controller().robot().surfaceWrench(footTask->surface()).force().z();
  return (xDist < 0.03 && yDist < 0.03 && zDist < 0.03 && Fz > 50.);
}

void states::SingleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
  if(ctl.isLastSSP() || ctl.pauseWalking)
  {
    ctl.nextDoubleSupportDuration(ctl.plan.finalDSPDuration());
    ctl.mpc().phaseDurations(remTime_, ctl.plan.finalDSPDuration(), 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(remTime_, ctl.doubleSupportDuration(), ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
    hasUpdatedMPCOnce_ = true;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("SingleSupport", lipm_walking::states::SingleSupport)
