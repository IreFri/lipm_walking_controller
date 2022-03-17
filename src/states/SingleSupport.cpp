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

namespace lipm_walking
{ 
void states::SingleSupport::start()
{
  auto & ctl = controller();
  auto & supportContact = ctl.supportContact();
  auto & targetContact = ctl.targetContact();

  duration_ = ctl.singleSupportDuration();
  hasUpdatedMPCOnce_ = false;
  remTime_ = ctl.singleSupportDuration();
  stateTime_ = 0.;
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
  logger().addLogEntry("walking_phase", []() { return 1.; });

  logger().addLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ra", [this]() { return Ra_; });
  logger().addLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Rb", [this]() { return Rb_; });
  logger().addLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ranger_01", [this]() { return range_01_; });
  logger().addLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ranger_10", [this]() { return range_10_; });

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
  logger().removeLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ra");
  logger().removeLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Rb");
  logger().removeLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ranger_01");
  logger().removeLogEntry("MyMeasuresInSSP_"+targetContact.surfaceName+"_Ranger_10");
  
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
  
  // {
  //   // SoftFoot: calculate ground roughness with range sensors for variable stiffness adaptation
  //   std::string ranger_sensor;
  //   if(ctl.targetContact().surfaceName == "LeftFootCenter")
  //   { 
  //     ranger_sensor = "LeftFootRangeSensor";  // I say the name that then I will use to extract the data under in the two lines. 
  //   }
  //   else // if(ctl.targetContact().surfaceName == "RightFootCenter")
  //   {
  //     ranger_sensor = "RightFootRangeSensor";
  //   }
  //   // At the end of the if, ranger_sensor contains the range sensor associated to the targetContact.surfaceName i.e. the foot moving

  //   // In the following line const auto & is equal to const std::vector<double> &. auto automatically replace the type when it can be deduce automatically.
  //   // The compilator knows that robot().device<mc_openrtm::RangeSensor>("NameOfSensor").data() will return a const std::vector<double> &
  //   const double range = ctl.robot().device<mc_mujoco::RangeSensor>(ranger_sensor).data(); // He takes the correspond right and left depending of the ContactSurface

  //   // check if you have data from your sensor
  //   // Get the data from the sensor

  //   mc_rtc::log::info("Data for range sensor of {}: {}", ctl.targetContact().surfaceName, range);

  //   if(std::isnan(range))
  //   {
  //     // Only to log 0 to show that you miss data
  //     Ra_ = 0.0;
  //     Rb_ = 0.0; 
  //     mc_rtc::log::error("There is a nan element or infinite element");
  //   }
  //   else
  //   {
  //     // Computation for the current foot in the air, not for both 
  //     const double diff_square = pow(diff_between_01_10, 2);
  //     ranger_square_.push_back(diff_square);
  //     // For accumulate I have to put 0.0 and not 0. 0 means it will cast everyting in int and then do the sum, so 0.000215 will be convert to 0 and then sum, that's why it did not work: sorry
  //     const double sum = std::accumulate(ranger_square_.begin(), ranger_square_.end(), 0.0);
  //     const double mean = std::abs(sum) / (ranger_square_.size() + 1.0);
  //     Ra_ = std::sqrt(mean);
  //     Rb_ = diff_between_01_10; 

  //     // For both feet I have a lot of negative values
  //     mc_rtc::log::success("Foot in motion {}", ctl.targetContact().surfaceName);
  //     mc_rtc::log::success("ranger_square_.size() : {}", ranger_square_.size());
  //     mc_rtc::log::success("sum : {:.0e}", sum);
  //     mc_rtc::log::success("mean : {}", mean);
  //     mc_rtc::log::success("Ra_ : {}", Ra_);
  //     mc_rtc::log::success("Rb_ : {}", Rb_);
  //   }

  // }
  
  // if(ctl.supportContact.surfaceName == "LeftFootCenter")
  // {
  //   double RangeSensor_L1 = Range_L1[0];
  //   double RangeSensor_L10 = Range_L10[0];
  //   double RangeSensor_R1 = Range_R1[0];
  //   double RangeSensor_R10 = Range_R10[0];
  //   // mc_rtc::log::success("Range_L1 : {}", Range_L1[0]);
  //   // mc_rtc::log::success("Range_L10 : {}", Range_L10[0]);
  //   // mc_rtc::log::success("Range_R1 : {}", Range_R1[0]);
  //   // mc_rtc::log::success("Range_R10 : {}", Range_R10[0]);
  //   if (Range_L1.size() > 1 & Range_L10.size() > 1 & Range_R1.size() > 1 & Range_R10.size() > 1)
  //   { 
  //     double diff_Left = RangeSensor_L1- RangeSensor_L10;
  //     double diff_Right = RangeSensor_R1 - RangeSensor_R10;
  //     if(std::isinf(diff_Left))
  //     {
  //       diff_Left = 0.0;
  //     }
  //     if(std::isinf(diff_Right))
  //     {
  //       diff_Right = 0.0;
  //     }
  //     if(std::isnan(diff_Left) || std::isinf(diff_Left))
  //     {
  //       mc_rtc::log::error("There is a nan element or infinite element");
  //     }
  //     else
  //     {
  //       double diff_Left_square = pow(diff_Left,2);
  //       double diff_Right_square = pow(diff_Right,2);
  //       Range_left_square.push_back(diff_Left_square);
  //       Range_right_square.push_back(diff_Right_square);
  //       double sum_left = 0.0;
  //       double sum_right = 0.0;
  //       for(int i = 0; i < Range_left_square.size(); i++)
  //       {
  //         sum_left = sum_left + Range_left_square[i];
  //       }
  //       for(int i = 0; i < Range_right_square.size(); i++)
  //       {
  //         sum_right = sum_right + Range_right_square[i];
  //       }
  //       // double sum_left = std::accumulate(Range_left_square.begin(), Range_left_square.end(), 0);
  //       // double sum_right = std::accumulate(Range_right_square.begin(), Range_right_square.end(), 0);
  //       double mean_left = abs(sum_left)/Range_left_square.size();
  //       double mean_right = abs(sum_right)/Range_right_square.size();
  //       Ra_Left_ = sqrt(mean_left); 
  //       Ra_Right_ = sqrt(mean_right);
  //       Rb_Left_ = diff_Left; 
  //       Rb_Right_ = diff_Right;

  //       // mc_rtc::log::success("diff_Left : {}", diff_Left);
  //       // // mc_rtc::log::success(" Range_left_square : {}", fmt::join( Range_left_square, ", "));
  //       // mc_rtc::log::success("Range_left_square.size() : {}", Range_left_square.size());
  //       // mc_rtc::log::success("sum_left : {:.0e}", sum_left);
  //       // mc_rtc::log::success("mean_left : {}", mean_left);
  //       // mc_rtc::log::success("Ra_Left_ : {}", Ra_Left_);

        
  //     }      
  //   }
  //   else
  //   {
  //     mc_rtc::log::error("Range sensor does not have value");
  //   }
  // }











  
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
