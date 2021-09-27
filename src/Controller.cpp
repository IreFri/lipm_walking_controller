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

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/utils/clamp.h>

using Color = mc_rtc::gui::Color;

namespace lipm_walking
{

Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> robotModule,
                       double dt,
                       const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(robotModule, dt, config), planInterpolator(gui()), externalFootstepPlanner(*this),
  halfSitPose(controlRobot().mbc().q)
{
  auto robotConfig = config("robot_models")(controlRobot().name());
  auto planConfig = config("plans")(controlRobot().name());
  if(planConfig.has("external"))
  {
    externalFootstepPlanner.configure(planConfig("external"));
  }

  mc_rtc::log::info("Loading default stabilizer configuration");
  defaultStabilizerConfig_ = robot().module().defaultLIPMStabilizerConfiguration();
  if(robotConfig.has("stabilizer"))
  {
    mc_rtc::log::info("Loading additional stabilizer configuration");
    defaultStabilizerConfig_.load(robotConfig("stabilizer"));
    // mc_rtc::log::info("Stabilizer Configuration:\n{}", defaultStabilizerConfig_.save().dump(true));
  }

  // Patch CoM height and step width in all plans
  std::vector<std::string> plans = planConfig.keys();
  double comHeight = defaultStabilizerConfig_.comHeight;
  for(const auto & p : plans)
  {
    auto plan = planConfig(p);
    if(!plan.has("com_height"))
    {
      plan.add("com_height", comHeight);
    }
  }

  double postureStiffness = config("tasks")("posture")("stiffness");
  double postureWeight = config("tasks")("posture")("weight");
  postureTask = getPostureTask(robot().name());
  postureTask->stiffness(postureStiffness);
  postureTask->weight(postureWeight);

  // Set half-sitting pose for posture task
  const auto & halfSit = robotModule->stance();
  const auto & refJointOrder = robot().refJointOrder();
  for(unsigned i = 0; i < refJointOrder.size(); ++i)
  {
    if(robot().hasJoint(refJointOrder[i]))
    {
      halfSitPose[robot().jointIndexByName(refJointOrder[i])] = halfSit.at(refJointOrder[i]);
    }
  }

  // Read sole properties from robot model and configuration file
  sva::PTransformd X_0_lfc = controlRobot().surfacePose("LeftFootCenter");
  sva::PTransformd X_0_rfc = controlRobot().surfacePose("RightFootCenter");
  sva::PTransformd X_0_lf = controlRobot().surfacePose("LeftFoot");
  sva::PTransformd X_lfc_lf = X_0_lf * X_0_lfc.inv();
  sva::PTransformd X_rfc_lfc = X_0_lfc * X_0_rfc.inv();
  double stepWidth = X_rfc_lfc.translation().y();
  sole_ = robotConfig("sole");

  // If an ankle offset is specified use it, otherwise compute it
  if(robotConfig("sole").has("leftAnkleOffset"))
  {
    sole_.leftAnkleOffset = robotConfig("sole")("leftAnkleOffset");
  }
  else
  {
    sole_.leftAnkleOffset = X_lfc_lf.translation().head<2>();
  }

  // Configure MPC solver
  mpcConfig_ = config("mpc");
  mpc_.sole(sole_);

  // ====================
  // Create Stabilizer
  // - Default configuration from the robot module
  // - Additional configuration from the configuration, in section robot_models/robot_name/stabilizer
  // ====================
  // clang-format off
  stabilizer_.reset(
    new mc_tasks::lipm_stabilizer::StabilizerTask(
      solver().robots(),
      solver().realRobots(),
      robot().robotIndex(),
      defaultStabilizerConfig_.leftFootSurface,
      defaultStabilizerConfig_.rightFootSurface,
      defaultStabilizerConfig_.torsoBodyName,
      solver().dt()));
  // clang-format on
  stabilizer_->configure(defaultStabilizerConfig_);

  // Read footstep plans from configuration
  planInterpolator.configure(planConfig);
  planInterpolator.stepWidth(stepWidth);
  std::string initialPlan = planInterpolator.availablePlans()[0];
  config("initial_plan", initialPlan);
  loadFootstepPlan(initialPlan);

  // =========================
  // Create Swing foot tasks
  // =========================
  double swingWeight = 1000;
  double swingStiffness = 500;
  if(robotConfig.has("swingfoot"))
  {
    robotConfig("swingfoot")("weight", swingWeight);
    robotConfig("swingfoot")("stiffness", swingStiffness);
  }
  swingFootTaskRight_.reset(new mc_tasks::SurfaceTransformTask("RightFootCenter", robots(), robots().robotIndex(),
                                                               swingWeight, swingStiffness));
  swingFootTaskLeft_.reset(new mc_tasks::SurfaceTransformTask("LeftFootCenter", robots(), robots().robotIndex(),
                                                              swingWeight, swingStiffness));

  addLogEntries(logger());
  mpc_.addLogEntries(logger());

  if(gui_)
  {
    addGUIElements(gui_);
    mpc_.addGUIElements(gui_);
  }

  // Update observers
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio_);
  });

  for(const auto & r: realRobots())
  {
    const std::string name = r.name();
    gui()->addElement({"RealRobots"},
      mc_rtc::gui::Robot(name,
        [name, this]() -> const mc_rbdyn::Robot &
        {
          return this->realRobot(name);
        }
      )
    );
  }

  // Softfoot
  gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Label("nrFootsteps", [this]() { return this->nrFootsteps_; }));
  logger().addLogEntry("nrFootsteps", [this]() { return this->nrFootsteps_; });

  gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Label("cost", [this]() { return this->cost_; }));
  logger().addLogEntry("cost", [this]() { return cost_; });

  mc_rtc::log::success("LIPMWalking controller init done.");
}

void Controller::addLogEntries(mc_rtc::Logger & logger)
{
  logger.addLogEntry("controlRobot_LeftFoot", [this]() { return controlRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry("controlRobot_LeftFootCenter", [this]() { return controlRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("controlRobot_RightFoot", [this]() { return controlRobot().surfacePose("RightFoot"); });
  logger.addLogEntry("controlRobot_RightFootCenter",
                     [this]() { return controlRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("mpc_failures", [this]() { return nbMPCFailures_; });
  logger.addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
  logger.addLogEntry("left_foot_ratio_measured", [this]() { return measuredLeftFootRatio(); });
  logger.addLogEntry("plan_com_height", [this]() { return plan.comHeight(); });
  logger.addLogEntry("plan_double_support_duration", [this]() { return plan.doubleSupportDuration(); });
  logger.addLogEntry("plan_final_dsp_duration", [this]() { return plan.finalDSPDuration(); });
  logger.addLogEntry("plan_init_dsp_duration", [this]() { return plan.initDSPDuration(); });
  logger.addLogEntry("plan_landing_duration", [this]() { return plan.landingDuration(); });
  logger.addLogEntry("plan_landing_pitch", [this]() { return plan.landingPitch(); });
  logger.addLogEntry("plan_ref_vel", [this]() { return plan.supportContact().refVel; });
  logger.addLogEntry("plan_single_support_duration", [this]() { return plan.singleSupportDuration(); });
  logger.addLogEntry("plan_swing_height", [this]() { return plan.swingHeight(); });
  logger.addLogEntry("plan_takeoff_duration", [this]() { return plan.takeoffDuration(); });
  logger.addLogEntry("plan_takeoff_offset", [this]() { return plan.takeoffOffset(); });
  logger.addLogEntry("plan_takeoff_pitch", [this]() { return plan.takeoffPitch(); });
  logger.addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
  logger.addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
}

void Controller::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  using namespace mc_rtc::gui;

  gui->addElement(
      {"Markers", "Sole"},
      mc_rtc::gui::Point3D("Target Ankle Pos", [this]() { return this->targetContact().anklePos(sole_); }),
      mc_rtc::gui::Point3D("Support Ankle Pos", [this]() { return this->supportContact().anklePos(sole_); }));

  gui->addElement({"Walking", "Main"},
                  Button("# EMERGENCY STOP",
                         [this]() {
                           mc_rtc::log::error("EMERGENCY STOP!");
                           emergencyStop = true;
                           this->interrupt();
                         }),
                  Button("Reset", [this]() {
                    mc_rtc::log::warning("Reset to Initial state");
                    this->resume("Initial");
                  }));

  gui->addElement({"Walking", "CoM"}, Label("Plan name", [this]() { return plan.name; }));

  gui->addElement({"Walking", "Swing"}, Label("Plan name", [this]() { return plan.name; }),
                  NumberInput(
                      "Swing height [m]", [this]() { return plan.swingHeight(); },
                      [this](double height) { plan.swingHeight(height); }),
                  NumberInput(
                      "Takeoff duration [s]", [this]() { return plan.takeoffDuration(); },
                      [this](double duration) { plan.takeoffDuration(duration); }),
                  NumberInput(
                      "Takeoff pitch [rad]", [this]() { return plan.takeoffPitch(); },
                      [this](double pitch) { plan.takeoffPitch(pitch); }),
                  NumberInput(
                      "Landing duration [s]", [this]() { return plan.landingDuration(); },
                      [this](double duration) { plan.landingDuration(duration); }),
                  NumberInput(
                      "Landing pitch [rad]", [this]() { return plan.landingPitch(); },
                      [this](double pitch) { plan.landingPitch(pitch); }));

  gui->addElement({"Walking", "Timings"}, Label("Plan name", [this]() { return plan.name; }),
                  NumberInput(
                      "Initial DSP duration [s]", [this]() { return plan.initDSPDuration(); },
                      [this](double duration) { plan.initDSPDuration(duration); }),
                  NumberInput(
                      "SSP duration [s]", [this]() { return plan.singleSupportDuration(); },
                      [this](double duration) {
                        constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
                        duration = std::round(duration / T) * T;
                        plan.singleSupportDuration(duration);
                      }),
                  NumberInput(
                      "DSP duration [s]", [this]() { return plan.doubleSupportDuration(); },
                      [this](double duration) {
                        constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
                        duration = std::round(duration / T) * T;
                        plan.doubleSupportDuration(duration);
                      }),
                  NumberInput(
                      "Final DSP duration [s]", [this]() { return plan.finalDSPDuration(); },
                      [this](double duration) { plan.finalDSPDuration(duration); }));
  gui->addElement(
      {"Walking", "Sole"},
      mc_rtc::gui::Label("Ankle offset",
                         []() {
                           return std::string{
                               "position of ankle w.r.t to left foot center. The corresponding offset for the right "
                               "foot is computed assuming that the feet are symetrical in the lateral direction"};
                         }),
      mc_rtc::gui::ArrayInput(
          "Left Ankle Offset", [this]() -> const Eigen::Vector2d & { return sole_.leftAnkleOffset; },
          [this](const Eigen::Vector2d & offset) {
            sole_.leftAnkleOffset = offset;
            mpc_.sole(sole_);
          }),
      mc_rtc::gui::ArrayLabel("Right Ankle Offset",
                              [this]() -> Eigen::Vector2d {
                                return {sole_.leftAnkleOffset.x(), -sole_.leftAnkleOffset.y()};
                              }),
      mc_rtc::gui::ArrayLabel("Sole half width/length",
                              [this]() {
                                return Eigen::Vector2d{sole_.halfWidth, sole_.halfLength};
                              })

  );
}

void Controller::reset(const mc_control::ControllerResetData & data)
{
  config()("observerPipelineName", observerPipelineName_);
  if(!hasObserverPipeline(observerPipelineName_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("LIPMWalking requires an observer pipeline named \"{}\"",
                                                     observerPipelineName_);
  }

  mc_control::fsm::Controller::reset(data);

  stabilizer_->reset();
  setContacts({{ContactState::Left, supportContact().pose}, {ContactState::Right, targetContact().pose}}, true);

  if(gui_)
  {
    gui_->removeCategory({"Contacts"});
  }
}

void Controller::setContacts(
    const std::vector<std::pair<mc_tasks::lipm_stabilizer::ContactState, sva::PTransformd>> & contacts,
    bool fullDoF)
{
  stabilizer()->setContacts(contacts);
  auto rName = robot().name();
  auto gName = "ground";
  auto gSurface = "AllGround";
  auto friction = stabilizer()->config().friction;
  removeContact({rName, gName, stabilizer()->footSurface(mc_tasks::lipm_stabilizer::ContactState::Left), gSurface});
  removeContact({rName, gName, stabilizer()->footSurface(mc_tasks::lipm_stabilizer::ContactState::Right), gSurface});
  for(const auto & contact : contacts)
  {
    const auto & rSurface = stabilizer()->footSurface(contact.first);
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    if(!fullDoF)
    {
      dof(0) = 0;
      dof(1) = 0;
      dof(5) = 0;
    }
    addContact({rName, gName, rSurface, gSurface, friction, dof});
  }
}

void Controller::leftFootRatio(double ratio)
{
  double maxRatioVar = 1.5 * timeStep / plan.doubleSupportDuration();
  if(std::abs(ratio - leftFootRatio_) > maxRatioVar)
  {
    mc_rtc::log::warning("Left foot ratio jumped from {} to {}", leftFootRatio_, ratio);
  }
  leftFootRatio_ = clamp(ratio, 0., 1., "leftFootRatio");
}

bool Controller::run()
{
  const auto & observerp = observerPipeline(observerPipelineName_);
  if(!observerp.success())
  {
    mc_rtc::log::error("Required pipeline \"{}\" for real robot observation failed to run!", observerPipelineName_);
    for(const auto & observer : observerp.observers())
    {
      if(!observer.success())
      {
        mc_rtc::log::error("Observer \"{}\" failed with error \"{}\"", observer.observer().name(),
                           observer.observer().error());
      }
    }
    return false;
  }
  if(emergencyStop)
  {
    return false;
  }
  if(pauseWalkingRequested)
  {
    pauseWalkingCallback();
  }
  if(!mc_control::fsm::Controller::running())
  {
    return mc_control::fsm::Controller::run();
  }

  ctlTime_ += timeStep;

  warnIfRobotIsInTheAir();

  // Update cost
  {
    double zmp = 0.0;
    // ZMP Error
    {
      const Eigen::Vector3d & zmp_ref = pendulum_.zmp();
      const Eigen::Vector3d & zmp_mes = stabilizer_->measuredZMP();
      const double zmp_error = (zmp_ref - zmp_mes).norm();
      // sum_zmp_error_ += zmp_error;
      // ++ nr_zmp_error_;
      // zmp = lambda_zmp_ * (sum_zmp_error_ / nr_zmp_error_);

      zmp_.push_back(zmp_error);
      double sum = std::accumulate(zmp_.begin(), zmp_.end(), 0.0);
      double mean = sum / zmp_.size();
      std::vector<double> diff(zmp_.size());
      std::transform(zmp_.begin(), zmp_.end(), diff.begin(), [mean](double x) { return x - mean; });
      double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      double stdev = std::sqrt(sq_sum / zmp_.size());
      zmp = lambda_zmp_ * mean + lambda_zmp_ * lambda_zmp_ * stdev;
    }

    double CoM = 0.0;
    // CoM error
    {
      const Eigen::Vector3d & CoM_ref = pendulum_.com();
      const Eigen::Vector3d & CoM_mes = stabilizer_->measuredCoM();
      const double CoM_error = (CoM_ref - CoM_mes).norm();
      // sum_CoM_error_ += CoM_error;
      // ++ nr_CoM_error_;
      // CoM = lambda_CoM_* (sum_CoM_error_ / nr_CoM_error_);

      CoM_.push_back(CoM_error);
      double sum = std::accumulate(CoM_.begin(), CoM_.end(), 0.0);
      double mean = sum / CoM_.size();
      std::vector<double> diff(CoM_.size());
      std::transform(CoM_.begin(), CoM_.end(), diff.begin(), [mean](double x) { return x - mean; });
      double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      double stdev = std::sqrt(sq_sum / CoM_.size());
      CoM = lambda_CoM_ * mean + lambda_CoM_ * lambda_CoM_ * stdev;
    }

    double R_Ankle_P_Torque = 0.0;
    double L_Ankle_P_Torque = 0.0;
    double R_Ankle_P_Torque_real = 0.0;
    double L_Ankle_P_Torque_real = 0.0;
    //double R_Ankle_P_Torque_target = 0.0;
    //double L_Ankle_P_Torque_target = 0.0;
    // Right and Left ankle pitch torque error
    // {
    //   R_Ankle_P_Torque_real = robot().jointTorques()[robot().jointIndexByName("R_ANKLE_P")]; 
    //   L_Ankle_P_Torque_real = robot().jointTorques()[robot().jointIndexByName("L_ANKLE_P")];  
    //   //R_Ankle_P_Torque_target = robot().jointTorque()[robot().jointIndexByName("R_ANKLE_P")][0]; 
    //   //L_Ankle_P_Torque_target = robot().jointTorque()[robot().jointIndexByName("L_ANKLE_P")][0];   
    //   //const double R_Ankle_P_Torque_error = R_Ankle_P_Torque_target - R_Ankle_P_Torque_target;
    //   //const double L_Ankle_P_Torque_error = L_Ankle_P_Torque_target - L_Ankle_P_Torque_target;

    //   //R_Ankle_P_Torque_.push_back(R_Ankle_P_Torque_error);
    //   R_Ankle_P_Torque_.push_back(R_Ankle_P_Torque_real);
    //   double sum_R = std::accumulate(R_Ankle_P_Torque_.begin(), R_Ankle_P_Torque_.end(), 0.0);
    //   double mean_R = sum_R / R_Ankle_P_Torque_.size();
    //   std::vector<double> diff_R(R_Ankle_P_Torque_.size());
    //   std::transform(R_Ankle_P_Torque_.begin(), R_Ankle_P_Torque_.end(), diff_R.begin(), [mean_R](double x) { return x - mean_R; });
    //   double sq_sum_R = std::inner_product(diff_R.begin(), diff_R.end(), diff_R.begin(), 0.0);
    //   double stdev_R = std::sqrt(sq_sum_R / R_Ankle_P_Torque_.size());
    //   R_Ankle_P_Torque = std::abs(lambda_R_Ankle_P_Torque_ * mean_R + lambda_R_Ankle_P_Torque_ * lambda_R_Ankle_P_Torque_ * stdev_R);

    //   //L_Ankle_P_Torque_.push_back(L_Ankle_P_Torque_error);
    //   L_Ankle_P_Torque_.push_back(L_Ankle_P_Torque_real);
    //   double sum_L = std::accumulate(L_Ankle_P_Torque_.begin(), L_Ankle_P_Torque_.end(), 0.0);
    //   double mean_L = sum_L / L_Ankle_P_Torque_.size();
    //   std::vector<double> diff_L(L_Ankle_P_Torque_.size());
    //   std::transform(L_Ankle_P_Torque_.begin(), L_Ankle_P_Torque_.end(), diff_L.begin(), [mean_L](double x) { return x - mean_L; });
    //   double sq_sum_L = std::inner_product(diff_L.begin(), diff_L.end(), diff_L.begin(), 0.0);
    //   double stdev_L = std::sqrt(sq_sum_L / L_Ankle_P_Torque_.size());
    //   L_Ankle_P_Torque = std::abs(lambda_L_Ankle_P_Torque_ * mean_L + lambda_L_Ankle_P_Torque_ * lambda_L_Ankle_P_Torque_ * stdev_L);

    // }

    double footstep = 0.0;
    // Footstep
    {
      //footstep_error_ = plan.contacts().size() / (static_cast<double>(nrFootsteps_) + 1.);
      // footstep_error_ = 1. / std::sqrt((static_cast<double>(nrFootsteps_) + 1.));
      footstep_error_ =  plan.contacts().size() - 2 - nrFootsteps_;
      footstep = lambda_footstep_ * footstep_error_;
    }

    // Distance between robot and final pose target 
    double Distance = 0.0;
    double error_distance = 0.0;
    if(datastore().has("SLAM::Robot"))   
    {
      if(!init_distance)
      {
        init_distance = true;
        const std::vector<Contact> & ListContacts = plan.contacts();
        sva::PTransformd Foot_Target_Pose = sva::interpolate(ListContacts[ListContacts.size()-2].pose, ListContacts[ListContacts.size()-1].pose, 0.5);
        sva::PTransformd Foot_Target_Initial_Pose = sva::interpolate(ListContacts[0].pose, ListContacts[1].pose, 0.5);
        sva::PTransformd initial_target = Foot_Target_Pose*Foot_Target_Initial_Pose.inv();
        const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot");
        sva::PTransformd R_Foot_Robot_Pose = estimatedRobot.surfacePose("RightFootCenter");
        sva::PTransformd L_Foot_Robot_Pose = estimatedRobot.surfacePose("LeftFootCenter");
        sva::PTransformd Foot_Robot_Pose = sva::interpolate(R_Foot_Robot_Pose, L_Foot_Robot_Pose, 0.5);
        foot_target_pose_ = initial_target*Foot_Robot_Pose;

        mc_rtc::log::success("foot_target_pose_ : {}", foot_target_pose_.translation().head<2>().transpose());

      }
      const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot");
      sva::PTransformd R_Foot_Robot_Pose = estimatedRobot.surfacePose("RightFootCenter");
      sva::PTransformd L_Foot_Robot_Pose = estimatedRobot.surfacePose("LeftFootCenter");
      sva::PTransformd Foot_Robot_Pose = sva::interpolate(R_Foot_Robot_Pose, L_Foot_Robot_Pose, 0.5);
      Distance = (foot_target_pose_.translation().head<2>()-Foot_Robot_Pose.translation().head<2>()).norm();
      error_distance = lambda_distance_ * Distance;

      //mc_rtc::log::success("foot_target_pose_ : {}, Foot_Robot_Pose : {}, error_distance : {}", foot_target_pose_.translation().head<2>().transpose(), Foot_Robot_Pose.translation().head<2>().transpose(), error_distance);
      mc_rtc::log::success("Distance : {}", Distance);

    }

    cost_ = - error_distance - zmp - CoM;
    //cost_ = - error_distance - zmp - R_Ankle_P_Torque - L_Ankle_P_Torque;
    //cost_ = - footstep - zmp - CoM;
    //mc_rtc::log::success("Foot_Target_Pose : {}, Foot_Robot_Pose : {}, error_distance : {}", Foot_Target_Pose.translation().head<2>(), Foot_Robot_Pose.translation().head<2>(), error_distance);
  }
  
  bool ret = mc_control::fsm::Controller::run();
  // if(mc_control::fsm::Controller::running())
  //{
  //  postureTask->posture(halfSitPose); // reset posture in case the FSM updated it
  //}
  return ret;
}

void Controller::pauseWalkingCallback(bool verbose)
{
  constexpr double MAX_HEIGHT_DIFF = 0.02; // [m]
  if(pauseWalking)
  {
    mc_rtc::log::warning("Already pausing, how did you get there?");
    return;
  }
  else if(std::abs(supportContact().z() - targetContact().z()) > MAX_HEIGHT_DIFF)
  {
    if(!pauseWalkingRequested || verbose)
    {
      mc_rtc::log::warning("Cannot pause on uneven ground, will pause later");
    }
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    pauseWalkingRequested = true;
  }
  else if(pauseWalkingRequested)
  {
    mc_rtc::log::warning("Pausing now that contacts are at same level");
    pauseWalkingRequested = false;
    pauseWalking = true;
  }
  else // (!pauseWalkingRequested)
  {
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    pauseWalking = true;
  }
}

void Controller::warnIfRobotIsInTheAir()
{
  static bool isInTheAir = false;
  constexpr double CONTACT_THRESHOLD = 30.; // [N]
  double leftFootForce = realRobot().forceSensor("LeftFootForceSensor").force().z();
  double rightFootForce = realRobot().forceSensor("RightFootForceSensor").force().z();
  if(leftFootForce < CONTACT_THRESHOLD && rightFootForce < CONTACT_THRESHOLD)
  {
    if(!isInTheAir)
    {
      mc_rtc::log::warning("Robot is in the air");
      isInTheAir = true;
    }
  }
  else
  {
    if(isInTheAir)
    {
      mc_rtc::log::info("Robot is on the ground again");
      isInTheAir = false;
    }
  }
}

void Controller::loadFootstepPlan(std::string name)
{
  bool loadingNewPlan = (plan.name != name);
  double initHeight = (plan.name.length() > 0) ? plan.supportContact().p().z() : 0.;
  FootstepPlan defaultPlan = planInterpolator.getPlan(name);
  if(loadingNewPlan)
  {
    plan.removeGUIElements(*gui());
    plan = defaultPlan;
    plan.name = name;
    mpc_.configure(mpcConfig_);
    if(!plan.mpcConfig.empty())
    {
      mpc_.configure(plan.mpcConfig);
    }
  }
  else if(plan.name != "external") // only reload contacts
  {
    plan.resetContacts(defaultPlan.contacts());
  }
  plan.complete(sole_);
  const sva::PTransformd & X_0_lf = controlRobot().surfacePose("LeftFootCenter");
  const sva::PTransformd & X_0_rf = controlRobot().surfacePose("RightFootCenter");
  plan.updateInitialTransform(X_0_lf, X_0_rf, initHeight);
  planInterpolator.worldReference(plan.initPose());
  planInterpolator.updateSupportPath(X_0_lf, X_0_rf);
  plan.rewind();

  double torsoPitch = plan.hasTorsoPitch() ? plan.torsoPitch() : defaultStabilizerConfig_.torsoPitch;
  stabilizer_->torsoPitch(torsoPitch);

  if(loadingNewPlan)
  {
    plan.addGUIElements(*gui());
    mc_rtc::log::info("Loaded footstep plan \"{}\"", name);
  }
}

void Controller::updatePlan(const std::string & name)
{
  if(name == "external")
  {
    planInterpolator.removeGUIElements();
    externalFootstepPlanner.activate();
    loadFootstepPlan(name);
    return;
  }
  if(plan.name == "external")
  {
    externalFootstepPlanner.deactivate();
  }

  planInterpolator.removeGUIElements();
  if(name.find("custom") != std::string::npos)
  {
    planInterpolator.addGUIElements();
    if(name.find("backward") != std::string::npos)
    {
      planInterpolator.restoreBackwardTarget();
    }
    else if(name.find("forward") != std::string::npos)
    {
      planInterpolator.restoreForwardTarget();
    }
    else if(name.find("lateral") != std::string::npos)
    {
      planInterpolator.restoreLateralTarget();
    }
    loadFootstepPlan(planInterpolator.customPlanName());
  }
  else // new plan is not custom
  {
    loadFootstepPlan(name);
  }
}

void Controller::startLogSegment(const std::string & label)
{
  if(segmentName_.length() > 0)
  {
    stopLogSegment();
  }
  segmentName_ = "t_" + std::to_string(++nbLogSegments_).erase(0, 1) + "_" + label;
  logger().addLogEntry(segmentName_, [this]() { return ctlTime_; });
}

void Controller::stopLogSegment()
{
  logger().removeLogEntry(segmentName_);
  segmentName_ = "";
}

bool Controller::updatePreview()
{
  mpc_.initState(pendulum());
  mpc_.comHeight(plan.comHeight());
  if(mpc_.buildAndSolve())
  {
    preview = mpc_.solution();
    return true;
  }
  else
  {
    nbMPCFailures_++;
    return false;
  }
}
} // namespace lipm_walking
