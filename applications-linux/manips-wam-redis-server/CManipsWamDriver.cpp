/*

*/

#include <iostream>
#include <chrono>
#include <thread>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include "CManipsWamDriver.hpp"

using namespace barrett;
using namespace redox;

# define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE true
# define BARRETT_SMF_PROMPT_ON_ZEROING true

#ifndef BARRETT_SMF_WAM_CONFIG_PATH
#  define BARRETT_SMF_WAM_CONFIG_PATH NULL
#endif

CManipsWamDriver::CManipsWamDriver(
    std::string redis_host, int redis_port,
    std::string sensors_key, std::string actuators_key
) : redis_host_(redis_host), redis_port_(redis_port),
    sensors_key_(sensors_key), actuators_key_(actuators_key),
    mode_(Mode::INIT)
{
  // Give us pretty stack-traces when things die
  barrett::installExceptionHandler();
}

CManipsWamDriver::~CManipsWamDriver() {

//  wam_->idle();

  // Wait for the user to press Shift-idle
  pm_.getSafetyModule()->waitForMode(SafetyModule::IDLE);
}

bool CManipsWamDriver::connect() {

//  pub_.noWait(true);
//  sub_.noWait(true);

  // Connect to Redis
  if(pub_.connect(redis_host_, redis_port_) && (sub_.connect(redis_host_, redis_port_))) {
    std::cout << "Connected to the Redis server at "
        << redis_host_ << ":" << redis_port_ << "." << std::endl;
  } else {
    std::cerr << "Could not connect to the Redis server at "
        << redis_host_ << ":" << redis_port_ << "." << std::endl;
    return false;
  }

  pm_.waitForWam(BARRETT_SMF_PROMPT_ON_ZEROING);
  pm_.wakeAllPucks();

  if (pm_.foundWam3()) {
    std::cerr << "Only 7 DOF WAM supported, found 3 DOF!" << std::endl;
    return false;
  } else if (pm_.foundWam4()) {
    std::cerr << "Only 7 DOF WAM supported, found 4 DOF!" << std::endl;
    return false;
  } else if (pm_.foundWam7()) {
    wam_ = pm_.getWam7(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH);
  } else {
    std::cerr << "No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam()" << std::endl;
    return false;
  }

  safety_module_ = pm_.getSafetyModule();

  wam_->gravityCompensate(true);

  running_ = true;

  return true;
}

template<int R, int C, typename Units>
bool jsonToVector(const Json::Value& v, math::Matrix<R,C, Units>& dest) {

  for(int i = 0; i < v.size(); i++) {

    if(!v[i].isConvertibleTo(Json::ValueType::realValue)) {
      std::cerr << "Cannot parse double from " << v[i] << "!" << std::endl;
      return false;
    }

    dest[i] = v[i].asDouble();
  }

  return true;
}

//template<int R, int C, typename Units>
//bool vectorToJson(math::Matrix<R,C, Units>& dest, Json::Value& v) {
//
//  v = Json::arrayValue;
//
//  for(int i = 0; i < dest.size(); i++) {
//    // Append as string to get fixed precision (1e-6)
//    v.append(std::to_string(dest[i]));
//  }
//
//  return true;
//}

void CManipsWamDriver::disconnectAllSystems() {
  wam_->gravityCompensate(true);
  wam_->supervisoryController.disconnectInput();
  systems::disconnect(wam_->input);
}

bool CManipsWamDriver::torqueCommand(const Json::Value& v) {

  if(v.size() != DOF) {
    std::cerr << "Expecting " << DOF << " torques!" << std::endl;
    return false;
  }

  // TODO time this
  if(!jsonToVector(v, jt_cmd_)) return false;

//  std::cout << "Torque command: " << jt_cmd_.transpose() << std::endl;

  jt_cmd_holder_.setValue(jt_cmd_);

  if(mode_ != Mode::TORQUE) {
    mode_ = Mode::TORQUE;
    std::cout << "Entering torque control mode." << std::endl;
    disconnectAllSystems();
    systems::connect(jt_cmd_holder_.output, wam_->input);
  }

  return true;
}

bool CManipsWamDriver::jointCommandRaw(const Json::Value& v) {

  if(v.size() != DOF) {
    std::cerr << "Expecting " << DOF << " joint positions!" << std::endl;
    return false;
  }

  if(!jsonToVector(v, jp_cmd_)) return false;

//  std::cout << "Joint command: " << jp_cmd_.transpose() << std::endl;

  jp_cmd_holder_.setValue(jp_cmd_);

  if(mode_ != Mode::JOINT_RAW) {
    mode_ = Mode::JOINT_RAW;
    std::cout << "Entering raw joint control mode." << std::endl;
    disconnectAllSystems();
    wam_->gravityCompensate(true);
    wam_->trackReferenceSignal(jp_cmd_holder_.output);
  }

  return true;
}

bool CManipsWamDriver::jointCommandEasy(const Json::Value& v) {

  if(v.size() != DOF) {
    std::cerr << "Expecting " << DOF << " joint positions!" << std::endl;
    return false;
  }

  if(!jsonToVector(v, jp_cmd_)) return false;

//  std::cout << "Joint command: " << jp_cmd_.transpose() << std::endl;

  if(mode_ != Mode::JOINT_EASY) {
    mode_ = Mode::JOINT_EASY;
    std::cout << "Entering easy joint control mode." << std::endl;
    disconnectAllSystems();
    wam_->gravityCompensate(true);
  }

  wam_->moveTo(jp_cmd_);

  return true;
}


bool CManipsWamDriver::cartesianCommandEasy(const Json::Value& v) {

  if(v.size() != 3) {
    std::cerr << "Expecting " << 3 << " cartesian positions!" << std::endl;
    return false;
  }

  if(!jsonToVector(v, cp_cmd_)) return false;

//  std::cout << "Cartesian command: " << cp_cmd_.transpose() << std::endl;

  if(mode_ != Mode::CARTESIAN_EASY) {
    mode_ = Mode::CARTESIAN_EASY;
    std::cout << "Entering cartesian control mode." << std::endl;
    disconnectAllSystems();
    wam_->gravityCompensate(true);
  }

  wam_->moveTo(cp_cmd_);

  return true;
}

void CManipsWamDriver::parseActuatorMessage(const std::string& msg) {

  reader_.parse(msg, actuator_msg_);

//  std::cout << "[RECV] Actuator msg: " << styled_writer_.write(actuator_msg_) << std::endl;

  if(!actuator_msg_.isObject()) {
    std::cerr << "Expecting top-level JSON object!" << std::endl;
    return;
  }

  if(actuator_msg_.isMember("tau") && actuator_msg_["tau"].isArray()) {
    torqueCommand(actuator_msg_["tau"]);
  } else if(actuator_msg_.isMember("q") && actuator_msg_["q"].isArray()) {
    jointCommandEasy(actuator_msg_["q"]);
  } else if(actuator_msg_.isMember("q_raw") && actuator_msg_["q_raw"].isArray()) {
    jointCommandRaw(actuator_msg_["q_raw"]);
  } else if(actuator_msg_.isMember("p") && actuator_msg_["p"].isArray()) {
    cartesianCommandEasy(actuator_msg_["p"]);
  } else if(actuator_msg_.isMember("mode") && actuator_msg_["mode"].isString()) {
    std::string mode = actuator_msg_["mode"].asString();
    if(mode == "idle") {
      std::cout << "Idling." << std::endl;
      disconnectAllSystems();
      wam_->idle();
    } else if(mode == "gravity") {
      std::cout << "Entering gravity comp mode." << std::endl;
      disconnectAllSystems();
      wam_->gravityCompensate(true);
    } else if(mode == "home") {
      std::cout << "Going to home position." << std::endl;
      disconnectAllSystems();
      wam_->gravityCompensate(true);
      jp_type home;
      home << 0.0, -2.0, 0.0, 3.1, 0.0, 0.0, 0.0;
      wam_->moveTo(home);
    }
  }
}

void CManipsWamDriver::sendSensorMessage() {

  // READING

  jt_obs_ = wam_->getJointTorques();
  jp_obs_ = wam_->getJointPositions();
  jv_obs_ = wam_->getJointVelocities();

  cp_obs_ = wam_->getToolPosition();
  cv_obs_ = wam_->getToolVelocity();

  // SENDING JOINT ANGLES AND VELOCITIES
  
  std::stringstream ss;
  ss << "{";

  ss << "\"q\":[";
  for(int i = 0; i < jp_obs_.size() - 1; i++)
    ss << jp_obs_[i] << ",";
  ss << jp_obs_[jp_obs_.size()-1] << "],";

  ss << "\"dq\":[";
  for(int i = 0; i < jv_obs_.size() - 1; i++)
    ss << jv_obs_[i] << ",";
  ss << jv_obs_[jv_obs_.size()-1] << "],";

  ss << "}";
  pub_.command({"PUBLISH", sensors_key_, ss.str()});

  // PRINTING JOINT ANGLES AND VELOCITIES
  
  std::cout << "[OUT: q] -> ";
  for(int i = 0; i < 6; i++) {
    std::cout << jp_obs_[i] << " ";
  }
  std::cout << jp_obs_[6] << std::endl;
  std::cout << std::endl;

  std::cout << "[OUT: dq] -> ";
  for(int i = 0; i < 6; i++) {
    std::cout << jv_obs_[i] << " ";
  }
  std::cout << jv_obs_[6] << std::endl;
  std::cout << std::endl;

}

void CManipsWamDriver::run() {

  sub_.subscribe(actuators_key_, [&](const std::string& topic, const std::string& msg) {

    reader_.parse(msg, actuator_msg_);

    float tau_rec[7];
    float ti;
    if(actuator_msg_.isMember("tau") && actuator_msg_["tau"].isArray()) {
      for(int i = 0; i < 7; i++) {
        ti = actuator_msg_["tau"][i].asFloat();
        tau_rec[i] = ti;
      }
    }

    // PRINTING TORQUES RECEIVED

    std::cout << "[IN: tau] -> ";
    for(int i = 0; i < 6; i++) {
     std::cout << tau_rec[i] << " ";
    }

   std::cout << tau_rec[6] << std::endl;
   std::cout << std::endl;

   // WRITING THE TORQUES TO THE ROBOT

   torqueCommand(actuator_msg_["tau"]);

    //parseActuatorMessage(msg);
  });

  auto t0 = std::chrono::system_clock::now();
  auto dt = std::chrono::microseconds(500);
  auto t_goal = t0;

  // TODO why doesn't this work
//  systems::connect(wam_->jpOutput, pub_sys_.input);

  while(running_) {

    sendSensorMessage();

    // TODO See if we have event based capability for libbarrett
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    t_goal += dt;
    std::this_thread::sleep_until(t_goal);
  }
}
