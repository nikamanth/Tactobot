/*
*
*/

#pragma once

#include <string>
#include <vector>
#include <redox.hpp>
#include <atomic>
#include <jsoncpp/json/json.h>

#include <barrett/products/product_manager.h>
#include <barrett/systems.h>

// Only support 7 DOF WAM right now
static const int DOF = 7;

// The macro below makes a number of typedefs that allow convenient access
// to commonly used barrett::units. For example, the typedefs establish
// "jp_type" as a synonym for "units::JointPositions<DOF>::type". This macro
// (along with a few others) is defined in barrett/units.h.
BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

/**
*
*/
class CManipsWamDriver {

public:

  /**
  *
  */
  CManipsWamDriver(
      std::string redis_host = "localhost",
      int redis_port = 6379,
      std::string sensors_key = "barrett:wam:sensors",
      std::string actuators_key = "barrett:wam:actuators"
  );

  /**
  *
  */
  ~CManipsWamDriver();

  /**
  *
  */
  bool connect();

  /**
  *
  */
  void run();

  /**
  * Control mode.
  */
  enum class Mode {
    INIT, // Start state
    TORQUE, // Receive torque commands directly
    JOINT_RAW, // Command joint position, low-level response
    JOINT_EASY, // Command joint positions, slow tracking
    CARTESIAN_EASY // Command tool positions, slow tracking
  };

private:

  void parseActuatorMessage(const std::string& msg);

  bool torqueCommand(const Json::Value& v);
  bool jointCommandEasy(const Json::Value& v);
  bool jointCommandRaw(const Json::Value& v);
  bool cartesianCommandEasy(const Json::Value& v);

  void sendSensorMessage();

  void disconnectAllSystems();

  const std::string redis_host_;
  const int redis_port_;

  const std::string sensors_key_;
  const std::string actuators_key_;

  redox::Redox pub_;
  redox::Subscriber sub_;

  Json::Reader reader_;
  Json::FastWriter writer_;
  Json::StyledWriter styled_writer_;

  Json::Value actuator_msg_;
  Json::Value sensor_msg_;

  Json::Value jp_obs_msg_;
  Json::Value jv_obs_msg_;
  Json::Value jt_obs_msg_;

  Json::Value cp_obs_msg_;
  Json::Value cv_obs_msg_;

  std::string sensor_command_;

  barrett::ProductManager pm_;

  barrett::systems::Wam<DOF>* wam_;

  barrett::systems::ExposedOutput<jp_type> jp_cmd_holder_;
  barrett::systems::ExposedOutput<jt_type> jt_cmd_holder_;

//  class PublisherSystem : public  barrett::systems::SingleIO<jp_type, jp_type> {
//  public:
//  protected:
//    virtual void operate() {
//      jp = input.getValue();
//      std::cout << "jp: " << jp.transpose() << std::endl;
//    }
//  private:
//    jp_type jp;
//  };
//
//  PublisherSystem pub_sys_;

  barrett::SafetyModule* safety_module_;
  std::atomic_bool running_ = {false};

  // Joint positions, velocities, accelerations
  jp_type jp_obs_;
  jv_type jv_obs_;
  ja_type ja_obs_;
  jt_type jt_obs_;

  // Cartesian force and torque
  cf_type cf_obs_;
  ct_type ct_obs_;

  // Tool position, velocity, acceleration
  cp_type cp_obs_;
  cv_type cv_obs_;
  ca_type ca_obs_;

  // Joint position and torque commands
  jp_type jp_cmd_;
  jt_type jt_cmd_;

  // Tool position and torque commands
  cp_type cp_cmd_;
  ct_type ct_cmd_;

  // Control mode
  Mode mode_;
};
