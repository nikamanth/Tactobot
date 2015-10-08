/* This file is part of scl, a control and simulation library
for robots and biomechanical models.

scl is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

Alternatively, you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of
the License, or (at your option) any later version.

scl is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License and a copy of the GNU General Public License along with
scl. If not, see <http://www.gnu.org/licenses/>.
 */
/* \file CExampleApp.cpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CExampleApp.hpp"
#include "tasks/CTaskGcExample.hpp"
#include "tasks/CTaskOpExample.hpp"

#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>

#include <sutil/CSystemClock.hpp>

#include <chai3d.h>

using namespace chai3d;
using namespace redox;

namespace scl_app
{

  /** Default constructor. Sets stuff to zero.
   * Uses a task controller*/
  CExampleApp::CExampleApp() : CRobotApp(),
      has_communicated_with_robot_once_(false),
      has_updated_dyn_after_robot_comm_(false),
      tau_fade_in_time_constt(0.1)
  { }

  scl::sBool CExampleApp::initMyController(const std::vector<std::string>& argv, scl::sUInt args_parsed)
  {
    try
    {
      initRedox();
      sub_.subscribe(sensors_key,
          [&](const std::string& topic, const std::string& msg)
          {
              json_reader_.parse(msg, json_sensor_);

              // READING JOINT ANGLES
              float qi;
              if(json_sensor_.isMember("q") && json_sensor_["q"].isArray()) {
                for(int i = 0; i < 7; i++) {
                  qi = json_sensor_["q"][i].asFloat();
                  measured_q[i] = qi;
                  robot_.getData()->io_data_->sensors_.q_(i) = measured_q[i];
                }
              }

              // PRINTING JOINT ANGLES
              std::cout << "[IN: q] -> ";
              for(int i = 0; i < 6; i++) {
                //std::cout << measured_q[i] << " ";
              }
             // std::cout << measured_q[6] << std::endl;

              // READING JOINT VELOCITIES
              float dqi;
              if(json_sensor_.isMember("dq") && json_sensor_["dq"].isArray()) {
                for(int i = 0; i < 7; i++) {
                  dqi = json_sensor_["dq"][i].asFloat();
                  measured_dq[i] = dqi;
                }
              }

              // PRINTING JOINT VELOCITIES
              std::cout << "[IN: dq] -> ";
              for(int i = 0; i < 6; i++) {
                //std::cout << measured_dq[i] << " ";
              }
              //std::cout << measured_dq[6] << std::endl;

              // GENERATING TORQUES :
              // This should resolve the initial bump that we feel...
              if(false == has_updated_dyn_after_robot_comm_ || false == db_->s_gui_.ui_flag_[5])
              {
                for(int i = 0; i < 7; i++)
                { computed_tau[i] = 0.0;  }
              }
              else {
                for(int i = 0; i < 7; i++)
                {
                  double tau_diff = robot_.getData()->io_data_->actuators_.force_gc_commanded_(i) - computed_tau[i];
                  if(fabs(tau_diff) < 0.1)
                  { computed_tau[i] = robot_.getData()->io_data_->actuators_.force_gc_commanded_(i);  }
                  else
                  { computed_tau[i] = computed_tau[i] + tau_fade_in_time_constt * tau_diff; }
                }
              }

              // SENDING TORQUES
              std::stringstream ss;
              ss << "{\"tau\":[";
              for(int i = 0; i < 6; i++){
                ss << computed_tau[i] << ", ";
              }
              ss << computed_tau[6] << "]}";
              rdx_.command({"PUBLISH", actuators_key, ss.str()});

              // PRINTING TORQUES SENT
              std::cout << "[OUT: tau] -> ";
              for(int i = 0; i < 6; i++) {
                std::cout << computed_tau[i] << " ";
              }
              std::cout << computed_tau[6] << std::endl;

              has_communicated_with_robot_once_ = true;

            });

      //Ctr in array of args_parsed = (args_parsed - 1)
      //So ctr for un-parsed arg = (args_parsed - 1) + 1
      scl::sUInt args_ctr = args_parsed;

      // Check that we haven't finished parsing everything
      while(args_ctr < argv.size())
      {
        /* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */
        // else if (argv[args_ctr] == "-p")
        // { }
        if(true)
        {
          std::cout<<"\n Possible example task options: -xxx (you can change me to suit your needs!)";
          args_ctr++;
        }
      }

      return true;
    }
    catch(std::exception &e)
    { std::cout<<"\nCExampleApp::initMyController() : "<<e.what(); }
    return false;
  }

  void CExampleApp::initRedox()
  {
    if (!rdx_.connect(REDIS_HOST, REDIS_PORT) || !sub_.connect(REDIS_HOST, REDIS_PORT)) {
      throw std::runtime_error("Could not connect to Redis!");
    }

//    // Declare app
//    rdx_.commandSync({"SADD", "scl:apps", APP_NAME});
//
//    // Watchdog timer to keep app active
//    rdx_.commandLoop<std::string>({"SETEX", APP_NAME + ":active", "2", "1"}, NULL, 1);
  }

  scl::sBool CExampleApp::registerCustomDynamicTypes()
  {
    bool flag;
    flag = registerType_TaskGcEmpty();
    flag = flag && registerType_TaskOpExample();
    return flag;
  }

  scl::sBool CExampleApp::setInitialStateForUIAndDynamics()
  {
    bool flag;
    try
    {

      // NOTE TODO : Vinay 
      //Get joint angles a few times here.... Set them in the robot io data structure
      robot_.getData()->io_data_->sensors_.q_; // Set me

      //Compute dynamics and servo once to initialize matrices.
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();
      robot_.setGeneralizedCoordinatesToZero();
      robot_.setGeneralizedVelocitiesToZero();
      robot_.setGeneralizedAccelerationsToZero();
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();

      //Update the operational point tasks (if any)
      std::vector<SUiCtrlPointData>::iterator it,ite;
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        if(false == it->has_been_init_)
        { throw(std::runtime_error(std::string("UI Task not intialized: ")+it->name_)); }

        if(NULL==it->chai_pos_des_)
        { throw(std::runtime_error(std::string("UI Task's chai position vector is NULL: ")+it->name_)); }

        if(NULL==it->task_)
        { throw(std::runtime_error(std::string("UI Task's control object is null: ")+it->name_)); }

        it->task_->getPos(it->pos_);

        flag = (3 == it->pos_.rows() && 1 == it->pos_.cols()) ||
            (1 == it->pos_.rows() && 3 == it->pos_.cols());
        if( false == flag )
        { throw(std::runtime_error(std::string("UI task's control position vector size is incorrect: ")+it->name_)); }

        db_->s_gui_.ui_point_[it->ui_pt_] = it->pos_;

        //Using a tmp ref to simplify code.
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));
      }

      {
        //Update the operational point tasks (if any)
        std::vector<SUiCtrlPointData>::iterator it,ite;

        static Eigen::VectorXd tmp_vec;
        for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
        {
          it->task_->getPos(tmp_vec);
          db_->s_gui_.ui_point_[it->ui_pt_] << tmp_vec(0),tmp_vec(1),tmp_vec(2);
        } //Get the goal position.
      }

      return true;
    }
    catch(std::exception &e)
    { std::cerr<<"\nCExampleApp::setInitialStateForUIAndDynamics() : "<<e.what(); }
    return false;
  }

  void CExampleApp::stepMySimulation()
  {
    sutil::CSystemClock::tick(db_->sim_dt_);//Tick the clock.

    //Update the operational point tasks (if any)
    std::vector<SUiCtrlPointData>::iterator it,ite;

    if(db_->s_gui_.ui_flag_[5])
    {
      //Press 5 to give the keyboard control
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        it->task_->setGoalPos(db_->s_gui_.ui_point_[it->ui_pt_]);
      } //Set the goal position.
    }
    else
    {// NOTE : If the ui flag 5 is false, we always send zero torques to the robot...
      //Press 5 again to set the control point to the robot's current position
      static Eigen::VectorXd tmp_vec;
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        it->task_->getPos(tmp_vec);
        db_->s_gui_.ui_point_[it->ui_pt_] << tmp_vec(0),tmp_vec(1),tmp_vec(2);
      } //Get the goal position.
    }

    if(ctrl_ctr_%20 == 0)
    { // Every 2ms
      //Set the positions of the ui points
      for(it = taskvec_ui_ctrl_point_.begin(), ite = taskvec_ui_ctrl_point_.end(); it!=ite; ++it )
      {
        Eigen::Vector3d& tmp_ref = db_->s_gui_.ui_point_[it->ui_pt_];
        it->chai_pos_des_->setLocalPos(tmp_ref(0),tmp_ref(1),tmp_ref(2));

        it->task_->getPos(it->pos_);
        Eigen::VectorXd& tmp_ref2 = it->pos_;
        it->chai_pos_->setLocalPos(tmp_ref2(0),tmp_ref2(1),tmp_ref2(2));
      }
    }

    if(true == has_communicated_with_robot_once_){
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();
      has_updated_dyn_after_robot_comm_ = true;
    }
    else{
      robot_.computeDynamics();
      robot_.computeNonControlOperations();
      robot_.computeServo();
    }

    ctrl_ctr_++;//Increment the counter for dynamics computed.
  }
}
