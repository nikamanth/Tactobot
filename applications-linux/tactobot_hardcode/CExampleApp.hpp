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
/* \file CExampleApp.hpp
 *
 *  Created on: Sep 16, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef CEXAMPLEAPP_HPP_
#define CEXAMPLEAPP_HPP_

#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/CTaskComPos.hpp>

#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>

#include <redox.hpp>
#include <atomic>
#include <jsoncpp/json/json.h>

#include <stdlib.h>
#include <iostream>
#include <stdexcept>

namespace scl_app
{
  static const std::string APP_NAME = "scl_example_control";
  static const std::string REDIS_HOST = "localhost";
  static const int REDIS_PORT = 6379;

  // Loop frequencies (Hz)
  static const int CONTROL_LOOP_FREQ = 1000;
  static const int GRAPHICS_LOOP_FREQ = 40;

  class CExampleApp : public scl::CRobotApp
  {
  public:

    Json::FastWriter json_writer_;
    Json::Reader json_reader_;
    Json::Value json_actuator_;
    Json::Value json_sensor_;

    redox::Redox rdx_;
    redox::Subscriber sub_;
    float measured_q[7];
    float measured_dq[7];
    float computed_tau[7];
    std::string sensors_key = "wamBot:sensors";
    std::string actuators_key = "wamBot:actuators";

    bool has_communicated_with_robot_once_;
    bool has_updated_dyn_after_robot_comm_;
    const double tau_fade_in_time_constt;

    // ****************************************************
    //                 The main functions
    // ****************************************************
    /** Runs the task controller. */
    virtual void stepMySimulation();

    // ****************************************************
    //           The initialization functions
    // ****************************************************
    /** Default constructor. Sets stuff to zero. */
    CExampleApp();

    void initRedox();

    /** Default destructor. Does nothing. */
    virtual ~CExampleApp(){}

    /** Sets up the task controller. */
    virtual scl::sBool initMyController(const std::vector<std::string>& argv,
        scl::sUInt args_parsed);

    /** Register any custom dynamic types that you have. */
    virtual scl::sBool registerCustomDynamicTypes();

    /** Sets all the ui points to their current position and
     * run the dynamics once to flush the state. */
    virtual scl::sBool setInitialStateForUIAndDynamics();
  };

}

#endif /* CEXAMPLEAPP_HPP_ */
