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
/* \file STaskOpPos.hpp
 *
 *  Created on: Jan 1, 2011
 *
 *  Copyright (C) 2011
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#ifndef STASKOPORI_HPP_
#define STASKOPORI_HPP_

#include <scl/DataTypes.hpp>
#include <scl/control/task/data_structs/STaskBase.hpp>

#include <Eigen/Dense>
#include <string>

namespace scl_app
{

  class STaskOpOri : public scl::STaskBase
  {
  public:
    //Computed attributes (last measured, in x dimensional task-space)
    Eigen::Quaterniond lam_;             //Euler parameter in the global frame
    Eigen::VectorXd dlam_;            //rate Euler parameter in the global frame
    Eigen::VectorXd ddlam_;           //Acce Euler parameter in the global frame
    Eigen::VectorXd delta_phi_;

    Eigen::Quaterniond lam_goal_;        //Goal ori in the global frame
    Eigen::VectorXd dlam_goal_;       //Goal ori Velocity in the global frame
    Eigen::VectorXd ddlam_goal_;      //Goal ori Acceleration in the global frame
    Eigen::MatrixXd omega_inp;			//input for Omega Matrix (switch b/w force and pos control)
    double mass;                        // mass of payload at end-effector
    Eigen::Vector3d radius;                      // radius of mass wrt end-effector point
    Eigen::Vector3d pos_in_parent_; //Position in the parent link's local frame (x,y,z)
    std::string link_name_;         //The parent link
    const scl::SRigidBody *link_ds_;     //The parent link's parsed data structure

    scl::sFloat spatial_resolution_;     //Meters

    const scl::SRigidBodyDyn *rbd_;   //For quickly obtaining a task Jacobian

    scl::sBool flag_compute_op_gravity_;  ///< Use operational point gravity? Default = true
    scl::sBool flag_compute_op_cc_forces_;///< Use operational centrifugal/coriolis forces? Default = false
    scl::sBool flag_compute_op_inertia_;  ///< Use operational inertia? If true, set to identity. Default = true

    /** Default constructor sets stuff to S_NULL */
    STaskOpOri();

    /** Default destructor does nothing */
    virtual ~STaskOpOri();

    /** 1. Initializes the task specific data members.
     *
     * 2. Parses non standard task parameters,
     * which are stored in STaskBase::task_nonstd_params_.
     * Namely:
     *  (a) parent link name
     *  (b) pos in parent.*/
    virtual bool initTaskParams();
  };

}

#endif /* SOPPOINTTASK_HPP_ */
