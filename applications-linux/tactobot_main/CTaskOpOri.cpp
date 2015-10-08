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
/* \file CTaskOpPos.cpp
 *
 *  Created on: Aug 19, 2010
 *
 *  Copyright (C) 2010
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

#include "CTaskOpOri.hpp"


#include "STaskOpOri.hpp"

#include <stdio.h>
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <scl/Singletons.hpp>
#include <sutil/CRegisteredDynamicTypes.hpp>

#ifdef DEBUG
#include <cassert>
#endif

#include <Eigen/Dense>

//Don't always use it. Read comments in the model update function
#include <Eigen/SVD>

using namespace scl;

namespace scl_app
{

  CTaskOpOri::CTaskOpOri() :
              CTaskBase(),
              data_(S_NULL),
              use_svd_for_lambda_inv_(false)
  { }

  //************************
  // Inherited stuff
  //************************
  bool CTaskOpOri::init(STaskBase* arg_task_data,
      CDynamicsBase* arg_dynamics)
  {
    try
    {
      if(S_NULL == arg_task_data)
      { throw(std::runtime_error("Passed a null task data structure"));  }

      if(false == arg_task_data->has_been_init_)
      { throw(std::runtime_error("Passed an uninitialized task data structure"));  }

      if(S_NULL == arg_dynamics)
      { throw(std::runtime_error("Passed a null dynamics object"));  }

      if(false == arg_dynamics->hasBeenInit())
      { throw(std::runtime_error("Passed an uninitialized dynamics object"));  }

      data_ = dynamic_cast<STaskOpOri*>(arg_task_data);

      dynamics_ = arg_dynamics;
      data_->rbd_ = data_->gc_model_->rbdyn_tree_.at_const(data_->link_name_);
      if(S_NULL == data_->rbd_)
      { throw(std::runtime_error("Couldn't find link dynamics object")); }

      //Defaults
      singular_values_.setZero();

      //Try to use the householder qr instead of the svd in general
      //Computing this once here initializes memory and resizes qr_
      //It will be used later.
      qr_.compute(data_->M_task_);

      has_been_init_ = true;
    }
    catch(std::exception& e)
    {
      std::cerr<<"\nCTaskOpOri::init() :"<<e.what();
      has_been_init_ = false;
    }
    return has_been_init_;
  }

  STaskBase* CTaskOpOri::getTaskData()
  { return data_; }

  /** Sets the current goal position */
  bool CTaskOpOri::setGoalOri(const Eigen::Quaterniond & arg_goal)
  {
    //TODO : lam has dimension 4 but dof is 3...how to make consistent
    if((data_->dof_task_ == arg_goal.coeffs().cols()-1 && 1 == arg_goal.coeffs().rows()) ||
        (1 == arg_goal.coeffs().cols() && data_->dof_task_ == arg_goal.coeffs().rows()-1) )
    {
      data_->lam_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpOri::setGoalOri() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal velocity */
  bool CTaskOpOri::setGoalVel(const Eigen::VectorXd & arg_goal)
  {
    if((data_->dof_task_ == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && data_->dof_task_ == arg_goal.rows()) )
    {
      data_->dlam_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpOri::setGoalVel() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  /** Sets the current goal acceleration */
  bool CTaskOpOri::setGoalAcc(const Eigen::VectorXd & arg_goal)
  {
    if((data_->dof_task_ == arg_goal.cols() && 1 == arg_goal.rows()) ||
        (1 == arg_goal.cols() && data_->dof_task_ == arg_goal.rows()) )
    {
      data_->ddlam_goal_ = arg_goal;
      return true;
    }
#ifdef DEBUG
    else
    {
      std::cerr<<"\nCTaskOpOri::setGoalAcc() : Error : Goal vector's size != data_->dof_task_"<<std::flush;
      assert(false);
    }
#endif
    return false;
  }

  void CTaskOpOri::reset()
  {
    data_ = S_NULL;
    dynamics_ = S_NULL;
    has_been_init_ = false;
  }


  bool CTaskOpOri::computeServo(const SRobotSensors* arg_sensors)
  {
    bool flag = true;
#ifdef DEBUG
    assert(has_been_init_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      //Step 1: Find position of the op_point
      //data_->x_ = data_->rbd_->T_o_lnk_ * data_->pos_in_parent_;
      //STEP 1: Find present orientation
      Eigen::Matrix3d R_on = data_->rbd_->T_o_lnk_.rotation();
      Eigen::Quaterniond lam_prs(R_on);

      data_->lam_.w()= lam_prs.w();
      data_->lam_.x()= lam_prs.x();
      data_->lam_.y()= lam_prs.y();
      data_->lam_.z()= lam_prs.z();


      Eigen::MatrixXd E_inv(3,4);
      E_inv(0,0)=-data_->lam_.x();E_inv(0,1)=data_->lam_.w();E_inv(0,2)=-data_->lam_.z();E_inv(0,3)=data_->lam_.y();
      E_inv(1,0)=-data_->lam_.y();E_inv(1,1)=data_->lam_.z();E_inv(1,2)=data_->lam_.w();E_inv(1,3)=-data_->lam_.x();
      E_inv(2,0)=-data_->lam_.z();E_inv(2,1)=-data_->lam_.y();E_inv(2,2)=data_->lam_.x();E_inv(2,3)=data_->lam_.w();

      E_inv = 2*E_inv;

      Eigen::VectorXd lam_goal_tmp(4);
      lam_goal_tmp(0)=data_->lam_goal_.w();
      lam_goal_tmp(1)=data_->lam_goal_.x();
      lam_goal_tmp(2)=data_->lam_goal_.y();
      lam_goal_tmp(3)=data_->lam_goal_.z();

      data_->delta_phi_ = E_inv * lam_goal_tmp;


      Eigen::VectorXd temp_vel;temp_vel.setZero(6);

      //Global coordinates : dx = J . dq
      data_->dlam_ = data_->J_ * arg_sensors->dq_; //using J-6 confirm whether correct variable

      //Compute the servo torques
      //tmp1 = (data_->x_goal_ - data_->x_);
      tmp1 =  (data_->kp_.array() * data_->delta_phi_.array());

      //tmp2 = (data_->dx_goal_ - data_->dx_);
      tmp2 = -1.0*(data_->kv_.array() * data_->dlam_.array());

      //Obtain force to be applied to a unit mass floating about
      //in space (ie. A dynamically decoupled mass).
      //data_->ddlam_ = data_->ka_.array() * (data_->ddlam_goal_ - data_->ddlam_).array();//what is this
      data_->ddlam_.setZero(data_->dof_task_);
      data_->ddlam_ += tmp2 + tmp1;
      data_->ddlam_ = data_->omega_inp * data_->ddlam_;	
      Eigen::Vector3d Force, Force_ef, moment_ef, moment_global;

      double val=data_->mass*9.81;
      Force << 0,0,val;
      Force_ef = R_on.transpose() * Force;
      moment_ef = data_->radius.cross(Force_ef);
      moment_global = R_on * moment_ef ;
      // NOTE : We apply the force limits in "task space". This is the whole point of
      // using operational space control. Since the control point is effectively a unit
      // inertia object floating in space, it is very easy to specify force limits. Moreover,
      // one can usually specify isotropic limits instead of hand-tuning different limits
      // for different directions.
      data_->ddlam_ = data_->ddlam_.array().min(data_->force_task_max_.array());//Min of self and max
      data_->ddlam_ = data_->ddlam_.array().max(data_->force_task_min_.array());//Max of self and min


      if(data_->flag_compute_op_inertia_)
      { data_->force_task_ = data_->M_task_ * data_->ddlam_;  }
      else
      { data_->force_task_ = data_->ddlam_;}

      if(data_->flag_compute_op_cc_forces_)
      { data_->force_task_ += data_->force_task_cc_;  }

      // NOTE : We subtract gravity (since we want to apply an equal and opposite force
      if(data_->flag_compute_op_gravity_)
      { data_->force_task_ -= data_->force_task_grav_;  }

      // T = J' ( M x F* + p)
      // We do not use the centrifugal/coriolis forces. They can cause instabilities.
      /*************************************ADD THE END_EFFECTOR PAYLOAD MASS MOMENT************************/
      data_->force_task_ += moment_global;

      data_->force_gc_ = data_->J_.transpose() * data_->force_task_;
//      data_->force_gc_.setZero(data_->robot_->dof_);


    }
    else
    { return false; }

    return flag;
  }

  /** Computes the dynamics (task model)
   * Assumes that the data_->model_.gc_model_ has been updated. */
  bool CTaskOpOri::computeModel(const SRobotSensors* arg_sensors)
  {
#ifdef DEBUG
    assert(has_been_init_);
    assert(data_->has_been_init_);
    assert(S_NULL!=data_->rbd_);
    assert(S_NULL!=dynamics_);
#endif
    if(data_->has_been_init_)
    {
      bool flag = true;
      const SGcModel* gcm = data_->gc_model_;

      flag = flag && dynamics_->computeJacobian(data_->J_6_,*(data_->rbd_),
          arg_sensors->q_,data_->pos_in_parent_);

      //Use the orientation jacobian only. This is an op-ori task.
      data_->J_ = data_->J_6_.block(3,0,3,data_->robot_->dof_);

      //Operational space mass/KE matrix:
      //    // NOTE TODO : Decide a good scheme for disabling operational space inertia
      //    if(data_->flag_compute_op_inertia_)
      //    {   }
      //    else
      //    { data_->M_task_inv_ = Eigen::Matrix3d::Identity();  }

      //Lambda = (J * Ainv * J')^-1
      data_->M_task_inv_ = data_->J_ * gcm->M_gc_inv_ * data_->J_.transpose();

#ifdef SCL_PRINT_INFO_MESSAGES
      std::cout<<"\n\tJx6:\n"<<data_->J_6_
          <<"\n\tFgrav_gc:\n"<<data_->gc_model_->force_gc_grav_.transpose();

      std::cout<<"\n\tMx_inv:\n"<<data_->M_task_inv_
          <<"\n\tJx:\n"<<data_->J_
          <<"\n\tJx6:\n"<<data_->J_6_
          <<"\n\tMgcinv:\n"<<gcm->M_gc_inv_;

      std::cout<<"\n\tTo_lnk: \n"<<data_->rbd_->T_o_lnk_.matrix()
                      <<"\n\tPosInPar: "<<data_->pos_in_parent_.transpose()
                      <<"\n\t       X: "<<data_->x_.transpose()
                      <<"\n\t   Xgoal: "<<data_->x_goal_.transpose()
                      <<"\n\t   Ftask: "<<data_->force_task_.transpose()
                      <<"\n\t Ftaskgc: "<<data_->force_gc_.transpose()
                      <<"\n\t  Fxgrav: "<<data_->force_task_grav_.transpose();
#endif

      if(!use_svd_for_lambda_inv_)
      {
        //The general inverse function works very well for op-point controllers.
        //3x3 matrix inversion behaves quite well. Even near singularities where
        //singular values go down to ~0.001. If the model is coarse, use a n-k rank
        //approximation with the SVD for a k rank loss in a singularity.
        qr_.compute(data_->M_task_inv_);
        if(qr_.isInvertible())
        { data_->M_task_ = qr_.inverse();  }
        else
        { use_svd_for_lambda_inv_ = true; }
      }

      if(use_svd_for_lambda_inv_)
      {
        //Use a Jacobi svd. No preconditioner is required coz lambda inv is square.
        //NOTE : This is slower and generally performs worse than the simple inversion
        //for small (3x3) matrices that are usually used in op-space controllers.
        svd_.compute(data_->M_task_inv_,
            Eigen::ComputeFullU | Eigen::ComputeFullV | Eigen::ColPivHouseholderQRPreconditioner);

#ifdef DEBUG
        std::cout<<"\n Singular values : "<<svd_.singularValues().transpose();
#endif
        int rank_loss=0;

        //NOTE : A threshold of .005 works quite well for most robots.
        //Experimentally determined: Take the robot to a singularity
        //and observe the response as you allow the min singular values
        //to decrease. Stop when the robot starts to go unstable.
        //NOTE : This also strongly depends on how good your model is
        //and how fast you update it. A bad model will require higher
        //thresholds and will result in coarse motions. A better model
        //will allow much lower thresholds and will result in smooth
        //motions.
        if(svd_.singularValues()(0) > 0.005)
        { singular_values_(0,0) = 1.0/svd_.singularValues()(0);  }
        else { singular_values_(0,0) = 0.0; rank_loss++; }
        if(svd_.singularValues()(1) > 0.005)
        { singular_values_(1,1) = 1.0/svd_.singularValues()(1);  }
        else { singular_values_(1,1) = 0.0; rank_loss++; }
        if(svd_.singularValues()(2) > 0.005)
        { singular_values_(2,2) = 1.0/svd_.singularValues()(2);  }
        else { singular_values_(2,2) = 0.0; rank_loss++; }

        if(0 < rank_loss)
        { std::cout<<"\nCTaskOpOri::computeModel() : Warning. Lambda_inv is ill conditioned. SVD rank loss (@.005) = "<<rank_loss; }

        data_->M_task_ = svd_.matrixV() * singular_values_ * svd_.matrixU().transpose();

        //Turn off the svd after 50 iterations
        //Don't worry, the qr will pop back to svd if it is still singular
        static sInt svd_ctr = 0; svd_ctr++;
        if(50>=svd_ctr)
        { svd_ctr = 0; use_svd_for_lambda_inv_ = false;  }
      }

      //Compute the Jacobian dynamically consistent generalized inverse :
      //J_dyn_inv = Ainv * J' (J * Ainv * J')^-1
      data_->J_dyn_inv_ = gcm->M_gc_inv_ * data_->J_.transpose() * data_->M_task_;

      //J' * J_dyn_inv'
      sUInt dof = data_->robot_->dof_;
      data_->null_space_ = Eigen::MatrixXd::Identity(dof, dof) -
          data_->J_.transpose() * data_->J_dyn_inv_.transpose();

      // We do not use the centrifugal/coriolis forces. They can cause instabilities.
      // NOTE TODO : Fix this...
      //    if(data_->flag_compute_op_gravity_)
      //    { /** I need some code */ }
      //    else
      //    { data_->force_task_cc_.setZero(data_->dof_task_,1);  }
      data_->force_task_cc_.setZero(data_->dof_task_,1);

      // J' * J_dyn_inv' * g(q)
      if(data_->flag_compute_op_gravity_)
      { data_->force_task_grav_ =  data_->J_dyn_inv_.transpose() * gcm->force_gc_grav_;  }

      return flag;
    }
    else
    { return false; }
  }


  //************************
  // Task specific stuff
  //************************

  bool CTaskOpOri::achievedGoalOri()
  {
    sFloat dist;
    dist = fabs(data_->delta_phi_.norm());

    if(dist > data_->spatial_resolution_)
    { return false; }
    else
    { return true;  }
  }

  /*******************************************
              Dynamic Type : CTaskOpOri

     NOTE : To enable dynamic typing for tasks, you
     must define the types for the "CTaskName" computation
     object AND the "STaskName" data structure. THIS IS NECESSARY.

     Why? So that you have a quick and easy way to specify
     custom xml parameters in the *Cfg.xml file.
   *******************************************/
  scl::sBool registerType_TaskOpOri()
  {
    bool flag;
    try
    {
      sutil::CDynamicType<std::string,scl_app::CTaskOpOri> typeCTaskOpOri(std::string("CTaskOpOri"));
      flag = typeCTaskOpOri.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type CTaskOpOri"));}

      sutil::CDynamicType<std::string,scl_app::STaskOpOri> typeSTaskOpOri(std::string("STaskOpOri"));
      flag = typeSTaskOpOri.registerType();
      if(false == flag) {throw(std::runtime_error("Could not register type STaskOpOri"));}

#ifdef DEBUG
      std::cout<<"\nregisterType_TaskOpOri() : Registered my cool orientation control task with the database";
#endif
    }
    catch (std::exception& e)
    {
      std::cout<<"\nregisterType_TaskOpOri() : Error : "<<e.what();
      return false;
    }
    return true;
  }
}

