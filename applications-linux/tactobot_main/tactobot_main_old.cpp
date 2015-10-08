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
/* \file scl_tutorial5_multi_task.cpp
 *
 *  Created on: Aug 10, 2014
 *
 *  Copyright (C) 2014
 *
 *  Author: Samir Menon <smenon@stanford.edu>
 */

//Optitrack
#include "OptiTrack.h"



//JR3 Force Sensor
#include "Jr3Sensor.h"

//scl lib
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SGcModel.hpp>
#include <scl/dynamics/scl/CDynamicsScl.hpp>
#include <scl/dynamics/tao/CDynamicsTao.hpp>
#include <scl/parser/sclparser/CParserScl.hpp>
#include <scl/graphics/chai/CGraphicsChai.hpp>
#include <scl/graphics/chai/ChaiGlutHandlers.hpp>
#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/data_structs/STaskOpPos.hpp>
#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>
#include "STaskOpOri.hpp"
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include "CTaskOpOri.hpp"

//scl functions to simplify dynamic typing and data sharing
#include <scl/robot/DbRegisterFunctions.hpp>
#include <scl/util/DatabaseUtils.hpp>

//For timing
#include <sutil/CSystemClock.hpp>

//Eigen 3rd party lib
#include <Eigen/Dense>

//Standard includes (for printing and multi-threading)
#include <iostream>
#include <omp.h>

//Freeglut windowing environment
#include <GL/freeglut.h>


int main(int argc, char** argv)
{
  std::cout<<"\n***************************************\n";
  std::cout<<"Standard Control Library Tutorial #5";
  std::cout<<"\n***************************************\n";

  scl::SRobotParsed rds;     //Robot data structure....
  scl::SGraphicsParsed rgr;  //Robot graphics data structure...
  scl::SGcModel rgcm;        //Robot data structure with dynamic quantities...
  scl::SRobotIO rio;         //I/O data structure
  scl::CGraphicsChai rchai;  //Chai interface (updates graphics rendering tree etc.)
  scl::CDynamicsScl dyn_scl; //Robot kinematics and dynamics computation object...
  scl::CDynamicsTao dyn_tao; //Robot physics integrator
  scl::CParserScl p;         //This time, we'll parse the tree from a file...

  scl::SControllerMultiTask rctr_ds; //A multi-task controller data structure
  scl::CControllerMultiTask rctr;    //A multi-task controller
  std::vector<scl::STaskBase*> rtasks;              //A set of executable tasks
  std::vector<scl::SNonControlTaskBase*> rtasks_nc; //A set of non-control tasks
  std::vector<scl::sString2> ctrl_params;        //Used to parse extra xml tags
  scl::STaskOpPos* rtask_hand;       //Will need to set hand desired positions etc.
  scl_app::STaskOpOri* rtask_wrist;

  //Optitrack variables
  OptiTrack *objects;
  bool b_useCalibration=false;
  int nObjects=3;
  bool ENABLE_OPTITRACK=true;


  double *currentPosition;
      currentPosition = new double[3];
      double **currentRotation;
      currentRotation = new double*[3];
      for (int i = 0; i < 3; i++) {
          currentRotation[i] = new double[3];
      }

 std::string vrpn_server_ip = "172.24.68.48:3883";
 printf("Initializing OptiTrack vrpn_server_ip: %s....", vrpn_server_ip.c_str());
 printf("done\n");

 std::string grasped_object_name = "tool";
 std::string env_object_name = "env";
 objects = new OptiTrack[nObjects];
 objects[0].Init(vrpn_server_ip, "Filler");
 objects[1].Init(vrpn_server_ip, grasped_object_name);
 objects[2].Init(vrpn_server_ip, env_object_name);

 //JR3 Force Sensor
 Jr3Sensor *ForceSensor;
 Eigen::Vector3d force;
 Eigen::Vector3d moment;

 float jr3_forces[6];
 memset(jr3_forces, 0x0, 6 * sizeof(float));

 printf("Initializing force sensor....");
     ForceSensor = new Jr3Sensor;
     printf("done\n");
bool ENABLE_FORCE_SENSOR=true;

  //Graphics Variables
  Eigen::Vector3d tray_pos;
  double tray_width=0.24;
  double tray_height=0.26;
  double tray_full_length=0.4;
  double tray_mid_section_height=0.06;
  double tray_length=0.3;
  double tray_slot_height=0.02;
  //All in meters

  //Graphics Objects
  chai3d::cGenericObject *goal_sphere;


  //Position and rotation from OptiTrack
  Eigen::Vector3d ShelfPositionOpti;
  Eigen::Matrix3d ShelfRotationOpti;

    sutil::CSystemClock::start(); //Start the clock

  /******************************Set up Dynamic Type Info************************************/
  bool flag = scl_app::registerType_TaskOpOri();
  if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.

  /******************************Load Robot Specification************************************/
  //We will use a slightly more complex xml spec than the first few tutorials
  flag = p.readRobotFromFile("../../specs/Barrett/wamCfg.xml","../../specs/","wamBot",rds);
  flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
  flag = flag && dyn_tao.init(rds);         //Set up integrator object
  flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
  flag = flag && rio.init(rds.name_,rds.dof_);        //Set up the I/O data structure
  for(unsigned int i=0;i<rds.dof_;++i){ rio.sensors_.q_(i) = rds.rb_tree_.at(i)->joint_default_pos_; }
  if(false == flag){ return 1; }            //Edata_->rror check.
  rgcm.rbdyn_tree_.at("link7");
  /******************************Set up Controller Specification************************************/
  // Read xml file info into task specifications.
  flag = p.readTaskControllerFromFile("../../specs/Barrett/wamCfg.xml","opc",rtasks,rtasks_nc,ctrl_params);
  flag = flag && rctr_ds.init("opc",&rds,&rio,&rgcm); //Set up the control data structure..
  // Tasks are initialized after find their type with dynamic typing.
  flag = flag && scl_registry::registerNativeDynamicTypes();
  flag = flag && scl_util::initMultiTaskCtrlDsFromParsedTasks(rtasks,rtasks_nc,rctr_ds);
  flag = flag && rctr.init(&rctr_ds,&dyn_scl);        //Set up the controller (needs parsed data and a dyn object)
  if(false == flag){ return 1; }            //Error check.

  rtask_hand = dynamic_cast<scl::STaskOpPos*>( *(rctr_ds.tasks_.at("hand")));
  if(NULL == rtask_hand)  {return 1;}       //Error check

//  rtask_NullDamp = dynamic_cast<scl::STaskNullSpaceDamping*>( *(rctr_ds.tasks_.at("NullSpaceDampingTask")));
//    if(NULL == rtask_NullDamp)  {return 1;}       //Error check

  rtask_wrist = dynamic_cast<scl_app::STaskOpOri*>( *(rctr_ds.tasks_.at("wrist")));
   if(NULL == rtask_wrist)  {return 1;}       //Error check

  /******************************ChaiGlut Graphics************************************/
  glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

  flag = p.readGraphicsFromFile("../../specs/Barrett/wamCfg.xml","wamBotStdView",rgr);
  flag = flag && rchai.initGraphics(&rgr);
  flag = flag && rchai.addRobotToRender(&rds,&rio);
  flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
  if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }

  /******************************Add models to render************************************/
      flag = rchai.addMeshToRender("plane","../../specs/GraphicObjects/Plane.obj",Eigen::Vector3d(0,0,-0.5), Eigen::Matrix3d::Identity());
      if(false == flag){std::cout<<"\nCould not load the plane"; return 1; }
      scl::SGraphicsChaiMesh *plane_mesh = rchai.getChaiData()->meshes_rendered_.at("plane");


      tray_pos<<0,1,0.5;
      flag = rchai.addMeshToRender("tray","../../specs/GraphicObjects/Tray.obj",tray_pos, Eigen::Matrix3d::Identity());
      if(false == flag){std::cout<<"\nCould not load the tray"; return 1; }
      scl::SGraphicsChaiMesh *tray_mesh = rchai.getChaiData()->meshes_rendered_.at("tray");

       //Show Frames
      rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setShowFrame(true);
      rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setFrameSize(0.1);

      rchai.getChaiData()->meshes_rendered_.at("tray")->graphics_obj_->setShowFrame(true);
      rchai.getChaiData()->meshes_rendered_.at("tray")->graphics_obj_->setFrameSize(0.1);

      Eigen::Vector3d sphere_pos(-0.5,-0.5,0.5);
      flag = rchai.addSphereToRender(sphere_pos, goal_sphere,0.03);
      if(false == flag){std::cout<<"\nCould not load the sphere"; return 1; }

  /******************************Simulation************************************/
  // Now let us integrate the model for a variety of timesteps and see energy stability
  std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
  long long iter = 0; double dt=0.001;

  omp_set_num_threads(4);
  int thread_id; double tstart, tcurr; flag = false;

#pragma omp parallel private(thread_id)
  {
    thread_id = omp_get_thread_num();

    if(thread_id==1) //Simulate physics and update the rio data structure..
    {
      // Controller : Operational space controller
      std::cout<<"\n\n***************************************************************"
          <<"\n Starting op space (task coordinate) controller..."        
          <<"\n***************************************************************";

      tstart = sutil::CSystemClock::getSysTime(); iter = 0;
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      {
         std::cout<<"Thread ID: "<<thread_id<<std::endl;
         tcurr = sutil::CSystemClock::getSysTime();


        //SET PLANE POSITION
        Eigen::MatrixXd T_matrix;
        scl::SRigidBodyDyn *rbd=rgcm.rbdyn_tree_.at("link7");
        T_matrix=rbd->T_o_lnk_.matrix();
        Eigen::Vector4d zero_vector(0,0,0,1);
        Eigen::Vector4d plane_pos=T_matrix*zero_vector;
        chai3d::cMatrix3d rot_matrix ,_90matrix;
        rot_matrix.set(T_matrix(0,0),T_matrix(0,1),T_matrix(0,2),T_matrix(1,0),T_matrix(1,1),T_matrix(1,2),T_matrix(2,0),T_matrix(2,1),T_matrix(2,2));
        _90matrix.set(1,0,0,0,0,1,0,-1,0);
        rot_matrix=rot_matrix*_90matrix;
        Eigen::Vector3d plane_in_ee(0,0,0.3);

        Eigen::Vector3d end_eff_pos=rtask_hand->x_+rbd->T_o_lnk_.rotation()*plane_in_ee;

                    // Move the left hand in a different sine wave



                        Eigen::Matrix3d R,R_des2;
                            R<<0,0,1,1,0,0,0,1,0;
                            R_des2<<1,0,0,0,0,1,0,-1,0;



                           if(!scl::CDatabase::getData()->s_gui_.ui_flag_[1]) //OPTITRACK CONTROL
                           {
                        	   //TODO : change R according to opti track frame
                               rtask_hand->x_goal_ =R*ShelfPositionOpti;
                               //rtask_wrist->lam_goal_=SharedRotation;
                               rtask_wrist->lam_goal_=R*SharedRotation;


                           }

                           else  //KEYBOARD CONTROL
                           {

                             //rtask_hand->x_goal_ =SharedPosition;
                             rtask_hand->x_goal_ = scl::CDatabase::getData()->s_gui_.ui_point_[0];
                             rtask_wrist->lam_goal_=R_des2;
                           }

                       std::cout<<"\nDelta Phi: "<<rtask_wrist->delta_phi_<<"\nLambda Goal: "<<rtask_wrist->lam_goal_.coeffs().transpose();
                       std::cout<<"\nLambda Current: "<<rtask_wrist->lam_.coeffs().transpose();




        // Compute control forces (note that these directly have access to the io data ds).
        rctr.computeDynamics();
        rctr.computeControlForces();

        //rtask_wrist->force_task_


        // Integrate the dynamics
        dyn_tao.integrate(rio,dt); iter++;

        /**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/


                        rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalRot(rot_matrix);
                        rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalPos(plane_pos[0],plane_pos[1],plane_pos[2]);

                        //SET GOAL SPHERE POSITION
                        goal_sphere->setLocalPos(rtask_hand->x_goal_[0],rtask_hand->x_goal_[1],rtask_hand->x_goal_[2]);

           /**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/



        if(iter % 5000 == 0){std::cout<<"\nLam-goal"<<rtask_wrist->lam_goal_.coeffs().transpose()
          <<"\nlam-actual"<<rtask_wrist->lam_.coeffs().transpose()
          <<"\ndphi"<<rtask_wrist->delta_phi_.transpose();

        }
      }
      //Then terminate
      scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
    }

    else if (thread_id==0)  //Read the rio data structure and updated rendererd robot..
      while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
      { glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }

    else if(thread_id==2) //Optitrack
    {
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
       {
            if(ENABLE_OPTITRACK)
            {
            //Update Optitrack locations
            for (int k=0; k<nObjects; k++)
                    {
                        if (objects[k].IsEnabled())
                        {
                            //reading new values, if there is any
                            objects[k].Update();
                            // Getting the object position
                            objects[k].GetPosition(currentPosition, b_useCalibration);
                            // Getting the object orientation
                            objects[k].GetRotationMatrix(currentRotation, b_useCalibration);
                           // objects[k].Print();
                        }

                            Eigen::VectorXd pos = Eigen::VectorXd::Zero(3);
                            Eigen::VectorXd rot = Eigen::VectorXd::Zero(9);
                            for (unsigned int i = 0; i < 3; i++) {
                                pos(i) = currentPosition[i];
                            }
                            for (unsigned int i = 0; i < 3; i++) {
                                for (unsigned int j = 0; j < 3; j++) {
                                    rot(i * 3 + j) = currentRotation[i][j];
                                }
                            }

                            if(k==1) //TOOL
                            {
                            	for (unsigned int i = 0; i < 3; i++)
                            	{
                            		for (unsigned int j = 0; j < 3; j++)
                            		{
                            			SharedRotation(i,j)=currentRotation[i][j];
                            		}
                            	}
                            	SharedPosition=pos;


                     }
                 }
       }
        }
    }

    else if(thread_id==3) //Force Sensor Thread
    {
        while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
        {
        if(ENABLE_FORCE_SENSOR==1)
        {

        if (ForceSensor->GetCartesianForcesTorques(jr3_forces, 1) == false)
                   {
                        std::cout << "Could not read from force/torque sensor!\n"<< std::flush;
                    }


                    for (unsigned int i = 0; i < 3; i++) {
                        force(i) = (double) jr3_forces[i];
                    }
                    for (unsigned int i = 0; i < 3; i++) {
                        moment(i) = (double) jr3_forces[i + 3];
                    }
                    std::cout<<"Force: "<<force<<std::endl;
                    std::cout<<"Moment: "<<moment<<std::endl;

                    //TODO: Subtract Offset here
                    //currentForce = currentForce - jr3_offset.offset_force;
                    //currentMoment = currentMoment - jr3_offset.offset_moment;


        }
        }
  }
  }

  /******************************Exit Gracefully************************************/
  std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
  std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
  std::cout<<"\n\nExecuted Successfully";
  std::cout<<"\n**********************************\n"<<std::flush;

  delete ForceSensor;
  delete[] objects;

  return 0;
}
