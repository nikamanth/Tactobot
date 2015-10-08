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

//WAM Comm
#include "CExampleApp.hpp"

//Optitrack
#include "OptiTrack.h"

//Trajectory Generation
#include<ReflexxesAPI.h>

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
#include <scl/control/task/tasks/data_structs/STaskGc.hpp>
//#include <scl/control/task/tasks/data_structs/STaskGcSet.hpp>


#include <scl/control/task/tasks/data_structs/STaskNullSpaceDamping.hpp>
#include "STaskOpOri.hpp"
//#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include "STaskOpPosition.hpp"
#include "CTaskOpPosition.hpp"


#include "CTaskOpOri.hpp"
#include <scl/DataTypes.hpp>
#include <scl/data_structs/SDatabase.hpp>
#include <scl/robot/CRobotApp.hpp>

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

#include <redox.hpp>


//WAM VARIABLES

using namespace chai3d;
using namespace redox;

static const std::string APP_NAME = "scl_example_control";
static const std::string REDIS_HOST = "localhost";
static const int REDIS_PORT = 6379;

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

bool has_communicated_with_robot_once_=false;
bool has_updated_dyn_after_robot_comm_=false;
const double tau_fade_in_time_constt=0.1;
// END WAM VARIABLES

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
scl_app::STaskOpPosition* rtask_hand;       //Will need to set hand desired positions etc.
scl_app::STaskOpOri* rtask_wrist;
//scl::STaskGc* rtask_gc;

scl::sBool InitController();
void initRedox();

int main(int argc, char** argv)
{
	std::cout<<"\n***************************************\n";
	std::cout<<"Standard Control Library Tutorial #5";
	std::cout<<"\n***************************************\n";




	//Optitrack variables
	OptiTrack *objects;
	bool b_useCalibration=false;
	int nObjects=3;

	double *currentPosition;
	currentPosition = new double[3];
	double **currentRotation;
	currentRotation = new double*[3];
	for (int i = 0; i < 3; i++)
	{
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
	Eigen::Vector3d SharedPosition;
	Eigen::Matrix3d SharedRotation;

	//Init shared variables
	//SharedPosition.setZero(3); //Init to zero pos
	SharedPosition<<0.65,0.15,0.5;
	SharedRotation.setZero(3,3); //Init to identity
	SharedRotation(0,0)=1;
	SharedRotation(1,1)=1;
	SharedRotation(2,2)=1;


	sutil::CSystemClock::start(); //Start the clock

	/******************************Set up Dynamic Type Info************************************/
	bool flag = scl_app::registerType_TaskOpPosition();
	if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.


	flag = scl_app::registerType_TaskOpOri();
	if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.

	/******************************Load Robot Specification************************************/
	//We will use a slightly more complex xml spec than the first few tutorials
	flag = p.readRobotFromFile("../../specs/Barrett/wamCfg.xml","../../specs/","wamBot",rds);
	flag = flag && rgcm.init(rds);            //Simple way to set up dynamic tree...
	flag = flag && dyn_tao.init(rds);         //Set up integrator object
	flag = flag && dyn_scl.init(rds);         //Set up kinematics and dynamics object
	flag = flag && rio.init(rds);        //Set up the I/O data structure
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

	rtask_hand = dynamic_cast<scl_app::STaskOpPosition*>( *(rctr_ds.tasks_.at("hand")));
	if(NULL == rtask_hand)  {return 1;}       //Error check


	//  rtask_gc = dynamic_cast<scl::STaskGc*>( *(rctr_ds.tasks_.at("GcTask")));
	//  if(NULL == rtask_hand)  {return 1;}       //Error check

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
	flag = rchai.addMeshToRender("plane","../../specs/GraphicObjects/Tool.obj",Eigen::Vector3d(0,0,-0.5), Eigen::Matrix3d::Identity());
	if(false == flag){std::cout<<"\nCould not load the plane"; return 1; }
	scl::SGraphicsChaiMesh *plane_mesh = rchai.getChaiData()->meshes_rendered_.at("plane");


	tray_pos<<0.8,1.0,0.3;
	flag = rchai.addMeshToRender("tray","../../specs/GraphicObjects/Env.obj",tray_pos, Eigen::Matrix3d::Identity());
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
	long long iter = 0, cur_iter=0; double dt=0.0001;

	/******************************TRAJECTORY GENERATION************************************/
	ReflexxesAPI *RML = NULL;
	RMLPositionInputParameters *IP = NULL;
	RMLPositionOutputParameters	*OP =NULL;
	RMLPositionFlags Flags;

	int dof = 3;

	// Initialize variables
	RML = new ReflexxesAPI(dof, dt);
	IP = new RMLPositionInputParameters(dof);
	OP = new RMLPositionOutputParameters(dof);

	for(int i = 0; i < dof; i++) IP->MaxVelocityVector->VecData[i] = .5;
	for(int i = 0; i < dof; i++) IP->MaxAccelerationVector->VecData[i] = 1;
	for(int i = 0; i < dof; i++) IP->MaxJerkVector->VecData[i] = 0.5;


	// Can optionally choose to ignore certain directions
	IP->SelectionVector->VecData[0]	=	true		;
	IP->SelectionVector->VecData[1]	=	true		;
	IP->SelectionVector->VecData[2]	=	true		;

	Eigen::VectorXd last_v(3);
	last_v.setZero(3);

	//REDIS SERVER CONNECT

	//simulatio or actual bot

	bool sim_only= false;

	if (!sim_only)
	{
		InitController();


		//Check if redis has read robot
		std::cout<<"Waiting For redis server"<<std::endl;
		while(!has_communicated_with_robot_once_)
		{
			sleep(2);
			std::cout<<"in sleep loop";
		}
		if(true == has_communicated_with_robot_once_){
			rctr.computeDynamics();
			rctr.computeNonControlOperations();
			rctr.computeControlForces();
			has_updated_dyn_after_robot_comm_ = true;
		}
	}

	std::cout<<"Redis Server has updated dynamics"<<std::endl;

	//Get initial position of robot
	//SET PLANE POSITION


	Eigen::VectorXd init_q(7);
	if(sim_only){
		// initialize Q only in simulation
		rio.sensors_.q_<<0, 1.57, 1.2, 1.57, 0, 0, 0;
	}



	//      rtask_hand->omega_inp;
	//      Eigen::MatrixXd T_matrix_init;
	//      scl::SRigidBodyDyn *rbd_init=rgcm.rbdyn_tree_.at("link7");
	//      T_matrix_init=rbd_init->T_o_lnk_.matrix();
	//      Eigen::Vector4d zero_vector_init(0,0,0.02,1);
	//      Eigen::Vector4d end_eff_pos_init=T_matrix_init*zero_vector_init;
	//      Eigen::VectorXd pos(3);
	//      pos=end_eff_pos_init.block(0,0,3,1);

	Eigen::VectorXd pos(3);
	//      pos<<0,0,1;


	rctr.computeDynamics();


	Eigen::Matrix3d init_rot;
	init_rot = rtask_wrist->rbd_->T_o_lnk_.rotation();
	//Generate Trajectory from current robot position to our initial robot state



	/*****************************Trajectory generation************************************/

	omp_set_num_threads(3);
	int thread_id; double tstart, tcurr; flag = false;

	Eigen::Matrix3d R_des1,R_des2;
	R_des1<<1,0,0,0,1,0,0,0,1;
	R_des2<<1,0,0,0,0,-1,0,1,0;

	bool movetray=true;
	int loop_ctr = 0;
	bool inter1_goal=false;
	bool inter_goal=false;
	int update_period = 1;



#pragma omp parallel private(thread_id)
	{
		thread_id = omp_get_thread_num();

		if(thread_id==1) //Simulate physics and update the rio data structure..
		{
			// Controller : Operational space controller
			std::cout<<"\n\n***************************************************************"
					<<"\n Starting op space (task coordinate) controller..."
					<<"\n***************************************************************";

			Eigen::VectorXd init_pos;
			init_pos = rtask_hand->rbd_->T_o_lnk_*rtask_hand->pos_in_parent_;
			pos=init_pos;
			pos(1) -= 0.1;

			//     R_des1 = rtask_wrist->rbd_->T_o_lnk_.rotation();

			for(int i = 0; i < dof; i++) IP->CurrentPositionVector->VecData[i] = init_pos(i);
			for(int i = 0; i < dof; i++) IP->CurrentVelocityVector->VecData[i] = 0;
			for(int i = 0; i < dof; i++) IP->CurrentAccelerationVector->VecData[i] = 0;

			tstart = sutil::CSystemClock::getSysTime(); iter = 0;
			std::cout<<"Thread ID: "<<thread_id<<std::endl;
			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{
				tcurr = sutil::CSystemClock::getSysTime();
				Eigen::Vector3d end_eff_pos=rtask_hand->x_;
				double error=0.0;
				int sec_iter=35000;

				Eigen::VectorXd Goal(3);
				if(inter1_goal==false)
				{
					Goal=pos;

					if(loop_ctr % update_period == 0) {
						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = Goal(0);
						IP->TargetPositionVector->VecData[1] = Goal(1);
						IP->TargetPositionVector->VecData[2] = Goal(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;
					}
					loop_ctr++;

					rtask_wrist->has_been_activated_=false;
					// ResultValue contains status information
					int ResultValue	= RML->RMLPosition(*IP, OP, Flags);

					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

					for(int i = 0; i < dof; i++) SharedPosition(i) = OP->NewPositionVector->VecData[i];
					R_des1<<1,0,0,0,0,1,0,-1,0;

					error=(Goal-rtask_hand->x_).norm();
					std::cout <<"\nnorm error" <<error;
					if (error<0.02){
						inter1_goal=true;
						cur_iter=iter;}

				}
				else if(inter_goal==false)
				{
					/* ---Traj gen code---*/
					Goal(0)=tray_pos(0);
					Goal(1)=tray_pos(1)-tray_length-0.3;
					Goal(2)=tray_pos(2);
					if(loop_ctr % update_period == 0) {
						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = Goal(0);
						IP->TargetPositionVector->VecData[1] = Goal(1);
						IP->TargetPositionVector->VecData[2] = Goal(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;
					}
					loop_ctr++;

					rtask_wrist->has_been_activated_=true;

					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

					// ResultValue contains status information
					int ResultValue	= RML->RMLPosition(*IP, OP, Flags);
					for(int i = 0; i < dof; i++) SharedPosition(i) = OP->NewPositionVector->VecData[i];
					R_des1<<1,0,0,0,0,1,0,-1,0;

					error=(Goal-rtask_hand->x_).norm();
					std::cout <<"\nnorm error" <<error;
					if (error<0.02){
						inter_goal=true;
						cur_iter=iter;}
				}

				rtask_hand->x_goal_ = SharedPosition;
				rtask_wrist->lam_goal_ =init_rot;
				// rtask_wrist->lam_goal_ = R_des1;

				std::cout<<"\nForce pos: "<<rtask_hand->force_task_.transpose() <<"\ndeltax:"<< rtask_hand->x_goal_-rtask_hand->x_.transpose();
				std::cout<<"\nQ:" <<rio.sensors_.q_.transpose();

				// Compute control forces (note that these directly have access to the io data ds).
				rctr.computeDynamics();
				rctr.computeControlForces();

				if(sim_only){
					dyn_tao.integrate(rio,dt);
				}

				iter++;

				/**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/

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
				rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalRot(rot_matrix);
				if(movetray)
					rchai.getChaiData()->meshes_rendered_.at("plane")->graphics_obj_->setLocalPos(plane_pos[0],plane_pos[1],plane_pos[2]);

				//SET GOAL SPHERE POSITION
				goal_sphere->setLocalPos(rtask_hand->x_goal_[0],rtask_hand->x_goal_[1],rtask_hand->x_goal_[2]);

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
								//SharedRotation(i,j)=currentRotation[i][j];
							}
						}
						//SharedPosition=pos;

							}
				}
			}
		}
	}

	/******************************Exit Gracefully************************************/
	std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
	std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
	std::cout<<"\n\nExecuted Successfully";
	std::cout<<"\n**********************************\n"<<std::flush;

	return 0;
}

scl::sBool InitController()
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
					rio.sensors_.q_(i) = measured_q[i];
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
			if(false == has_updated_dyn_after_robot_comm_ || false == scl::CDatabase::getData()->s_gui_.ui_flag_[5])
			{
				for(int i = 0; i < 7; i++)
				{ computed_tau[i] = 0.0;  }
			}
			else {
				for(int i = 0; i < 7; i++)
				{
					double tau_diff = rio.actuators_.force_gc_commanded_(i) - computed_tau[i];
					if(fabs(tau_diff) < 0.1)
					{ computed_tau[i] = rio.actuators_.force_gc_commanded_(i);  }
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
		//scl::sUInt args_ctr = args_parsed;

		// Check that we haven't finished parsing everything
		// while(args_ctr < argv.size())
		// {
		/* NOTE : ADD MORE COMMAND LINE PARSING OPTIONS IF REQUIRED */

		//if(true)
		//{
		//  std::cout<<"\n Possible example task options: -xxx (you can change me to suit your needs!)";
		//  args_ctr++;
		// }
		// }

		return true;
	}
	catch(std::exception &e)
	{ std::cout<<"\nCExampleApp::initMyController() : "<<e.what(); }

}

void initRedox()
{
	if (!rdx_.connect(REDIS_HOST, REDIS_PORT) || !sub_.connect(REDIS_HOST, REDIS_PORT))
	{
		throw std::runtime_error("Could not connect to Redis!");
	}
}
