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
#include "STaskOpForce.hpp"
#include "CTaskOpForce.hpp"


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
#include <jsoncpp/json/json.h>


#include <scl/robot/CRobotApp.hpp>

#include <scl/control/task/CControllerMultiTask.hpp>
#include <scl/control/task/tasks/CTaskOpPos.hpp>
#include <scl/control/task/tasks/CTaskComPos.hpp>

#include <scl/graphics/chai/data_structs/SGraphicsChai.hpp>

#include <redox.hpp>

//WAM VARIABLES
#include "CExampleApp.hpp"

#include<fstream>

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
scl_app::STaskOpForce* rtask_force;


bool ENABLE_OPTITRACK=true;
bool ENABLE_FORCE_SENSOR=true;

scl::sBool InitController();
void initRedox();

int main(int argc, char** argv)
{


    Eigen::Matrix3d Graphic_R_Optienv_simenv;
    Graphic_R_Optienv_simenv<<0,0,1,1,0,0,0,1,0;

	//Optitrack variables
	OptiTrack *objects;
	bool b_useCalibration=false;
    int nObjects=4;
    Eigen::Affine3d T_opti_robot;
    T_opti_robot.matrix().block(0,0,3,3)<< 0,0,-1,-1,0,0,0,1,0;


    Eigen::Matrix3d R_opti_robot;
    R_opti_robot<< 0,0,-1,-1,0,0,0,1,0;



	//Force Sensor
	//JR3 Force Sensor
	Eigen::Vector3d force;
	Eigen::Vector3d moment;

	std::string vrpn_server_ip = "172.24.68.48:3883";
	printf("Initializing OptiTrack vrpn_server_ip: %s....", vrpn_server_ip.c_str());
	printf("done\n");

	std::string grasped_object_name = "tool";
    std::string env_object_name = "env2";
    std::string robot_object_name = "robot";
    objects = new OptiTrack[nObjects];
	objects[0].Init(vrpn_server_ip, "Filler");
	objects[1].Init(vrpn_server_ip, grasped_object_name);
	objects[2].Init(vrpn_server_ip, env_object_name);
    objects[3].Init(vrpn_server_ip, robot_object_name);

    Eigen::Vector3d RobotPosOpti;

	//Graphics Variables
	Eigen::Vector3d env_pos_sim;

	//Graphics Objects
	chai3d::cGenericObject *goal_sphere;
    chai3d::cGenericObject *ultimate_goal_sphere;


	//Position and rotation from OptiTrack
	Eigen::Vector3d EnvPosOpti;
	Eigen::Matrix3d EnvRotOpti;

	sutil::CSystemClock::start(); //Start the clock

	/******************************Set up Dynamic Type Info************************************/
	bool flag = scl_app::registerType_TaskOpPosition();
	if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.


	flag = scl_app::registerType_TaskOpOri();
	if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.

  // flag = scl_app::registerType_TaskOpForce();
  //if(false == flag){ std::cout<<"\nCould not init task dyn type\n"; return 1; } //Error check.

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

       // rtask_force = dynamic_cast<scl_app::STaskOpForce*>( *(rctr_ds.tasks_.at("force")));
      //  if(NULL == rtask_wrist)  {return 1;}       //Error check





	/******************************ChaiGlut Graphics************************************/
	glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

	flag = p.readGraphicsFromFile("../../specs/Barrett/wamCfg.xml","wamBotStdView",rgr);
	flag = flag && rchai.initGraphics(&rgr);
	flag = flag && rchai.addRobotToRender(&rds,&rio);
	flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
	if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }



	/******************************Add models to render************************************/
	flag = rchai.addMeshToRender("tool","../../specs/GraphicObjects/Tool.obj",Eigen::Vector3d(0,0,-0.5), Eigen::Matrix3d::Identity());
	if(false == flag){std::cout<<"\nCould not load the tool"; return 1; }
	scl::SGraphicsChaiMesh *tool_mesh = rchai.getChaiData()->meshes_rendered_.at("tool");


	env_pos_sim<<0,0,0;
    flag = rchai.addMeshToRender("env","../../specs/GraphicObjects/Env.obj",env_pos_sim, Eigen::Matrix3d::Identity());
	if(false == flag){std::cout<<"\nCould not load the env object"; return 1; }
	scl::SGraphicsChaiMesh *env_mesh = rchai.getChaiData()->meshes_rendered_.at("env");

    scl::SRigidBodyDyn *rbd=rgcm.rbdyn_tree_.at("link7");
	//Show Frames
	rchai.getChaiData()->meshes_rendered_.at("tool")->graphics_obj_->setShowFrame(true);
	rchai.getChaiData()->meshes_rendered_.at("tool")->graphics_obj_->setFrameSize(0.1);

	rchai.getChaiData()->meshes_rendered_.at("env")->graphics_obj_->setShowFrame(true);
	rchai.getChaiData()->meshes_rendered_.at("env")->graphics_obj_->setFrameSize(0.1);

	Eigen::Vector3d sphere_pos(-0.5,-0.5,0.5);
	flag = rchai.addSphereToRender(sphere_pos, goal_sphere,0.03);
	if(false == flag){std::cout<<"\nCould not load the sphere"; return 1; }

    flag = rchai.addMeshToRender("RotDes","../../specs/GraphicObjects/Tool.obj",Eigen::Vector3d(0,0,-0.5), Eigen::Matrix3d::Identity());
    if(false == flag){std::cout<<"\nCould not load the tool"; return 1; }


    flag = rchai.addSphereToRender(sphere_pos, ultimate_goal_sphere,0.03);
    if(false == flag){std::cout<<"\nCould not load the sphere"; return 1; }
    chai3d::cMaterial material;
    material.m_diffuse.setRed();
    material.m_emission.setRed();
    material.m_specular.setRed();
    material.m_ambient.setRed();
    ultimate_goal_sphere->setMaterial(material);

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

    for(int i = 0; i < dof; i++) IP->MaxVelocityVector->VecData[i] = .6;
    for(int i = 0; i < dof; i++) IP->MaxAccelerationVector->VecData[i] = 0.3;
    for(int i = 0; i < dof; i++) IP->MaxJerkVector->VecData[i] = 0.2;

    // Can optionally choose to ignore certain directions
    IP->SelectionVector->VecData[0]	=	true		;
    IP->SelectionVector->VecData[1]	=	true		;
    IP->SelectionVector->VecData[2]	=	true		;



    //INSERTION TRAJECTORY
    ReflexxesAPI *RMLinsertion = NULL;
    RMLPositionInputParameters *IPinsertion = NULL;
    RMLPositionOutputParameters	*OPinsertion =NULL;
    RMLPositionFlags Flagsinsertion;

    int dofinsertion = 3;

    // Initialize variables
    RMLinsertion = new ReflexxesAPI(dofinsertion, dt);
    IPinsertion = new RMLPositionInputParameters(dofinsertion);
    OPinsertion = new RMLPositionOutputParameters(dofinsertion);

    for(int i = 0; i < dofinsertion; i++) IPinsertion->MaxVelocityVector->VecData[i] = .05;
    for(int i = 0; i < dofinsertion; i++) IPinsertion->MaxAccelerationVector->VecData[i] = .01;
    for(int i = 0; i < dofinsertion; i++) IPinsertion->MaxJerkVector->VecData[i] = 0.005;

    // Can optionally choose to ignore certain directions
    IPinsertion->SelectionVector->VecData[0]	=	true		;
    IPinsertion->SelectionVector->VecData[1]	=	true		;
    IPinsertion->SelectionVector->VecData[2]	=	true		;


    //Orientation Trajectory
    ReflexxesAPI *RMLori = NULL;
    RMLPositionInputParameters *IPori = NULL;
    RMLPositionOutputParameters	*OPori =NULL;
    RMLPositionFlags Flagsori;

    int dofori = 4;

    // Initialize variables
    RMLori = new ReflexxesAPI(dofori, dt);
    IPori = new RMLPositionInputParameters(dofori);
    OPori = new RMLPositionOutputParameters(dofori);

    for(int i = 0; i < dofori; i++) IPori->MaxVelocityVector->VecData[i] = .004;
    for(int i = 0; i < dofori; i++) IPori->MaxAccelerationVector->VecData[i] = .001;
    for(int i = 0; i < dofori; i++) IPori->MaxJerkVector->VecData[i] = 0.001;


    // Can optionally choose to ignore certain directions
    IPori->SelectionVector->VecData[0]	=	true		;
    IPori->SelectionVector->VecData[1]	=	true		;
    IPori->SelectionVector->VecData[2]	=	true        ;

    Eigen::VectorXd last_v(3);
    last_v.setZero(3);
    // testing *****************QUATERNION
//    Eigen::Quaterniond laam;
//    laam.w()=0.2;
//    laam.x()=0.4;
//    laam.y()=0.5;
//    laam.z()=0.3;
//    laam.normalize();
//    Eigen::Matrix3d RRR;
//    RRR=laam;
//    std::cout << "\nrotation matrix" << RRR;
//    Eigen::Quaterniond lam2;
//    lam2=RRR;
//    std::cout <<"\noriginal quaternion" << laam.coeffs();
//    std::cout <<"\nafter rotation quaternion" << lam2.coeffs();

     //REDIS SERVER CONNECT

	//simulation  or actual bot
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
            //rctr.computeControlForces();
			has_updated_dyn_after_robot_comm_ = true;
		}
	}


	std::cout<<"Redis Server has updated dynamics"<<std::endl;
     /********************************* MASS OF PAYLOAD (for actual robot)***************************/
    rtask_hand->mass = 0.507;
    rtask_wrist->mass = 0.507;
    rtask_wrist->radius << 0.0,0.0,0.09;


	Eigen::VectorXd init_q(7);
	if(sim_only){
		// initialize Q only in simulation
		rio.sensors_.q_<<0, 1.57, 1.2, 1.57, 0, 0, 0;
        rtask_hand->flag_compute_op_gravity_=true;
        rtask_wrist->flag_compute_op_gravity_=true;
        rtask_hand->flag_compute_op_inertia_=true;
        rtask_wrist->flag_compute_op_inertia_=true;
        rctr.computeDynamics();

        rtask_hand->mass = 0.0;
        rtask_wrist->mass = 0.0;
        rtask_wrist->radius << 0.0,0.0,0.0 ;
        rtask_hand->pos_in_parent_ << 0,0,0;
        rtask_wrist->pos_in_parent_ << 0,0,0;
	}

    Eigen::VectorXd pos(4);

    omp_set_num_threads(3);
	int thread_id; double tstart, tcurr; flag = false;

    Eigen::Matrix3d R_des1,R_des2,R_des3;
	R_des1<<1,0,0,0,1,0,0,0,1;
    R_des2<<0,0,-1,0,1,0,1,0,0;

    int loop_ctr = 0, loop_ctr1=0;

	bool first_moveback_done=false; //STATE 1
	bool start_insertion=false; //STATE 2
    bool start_insertion_first_run=true;

	int update_period = 1;

	Eigen::Vector3d Traj_Intr_Point;
    Eigen::VectorXd Traj_Intr_lam_vec(4);
    Eigen::Quaterniond Traj_Intr_lam;

    Eigen::MatrixXd mov_dq(7,10);
    Eigen::VectorXd mov_avg_dq(7);


    bool has_keyboard_run=false;
    bool OptitrackCalibrated=false;

#pragma omp parallel private(thread_id)
	{
		thread_id = omp_get_thread_num();


		if(thread_id==1) //Simulate physics and update the rio data structure..
		{


            /*****************************************SELECTION MATRICES FOR FORCE COMPLIANT CONTROL**************************/

            Eigen::Matrix3d R_o_lnk, omega_EF_pos, omega_EF_ori , zeroM;
            Eigen::MatrixXd R_tilde(6,6) ,omega_EF_force(6,6), omega_EF_posori(6,6);
            /*omega_EF_pos<<1,0,0,0,1,0,0,0,1;
            omega_EF_ori<<1,0,0,0,1,0,0,0,1;
            R_o_lnk = rtask_hand->rbd_->T_o_lnk_.rotation();
            zeroM.setZero();

            rtask_hand->omega_inp = R_o_lnk.transpose()*omega_EF_pos*R_o_lnk;
            rtask_wrist->omega_inp = R_o_lnk.transpose()*omega_EF_ori*R_o_lnk;
            omega_EF_posori.setIdentity();
            omega_EF_posori<<omega_EF_pos,zeroM,zeroM,omega_EF_ori;
            R_tilde<<R_o_lnk,zeroM,zeroM,R_o_lnk;

            omega_EF_force.setIdentity();
            omega_EF_force -= omega_EF_posori;

            Eigen::MatrixXd TEMP(6,6);
            rtask_force->Omega_ = R_tilde.transpose()*omega_EF_force*R_tilde;


            std::cout<<"\rtask_force->Omega_ \n"<< rtask_force->Omega_<<std::endl; */

            Eigen::Matrix3d init_rot;
            Eigen::VectorXd init_rot_lam_vec(4);
            Eigen::Quaterniond init_rot_lam;
            init_rot = rtask_wrist->rbd_->T_o_lnk_.rotation();
            init_rot_lam = init_rot;

            init_rot_lam_vec(0) = init_rot_lam.x();
            init_rot_lam_vec(1) = init_rot_lam.y();
            init_rot_lam_vec(2) = init_rot_lam.z();
            init_rot_lam_vec(3) = init_rot_lam.w();

			Eigen::VectorXd init_pos;
			init_pos = rtask_hand->rbd_->T_o_lnk_*rtask_hand->pos_in_parent_;
			pos=init_pos;
            pos(1) -= 0;

            while(!OptitrackCalibrated)
                usleep(50000);

            Eigen::VectorXd GoalPos(3);

            Eigen::Matrix3d GoalRot;
            Eigen::Quaterniond Goal_Rot_lam;

			for(int i = 0; i < dof; i++) IP->CurrentPositionVector->VecData[i] = init_pos(i);
			for(int i = 0; i < dof; i++) IP->CurrentVelocityVector->VecData[i] = 0;
			for(int i = 0; i < dof; i++) IP->CurrentAccelerationVector->VecData[i] = 0;

            for(int i = 0; i < dofori; i++) IPori->CurrentPositionVector->VecData[i] = init_rot_lam_vec(i);
            for(int i = 0; i < dofori; i++) IPori->CurrentVelocityVector->VecData[i] = 0;
            for(int i = 0; i < dofori; i++) IPori->CurrentAccelerationVector->VecData[i] = 0;
			tstart = sutil::CSystemClock::getSysTime(); iter = 0;

			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{

                Eigen::Vector3d p_opti_in_robot=R_opti_robot*(-1*RobotPosOpti);
                T_opti_robot.translation()<<p_opti_in_robot(0),p_opti_in_robot(1),p_opti_in_robot(2)-0.15;
                std::cout<<"P_OPTI_IN_ROBOT: "<<p_opti_in_robot.transpose()<<std::endl;

				tcurr = sutil::CSystemClock::getSysTime();
				Eigen::Vector3d end_eff_pos=rtask_hand->x_;
				double error=0.0;
				//STATE 1
				if(first_moveback_done==false)
				{
                    GoalPos=pos;

					if(loop_ctr % update_period == 0)
					{
						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = GoalPos(0);
						IP->TargetPositionVector->VecData[1] = GoalPos(1);
						IP->TargetPositionVector->VecData[2] = GoalPos(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;
					}
					loop_ctr++;

                    rtask_wrist->has_been_activated_=false;
					int ResultValue	= RML->RMLPosition(*IP, OP, Flags);

					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

					for(int i = 0; i < dof; i++) Traj_Intr_Point(i) = OP->NewPositionVector->VecData[i];
					R_des1<<1,0,0,0,0,1,0,-1,0;

					error=(GoalPos-rtask_hand->x_).norm();
                    //std::cout <<"\n Norm error" <<error;

                    if (error<0.01)
					{
						first_moveback_done=true;


					}

				}

                else //if (start_insertion == false)
				{


                    //Move goal position so that tray is at start of insertion (As goal point is further tray distance away )
                    Eigen::Vector3d tray_goal_offset(0.05,0,0);
                   // end_eff_pos=rbd->T_o_lnk_.rotation()*Graphic_R_Optienv_simenv*p_tool_ee;
                    GoalPos=EnvPosOpti-EnvRotOpti*tray_goal_offset;

                    R_des2<<0,0,1,0,1,0,-1,0,0;
                    R_des3<<0,-1,0,1,0,0,0,0,1;
                    R_des1 = EnvRotOpti*R_des2 ;
                    Goal_Rot_lam = R_des1;

					if(loop_ctr % update_period == 0)
					{
						// IMPORTANT: set target values
						IP->TargetPositionVector->VecData[0] = GoalPos(0);
						IP->TargetPositionVector->VecData[1] = GoalPos(1);
						IP->TargetPositionVector->VecData[2] = GoalPos(2);

						IP->TargetVelocityVector->VecData[0] = 0.0;
						IP->TargetVelocityVector->VecData[1] = 0.0;
						IP->TargetVelocityVector->VecData[2] = 0.0;

                        // IMPORTANT: set target values orientation
                        IPori->TargetPositionVector->VecData[0] = Goal_Rot_lam.x();
                        IPori->TargetPositionVector->VecData[1] = Goal_Rot_lam.y();
                        IPori->TargetPositionVector->VecData[2] = Goal_Rot_lam.z();
                        IPori->TargetPositionVector->VecData[3] = Goal_Rot_lam.w();


                        IPori->TargetVelocityVector->VecData[0] = 0.0;
                        IPori->TargetVelocityVector->VecData[1] = 0.0;
                        IPori->TargetVelocityVector->VecData[2] = 0.0;
                        IPori->TargetVelocityVector->VecData[3] = 0.0;
					} loop_ctr++;

					rtask_wrist->has_been_activated_=true;

                    // ResultValue contains status information
                    int ResultValue	= RML->RMLPosition(*IP, OP, Flags);
                    for(int i = 0; i < dof; i++) Traj_Intr_Point(i) = OP->NewPositionVector->VecData[i];

                    // ResultValue contains status information orientation
                   int ResultValue_ori	= RMLori->RMLPosition(*IPori, OPori, Flagsori);
                    for(int i = 0; i < dofori; i++) Traj_Intr_lam_vec(i) = OPori->NewPositionVector->VecData[i];

					// IMPORTANT: set the data to some initial values for the current state
					*IP->CurrentPositionVector	=	*OP->NewPositionVector		;
					*IP->CurrentVelocityVector	=	*OP->NewVelocityVector		;
					*IP->CurrentAccelerationVector	=	*OP->NewAccelerationVector	;

                    // IMPORTANT: set the data to some initial values for the current state
                    *IPori->CurrentPositionVector	=	*OPori->NewPositionVector		;
                    *IPori->CurrentVelocityVector	=	*OPori->NewVelocityVector		;
                    *IPori->CurrentAccelerationVector	=	*OPori->NewAccelerationVector	;

                    Traj_Intr_lam.x() = Traj_Intr_lam_vec(0);
                    Traj_Intr_lam.y() = Traj_Intr_lam_vec(1);
                    Traj_Intr_lam.z() = Traj_Intr_lam_vec(2);
                    Traj_Intr_lam.w() = Traj_Intr_lam_vec(3);

                    Traj_Intr_lam.normalize();
                    R_des1 = Traj_Intr_lam;

					error=(GoalPos-rtask_hand->x_).norm();
                    //std::cout <<"\nNorm error" <<error;

                    if (error<0.02)
					{
						start_insertion=true;
					}
                }
//                else if (start_insertion == true) {

//                    omega_EF_pos<<0,0,0,0,0,0,0,0,0;        // for compliant insertion
//                    omega_EF_ori<<0,0,0,0,0,0,0,0,0;
//                    R_o_lnk = rtask_hand->rbd_->T_o_lnk_.rotation();

//                    if(start_insertion_first_run)
//                    {

//                        Eigen::Vector3d temp_position =rtask_hand->rbd_->T_o_lnk_*rtask_hand->pos_in_parent_;

//                        for(int i = 0; i < dof; i++) IPinsertion->CurrentPositionVector->VecData[i] = temp_position(i);
//                        for(int i = 0; i < dof; i++) IPinsertion->CurrentVelocityVector->VecData[i] = 0;
//                        for(int i = 0; i < dof; i++) IPinsertion->CurrentAccelerationVector->VecData[i] = 0;
//                        start_insertion_first_run=false;
//                    }

//                    rtask_hand->omega_inp = R_o_lnk.transpose()*omega_EF_pos*R_o_lnk;
//                    rtask_wrist->omega_inp = R_o_lnk.transpose()*omega_EF_ori*R_o_lnk;

//                    Eigen::Vector3d goal_final(0,0.1,0); // offset in optitrack frame
//                    GoalPos=EnvPosOpti-EnvRotOpti*goal_final;
//                    R_des1 = EnvRotOpti*R_des2 ;

//                    if(loop_ctr1 % update_period == 0)
//                    {
//                        // IMPORTANT: set target values
//                        IPinsertion->TargetPositionVector->VecData[0] = GoalPos(0);
//                        IPinsertion->TargetPositionVector->VecData[1] = GoalPos(1);
//                        IPinsertion->TargetPositionVector->VecData[2] = GoalPos(2);

//                        IPinsertion->TargetVelocityVector->VecData[0] = 0.0;
//                        IPinsertion->TargetVelocityVector->VecData[1] = 0.0;
//                        IPinsertion->TargetVelocityVector->VecData[2] = 0.0;

//                    } loop_ctr1++;

//                    rtask_wrist->has_been_activated_=true;

//                    // ResultValue contains status information
//                    int ResultValue	= RMLinsertion->RMLPosition(*IPinsertion, OPinsertion, Flagsinsertion);
//                    for(int i = 0; i < dofinsertion; i++) Traj_Intr_Point(i) = OPinsertion->NewPositionVector->VecData[i];

//                    // IMPORTANT: set the data to some initial values for the current state
//                    *IPinsertion->CurrentPositionVector	=	*OPinsertion->NewPositionVector		;
//                    *IPinsertion->CurrentVelocityVector	=	*OPinsertion->NewVelocityVector		;
//                    *IPinsertion->CurrentAccelerationVector	=	*OPinsertion->NewAccelerationVector	;

//                }


                if(!scl::CDatabase::getData()->s_gui_.ui_flag_[1]) //OPTITRACK CONTROL
                {
                 rtask_hand->x_goal_ = Traj_Intr_Point;
                 rtask_wrist->lam_goal_ = R_des1;
                 has_keyboard_run = false;

                }

                else //1 is pressed again
                {
                   if(!has_keyboard_run)
                   {
                    scl::CDatabase::getData()->s_gui_.ui_point_[0]=rtask_hand->x_;
                    has_keyboard_run = true;
                   }
                   rtask_wrist->has_been_activated_=true;
                   rtask_hand->x_goal_ = scl::CDatabase::getData()->s_gui_.ui_point_[0];
                   rtask_wrist->lam_goal_ = init_rot;

                }

                /* ***************************************MOVING AVERAGE FOR Q's**********************************/
                /*if(iter<10) {
                for (int j=0;j<8;j++)
                { mov_dq(j,iter)=rio.sensors_.dq_(j);
                  mov_avg_dq(j)=0;
                    }
                }
                if (iter>=10) {
                    for (int j=0;j<8;j++) {
                      for  (int k=0; k<9; k++) {
                          mov_dq(j,k)= mov_dq(j,k+1); // shift the elements left in every iteration
                      }
                        mov_dq(j,9) = rio.sensors_.dq_(j); // set the last one to be present q values
                       for (int k=0;k<10;k++) {
                           mov_avg_dq(j) +=mov_dq(j,k);
                            }
                       mov_avg_dq(j)=mov_avg_dq(j)/10;
                    }
                   // rio.sensors_.dq_ = 0.95*mov_avg_dq + 0.05*rio.sensors_.dq_;
                }*/

                // set the present dq to the average
                //std::cout << "\naverage q's" << mov_avg_dq.transpose() <<"\n";
                //std::cout << "\npresent q's" << rio.sensors_.dq_.transpose() <<"\n";

				// Compute control forces (note that these directly have access to the io data ds).
				rctr.computeDynamics();
				rctr.computeControlForces();

				if(sim_only)
				{
					dyn_tao.integrate(rio,dt);
				}

				iter++;

				/**************************************SET POSTIION OF GRAPHICS OBJECTS***************************/
				//SET TOOL POSITION
                Eigen::MatrixXd T_matrix;
                Eigen::Matrix3d Traj_rot;
				T_matrix=rbd->T_o_lnk_.matrix();
				Eigen::Vector4d zero_vector(0,0,0,1);
				Eigen::Vector4d tool_pos=T_matrix*zero_vector;
                chai3d::cMatrix3d rot_matrix ,_90matrix, Traj_rotc;
				rot_matrix.set(T_matrix(0,0),T_matrix(0,1),T_matrix(0,2),T_matrix(1,0),T_matrix(1,1),T_matrix(1,2),T_matrix(2,0),T_matrix(2,1),T_matrix(2,2));
                _90matrix.set(1,0,0,0,0,1,0,-1,0);
				rot_matrix=rot_matrix*_90matrix;
                Traj_rot = Traj_Intr_lam;
                Traj_rotc.set(Traj_rot(0,0),Traj_rot(0,1),Traj_rot(0,2),Traj_rot(1,0),Traj_rot(1,1),Traj_rot(1,2),Traj_rot(2,0),Traj_rot(2,1),Traj_rot(2,2));
                Traj_rotc = Traj_rotc*_90matrix;


				rchai.getChaiData()->meshes_rendered_.at("tool")->graphics_obj_->setLocalRot(rot_matrix);
				rchai.getChaiData()->meshes_rendered_.at("tool")->graphics_obj_->setLocalPos(tool_pos[0],tool_pos[1],tool_pos[2]);

                rchai.getChaiData()->meshes_rendered_.at("RotDes")->graphics_obj_->setLocalRot(Traj_rotc);
                rchai.getChaiData()->meshes_rendered_.at("RotDes")->graphics_obj_->setLocalPos(Traj_Intr_Point[0],Traj_Intr_Point[1],Traj_Intr_Point[2]);

				if(ENABLE_OPTITRACK)
				{

                    Eigen::Matrix3d LocalRot=EnvRotOpti;
                    Eigen::Vector3d LocalPos=EnvPosOpti;

                    rot_matrix.set(LocalRot(0,0),LocalRot(0,1),LocalRot(0,2),LocalRot(1,0),LocalRot(1,1),LocalRot(1,2),LocalRot(2,0),LocalRot(2,1),LocalRot(2,2));
                    rchai.getChaiData()->meshes_rendered_.at("env")->graphics_obj_->setLocalRot(rot_matrix);
                    rchai.getChaiData()->meshes_rendered_.at("env")->graphics_obj_->setLocalPos(LocalPos[0],LocalPos[1],LocalPos[2]);

				}

				//SET GOAL SPHERE POSITION
                goal_sphere->setLocalPos(rtask_hand->x_goal_[0],rtask_hand->x_goal_[1],rtask_hand->x_goal_[2]);
                ultimate_goal_sphere->setLocalPos(GoalPos(0),GoalPos(1),GoalPos(2));


			}
			//Then terminate
			scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running = false;
		}

		else if (thread_id==0)  //Read the rio data structure and updated rendererd robot..
			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{ glutMainLoopEvent(); const timespec ts = {0, 15000000};/*15ms*/ nanosleep(&ts,NULL); }

		else if(thread_id==2) //Optitrack
		{

            Eigen::Matrix3d TempRotOpti;
            Eigen::Vector3d TempPosOpti;

            Eigen::Vector3d TempRobotPosOpti;

            bool RobotPositionCallibrate=false;

            int Optitrackiter=0;

			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{
				if(ENABLE_OPTITRACK)
				{
					double *currentPosition;
					currentPosition = new double[3];
					double **currentRotation;
					currentRotation = new double*[3];
					for (int i = 0; i < 3; i++)
					{
						currentRotation[i] = new double[3];
					}

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


						if(k==0)
						{
							//FILLER
						}

						if(k==1)
						{
							//TOOL
						}

						if(k==2) //ENV
						{
							for (unsigned int i = 0; i < 3; i++)
							{
								for (unsigned int j = 0; j < 3; j++)
								{
                                    TempRotOpti(i,j)=currentRotation[i][j];
								}
							}

                            TempPosOpti(0)=currentPosition[0];
                            TempPosOpti(1)=currentPosition[1];
                            TempPosOpti(2)=currentPosition[2];

                            EnvPosOpti=T_opti_robot*TempPosOpti;
                            EnvRotOpti=R_opti_robot*TempRotOpti;


						}

                        if(k==3)
                        {
                            TempRobotPosOpti(0)=currentPosition[0];
                            TempRobotPosOpti(1)=currentPosition[1];
                            TempRobotPosOpti(2)=currentPosition[2];

                            std::cout<<"RAW ROBOT POS: "<<TempRobotPosOpti.transpose()<<std::endl;


                            if(!OptitrackCalibrated)
                            {
                            RobotPosOpti=TempRobotPosOpti;
                            Optitrackiter++;
                            if(Optitrackiter>100)
                            {
                                OptitrackCalibrated=true;
                            }

                            }
                            //ROBOT
                        }
					}
				}
			}
		}

		else if(thread_id==3) //Force Sensor Thread
		{

            std::string host_ip = "127.0.0.1";
            int host_port = 6379;

            std::string key = "jr3_ft_sensor";

            redox::Redox rdx;

            if(!rdx.connect(host_ip, host_port))
            {
                std::cout<<"UNABLE TO CONNECT TO FORCE SENSOR SERVER!"<<std::endl;
                ENABLE_FORCE_SENSOR=false;
            }



			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{
                if(ENABLE_FORCE_SENSOR)
                {
                        double arr_in[6] = {}; // input array
                        std::istringstream iss(rdx.get(key)); // for converting to double

                        // read doubles from string
                        for(int i = 0; i < 6; i++) iss >> arr_in[i];

                        std::cout << "read: ";
                        for(int i = 0; i < 6; i++) std::cout << arr_in[i] << " ";
                        std::cout << "\n";
                        usleep(1000);

				}
                else
                {
                    std::cout<<"FORCE SENSOR DEACTIVATED!"<<std::endl;
                }
			}
		}


	}

    rtask_hand->force_gc_.setZero(7);
    rtask_wrist->force_gc_.setZero(7);
    rtask_force->force_gc_.setZero(7);


	/******************************Exit Gracefully************************************/
	std::cout<<"\n\n\tSystem time = "<<sutil::CSystemClock::getSysTime()-tstart;
	std::cout<<"\n\tSimulated time = "<<static_cast<double>(iter)*dt;
	std::cout<<"\n\nExecuted Successfully";
	std::cout<<"\n**********************************\n"<<std::flush;
    delete[] objects;    
	return 0;
}

scl::sBool InitController()
{
	try
	{
        // Make sure you get zero torques at the start...
        scl::CDatabase::getData()->s_gui_.ui_flag_[5] = false;
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
                    rio.sensors_.dq_(i) = measured_dq[i];
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
