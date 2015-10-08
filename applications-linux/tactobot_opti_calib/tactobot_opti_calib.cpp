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
#include <fstream>

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

scl::sBool InitController();
void initRedox();

int main(int argc, char** argv)
{

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

    std::string grasped_object_name = "robot";
	std::string env_object_name = "env";
	objects = new OptiTrack[nObjects];
	objects[0].Init(vrpn_server_ip, "Filler");
	objects[1].Init(vrpn_server_ip, grasped_object_name);
	objects[2].Init(vrpn_server_ip, env_object_name);

	//Graphics Objects
	chai3d::cGenericObject *goal_sphere;


	//Position and rotation from OptiTrack
	Eigen::Vector3d SharedPosition;
	Eigen::Matrix3d SharedRotation;


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

	rtask_wrist = dynamic_cast<scl_app::STaskOpOri*>( *(rctr_ds.tasks_.at("wrist")));
	if(NULL == rtask_wrist)  {return 1;}       //Error check



	/******************************ChaiGlut Graphics************************************/
	glutInit(&argc, argv); // We will use glut for the window pane (not the graphics).

	flag = p.readGraphicsFromFile("../../specs/Barrett/wamCfg.xml","wamBotStdView",rgr);
	flag = flag && rchai.initGraphics(&rgr);
	flag = flag && rchai.addRobotToRender(&rds,&rio);
	flag = flag && scl_chai_glut_interface::initializeGlutForChai(&rgr, &rchai);
	if(false==flag) { std::cout<<"\nCouldn't initialize chai graphics\n"; return 1; }


	Eigen::Vector3d sphere_pos(-0.5,-0.5,0.5);
	flag = rchai.addSphereToRender(sphere_pos, goal_sphere,0.03);
	if(false == flag){std::cout<<"\nCould not load the sphere"; return 1; }

	/******************************Simulation************************************/
	// Now let us integrate the model for a variety of timesteps and see energy stability
	std::cout<<"\nIntegrating the r6bot's physics. \nWill test two different controllers.\n Press (x) to exit at anytime.";
	long long iter = 0, cur_iter=0; double dt=0.0001;

	

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

	Eigen::VectorXd init_q(7);
    if(sim_only)
    {
		// initialize Q only in simulation
		rio.sensors_.q_<<0, 1.57, 1.2, 1.57, 0, 0, 0;
	}

	Eigen::VectorXd pos(3);

	rctr.computeDynamics();

	Eigen::Matrix3d init_rot;
	init_rot = rtask_wrist->rbd_->T_o_lnk_.rotation();

	omp_set_num_threads(3);
	int thread_id; double tstart, tcurr; flag = false;

    std::ofstream myfile;
    myfile.open ("OptitrackCallib.txt");

#pragma omp parallel private(thread_id)
	{
		thread_id = omp_get_thread_num();

		if(thread_id==1) //Simulate physics and update the rio data structure..
		{

            int loop_ctr = 0;

            std::vector<Eigen::Vector3d> RobotPositions;
            std::vector<Eigen::Vector3d> OptitrackPositions;

			while(true == scl_chai_glut_interface::CChaiGlobals::getData()->chai_glut_running)
			{
                std::string a;

                //std::cin>>a;

                //SET TOOL POSITION
                 rctr.computeDynamics();
                 Eigen::MatrixXd T_matrix;
                 scl::SRigidBodyDyn *rbd=rgcm.rbdyn_tree_.at("link7");
                 Eigen::Vector3d zero_vector(0,0,0);
                 Eigen::Vector3d tool_pos=rbd->T_o_lnk_*zero_vector;

                 if(loop_ctr++ % 100 == 0) {


                RobotPositions.push_back(tool_pos);
                OptitrackPositions.push_back(SharedPosition);
                std::cout<<"Got a position: \n";
                std::cout<<"Robot position (CHAI): "<<tool_pos.transpose()<<std::endl;
                std::cout<<"Robot position (OPTITRACK): "<<SharedPosition.transpose()<<std::endl;


                //myfile<<"Robot position (CHAI): "<<tool_pos.transpose()<<std::endl;
                //myfile<<"Robot position (OPTITRACK): "<<SharedPosition.transpose()<<std::endl<<std::endl;
                 }

                 usleep(1000);

            }

            for(int i = 0; i < RobotPositions.size(); i++) {
                myfile << RobotPositions[i].transpose() << "\n";
            }

            myfile << "\n";

            for(int i = 0; i < OptitrackPositions.size(); i++) {
                myfile << OptitrackPositions[i].transpose() << "\n";
            }

            myfile.close();
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
								SharedRotation(i,j)=currentRotation[i][j];
							}
						}
						SharedPosition=pos;

							}
				}
			}
		}
	}

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



			// READING JOINT VELOCITIES
			float dqi;
			if(json_sensor_.isMember("dq") && json_sensor_["dq"].isArray()) {
				for(int i = 0; i < 7; i++) {
					dqi = json_sensor_["dq"][i].asFloat();
					measured_dq[i] = dqi;
				}
			}

			// PRINTING JOINT VELOCITIES



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
            //std::cout << "[OUT: tau] -> ";
			for(int i = 0; i < 6; i++) {
                //std::cout << computed_tau[i] << " ";
			}
            //std::cout << computed_tau[6] << std::endl;

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
