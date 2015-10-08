/** A simple application to talk to the Barrett WAM
 * robot. The application communicates with a controller
 * over a shared memory interface */

// This provides the default data structure
#include <scl-wam/SShmObjWAM.hpp>

// Shared Memory Includes
#include <sutil/CSharedMemory.hpp>

// For Timing
// #include <sutil/CSystemClock.hpp>
#include <ctime>
#include <chrono>
#include <sstream> // stringstream
#include <iomanip> // put_time
#include <string>  // string
#include <iostream>
#include <vector>
#include <stdio.h>
#include <signal.h>

// For setting affinity
#include <sys/time.h>
#include <sys/resource.h>


bool running = true;

/** INThandler
 * Handles an interrupt (^c) from the command line
 * to kill the process and child process.
 */
void INThandler(int sig) {
  std::cout<<"Signal Caught!"<<std::endl;
  running = false;
}

/** setVector
 * Sets all elements of a vector of doubles to a certain value.
 */
void setVector(std::vector<double>& v, double value) {
  for (unsigned int i = 0; i < v.size(); i++) v[i] = value;
}

/** printData
 * Prints the position, velocity, and torque vectors.
 */
void printData(const double t_curr, const std::vector<double>& p, const std::vector<double>& v,
    const std::vector<double>& a, const std::vector<double>& t){
  printf("t(%lf) q(%lf, %lf, %lf) ",t_curr, p[0],p[1],p[2]);
  printf("dq(%lf, %lf, %lf) ",v[0],v[1],v[2]);
  printf("ddq(%lf, %lf, %lf) ",a[0],a[1],a[2]);
  printf("fq(%lf, %lf, %lf)\n",t[0],t[1],t[2]);
}

// Sets all the torques to 0. Should be ran on startup, before anything
// else should have touched the virtual memory.
void resetSHMemTorques( sutil::CSharedMemory<SShmObjWAM, char>& shmem){
  for(int i=0; i<3; i++){
    shmem.data_->f_q_[i] = 0;
  }
}

/** setSHMem
 * Sets the position and velocity data in shared memory and
 * reads the torque data.  Returns true on success or false
 * if the child GUI process was busy with the shared memory.
 */
bool setSHMem( sutil::CSharedMemory<SShmObjWAM, char>& shmem,
    const std::vector<double>& pos,
    const std::vector<double>& vel,
    const std::vector<double>& acc,
    std::vector<double>& tor,
    bool& log_enabled,
    double server_time) {
  if('1' == *(shmem.data_signal_) ) {
    *(shmem.data_signal_) = '0';

    for(int i=0; i<3; i++){
      shmem.data_->q_[i] = pos[i];
      shmem.data_->dq_[i] = vel[i];
      shmem.data_->ddq_[i] = acc[i];
      tor[i] = shmem.data_->f_q_[i];
    }

    log_enabled = shmem.data_->log_enabled;
    shmem.data_->server_time = server_time;

    *(shmem.data_signal_) = '1';

    // Give the other guy some time to do stuff
    // const timespec ts = {0, 500000};//Sleep for 0.5ms
    // nanosleep(&ts,NULL);

    return true;
  }
  return false;
}

/** torqueCheckAndSaturate
 * Currently sets all torques to zero.
 */
bool torqueCheckAndSaturate(std::vector<double>& torque, double torque_max) {
  bool flag=false;
  for (unsigned int i = 0; i < 3; i++) {
    if (torque[i] > torque_max)
    { torque[i] = torque_max;  }
    else if(torque[i] < -torque_max)
    { torque[i] = -torque_max;  }
    else { continue; }
    flag=true;
  }
  return flag;
}

std::string current_time_and_date()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];

  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime (buffer,80,"%F_%H-%M-%S",timeinfo);
  return std::string(buffer);

  return 0;
}

/**
 * The main server loop
 */
void serverLoop(double target_frequency, int num_motors, int num_encoders, double torque_max,
    sutil::CSharedMemory<SShmObjWAM, char>& shmem){

  printf("Beginning server loop at %f.\n",target_frequency);

  //Set up data variables.
  // == num motors
  std::vector<double> torque(num_motors); // Command Vectors
  std::vector<double> torque_des(num_motors); // Command Vectors

  // == num encoders
  std::vector<double> q_ret(num_encoders), dq_ret(num_encoders), ddq_ret(num_encoders); // Response Vectors
  std::vector<double> t_ret(num_encoders); // Timestamp Information

  //Zero everything..
  for(int i=0;i<num_motors;++i)
  {
    torque[i] = 0;
    torque_des[i] = 0;
  }
  for(int i=0;i<num_encoders;++i)
  {
    q_ret[i] = 0;
    dq_ret[i] = 0;
    ddq_ret[i] = 0;
    t_ret[i] = 0;
  }

  //Initialize the time
  auto t0 = std::chrono::high_resolution_clock::now();
  int loop_counter = 0;

  // Frequency Limiting Information
  auto t_curr = t0;
  auto t_last = t0;

  // Loop Print Constants
  auto t_last_loop = t0;
  double dt = 1.0/target_frequency;
  int loops_per_print = 5000;

  //Settings to log the output
  bool print = true;


  // TODO: Move Files to temp folder for logging.
  // NOTE: SYSTEM CALLS SHOULD BE DONE IN SCRIPT. Just lazy for now.
  auto r = system("cp /mnt/ram/* /home/data-shared/wam_logs/");
  r = system("mkdir /mnt/ram"); /* to suppress compiler warnings */ if (r){} 
  r = system("umount /mnt/ram");
  r = system("mount -t ramfs -o size=2000m ramfs /mnt/ram");
  bool log_enabled = false;
  bool last_log_state = log_enabled;
  FILE* logfile = NULL;

  const double kMicrosInSec = 1000000.0;

  while(running){
    t_last = t_curr;
    t_curr = std::chrono::high_resolution_clock::now();

    //** The actual server work **//

    // Set the applied torque to the desired torque..
    for(int i=0;i<num_motors;++i)
    { torque[i] = torque_des[i];  }

    // Check torque is not too high, apply torque, and check shared memory
    torqueCheckAndSaturate(torque, torque_max);

    // Read the encoders and set the torque for the motors
    // NOTE TODO :
    //driver.readEncodersAndCommandMotors(torque, t_ret, q_ret, dq_ret, ddq_ret);

    double currtime_sec = std::chrono::duration_cast<std::chrono::microseconds>(t_curr-t0).count()/kMicrosInSec;

    // Set the encoder stuff in the shared memory and read in desired torques..
    setSHMem(shmem, q_ret, dq_ret, ddq_ret, torque_des, log_enabled, currtime_sec);

    //** Logging Work : NOTE that we log applied torques, not desired torques **//
    if(print && loop_counter % loops_per_print == 0 && loop_counter > 0) {
      printf("%f Hz, ", 
          (kMicrosInSec/std::chrono::duration_cast<std::chrono::microseconds>(t_curr-t_last_loop).count())*loops_per_print);

      t_last_loop = t_curr;
      printData(
          std::chrono::duration_cast<std::chrono::microseconds>(t_curr-t0).count()/kMicrosInSec
          , q_ret, dq_ret, ddq_ret, torque);
    }

    // Custom logging code for wam
    if(!last_log_state && log_enabled){
      printf("Started Log \n");
      std::string fname = std::string("/mnt/ram/");
      fname += (current_time_and_date());
      fname += (std::string(".log"));
      logfile = fopen(fname.c_str(),"a");
      last_log_state = log_enabled;
    } else if (last_log_state && !log_enabled){
      printf("Stopped Logging \n");
      fclose(logfile);
      last_log_state = log_enabled;
      r = system("cp /mnt/ram/* /home/data-shared/wam_logs/");
    }

    if(log_enabled){
      fprintf(logfile,"%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\n",
          currtime_sec,
          q_ret[0], dq_ret[0], ddq_ret[0],
          q_ret[1], dq_ret[1], ddq_ret[1],
          q_ret[2], dq_ret[2], ddq_ret[2],
          torque[0], torque[1], torque[2]);
      fflush(logfile);
    }

    //** Prepare to loop again **//
    loop_counter++;

    //** Frequency Limiting (busy wait to keep processor)**//
    while(running && std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now()-t_curr).count()/kMicrosInSec < dt){}
  }

  if(log_enabled){
    fclose(logfile);
    printf("File Closed\n");
  }
  r = system("cp /mnt/ram/* /home/data-shared/wam_logs/");
}


/**
 * Main Function
 */ 

int main(int argc, char** argv){
  if(argc != 2){
    std::cout<<"\n==== WAM Control Server ==== "
        <<"\nSyntax :"
        <<"\n\t$sudo ./wam_shmem_serv <Force Limits> \n\n";
    return 1;
  }

  printf("Setting Affinity\n");
  if(setpriority(PRIO_PROCESS, 0, -20)<0){
    printf("Error Setting Affinity. Run as Sudo.\n");
    return 1;
  }

  int num_motors = DOF_WAM, num_encoders = DOF_WAM;
  double arg_torque_max = atof(argv[1]);

  // Set up WAM driver..
  // NOTE TODO : Init "driver" object here..

  // Create the shared memory
  sutil::CSharedMemory<SShmObjWAM, char> shmem(8080,'x');
  if (shmem.shmCreate()) { printf("Created shared memory\n"); }
  else { printf("Failed to create shared memory\n"); return 1; }
  *(shmem.data_signal_) = '1';

  //Set all data in the shared memory to zero
  std::vector<double> tmp(num_motors);
  bool tmpbool;
  double tmpdouble = 0;
  setVector(tmp,0.0);
  setSHMem(shmem,tmp,tmp,tmp,tmp,tmpbool,tmpdouble);  
  resetSHMemTorques(shmem);

  // Register signal handler
  signal(SIGINT, INThandler);

  double target_frequency = 750;

  // Run the Server loop.
  serverLoop(target_frequency, num_motors, num_encoders, arg_torque_max, shmem, driver);

  //Checks whether the shmem signal is 'x', and terminates when it is
  // while(!shmem.shmCheckDetach()) {}
  // std::cout<<"\nTest Result Server: Client terminated, detaching shared memory";

  // NOTE TODO : Shut down driver here
  // driver.shutdown();
}
