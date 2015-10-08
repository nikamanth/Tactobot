/*
 * File : main.cpp
 * Date : 23 May, 2015
 *
 * Code for:
 * CS225A : Experimental Robotics
 * Stanford University
 *
 * Description : This provides a basic driver application that communicates
 * with the Barrett driver and sends/receives commands to a redis server.
 *
 * Control applications then communicate with the redis server and send
 * position/torque commands to the wam robot.
 *
 * NOTE : This application is under development and the API may change at
 * any given point of time.
 */

#include "CManipsWamDriver.hpp"

using namespace barrett;

int main(int argc, char* argv[]) {

  //std::string redis_host = "192.168.0.111";
  std::string redis_host = "localhost";
  int redis_port = 6379;

  std::string sensors_key = "wamBot:sensors";
  std::string actuators_key = "wamBot:actuators";

  CManipsWamDriver driver(redis_host, redis_port, sensors_key, actuators_key);

  if(!driver.connect()) return 1;

  driver.run();

  return 0;
}
