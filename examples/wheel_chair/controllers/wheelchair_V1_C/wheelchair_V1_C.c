/*
 * File:          wheelchair_V1_C.c
 * Date:          03-26-2022
 * Description:   wheelchair controller
 * Author:        DrakerDG
 * Modifications: 1
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>

#define MAX_SP 4
#define MIN_DT 850

int MAX_TM = 65 / MAX_SP;

int timestep = 0;
int left_speed = MAX_SP;
int right_speed = MAX_SP;  
int turn = 0;
int timer = 0;

// Motors
WbDeviceTag left_motor, right_motor;

// IR Sensors
WbDeviceTag left_IR, right_IR;

int Obstacle_Detection(void) {
  int go_turn = 0;
  int LIR =  wb_distance_sensor_get_value(left_IR);
  int RIR =  wb_distance_sensor_get_value(right_IR);
  if (LIR < MIN_DT || RIR < MIN_DT) go_turn = 1;
  if (LIR > RIR) go_turn *= -1;
  return go_turn;  
}

void Set_Direction(void) {
  if (turn == 0) turn = Obstacle_Detection();
  else {
    if (timer == 0) {
      left_speed = MAX_SP * turn;
      right_speed = - MAX_SP * turn;
    }
    timer += 1;
    if (timer > MAX_TM) {
      left_speed = MAX_SP;
      right_speed = MAX_SP;
      timer = 0;
      turn = 0;
    }
  }
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
}


int main(int argc, char **argv) {
  wb_robot_init();

  // Initalize time step
  timestep = wb_robot_get_basic_time_step();

  // Intialize motors
  left_motor = wb_robot_get_device("left motor");
  right_motor = wb_robot_get_device("right motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);

  // Intialize sensors
  left_IR = wb_robot_get_device("left IR");
  right_IR = wb_robot_get_device("right IR");
  wb_distance_sensor_enable(left_IR, timestep);
  wb_distance_sensor_enable(right_IR, timestep);

  // Intialize cameras
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep * 2);

  // main loop
  while (wb_robot_step(timestep) != -1) {
    
    Set_Direction();  

  };

  wb_robot_cleanup();
  return 0;
}
