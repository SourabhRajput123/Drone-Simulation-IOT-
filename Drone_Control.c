/*
 * Crazyflie Controller for Webots
 * Author: Sourabh Rajput (based on Bitcraze AB example)
 */

#include <math.h>
#include <stdio.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Add external controller (PID controller file)
#include "pid_controller.h"

#define FLYING_ALTITUDE 1.0 // Desired flying altitude

int main(int argc, char **argv) {
  wb_robot_init();  // Initialize the robot

  const int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");

  // Set motors to velocity mode
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_position(m4_motor, INFINITY);

  wb_motor_set_velocity(m1_motor, -1.0);  // Motor 1 initial velocity
  wb_motor_set_velocity(m2_motor, 1.0);   // Motor 2 initial velocity
  wb_motor_set_velocity(m3_motor, -1.0);  // Motor 3 initial velocity
  wb_motor_set_velocity(m4_motor, 1.0);   // Motor 4 initial velocity

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  WbDeviceTag camera = wb_robot_get_device("camera");

  // Enable sensors with timestep
  wb_inertial_unit_enable(imu, timestep);
  wb_gps_enable(gps, timestep);
  wb_gyro_enable(gyro, timestep);
  wb_camera_enable(camera, timestep);
  
  // Distance sensors for obstacle detection
  WbDeviceTag range_front = wb_robot_get_device("range_front");
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  WbDeviceTag range_right = wb_robot_get_device("range_right");

  wb_distance_sensor_enable(range_front, timestep);
  wb_distance_sensor_enable(range_left, timestep);
  wb_distance_sensor_enable(range_back, timestep);
  wb_distance_sensor_enable(range_right, timestep);

  // Initialize keyboard for manual control
  wb_keyboard_enable(timestep);

  // Wait for 2 seconds to stabilize
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize variables for drone control
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  double past_x_global = 0, past_y_global = 0;
  double past_time = wb_robot_get_time();
  double height_desired = FLYING_ALTITUDE;

  // Initialize PID controller gains
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();  // Initialize PID controller

  motor_power_t motor_power;  // Struct for motor power control

  printf("\n====== Drone Controls ======\n");
  printf("Use keyboard to control:\n");
  printf("- Arrow keys for horizontal movement\n");
  printf("- W/S to increase/decrease altitude\n");
  printf("- Q/E to rotate around yaw axis\n");

  while (wb_robot_step(timestep) != -1) {
    const double dt = wb_robot_get_time() - past_time;

    // Get sensor measurements
    actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    actual_state.altitude = wb_gps_get_values(gps)[2];
    double x_global = wb_gps_get_values(gps)[0];
    double vx_global = (x_global - past_x_global) / dt;
    double y_global = wb_gps_get_values(gps)[1];
    double vy_global = (y_global - past_y_global) / dt;

    // Convert to body-fixed velocities
    double actual_yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actual_yaw);
    double sinyaw = sin(actual_yaw);
    actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

    // Initialize desired states
    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = 0;
    desired_state.vy = 0;
    desired_state.yaw_rate = 0;
    desired_state.altitude = height_desired;

    double forward_desired = 0;
    double sideways_desired = 0;
    double yaw_desired = 0;
    double height_diff_desired = 0;

    // Manual control through keyboard
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          forward_desired = +0.5;
          break;
        case WB_KEYBOARD_DOWN:
          forward_desired = -0.5;
          break;
        case WB_KEYBOARD_RIGHT:
          sideways_desired = -0.5;
          break;
        case WB_KEYBOARD_LEFT:
          sideways_desired = +0.5;
          break;
        case 'Q':
          yaw_desired = 1.0;
          break;
        case 'E':
          yaw_desired = -1.0;
          break;
        case 'W':
          height_diff_desired = 0.1;
          break;
        case 'S':
          height_diff_desired = -0.1;
          break;
      }
      key = wb_keyboard_get_key();
    }

    height_desired += height_diff_desired * dt;  // Update altitude based on input

    // Example sensor data usage (e.g., obstacle avoidance):
    // double range_front_value = wb_distance_sensor_get_value(range_front);
    // const unsigned char *image = wb_camera_get_image(camera);

    // Set desired state for PID control
    desired_state.yaw_rate = yaw_desired;
    desired_state.vy = sideways_desired;
    desired_state.vx = forward_desired;
    desired_state.altitude = height_desired;

    // Call PID controller
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // Set motor velocities based on PID output
    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    // Save past time for the next iteration
    past_time = wb_robot_get_time();
    past_x_global = x_global;
    past_y_global = y_global;
  }

  // Clean up the robot simulation
  wb_robot_cleanup();

  return 0;
}
