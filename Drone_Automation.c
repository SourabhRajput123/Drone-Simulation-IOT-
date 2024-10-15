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
#include "pid_controller.h"

#define FLYING_ALTITUDE 1.0
#define NUM_WAYPOINTS 4
#define LANDING_ALTITUDE 0.1
#define THRESHOLD 0.5 // Adjusted threshold for reaching waypoint
#define MAX_MOTOR_VELOCITY 600.0 // Crazyflie motor maximum velocity

typedef struct {
    double x;
    double y;
    double z;
} waypoint_t;

waypoint_t waypoints[NUM_WAYPOINTS] = {
    {1.0, 1.0, FLYING_ALTITUDE},
    {1.0, -1.0, FLYING_ALTITUDE},
    {-1.0, -1.0, FLYING_ALTITUDE},
    {-1.0, 1.0, FLYING_ALTITUDE}
};

// Declare variables for state and control
actual_state_t actual_state; // Ensure actual_state_t is defined
desired_state_t desired_state; // Ensure desired_state_t is defined
gains_pid_t gains_pid; // Ensure gains_pid_t is defined
motor_power_t motor_power; // Ensure motor_power_t is defined

double past_time = 0.0;

gains_pid_t init_pid_controller() {
    gains_pid_t gains;
    gains.kp_z = 1.0; // Proportional gain for altitude control
    gains.ki_z = 0.1; // Integral gain for altitude control
    gains.kd_z = 0.5; // Derivative gain for altitude control
    return gains;
}

void read_sensors(WbDeviceTag gps, WbDeviceTag distance_sensor) {
    const double *gps_values = wb_gps_get_values(gps);
    actual_state.vx = gps_values[0]; // Read x position
    actual_state.vy = gps_values[1]; // Read y position
    
    // Ensure valid sensor readings
    if (distance_sensor != -1) {
        actual_state.altitude = wb_distance_sensor_get_value(distance_sensor); // Read altitude
        if (isnan(actual_state.altitude) || actual_state.altitude > 10.0) { // Check if the altitude is too high or invalid
            actual_state.altitude = FLYING_ALTITUDE; // Default to flying altitude if sensor fails or incorrect
        }
    } else {
        printf("Error: Distance sensor is not initialized or configured properly.\n");
    }
    printf("Position: (%.2f, %.2f), Altitude: %.2f\n", actual_state.vx, actual_state.vy, actual_state.altitude);
}

void limit_motor_speed(motor_power_t *motor_power) {
    // Cap motor velocities at MAX_MOTOR_VELOCITY
    if (motor_power->m1 > MAX_MOTOR_VELOCITY) motor_power->m1 = MAX_MOTOR_VELOCITY;
    if (motor_power->m2 > MAX_MOTOR_VELOCITY) motor_power->m2 = MAX_MOTOR_VELOCITY;
    if (motor_power->m3 > MAX_MOTOR_VELOCITY) motor_power->m3 = MAX_MOTOR_VELOCITY;
    if (motor_power->m4 > MAX_MOTOR_VELOCITY) motor_power->m4 = MAX_MOTOR_VELOCITY;

    if (motor_power->m1 < 0) motor_power->m1 = 0;
    if (motor_power->m2 < 0) motor_power->m2 = 0;
    if (motor_power->m3 < 0) motor_power->m3 = 0;
    if (motor_power->m4 < 0) motor_power->m4 = 0;
}

int main(int argc, char **argv) {
    wb_robot_init();
    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors and sensors
    WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
    WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
    WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
    WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
    WbDeviceTag gps = wb_robot_get_device("gps");
    WbDeviceTag distance_sensor = wb_robot_get_device("distance_sensor");

    // Ensure sensors are enabled
    wb_gps_enable(gps, timestep);
    wb_distance_sensor_enable(distance_sensor, timestep);

    // Initialize the PID controller gains
    gains_pid = init_pid_controller();

    // Initialize variables for path following
    int currentWaypoint = 0;

    while (wb_robot_step(timestep) != -1) {
        // Read the actual state from sensors
        read_sensors(gps, distance_sensor);

        // Check if the drone has reached the current waypoint
        double distance_to_waypoint = sqrt(pow(actual_state.vx - waypoints[currentWaypoint].x, 2) +
                                           pow(actual_state.vy - waypoints[currentWaypoint].y, 2));

        if (distance_to_waypoint < THRESHOLD) {  // Threshold for reaching waypoint
            currentWaypoint++;
            if (currentWaypoint >= NUM_WAYPOINTS) {
                // If last waypoint reached, return to starting point
                currentWaypoint = 0;  // Go back to the first waypoint
            }
        }

        // Control logic to navigate to the current waypoint
        desired_state.altitude = FLYING_ALTITUDE;  // Maintain flying altitude
        desired_state.vx = (waypoints[currentWaypoint].x - actual_state.vx) * 0.5;  // Adjust for your PID controller
        desired_state.vy = (waypoints[currentWaypoint].y - actual_state.vy) * 0.5;

        // If returning to the starting point, start descending
        if (currentWaypoint == 0 && distance_to_waypoint < THRESHOLD) {
            desired_state.altitude -= 0.01;  // Descend for landing
            if (desired_state.altitude < LANDING_ALTITUDE) {
                desired_state.altitude = LANDING_ALTITUDE;  // Ensure it doesn't go below landing altitude
            }
        }

        // Calculate dt as the time step
        double dt = timestep / 1000.0; // Use timestep directly for consistency
        
        // Update motor speeds using PID controller
        pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);
        
        // Limit motor speeds to max allowable values
        limit_motor_speed(&motor_power);

        // Set motor velocities
        wb_motor_set_velocity(m1_motor, motor_power.m1);
        wb_motor_set_velocity(m2_motor, motor_power.m2);
        wb_motor_set_velocity(m3_motor, motor_power.m3);
        wb_motor_set_velocity(m4_motor, motor_power.m4);

        // Debugging: Print motor power values
        printf("Motor Powers: M1: %f, M2: %f, M3: %f, M4: %f\n", motor_power.m1, motor_power.m2, motor_power.m3, motor_power.m4);
    }

    wb_robot_cleanup();
    return 0;
}
