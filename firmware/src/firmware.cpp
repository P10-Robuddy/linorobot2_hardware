// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "time.h"
#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
    }
}

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU bno;

unsigned long long next_run_time = 0;

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 500)) 
    {
        fullStop();
        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );
    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

#define CONTROL_INTERVAL 20  // ms

struct robot_measurement_packet {
  float imu_yaw_rate;
  float wheel_lin_vel;
  float wheel_angular_vel;
  float odom_x_pos;
  float odom_y_pos;
  float odom_yaw;
  float cmd_vel_x;
  float cmd_vel_z;
};

void send_packet(robot_measurement_packet* packet) {
  // protocol:
  // header, 3x floats of 4 bytes,2 CRC bytes, end of message
  // [0xFF][float][float][float][CRC16][0xFE]

  size_t SERIAL_BUFFER_LENGTH = 36;
  uint8_t buffer[SERIAL_BUFFER_LENGTH];
  buffer[0] = 0xFF;  // start of msg
  const int FLOAT_BYTES = 4;
  memcpy(&buffer[0*FLOAT_BYTES + 1], &packet->imu_yaw_rate, 4);
  memcpy(&buffer[1*FLOAT_BYTES + 1], &packet->wheel_lin_vel, 4);
  memcpy(&buffer[2*FLOAT_BYTES + 1], &packet->wheel_angular_vel, 4);
  memcpy(&buffer[3*FLOAT_BYTES + 1], &packet->odom_x_pos, 4);
  memcpy(&buffer[4*FLOAT_BYTES + 1], &packet->odom_y_pos, 4);
  memcpy(&buffer[5*FLOAT_BYTES + 1], &packet->odom_yaw, 4);
  memcpy(&buffer[6*FLOAT_BYTES + 1], &packet->cmd_vel_x, 4);
  memcpy(&buffer[7*FLOAT_BYTES + 1], &packet->cmd_vel_z, 4);

  buffer[SERIAL_BUFFER_LENGTH-3] = 0x02;  // crc high byte
  buffer[SERIAL_BUFFER_LENGTH-2] = 0x01;  // crc low byte
  buffer[SERIAL_BUFFER_LENGTH-1] = 0x00;  // end of message

  Serial.write(buffer, SERIAL_BUFFER_LENGTH);
}

void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    flashLED(3);
    bool imu_ok = bno.init();
    if(!imu_ok)
    {
        while(1)
        {
            flashLED(3);
        }
    }
    fullStop();
    while(!Serial)
    {
        flashLED(1);
    }
    next_run_time = millis();
}

const unsigned int MAX_BUFFER_SIZE = 512;
const int INCOMING_PACKET_SIZE = 12;
char buffer[MAX_BUFFER_SIZE];
char* buffer_pos = buffer;
uint8_t buffer_size = 0;
void loop() {
    // check for incoming data
    float cmd_lin_x, cmd_ang_z;
    if(Serial.available())
    {
        int readable_bytes = Serial.available();
        if(readable_bytes > MAX_BUFFER_SIZE-buffer_size)
        {
            readable_bytes = MAX_BUFFER_SIZE-buffer_size;
        }
        Serial.readBytes(buffer_pos+buffer_size, readable_bytes);
        buffer_size += readable_bytes;

        while(buffer_pos+buffer_size - buffer >= INCOMING_PACKET_SIZE)
        {
            char* current_char = buffer_pos;
            if(*current_char == 0xFF) // start of message
            {
                //Serial.println("we should have a message");
                if(*(current_char+INCOMING_PACKET_SIZE-3) == 2 && *(current_char+INCOMING_PACKET_SIZE-2) == 1 && *(current_char+INCOMING_PACKET_SIZE-1) == 0)
                {
                    memcpy(&cmd_lin_x, current_char+1, 4); // cmd.linear.x
                    memcpy(&cmd_ang_z, current_char+5, 4); // cmd.angular.z

                    // store the result
                    twist_msg.linear.x = cmd_lin_x;
                    twist_msg.angular.z = cmd_ang_z;
                    prev_cmd_time = millis();

                    // move the buffer back
                    unsigned int current_pos = buffer_pos - buffer;
                    for(unsigned int i = current_pos; i < MAX_BUFFER_SIZE; i++)
                    {
                        buffer[i-current_pos] = buffer[i];
                    }
                    buffer_pos = buffer;
                    buffer_size -= INCOMING_PACKET_SIZE;
                    break;
                }
            }
            buffer_pos++;
            buffer_size--;
        }
    }

    // check if time to send data
    unsigned long long now = millis();
    if (now > next_run_time) {
        if (now - next_run_time > CONTROL_INTERVAL * 10) {
            digitalWrite(LED_PIN, HIGH); // turn on if schedule is missed
        }
        next_run_time += CONTROL_INTERVAL;

        moveBase();

        // send data
        odom_msg = odometry.getData();
        imu_msg = bno.getData();
        robot_measurement_packet packet;
        packet.imu_yaw_rate = imu_msg.angular_velocity.z;
        packet.wheel_lin_vel = odom_msg.twist.twist.linear.x;
        packet.wheel_angular_vel = odom_msg.twist.twist.angular.z;
        packet.odom_x_pos = odom_msg.pose.pose.position.x;
        packet.odom_y_pos = odom_msg.pose.pose.position.y;
        packet.odom_yaw = odometry.get_heading();
        packet.cmd_vel_x = twist_msg.linear.x;
        packet.cmd_vel_z = twist_msg.angular.z;
        send_packet(&packet);
    }
}