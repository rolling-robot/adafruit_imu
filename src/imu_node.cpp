#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"


#include <sstream>

#ifndef IMU_HPP_
#define IMU_HPP_

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "../IMU/L3G.h"
#include "../IMU/LSM303.h"
#include "../IMU/IMU.h"
#include "../IMU/Compass.h"
#include "../IMU/DCM.h"
#include "../sys/Timer.h"

#endif

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

ros::Time timer(0);   //general purpuse timer
ros::Time timer_old(0);
//long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

bool imu_on = false;

ros::Publisher imu_pub;




void setup_IMU()
{

	ROS_INFO ("Start initialization IMU");
	IMU_Init();
	usleep (20000);
	for(int i=0;i<32;i++)    // We take some readings...
    {
		Read_Gyro();
		Read_Accel();
		for(int y=0; y<6; y++)   // Cumulate values
			AN_OFFSET[y] += AN[y];
		usleep (20000);
    }

	for(int y=0; y<6; y++)
		AN_OFFSET[y] = AN_OFFSET[y]/32;

	AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

	timer=ros::Time::now();
	usleep (20000);
	counter=0;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::Time::init();
  ros::NodeHandle node;
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu", 1000);
  ros::Publisher mag_pub = node.advertise<sensor_msgs::MagneticField>("mag", 1000);
  
  ros::Rate loop_rate(45);
  ROS_INFO("Starting imu advertising, %d", imu_on);
  int counter = 0;
  setup_IMU();
  imu_on = true;
  while (ros::ok())
  {

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    if (imu_on){
      Read_Gyro();
      Read_Accel();

      imu_msg.header.stamp = ros::Time::now();

      imu_msg.linear_acceleration.x = ToSi(accel_x); //(float)accel_x / GRAVITY * FREE_ACC;
      imu_msg.linear_acceleration.y = ToSi(accel_y); //(float)accel_y / GRAVITY * FREE_ACC;
      imu_msg.linear_acceleration.z = ToSi(accel_z); //(float)accel_z / GRAVITY * FREE_ACC;

      imu_msg.angular_velocity.x = Gyro_Scaled_X(gyro_x);
      imu_msg.angular_velocity.y = Gyro_Scaled_Y(gyro_y);
      imu_msg.angular_velocity.z = Gyro_Scaled_Z(gyro_z);

      imu_pub.publish(imu_msg);

      if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
	{
	  counter=0;
	  Read_Compass();    // Read I2C magnetometer
	  Compass_Heading(); // Calculate magnetic heading

	  mag_msg.header.stamp = ros::Time::now();
	  mag_msg.magnetic_field.x = magnetom_x;  
	  mag_msg.magnetic_field.y = magnetom_y;  
	  mag_msg.magnetic_field.z = magnetom_z;
	  mag_pub.publish(mag_msg);
	}
      //Matrix_update();
      //Normalize();
      //Drift_correction();
      //Euler_angles();
      //printf ("\033[1Aroll: %.2f \tpitch: %.2f  \tyaw: %.2f\n", roll, pitch, yaw);
      

    }

    
    ros::spinOnce();

    loop_rate.sleep();
    ++counter;
  }


  return 0;
}
