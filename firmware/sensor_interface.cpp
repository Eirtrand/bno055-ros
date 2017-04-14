#include <ros.h>
#include <string.h>
#include <Arduino.h>
#include "MS5837.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <math.h>






#define STANDARD_GRAVITY 9.08665      // [m/s^2]
#define RAD_PER_DEG 0.01745329252     // [1/deg]
#define PASCAL_PER_MILLIBAR 0.01      // [Pa/mbar]
#define MICROTESLA_PER_TESLA 0.000001 // [uT/T]

int id = -1;

ros::NodeHandle nh;

sensor_msgs::Imu imu_raw_msg;
geometry_msgs::Vector3 euler_msg;
sensor_msgs::Imu imu_quaterions;


ros::Publisher pub_euler("imu/euler", &euler_msg);
ros::Publisher pub_imu ("imu/raw", &imu_raw_msg);
ros::Publisher imu_publisher("bno055",&imu_quaterions);


// float DBG_AXIS_SIGN_VALUE = -1.0;

const int SensorReadDelay = 40;
unsigned long PrevoiusSensorReadMillis = 0;

int dbg_count = 0;

MS5837 barometer;
Adafruit_BNO055 bno055 = Adafruit_BNO055(55);

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);


// double GyroLsbSens, AccelLsbSens;
void setupIMU();
void getRawIMU();

void setup() {

    nh.initNode();

    nh.advertise(pub_imu);
    nh.advertise(pub_euler);
    nh.advertise(imu_publisher);

    Wire.begin();
    setupIMU();
}

void setupIMU(){ /*
                  Sets opmode to NDOF by default

                  From Adafruit_BNO055.h:

                  OPERATION_MODE_CONFIG                                   = 0X00,
                  OPERATION_MODE_ACCONLY                                  = 0X01,
                  OPERATION_MODE_MAGONLY                                  = 0X02,
                  OPERATION_MODE_GYRONLY                                  = 0X03,
                  OPERATION_MODE_ACCMAG                                   = 0X04,
                  OPERATION_MODE_ACCGYRO                                  = 0X05,
                  OPERATION_MODE_MAGGYRO                                  = 0X06,
                  OPERATION_MODE_AMG                                      = 0X07,
                  OPERATION_MODE_IMUPLUS                                  = 0X08,
                  OPERATION_MODE_COMPASS                                  = 0X09,
                  OPERATION_MODE_M4G                                      = 0X0A,
                  OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
                  OPERATION_MODE_NDOF                                     = 0X0C


                  ACCEL   MAG     GYRO    RELATIVE        ABSOLUTE
                  ORIENTATION     ORIENTATION
                  ---------------------------------------------------------------
                  IMU     |    X    |   -    |   X  |       X        |     -
                  COMPASS |    X    |   X    |   -  |       -        |     X
                  M4G     |    X    |   X    |   -  |       X        |     -
                  NDOF    |    X    |   X    |   X  |       -        |     X
                */

    id = bno055.begin(bno055.OPERATION_MODE_CONFIG);
    bno055.setAxisConfig(bno055.REMAP_CONFIG_P6);
    bno055.setAxisSign(bno055.REMAP_SIGN_P6);
    bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);
}



void getRawIMU() {

    // sensors_event_t event;
    // bno055.getEvent(&event);

    imu::Vector<3> gyro = bno055.getVector(bno055.VECTOR_GYROSCOPE);
    imu_raw_msg.linear_acceleration.x = gyro[0];
    imu_raw_msg.linear_acceleration.y = gyro[1];
    imu_raw_msg.linear_acceleration.z = gyro[2];


    imu::Vector<3> lin_acc = bno055.getVector(bno055.VECTOR_LINEARACCEL);
    imu_raw_msg.angular_velocity.x = lin_acc[0];
    imu_raw_msg.angular_velocity.y = lin_acc[1];
    imu_raw_msg.angular_velocity.z = lin_acc[2];


    imu::Quaternion qs = bno055.getQuat();
    imu_raw_msg.orientation.x = qs.x();
    imu_raw_msg.orientation.y = qs.y();
    imu_raw_msg.orientation.z = qs.z();
    imu_raw_msg.orientation.w = qs.w();

    imu_raw_msg.header.stamp = ros::Time(millis()/1000.0, 0);
    pub_imu.publish(&imu_raw_msg);

}

bool calibrated = false;

void e2q(float roll, float pitch, float yaw, sensor_msgs::Imu *y)
{  
    double cosPhi_2   = cos(roll / 2.0);
    double sinPhi_2   = sin(roll / 2.0);
    double cosTheta_2 = cos(pitch / 2.0);
    double sinTheta_2 = sin(pitch / 2.0);
    double cosPsi_2   = cos(yaw / 2.0);
    double sinPsi_2   = sin(yaw / 2.0);

    y->orientation.x = (cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2) * -1;
    y->orientation.y = (sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2) * -1;
    y->orientation.z = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
    y->orientation.w = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
 
    return;
}


void getEuler()
{
    imu::Vector<3> euler = bno055.getVector(bno055.VECTOR_EULER);
    imu::Vector<3> lineacc =bno055.getVector(bno055.VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno055.getVector(bno055.VECTOR_GYROSCOPE);

    euler_msg.x = euler[0];
    euler_msg.y = euler[1];
    euler_msg.z = euler[2];
    imu::Quaternion quat = bno055.getQuat();
    
    e2q(euler_msg.x, euler_msg.y, euler_msg.z, &imu_quaterions);
/*
    imu_quaterions.orientation.x = quat[0];
    imu_quaterions.orientation.x = quat[1];
    imu_quaterions.orientation.x = quat[2];
    imu_quaterions.orientation.x = quat[0];
   */ 
    imu_quaterions.angular_velocity.x = gyro[0];
    imu_quaterions.angular_velocity.y = gyro[1];
    imu_quaterions.angular_velocity.z = gyro[2];

    imu_quaterions.linear_acceleration.x = lineacc[0];
    imu_quaterions.linear_acceleration.y = lineacc[1];
    imu_quaterions.linear_acceleration.z = lineacc[2];
    
    imu_quaterions.header.frame_id = "AUV";
    pub_euler.publish(&euler_msg);
    imu_publisher.publish(&imu_quaterions);
}


void loop(){
    nh.spinOnce();

    if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {

        PrevoiusSensorReadMillis = millis();

        getRawIMU();
        nh.spinOnce();

        getEuler();
        nh.spinOnce();

    }
}
