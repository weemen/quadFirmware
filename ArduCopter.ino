/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */
#include <stdlib.h>
// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Scheduler.h>       // main loop scheduler
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library

#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_Mission.h>

#include <AC_PID.h>  
// PID library
#include <AC_P.h>               // P library
#include <AP_BoardLED.h>
#include <AP_NavEKF.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <PID.h>

// Local modules
// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

#include "Parameters.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

AP_Compass_HMC5843 compass;

AP_GPS_UBLOX gps;
#define T6 1000000
#define T7 10000000

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

AP_MotorsQuad   motors(rc1, rc2, rc3, rc4);

uint32_t timer;

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");
    
//    if (!ins.wait_for_sample(1000)) {
//      hal.console->println("whoops AP_InertialSensor_MPU6000 - accel/gyro chip");
//    }


//    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
//    motors.set_min_throttle(88); //88 in minimum && 1100 +/- max
    motors.Init();      // initialise motors
//
    if (rc3.radio_min == 0) rc3.radio_min = 1000;
    if (rc3.radio_max == 0) rc3.radio_max = 2000;

    motors.enable();
    motors.output_min();
//
//    hal.uartA->begin(115200);
    hal.uartC->begin(115200);
//    
//    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
//    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

    hal.console->println("Compass library test");
    
//    if (!compass.init()) {
//        hal.console->println("compass initialisation failed!");
//        while (1) ;
//    }
    
    hal.uartB->begin(38400);
    gps.print_errors = true;

//    hal.console->println("GPS UBLOX library test");
    gps.init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);       // GPS Initialization
}

// loop
void loop()
{
    
    uartcRx_loop();   //read incoming data from UartC
//    update_compass(); //update compas
    update_gps();     //update gps
}


static void uartcRx_loop()
{
    static int counter = 0;
    static char buffer[256];
    char *ptr;
    int value;

    // test motors
    while(hal.uartC->available()) {
      value = hal.uartC->read();
      buffer[counter] = value;
      counter = counter + 1;
      
      if(value == '\n') {
        buffer[counter] = '\0';
        hal.console->printf("FULL Message recieved: %s\n", buffer);
        
        char command[4];
        memcpy( command, &buffer[0], 3 );
        command[3] = '\0';
        hal.console->printf("BUFFER: %s\n", command);
        
        if (strcmp("thr",command) == 0) {
            
          //fetch parameters from buffer (4 characters from position 5 till 8)
          // substract 500 to get input value
          //throttle no substract always 4 positions
          //yaw    -500 (3 pos)
          //pitch  -500 (3 pos)
          //roll   -500 (3 pos)
          
          char paramThrottle[5];
          memcpy( paramThrottle, &buffer[4], 4);
          paramThrottle[4] = '\0';
          int throttle = atoi(paramThrottle);
          
          char paramYaw[4];
          memcpy( paramYaw, &buffer[8], 3);
          paramYaw[3] = '\0';
          int yaw = (atoi(paramYaw) - 500);
          
          char paramPitch[4];
          memcpy( paramPitch, &buffer[11], 3);
          paramPitch[3] = '\0';
          int pitch = (atoi(paramPitch) - 500);
          
          char paramRoll[4];
          memcpy( paramRoll, &buffer[14], 3);
          paramRoll[3] = '\0';
          int roll = (atoi(paramRoll) - 500);
          
          hal.console->printf("PARAM1:  %i\n", throttle);
          hal.console->printf("PARAM2:  %i\n", yaw);
          hal.console->printf("PARAM3:  %i\n", pitch);
          hal.console->printf("PARAM4:  %i\n", roll);
          
          update_throttle(throttle);
          //update_movement(throttle, yaw, pitch, roll)
        }
 
        buffer[0]  = '\0';
        counter    = 0;
        hal.uartC->flush();
      }
    }
}

void update_throttle(int throttle)
{
  //hal.console->printf("Throttle will be updated to value %d\n", throttle);
  motors.armed(true);
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[0]), throttle);
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[1]), throttle);
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[2]), throttle);
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[3]), throttle);
  //motors.armed(false);
}

void update_movement(int rcThrottle, int rcYaw, int rcPitch, int rcRoll)
{
  // Ask MPU6050 for orientation
  if (rcThrottle < 1170) {
    return;
  }
  
  Quaternion q;
  
  float roll,pitch,yaw;  
  ins.update();
  //ins.quaternion.to_euler(&roll, &pitch, &yaw); //<-- WTF
  
  q.to_euler(&roll, &pitch, &yaw);
  
  roll  = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw   = ToDeg(yaw) ;
  
  // Ask MPU6050 for gyro data
  Vector3f gyro     = ins.get_gyro();
  float gyroPitch   = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  long pitch_output =   pids[PID_PITCH_RATE].get_pid(gyroPitch - rcPitch, 1);  
  long roll_output  =   pids[PID_ROLL_RATE].get_pid(gyroRoll - rcRoll, 1);  
  long yaw_output   =   pids[PID_YAW_RATE].get_pid(gyroYaw - rcYaw, 1);  
  
  int throttleFL    = rcThrottle - roll_output - pitch_output;
  int throttleBL    = rcThrottle - roll_output + pitch_output;
  int throttleFR    = rcThrottle + roll_output - pitch_output;
  int throttleBR    = rcThrottle + roll_output + pitch_output;

  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[2]), throttleFL); // Front left
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[1]), throttleBL); // back left
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[0]), throttleFR); // Front right
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[3]), throttleBR); // back right
}

void update_compass()
{
  static float min[3], max[3], offset[3];

    compass.accumulate();
    if((hal.scheduler->micros() - timer) < 1650000L) {
        return;
    }
    
    timer = hal.scheduler->micros();
    compass.read();
    float heading;

    if (!compass.healthy()) {
        hal.console->println("not healthy");
        return;
    }
	Matrix3f dcm_matrix;
	// use roll = 0, pitch = 0 for this example
	dcm_matrix.from_euler(0, 0, 0);
    heading = compass.calculate_heading(dcm_matrix);
    heading = ToDeg(heading);
    
    if (heading < 0) {
      heading = heading + 360;
    }
 
    // display heading
    hal.console->printf("Heading: %.2f\n", heading);
    hal.uartC->printf("cps:%.1f\n",heading);
}

void update_gps()
{   
    gps.update();

    if((hal.scheduler->micros() - timer) < 1600000L) {
      return;
    }
    
    float lat   = 0;
    float longt = 0;
    if (gps.new_data) {
        lat   = (float)gps.latitude / T7, BASE_DEC;
        longt = (float)gps.longitude / T7, BASE_DEC;
        gps.new_data = 0; // We have readed the data
//        hal.console->printf("gps:%.6f %.6f\n",lat, longt);
        hal.uartC->printf("gps:%.6f %.6f\n",lat, longt);
    }
}

AP_HAL_MAIN();


