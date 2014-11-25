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

#define ENABLE_GPS 0
#define ENABLE_COMPASS 0

// enum for ESC CALIBRATION
enum ESCCalibrationModes {
    ESCCAL_NONE = 0,
    ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
    ESCCAL_PASSTHROUGH_ALWAYS = 2,
    ESCCAL_AUTO = 3
};

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// setup
void setup()
{
    // PID Configuration
    pids[PID_PITCH_RATE].kP(0.7);
    pids[PID_PITCH_RATE].kI(1);
    pids[PID_PITCH_RATE].imax(50);
    
    pids[PID_ROLL_RATE].kP(0.7);
    pids[PID_ROLL_RATE].kI(1);
    pids[PID_ROLL_RATE].imax(50);
    
    pids[PID_YAW_RATE].kP(2.7);
    pids[PID_YAW_RATE].kI(1);
    pids[PID_YAW_RATE].imax(50);
    
    pids[PID_PITCH_STAB].kP(4.5);
    pids[PID_ROLL_STAB].kP(4.5);
    pids[PID_YAW_STAB].kP(10);
    
    hal.console->println("Intialzing Ardupilot");    
    initializeGyro();
    initializeEngines();
    openingUartPorts();
//    initializeCompass();
//    intializeGPS();
    hal.console->println("Intialzing Complete");
//
//    hal.uartC->println("this is a test line on UART C\n");
//    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
//    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true nort    
    
}

// loop
void loop()
{
    
    uartcRx_loop();   //read incoming data from UartC
//    update_compass(); //update compas
//    update_gps();     //update gps
}

void initializeGyro()
{
    hal.console->println("\n ------------- Intialzing Gyro ----------------");
    
    hal.console->println(" - stop parameter from blocking SPI bus");
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);

    hal.console->println(" - initialize inertial sensor");
    ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
    
    hal.console->println(""); //improves readabilty
    
    hal.console->println(" - wait for inertial sensor data");
    if (!ins.wait_for_sample(1000)) {
      hal.console->println(" - AP_InertialSensor_MPU6000 accel/gyro chip seems to have serious problems, no init data recieved");
    } else {
      hal.console->println(" - AP_InertialSensor_MPU6000 accel/gyro OK");
    }

    hal.console->println("\n ---------------------------------------------\n");
}

void initializeEngines()
{
    hal.console->println("\n ----------- Intialzing Engines --------------");
    
    int update_rate = 490;
    int minimal_throttle   = 88;
    int radio_min          = 1000;
    int radio_max          = 2000;
    
    // motor initialisation
    motors.set_update_rate(update_rate);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_min_throttle(minimal_throttle); //88 in minimum && 1100 +/- max
    motors.Init();      // initialise motors

    if (rc3.radio_min == 0) rc3.radio_min = radio_min;
    if (rc3.radio_max == 0) rc3.radio_max = radio_max;

    hal.console->printf(" - setting engine update rate: %d\n", update_rate);
    hal.console->printf(" - setting frame orientation: %d (0 = PLUS-FRAME | 1 = X-FRAME)\n", AP_MOTORS_X_FRAME);
    hal.console->printf(" - setting engine minimal throttle: %d\n", minimal_throttle);
    hal.console->printf(" - setting minimal radio value: %d\n", radio_min);
    hal.console->printf(" - setting maximal radio value: %d\n", radio_max);
    
    motors.enable();
    
    hal.console->println(" - engines enabled");
    motors.output_min();
    hal.console->println(" - engines set to minimal output");
    hal.console->println("\n ---------------------------------------------\n");
}

void openingUartPorts()
{
   long uartASpeed = 115200;
   long uartCSpeed = 115200;
   
   hal.console->println("\n ----------- Opening UART ports --------------");
   hal.console->printf(" - opening UART-A at %lu Baud\n",uartASpeed);
   hal.uartA->begin(uartASpeed);
   hal.console->printf(" - opening UART-C at %lu Baud\n",uartCSpeed);
   hal.uartC->begin(uartCSpeed);
   hal.console->println("\n ---------------------------------------------\n");
}

void initializeCompass()
{
   hal.console->println("\n ----------- Initialize Compass --------------");
   if (ENABLE_COMPASS == 1) {
      hal.console->println(" - Compass library test");
      if (!compass.init()) {
        hal.console->println(" - compass initialisation failed!");
        while (1) ;
      }
   } else {
     hal.console->println(" - compass is not enabled");
   }

   hal.console->println("\n ---------------------------------------------\n");
}

void intializeGPS()
{
   hal.console->println("\n ------------- Initialize GPS ----------------");
   
   if (ENABLE_GPS == 1) {
     hal.uartB->begin(38400);
     gps.print_errors = true;

     hal.console->println(" - GPS UBLOX library test");
     gps.init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);       // GPS Initialization
   } else {
     hal.console->println(" - GPS is not enabled");
   }

   hal.console->println("\n ---------------------------------------------\n");
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
//        hal.console->printf("FULL Message recieved: %s\n", buffer);
        
        char command[4];
        memcpy( command, &buffer[0], 3 );
        command[3] = '\0';
//        hal.console->printf("BUFFER: %s\n", command);
        
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
          
          
//          hal.console->printf("Throttle: %i \tYAW: %i \tPITCH: %i \tROLL: %i\n", throttle, yaw, pitch, roll);
          
          //update_throttle(throttle);
          update_movement(throttle, yaw, pitch, roll);
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
  static float yaw_target = 0;
  
  // Ask MPU6050 for orientation
  if (rcThrottle < 1170) {
//    hal.console->printf("Throttle is below minimum!! Returning! \n");
    hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[2]), rcThrottle); // Front left
    hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[1]), rcThrottle); // back left
    hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[0]), rcThrottle); // Front right
    hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[3]), rcThrottle); // back right
    return;
  }
  
  hal.console->printf("thr: %i \t yaw: %i \t pitch: %i \t roll: %i\n", rcThrottle, rcYaw, rcPitch, rcRoll);
  
  Quaternion q;
  
  float roll,pitch,yaw;  
  ins.update();
  //ins.quaternion.to_euler(&roll, &pitch, &yaw); //<-- WTF
  
  Vector3f gyro     = ins.get_gyro();
  q.to_euler(&roll, &pitch, &yaw);
  
  roll  = ToDeg(gyro.x) ;
  pitch = ToDeg(gyro.y) ;
  yaw   = ToDeg(gyro.z) ;
  
  hal.console->printf("yaw: %i \t pitch: %i \t roll: %i\n", yaw, pitch, roll);
  
  // Ask MPU6050 for gyro data
  float gyroPitch   = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  
  hal.console->printf("gyroYaw: %10.7f (%10.7f)\n", gyroYaw, gyro.z);
  hal.console->printf("gyroPitch: %10.7f (%10.7f)\n", gyroPitch, gyro.y);
  hal.console->printf("gyroRoll: %10.7f (%10.7f)\n", gyroRoll, gyro.x);
  
  
  // Stablise PIDS
  float pitch_stab_output = pids[PID_PITCH_STAB].get_pid((float)rcPitch - pitch, 1); 
  float roll_stab_output  = pids[PID_ROLL_STAB].get_pid((float)rcRoll - roll, 1);
  float yaw_stab_output   = pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1);
  
  // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
  if(abs(rcYaw ) > 5) {
    yaw_stab_output = rcYaw;
    yaw_target = yaw;   // remember this yaw for when pilot stops
  }
  
  // rate PIDS
  int pitch_output =  (int) pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1);  
  int roll_output  =  (int) pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1);  
  int yaw_output   =  (int) pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1);  
    
//  float pitch_output =   (float) pids[PID_PITCH_RATE].get_pid(gyroPitch - (float) rcPitch, 1);  
//  float roll_output  =   (float) pids[PID_ROLL_RATE].get_pid(gyroRoll - (float) rcRoll, 1);  
//  float yaw_output   =   (float) pids[PID_YAW_RATE].get_pid(0 - (float) rcYaw, 1);  
  
  hal.console->printf("yaw_output: %i \n", yaw_output);
  hal.console->printf("pitch_output: %i \n", pitch_output);
  hal.console->printf("roll_output: %i \n", roll_output);
  
  int throttleFL    = rcThrottle - roll_output - pitch_output;
  int throttleBL    = rcThrottle - roll_output + pitch_output;
  int throttleFR    = rcThrottle + roll_output - pitch_output;
  int throttleBR    = rcThrottle + roll_output + pitch_output;

  hal.console->printf("thr: %i - %i - %i = %i\n", rcThrottle, roll_output, pitch_output, throttleFL);
  hal.console->printf("thr: %i - %i + %i = %i\n", rcThrottle, roll_output, pitch_output, throttleBL);
  hal.console->printf("thr: %i + %i - %i = %i\n", rcThrottle, roll_output, pitch_output, throttleFR);
  hal.console->printf("thr: %i + %i + %i = %i\n", rcThrottle, roll_output, pitch_output, throttleBR);
  
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[2]), throttleFL); // Front left
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[1]), throttleBL); // back left
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[0]), throttleFR); // Front right
  hal.rcout->write(pgm_read_byte(&motors._motor_to_channel_map[3]), throttleBR); // back right
}

void update_compass()
{
  if (ENABLE_COMPASS == 0) {
      return;
  }
    
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
    hal.uartA->printf("cps:%.1f\n",heading);
}

void update_gps()
{   
    if (ENABLE_GPS == 0) {
      return;
    }
    
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
        hal.uartA->printf("gps:%.6f %.6f\n",lat, longt);
    }
}

AP_HAL_MAIN();


