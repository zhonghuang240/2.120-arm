/**********************************************************************************

  Arduino code to control a DC motor
  MIT Department of Mechanical Engineering, 2.004, 2.12
  Rev. 1.0 -- 08/03/2020 H.C.
  Rev. 2.0 -- 08/25/2020 H.C.
  Rev. 3.0 -- 03/31/2021 H.S.
  Rev. 4.0 -- 04/11/2021 Phillip Daniel
***********************************************************************************/
#include <Encoder.h>
#include "Arduino.h"
#include "helper.h"


SerialComm          serialComm;       // serial communication class
unsigned long       prevTime = 0;

// Inverse Kinematics (IK)
#define IK

// Manipulator dimensions and joint angle limits
const float l_1 = 0.1524; // link1 lenth (m)
const float l_2 = 0.1778; // link2 lenth (m)
const float q1_limit = 113.7*PI/180; // joint angle limit for q1 (rad)
const float q2_limit = 161*PI/180; // joint angle limit for q2 (rad)

// ================================================================
// ===               PID FEEDBACK CONTROLLER                    ===
// ================================================================

// Provide PID controller gains here

float kp_1 = 400.0;
float kd_1 = 40;
float ki_1 = 0.0;

float kp_2 = 400.0;
float kd_2 = 40;
float ki_2 = 0.0;

// ================================================================
// ===               SERIAL OUTPUT CONTROL                      ===
// ================================================================

// Switch between built-in Serial Plotter and Matlab streaming
// comment out both to disable serial output

#define PRINT_DATA

// ================================================================
// ===               DFROBOT MOTOR SHIELD DEFINITION            ===
// ================================================================

const int
// motor connected to M1
PWM_1   = 10,
DIR_1   = 12;
const int
// motor connected to M2
PWM_2   = 11,
DIR_2   = 13;

// ================================================================
// ===               ENCODER DEFINITION                         ===
// ================================================================

// The motor has a dual channel encoder and each has 120 counts per revolution
// The effective resolution is 480 CPR with quadrature decoding.
// Calibrate the conversion factor by hand.

float C2Rad =  1176*4/ (2 * PI); // TO DO: replace it with your conversion factor

Encoder Mot1(3,4);
Encoder Mot2(2,5);

// ================================================================
// ===               VARIABLE DEFINITION                        ===
// ================================================================

float q_1 = 0.0, pre_q_1 = 0.0; // Current and previous joint angles of the motor 1
float q_2 = 0.0, pre_q_2 = 0.0; // Current and previous joint angles of the motor 2
float error_1, sum_error_1 = 0.0, d_error_1 = 0.0, filt_d_error_1 = 0.0, error_pre_1;
float error_2, sum_error_2 = 0.0, d_error_2 = 0.0, filt_d_error_2 = 0.0, error_pre_2;

// Circular path information
float CircleCenterX = 0.15;
float CircleCenterY = 0.1;
float Radius = 0.05;

float alpha = 0.25,alpha_ref = 0.75;    // low pass filter constant
unsigned long timer;
double loop_time = 0.01;
float Pcontrol, Icontrol, Dcontrol;
float pwm;
float vc;
float set_point_1 = 0; // Set point (desired position) for the motor 1
float set_point_2 = 0; // Set point (desired position) for the motor 2
float x_e=l_1+l_2, y_e=0, x_e_meas, y_e_meas; // end-effector position
float q_1_ik, q_2_ik; // inverse kinematics solutions
int i = 0; // To generate waypoints of the circular path.

// Convert from pixels to inches
float objectRadiusPixel=9,objectDiameterPixel=objectRadiusPixel*2, objectDiameterMeter=.04;
float scale=objectDiameterMeter/objectDiameterPixel; // meters/pixel

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Serial.begin(115200);

  // configure motor shield M1 and M2 outputs
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);

  delay(10);    // delay 0.01 second
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  timer = micros();

// Read position command
  if (timer - prevTime >= SERIAL_PERIOD_MICROS) {
        serialComm.receiveSerialData();
        prevTime = timer; // update time
    } 
	
  // Joint position from the encoder counts

  q_1 = Mot1.read() / C2Rad;    // convert to radians
  q_2 = Mot2.read() / C2Rad;    // convert to radians

// Inverse Kinematics
#ifdef IK
    // TO DO
    // Implement the Inverse_K() function below to compute the inverse kinematics of the manipulator.
    // Use x_e and y_e to give end-effector position.
    // Try to compare the true position and the actual position of the end-effector.
    
    x_e_meas=(serialComm.x-100)*scale+l_1+l_2;
    y_e_meas=(-serialComm.y+62)*scale;

    // Filtered endpoint
    x_e=alpha_ref*x_e+(1-alpha_ref)*x_e_meas;
    y_e=alpha_ref*y_e+(1-alpha_ref)*y_e_meas;

    // No filter on serial read
    //x_e=x_e_meas;
    //y_e=y_e_meas;

    // Constant endpoint position
    //x_e=l_1+l_2;
    //y_e=0;

    Inverse_K();
#endif

  // ================================================================
  // ===                    CONTROLLER CODE                       ===
  // ================================================================

    // PID controller for motor 1
    error_1 = set_point_1 - q_1;
    d_error_1 = (error_1 - error_pre_1) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_1 = alpha * d_error_1 + (1 - alpha) * filt_d_error_1;
    sum_error_1 += error_1 * loop_time;
    error_pre_1 = error_1;
    motorControl(DIR_1, PWM_1, error_1, filt_d_error_1, sum_error_1, kp_1, kd_1, ki_1);

    // PID controller for motor 2
    error_2 = set_point_2 - q_2;
    d_error_2 = (error_2 - error_pre_2) / loop_time;
    // 1st order filter to clean up noise
    filt_d_error_2 = alpha * d_error_2 + (1 - alpha) * filt_d_error_2;
    sum_error_2 += error_2 * loop_time;
    error_pre_2 = error_2;    
    motorControl(DIR_2, PWM_2, error_2, filt_d_error_2, sum_error_2, kp_2, kd_2, ki_2);

  // ================================================================
  // ===                    PRINT DATA                            ===
  // ================================================================


#ifdef PRINT_DATA
  Serial.print("x_e, y_e: ");
  Serial.print(x_e);  Serial.print(", ");
  Serial.print(y_e);  Serial.print("\t"); // print out x_e, y_e
  Serial.print("sp_1, q_1: ");
  Serial.print(set_point_1);  Serial.print(", "); 
  Serial.print(q_1); Serial.print("\t"); // set_point_1, q_1
  Serial.print("sp_2, q_2: ");
  Serial.print(set_point_2);  Serial.print(", ");
  Serial.print(q_2); Serial.print("\t"); // set_point_2, q_2  
  Serial.print("\n");
#endif
  
  delay(30);
  loop_time = (micros() - timer) / 1000000.0;  //compute actual sample time
}

// ================================================================
// ===                   Inverse Kinematics                     ===
// ================================================================
void Inverse_K()
{
  //TO DO
  //In the lab2, we have learned how to implement inverse kinematics of the 2 DOF manipulator.
  //You can use l_1, l_2 for the lenth of each link and x_e and y_e for the end-effector position.
  //You can use q_1_ik, and q_2_ik to store the IK solutions.
  q_2_ik = 2*atan2(sqrt(sq(l_1+l_2)-sq(x_e+y_e)),sqrt(sq(x_e+y_e)-sq(l_1-l_2)));
  q_1_ik = atan2(y_e,x_e) - atan2(l_2*sin(q_2_ik),l_1+l_2*cos(q_2_ik));
  
  // position limit constraints (update set_point_1 and set_point_2 when the solutions are within the limits)
  //if (-q2_limit< q_2_ik < q2_limit) set_point_2 = q_2_ik;
  //if (-q1_limit< q_1_ik < q1_limit) set_point_1 = q_1_ik;
	if(abs(q_1_ik)<q1_limit && abs(q_2_ik)<q2_limit)
	 {
		 set_point_1 = q_1_ik;
		 set_point_2 = q_2_ik;  
	 }

}

// ================================================================
// ===                   MOTOR CONTROLLER                       ===
// ================================================================

void motorControl(int DIR_x, int PWM_x, float error, float d_error, float sum_error, float kp_x, float kd_x, float ki_x)
{
    float pwm_command; 
  
    Pcontrol = error * kp_x;
    Icontrol = sum_error * ki_x;
    Dcontrol = d_error * kd_x; //Zero because the camera is too noisy
    
    Icontrol = constrain(Icontrol, -200, 200);  // I control saturation limits for anti-windup  
    
    pwm_command = Pcontrol + Icontrol + Dcontrol;
    
     if (pwm_command > 0)
     {  digitalWrite(DIR_x, LOW);
        analogWrite(PWM_x,(int)  constrain(pwm_command, 0, 255));
     }
     else
     {
        digitalWrite(DIR_x, HIGH);
        analogWrite(PWM_x, (int) constrain(abs(pwm_command), 0, 255));
     }
}
