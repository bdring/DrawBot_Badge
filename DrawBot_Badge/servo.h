/*
  servo.h - rs274/ngc parser.
  Part of Grbl
      
	copyright (c) 2018 -	Bart Dring This file was modified for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/



/*
Tower pro specs
min pulse = 0.008 seconds
max pulse = 0.220 seconds
total sweep angle = 150° ... 2.62 radians
*/

// ---- begin servo definition  ------
#define SERVO_MIN_RADIANS  0.0
#define SERVO_MAX_RADIANS  2.62 // 150°
#define SERVO_MIN_PULSE_MS 0.0008 // min pulse in seconds
#define SERVO_MAX_PULSE_MS 0.0022 // max pulse in seconds
// ---- begin servo definition  ------

#define SERVO_TIMER_NUM 1
#define SERVO_TIMER_INT_FREQ 50 // Hz This is the task freq

#define SERVO_A_CHANNEL_NUM 5
#define SERVO_B_CHANNEL_NUM 6
#define SERVO_C_CHANNEL_NUM 7

#ifdef CPU_MAP_WORKSHOP_PCB
	#define SERVO_A_PIN 		GPIO_NUM_12
	#define SERVO_B_PIN 		GPIO_NUM_27
	#define SERVO_C_PIN 		GPIO_NUM_14
#endif

#ifdef CPU_MAP_SIMPLE_PCB
	#define SERVO_A_PIN 		GPIO_NUM_16
	#define SERVO_B_PIN 		GPIO_NUM_5
	#define SERVO_C_PIN 		GPIO_NUM_19
#endif

#define SERVO_PULSE_FREQ 50 // 50Hz ... this is a standard analog servo value
#define SERVO_PULSE_RES_BITS 16 // bits of resolution of PWM (16 is max)
#define SERVO_PULSE_RES_COUNT 65535 // see above

#define SERVO_TIME_PER_BIT  ((1.0 / (float)SERVO_PULSE_FREQ) / ((float)SERVO_PULSE_RES_COUNT) ) // seconds

#define SERVO_MIN_PULSE    (uint16_t)(SERVO_MIN_PULSE_MS / SERVO_TIME_PER_BIT) // 
#define SERVO_MAX_PULSE    (uint16_t)(SERVO_MAX_PULSE_MS / SERVO_TIME_PER_BIT) 
#define SERVO_PULSE_RANGE  (SERVO_MAX_PULSE - SERVO_MIN_PULSE)   

// the servo are mounted in the same orientation, but the arms are not to optimize the travel.
// The A servo arm is mounted so the center of servo travel is 45° down from machine zero
// The B servo arm is mounted so the center of servo travel is 45° up from machine zero

// calculate the pulse duration of the zero (pointing right) location
// center of A is 45° above zero
#define SERVO_A_ZERO_ANG  (SERVO_MAX_RADIANS/2.0 + PI/4.0)  // referenced to servo
#define SERVO_A_ZERO_PT    (uint16_t)(((float)SERVO_PULSE_RANGE * ((float)SERVO_A_ZERO_ANG/(float)SERVO_MAX_RADIANS)) + SERVO_MIN_PULSE)
#define SERVO_A_RAD_CNT    (SERVO_PULSE_RANGE / SERVO_MAX_RADIANS)

// center of B is 45° below zero
#define SERVO_B_ZERO_ANG  (SERVO_MAX_RADIANS/2.0 - PI/4.0) // referenced to servo
#define SERVO_B_ZERO_PT (uint16_t)(((float)SERVO_PULSE_RANGE * (SERVO_B_ZERO_ANG/SERVO_MAX_RADIANS)) + SERVO_MIN_PULSE)
#define SERVO_B_RAD_CNT    (SERVO_PULSE_RANGE / SERVO_MAX_RADIANS)	 						// PWM counts per radian	 

// servo C, the Z, is much simpler. We only need to define the range we want use
#define SERVO_C_RADIANS_MIN SERVO_MAX_RADIANS - PI/2 // 90° of travel
#define SERVO_C_RADIANS_MAX SERVO_MAX_RADIANS

#define SERVO_C_RANGE_MM 3.0 // mm (full range of z)

// Kinematic Constants
// work area
#define MIN_WORK_X -30.0 // outside work area, but useful for calibration
#define MAX_WORK_X 75.0
#define MIN_WORK_Y 0.0  
#define MAX_WORK_Y 75.0
// arm lengths
#define LEN_PEN_FOREARM 	55.0
#define L_PEN_UPPERARM 		30.0
#define LEN_LOWER_CRANK 	50.0
#define LEN_UPPER_CRANK 	30.0
#define LEN_MID_LINK 			60.0
// static positions
#define SERVO_A_X	24.0
#define SERVO_A_Y 91.1
#define SERVO_B_X SERVO_A_X // both have same x position
#define SERVO_B_Y 103.1

#define SERVO_CAL_MIN 50.0 // the minimum allowable calibration value
#define SERVO_CAL_MAX 150.0 // the maximum allowable calibration value

#define BADGE_STARTUP_LINE "G0X58.3Y38.2Z0.0" // used to locate arms for assembly alignment

#define BADGE_MODE     (bit_istrue(settings.flags, BITFLAG_INVERT_PROBE_PIN))

#ifndef servo_h
#define servo_h

static TaskHandle_t servoSyncTaskHandle = 0;

void servo_init();

void servoSyncTask(void *pvParameters);

void calc_servo_cosines(float penX, float penY);
void calc_servo_intersect(float penX, float penY, float penZ);
uint8_t servo_constrain_cal_range(uint8_t cal_vsl);
bool validate_bagde_settings();
void badge_servos_disable();

int circle_circle_intersection(float x0, float y0, float r0,  // x,y and radius 
                               float x1, float y1, float r1,
                               float *xi, float *yi, // intersection
                               float *xi_prime, float *yi_prime);
															 


#endif