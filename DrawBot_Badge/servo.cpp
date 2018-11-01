/*
  servo.cpp 
  Add On for Grbl
      
	copyright (c) 2018 -	Bart Dring					
					
	Do not use this with Grbl for atMega328P
  
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
	
	This is a stand alone add on for ESP32_Grbl
	The only things you need to add to the main program are...
	- add #include "servo.h" to grbl.h
	- put servo_init() in void setup()
	
	This uses a vTaskDelayUntil(...) method of repeating this at a relatively 
	fixed frequency.
	
	After much experimentation taskENTER_CRITICAL(&myMutex)
	appear to be the key to stability. Other tasks and Interrupts
	should not be able to fire until thee PWM functions are complete.
	
	There are two ways to calculate the kinematics. - currently Using Intersection Method
	- Law of cosines method: More deterministic, but half as fast.
	- Intersection of circles, faster, but need to make assumptions 
	  about which of the two intersections it finds  
*/

#include "grbl.h"


void servo_init()
{
	if (!BADGE_MODE) {
		return;
	}	
	
	grbl_send(CLIENT_SERIAL, "[MSG:Badge Mode]\r\n");
	
	/* debug stuff
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo time per bit: %1.10f]\r\n", SERVO_TIME_PER_BIT);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo min pulse ms: %1.10f]\r\n", SERVO_MIN_PULSE_MS);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo max pulse ms: %1.10f]\r\n", SERVO_MAX_PULSE_MS);
	
	
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo min pulse count: %d]\r\n", SERVO_MIN_PULSE);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo max pulse count: %d]\r\n", SERVO_MAX_PULSE);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo range count: %d]\r\n", SERVO_PULSE_RANGE);
	
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo A zero ang: %1.10f]\r\n", SERVO_A_ZERO_ANG);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo B zero ang: %1.10f]\r\n", SERVO_B_ZERO_ANG);
	
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo A zero: %d]\r\n", SERVO_A_ZERO_PT);
	grbl_sendf(CLIENT_SERIAL, "[MSG:Servo B zero: %d]\r\n", SERVO_B_ZERO_PT);	
	*/
	
	
	//Servo A ledcSetup
	ledcSetup(SERVO_A_CHANNEL_NUM, SERVO_PULSE_FREQ, SERVO_PULSE_RES_BITS);
	ledcAttachPin(SERVO_A_PIN, SERVO_A_CHANNEL_NUM);
	
	//Servo B ledcSetup
	ledcSetup(SERVO_B_CHANNEL_NUM, SERVO_PULSE_FREQ, SERVO_PULSE_RES_BITS);
	ledcAttachPin(SERVO_B_PIN, SERVO_B_CHANNEL_NUM);
	
	//Servo C ledcSetup
	ledcSetup(SERVO_C_CHANNEL_NUM, SERVO_PULSE_FREQ, SERVO_PULSE_RES_BITS);
	ledcAttachPin(SERVO_C_PIN, SERVO_C_CHANNEL_NUM);
	
	// setup a task that will calculate the kinematics and set the PWM	
	
	
	xTaskCreatePinnedToCore(	servoSyncTask,    // task
   													"servoSyncTask", // name for task
													4096,   // size of task stack
													NULL,   // parameters
													1, // priority
													&servoSyncTaskHandle,
													0 // core
													); 
													
}

// turn off the PWM (0 duty) to prevent servo jitter when not in use.
void badge_servos_disable()
{
	ledcWrite(SERVO_A_CHANNEL_NUM, 0);
	ledcWrite(SERVO_B_CHANNEL_NUM, 0);		
	ledcWrite(SERVO_C_CHANNEL_NUM, 0);
}

bool validate_bagde_settings() // max sure the settings are reasonable..otherwise reset the settings to default
{
	
	for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
		if ( (settings.steps_per_mm[axis] < SERVO_CAL_MIN) || (settings.steps_per_mm[axis] > SERVO_CAL_MAX) ) {
			grbl_sendf(CLIENT_SERIAL, "Steps/mm setting error:%f4.4\r\n", settings.steps_per_mm[X_AXIS]);
			return false;
		}	
	
		// Note: Max travel is set positive via $$, but stored as a negative number
		if ( (settings.max_travel[axis] < -SERVO_CAL_MAX) || (settings.max_travel[axis] > -SERVO_CAL_MIN) ) {
			grbl_sendf(CLIENT_SERIAL, "Max travel setting error:%f4.4\r\n", settings.max_travel[X_AXIS]);
			return false;
		}
	}
	
	return true; // setting OK
}


// this is the task
void servoSyncTask(void *pvParameters)
{	
	if (!BADGE_MODE) {
		return;
	}	
	
	int32_t current_position[N_AXIS];
	float m_pos[N_AXIS];
	TickType_t xLastWakeTime;
	const TickType_t xServoFrequency = SERVO_TIMER_INT_FREQ;  // in ticks (ms)
	
	while(true) {		
	    
			vTaskDelayUntil(&xLastWakeTime, xServoFrequency);
			
			if(!stepper_idle) //sys.state == STATE_CYCLE) {				
				{
				memcpy(current_position,sys_position,sizeof(sys_position));  // get current position in step	
				system_convert_array_steps_to_mpos(m_pos,current_position); // convert to millimeters
			
				calc_servo_intersect(m_pos[X_AXIS], m_pos[Y_AXIS], m_pos[Z_AXIS]); // calculate kinematics and move the servos			
			}
	}	
}


// law of cosines method
/*
void calc_servo_cosines(float penX, float penY)
{	
	uint16_t pulseLen;	
	
	float deltaPx, deltaPy;  // X and Y distances of pen from ServoA
	float D1; // absolute distance of pen from ServoA
	float a1, a2; //
	float angle_servo_a; // the angle of servo A arm relative to machine
	uint16_t servo_a_center = SERVO_A_ZERO_PT;
	uint16_t servo_a_pulse_len;
	float servo_a_ticks_per_radian = SERVO_A_RAD_CNT;	
	
	float a3, a4; //
	float mid_x, mid_y; // location of the mid link and pen arm bearing
	float D2; 					// absolute distance from pen/mid arm bearing from Servo b
	float a5, a6; // 
	float angle_servo_b; // the angle of servo B arm relative to machine	
	uint16_t servo_b_center = SERVO_B_ZERO_PT;
	uint16_t servo_b_pulse_len;
	float servo_b_ticks_per_radian = SERVO_B_RAD_CNT;
	
	float angle_servo_c;
 		
	// determine the absolute distance from pen to servo a
	deltaPx = (-SERVO_A_X) + penX;
	deltaPy = (-SERVO_A_Y) + penY;
	D1 = sqrt(deltaPx * deltaPx + deltaPy * deltaPy);
	
	a1 = atan2(deltaPy, deltaPx);
	a2 = acos( (D1 * D1 + LEN_LOWER_CRANK * LEN_LOWER_CRANK - LEN_PEN_FOREARM * LEN_PEN_FOREARM) / (2 * D1 * LEN_LOWER_CRANK) );
	angle_servo_a = a1 + a2;	
	
	//===================================
	a3 = acos( (LEN_LOWER_CRANK*LEN_LOWER_CRANK + LEN_PEN_FOREARM*LEN_PEN_FOREARM - D1*D1) / (2 * LEN_LOWER_CRANK*LEN_PEN_FOREARM));
	a4 = a3 + angle_servo_a;
	mid_x = cos(a4)*(LEN_PEN_FOREARM+L_PEN_UPPERARM) + penX;
	mid_y = sin(a4)*(LEN_PEN_FOREARM+L_PEN_UPPERARM) + penY;	
	
	D2 =sqrt ( sq(mid_x-SERVO_B_X) + sq(mid_y - SERVO_B_Y) );
	a5 = atan2(mid_y - SERVO_B_Y, mid_x - SERVO_B_X);
	a6 = acos( (LEN_UPPER_CRANK*LEN_UPPER_CRANK + D2*D2 - LEN_MID_LINK*LEN_MID_LINK) / (2*LEN_UPPER_CRANK*D2));
	angle_servo_b = a5 + a6;				
}
*/


// intersection of circles method
void calc_servo_intersect(float penX, float penY, float penZ)
{
	
	
	float angle_servo_a, angle_servo_b, angle_servo_c; // the angle of servo A arm relative to machine		
	float intsecX0, intsecY0, intsecX1, intsecY1;  // two possible intersection points 	
	float pin1X, pin1Y , pin2X, pin2Y, pin3X, pin3Y;	
	uint16_t servo_a_pulse_len, servo_b_pulse_len, servo_c_pulse_len;
	float servo_a_ticks_per_radian = SERVO_A_RAD_CNT;
	float servo_b_ticks_per_radian = SERVO_B_RAD_CNT;
	uint16_t servo_a_center = SERVO_A_ZERO_PT;
	uint16_t servo_b_center = SERVO_B_ZERO_PT;
	
	float servo_c_pulse_min, servo_c_pulse_max;
	
	// range check
	if (penX < MIN_WORK_X || penX > MAX_WORK_X)
		return;
	
	if (penY < MIN_WORK_Y || penY > MAX_WORK_Y)
		return;
	
	// ============== find the location of pin1 ========================
	if (circle_circle_intersection(penX, penY, LEN_PEN_FOREARM,  // pen location and arm length 
                               SERVO_A_X, SERVO_A_Y, LEN_LOWER_CRANK, // servo a location and arm length
                               &intsecX0, &intsecY0, // intersection 0
                               &intsecX1, &intsecY1) == 0) // intersection 1
	
	{
		Serial.println("calc fail 0");	
		return;
	}																
	
	// right most link is correct
	if (intsecX0 > intsecX1)
	{
		pin1X = intsecX0;
		pin1Y = intsecY0;
	}
	else
	{
		pin1X = intsecX1;
		pin1Y = intsecY1;
	}
	
	// ============== find the angle of Servo A =======================
	angle_servo_a = atan2(pin1Y - SERVO_A_Y, pin1X - SERVO_A_X);
	
	// =============== find location of pin2 ========================
	pin2X = (pin1X - penX) * ((LEN_PEN_FOREARM + L_PEN_UPPERARM)/LEN_PEN_FOREARM) + penX;
	pin2Y = (pin1Y - penY) * ((LEN_PEN_FOREARM + L_PEN_UPPERARM)/LEN_PEN_FOREARM) + penY;	
		
	// ============== find the location of pin3 ========================
	if (circle_circle_intersection(pin2X, pin2Y, LEN_MID_LINK,  // pen location and arm length 
                               SERVO_B_X, SERVO_B_Y, LEN_UPPER_CRANK, // servo a location and arm length
                               &intsecX0, &intsecY0, // intersection 0
                               &intsecX1, &intsecY1) == 0) // intersection 1 
	
	{
		Serial.println("calc fail 1");
		return;
	}	
	
	if (intsecY0 > intsecY1) // higher Y is used
	{
		pin3X = intsecX0;
		pin3Y = intsecY0;
	}
	else
	{
		pin3X = intsecX1;
		pin3Y = intsecY1;
	}
	
	//Serial.printf("PIN3 X:%4.3f Y:%4.3f\r\n", pin3X, pin3Y);
	
	// ============== find the angle of Servo A =======================
	angle_servo_b = atan2(pin3Y - SERVO_B_Y, pin3X - SERVO_B_X);
	
		
	// apply calibration
	// step resolution which is meaningless is used as a percentage compensation...higher is positive angle
	servo_a_center = servo_a_center * (settings.steps_per_mm[X_AXIS] / 100.0);
	servo_b_center = servo_b_center * (settings.steps_per_mm[Y_AXIS] / 100.0);
	servo_a_ticks_per_radian = SERVO_A_RAD_CNT * (settings.max_travel[X_AXIS] / -100.0);
	servo_b_ticks_per_radian = SERVO_B_RAD_CNT * (settings.max_travel[Y_AXIS] / -100.0);
	
	// now convert angles to pulse length counts
	servo_a_pulse_len = servo_a_center + (angle_servo_a * servo_a_ticks_per_radian);
	servo_b_pulse_len = servo_b_center + (angle_servo_b * servo_b_ticks_per_radian);
	
	// ----- Begin SERVO C -----
	
	// map z to servo angles. Z is constrained to a range
	angle_servo_c = mapConstrain(penZ, 0.0, SERVO_C_RANGE_MM, SERVO_C_RADIANS_MIN, SERVO_C_RADIANS_MAX);
	
	// calculate and apply a calibration to the minimum position
	servo_c_pulse_min = map_float(SERVO_C_RADIANS_MIN, SERVO_MIN_RADIANS, SERVO_MAX_RADIANS, SERVO_MIN_PULSE, SERVO_MAX_PULSE) * (settings.				steps_per_mm[Z_AXIS] / 100.0);
	
	// calculate and apply a calibration to the maximum position
	servo_c_pulse_max = map_float(SERVO_C_RADIANS_MAX, SERVO_MIN_RADIANS, SERVO_MAX_RADIANS, SERVO_MIN_PULSE, SERVO_MAX_PULSE) * (settings.				max_travel[Z_AXIS] / -100.0);
	
	// determine the pulse length
	servo_c_pulse_len = map_float(angle_servo_c, SERVO_C_RADIANS_MIN, SERVO_C_RADIANS_MAX, servo_c_pulse_min, servo_c_pulse_max );
	
	// ----- End of SERVO C -----
	
	
	
	// update the PWM values
	// ledcWrite appears to have issues with interrupts, so make this a critical section
	portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
	taskENTER_CRITICAL(&myMutex);	
		ledcWrite(SERVO_A_CHANNEL_NUM, servo_a_pulse_len);
		ledcWrite(SERVO_B_CHANNEL_NUM, servo_b_pulse_len);		
		ledcWrite(SERVO_C_CHANNEL_NUM, servo_c_pulse_len);  
	taskEXIT_CRITICAL(&myMutex);
	
}

// ============================== 

int circle_circle_intersection(float x0, float y0, float r0,  // x,y and radius 
                               float x1, float y1, float r1,
                               float *xi, float *yi, // intersection
                               float *xi_prime, float *yi_prime)
{
  double a, dx, dy, d, h, rx, ry;
  double x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = x1 - x0;
  dy = y1 - y0;

  /* Determine the straight-line distance between the centers. */
  //d = sqrt((dy*dy) + (dx*dx));
  d = hypot(dx,dy); // Suggested by Keith Briggs

  /* Check for solvability. */
  if (d > (r0 + r1))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < fabs(r0 - r1))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.  
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  /* Determine the coordinates of point 2. */
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((r0*r0) - (a*a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h/d);
  ry = dx * (h/d);

  /* Determine the absolute intersection points. */
  *xi = x2 + rx;
  *xi_prime = x2 - rx;
  *yi = y2 + ry;
  *yi_prime = y2 - ry;

  return 1;
}

// constrain calibration values to the legal range
uint8_t servo_constrain_cal_range(uint8_t cal_val) {	
	return constrain(cal_val, SERVO_CAL_MIN, SERVO_CAL_MAX);
}
