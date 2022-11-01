// Imports
#include "system.h" // IO addresses & IRQs of ports
#include "alt_types.h"
#include "altera_up_avalon_parallel_port.h"
#include "sys/alt_irq.h" //needed to use interrupts
#include <priv/alt_legacy_irq.h> // alt_irq_register prototype
#include <stdio.h>
#include "includes.h"
#include <string.h>
#include <stdfix.h> // Get fixed point working

#define false 0
#define true 1

typedef INT8U bool;

/* Definition of Task Stacks */
#define TASK_STACKSIZE 2048
#define NUM_PROCS 5
OS_STK task_procs_stk[NUM_PROCS][TASK_STACKSIZE];  // task stack space
OS_STK start_task_stk[TASK_STACKSIZE];
OS_STK cpu_task_stk[TASK_STACKSIZE];

// ***************  RMS and EDF scheduling code starts here **************************************

// Priorities
#define TASK_START_PRIO 1
#define CPU_TASK_START_PRIO 2
#define MIN_PRIO_PERIOD CPU_TASK_START_PRIO + 1 // highest (ie lowest number) assigned priority to the processes in RMS
#define BEAM_UPDATE_TASK 0
#define PID_CONTROL_TASK 1
#define TRAJ_UPDATE_TASK 2
#define MOTION_UPDATE_TASK 3
#define ENCODER_ANGLE_HEX_OUTPUT_TASK 4

// Initialise System Variables

int priority[NUM_PROCS]; // uCOS-II priority assigned to each proc
int param[NUM_PROCS];// global memory reserved for passing parameters to tasks
int deadlines[NUM_PROCS];

OS_EVENT *SemProcMutex;  // semaphore for mutex on these process data structures
OS_EVENT *SemProcReady[NUM_PROCS];// semaphore for each periodic task to wait for new period
OS_EVENT *SemEncoderAngleMutex;// semaphore for each the encoder angle


#define CHECK_NULL(x) if (x == 0) printf("Runtime error NULL pointer: line %d\n", __LINE__);
INT8U err;
#define CE(x) if ((err = x) != OS_NO_ERR) \
	printf("Runtime error: %d line %d - see ucos_ii.h\n", err, __LINE__);

#define OPENDEV(DEV, PATH) DEV = alt_up_parallel_port_open_dev(PATH);\
	if (DEV == NULL) printf("Failed to open %s\n", PATH); \
	else printf("Opened %s\n", PATH);

#define WRITE(DEV, data) alt_up_parallel_port_write_data(DEV, data);

#define READ(DEV) alt_up_parallel_port_read_data(DEV);

// Initialize device driver pointers here
alt_up_parallel_port_dev *usecs_dev;
alt_up_parallel_port_dev *sliders_dev;
alt_up_parallel_port_dev *hex0_dev;
alt_up_parallel_port_dev *hex1_dev;
alt_up_parallel_port_dev *ledgreen_dev;
alt_up_parallel_port_dev *ledred_dev;
alt_up_parallel_port_dev *keys_dev;
alt_up_parallel_port_dev *encoder_dev;
alt_up_parallel_port_dev *timer_dev;
alt_up_parallel_port_dev *encoder_angle_dev;
alt_up_parallel_port_dev *hbridge_pwm_dev;
alt_up_parallel_port_dev *beam_dev;

#define ENCODER_INTERRUPT_MASK 0x3
#define BEAM_INTERRUPT_MASK 0x1
#define MASK_10BIT 0x3FF

// ISR Variables
void * context;

// Global variables
int encoder_angle = 0;
int estimate_angle = 0;
int desired_count = 0;
int enAin;
int enBin;
int enAin_prev;
int enBin_prev;
int motor_voltage;

#define BEAM_N 5
#define MASK_DIST 4 // Mask distance in cm
#define MASK_DIST_COUNT 13

// Beam constants
#define BEAM_THRESHOLD 10000
#define THRESHOLD_SCALING 300

// Pendulum trajectory constants
#define ARM_LENGTH 59
#define PI_SQUARE 2527
#define PI 804
#define PI2 1608 // 24.8 bit
#define A_MAX_CONST 6058 // 4*pi^2*0.6*256 to get into fixed point
int beam_times[BEAM_N];
int beam_values[BEAM_N];
int T;
int A_max;
int t0 = 0;
int dir;
int crossing_vel = 0;
int velocity_command;
int timestep_inter = 0; // This refers to how many times
// we have done intermittent 10ms for a given velocity command

#define Kp 30
#define Kd 300 //256
#define Ki 30
#define saturate_max 2147483647
#define saturate_min -2147483648

int sat_add(int A, int B) {
	int sum = A + B;
	if (A > 0 && B > 0 && sum < 0) {
		return saturate_max;
	} else if (A < 0 && B < 0 && sum > 0) {
		return saturate_min;
	} else {
		return sum;
	}
}

int sat_mult(int A, int B) {
	int ans = A * B;
	if (((A > 0 && B > 0) || (A < 0 && B < 0)) && ans < 0) {
		return saturate_max;
	} else if (((A < 0 && B > 0) || (A > 0 && B < 0)) && ans > 0) {
		return saturate_min;
	} else {
		return ans;
	}
}

// Interpolated sin lookup function
// input x is signed 32.0
// output is signed 24.8
int Sin(int x){
	int normalised_x = x % 360; // has to be between 0 <= x <= 359
	if (normalised_x < 0) normalised_x += 360;

	int quadrant_sign = (0 <= normalised_x & normalised_x <= 180)?1:-1; // 1 if above y axis

	// Normalise to first quadrant
	if (90 < normalised_x & normalised_x < 180){
		normalised_x = 180 - normalised_x;
	}
	else if (180 <= normalised_x & normalised_x < 270){
		normalised_x = normalised_x - 180;
	}
	else if (270 <= normalised_x & normalised_x < 360){
		normalised_x = 360 - normalised_x;
	}

	// Build look up table:
	// THese are the values of sin(10k) where k = 0,1,..,9
	int sin_lut[10] = {0, 44, 87, 128, 164, 196, 221, 240, 252, 256};
	for (int k = 0; k < 10; k++){
		// Found upper bound on multiple of 10 for x
		if (normalised_x <= 10*k){
			if (k == 0) return quadrant_sign * sin_lut[k];
			else {
				// Then x should be rounded down
				if (10*k - normalised_x > 5) return quadrant_sign * sin_lut[k - 1];
				else return quadrant_sign * sin_lut[k];
			}
		}
	}
}

// Define our tasks

void PIDControlTask(void *pdata){
	int error = 0;
	int error_prev = 0;
	int error_sum = 0;
	int pid_signal_10bit = 0;
	int start_time;
	int execution_time; // us
	int ms_count = 0;

	while(true){
		// Run task periodically at 10ms intervals
		OSTimeDlyHMSM(0,0,0,10);

		/*if (4000 >= ms_count && ms_count >= 2000){

			estimate_angle = 300;
		}
		else estimate_angle = 0;
	*/


		//start_time = READ(usecs_dev);
		//printf("Start time: %d", start_time);

		encoder_angle = READ(encoder_angle_dev);

//		if (ms_count <= 6000){
//
//			printf("%d,%d,%d\n", ms_count, estimate_angle, encoder_angle);
//		}

		// Simulate a predicted angle which will be later computed using sensor beam
		//estimate_angle = 300;
		error_prev = error;
		error = -encoder_angle + estimate_angle;
		error_sum += error;
		int P_term = sat_mult(Kp,error);
		int D_term = sat_mult(Kd,sat_add(error,-error_prev));
		int I_term = sat_add(sat_mult(Ki,error_sum),128)/256;

		motor_voltage = (P_term + D_term + I_term + 128)/256;

		// 512 = 2^9
		if (motor_voltage > 511) {
			motor_voltage = 511;
		} else if (motor_voltage < -512) {
			motor_voltage = -512; //Looks like 0b1 XXX..XXXX
		}
		//printf("Error %d P %d D %d I %d Mv %d\n",error,P_term,D_term,I_term, motor_voltage);
		WRITE(hbridge_pwm_dev, motor_voltage);
		ms_count += 10;
		//int end_time = READ(usecs_dev);
		//printf("Start time: %d", start_time);
		//printf("End time: %d", end_time);
		//execution_time = end_time - start_time;
		//printf("Motor_voltage %d, exec_time %d us\n",motor_voltage, execution_time);
	}
}

/* void EncoderAngleUpdaterTask(void *pdata){

	while (true){
		//printf("Waiting for ISR\n");

		OSSemPend(SemProcReady[ENCODER_ANGLE_UPDATE_TASK],0,&err); CE(err); // Wait for task to be resumed by ISR - meaning new encoder data available
		OSSemPend(SemEncoderAngleMutex,0,&err); CE(err);

		// Read new encoder values and compute bitwise
		int encoder_read = READ(encoder_dev);
		//printf("Encoder read: %d\n", encoder_read);
		//printf("Latency: %d\n", latency_us);
		if (encoder_read == 0) { enAin = 0; enBin = 0; }
		else if (encoder_read == 1) { enAin = 0; enBin = 1;}
		else if (encoder_read == 2) {enAin = 1; enBin = 0;}
		else if (encoder_read == 3) {enAin = 1; enBin = 1;}

		// Compute the angle based off the previous vs current encoder value
		if (!enAin_prev & !enBin_prev & enAin & !enBin | \
		enAin_prev & !enBin_prev & enAin & enBin | \
		enAin_prev & enBin_prev & !enAin & enBin | \
		!enAin_prev & enBin_prev & !enAin & !enBin
		) encoder_angle++;

		else if (!enAin_prev & !enBin_prev & !enAin & enBin | \
		!enAin_prev & enBin_prev & enAin & enBin | \
		enAin_prev & enBin_prev & enAin & !enBin | \
		enAin_prev & !enBin_prev & !enAin & !enBin
		) encoder_angle--;

		else printf("Incorrect encoder reading.\n");
		//printf("Encoder Angle: %d\n", encoder_angle);
		enAin_prev = enAin;
		enBin_prev = enBin;

		OSSemPost(SemEncoderAngleMutex);

	}
} */

void TrajectoryGeneratorTask(void *pdata) {
	int previous_count = 0;


	while (true) {
		previous_count = desired_count;
		desired_count = (50 * previous_count + (velocity_command)) / 50;
		desired_count = (abs(desired_count) >= 175)? desired_count/abs(desired_count)*175 : desired_count;

		// Now set this for our PID loop to process
		if (timestep_inter < 5){
//			printf("PREV %d DESIRED %d \n", previous_count, desired_count);
			estimate_angle = desired_count;
		}
		timestep_inter++;
		OSTimeDlyHMSM(0,0,0,10);
	
	}
}


// Take in crossing time, speed, direction, AMax, T
// Output the velocity our motor needs to rotate to reach position
void MotionGeneratorTask(void *pdata){
	int Adesired;
	int t;
	int A_waypoint_next;
	int A_current;

	while(true){
		// Need to compute next waypoint to get to within 100ms period

		// Compute the next time step we are up to:
		// By adding 100ms to next time
		t = READ(usecs_dev);// + 100000;
		A_current = READ(encoder_angle_dev);
		A_current = A_current % 600;

		// Generate the angle for that 100ms period that we wish to reach.
		// This desired angle will then be broken up further into smaller 10ms intermittent desired angles
		int deg = (t - t0) / 1000 * 360 / (T );
		A_waypoint_next = A_max * dir * Sin(deg) / 256;// Need to check if A_max non-zero?

		// Generate velocity command
		velocity_command = (A_waypoint_next - A_current) * 5; // counts/s

		printf("t %d t0 %d T %d deg %d Sin(deg) %d\n", t, t0, T, deg, Sin(deg));
		//printf("AMAX: %d Deg: %d Sin(x): %d A_Current %d WAYNEXT: %d Command %d \n", A_max, deg, Sin(deg), A_current, A_waypoint_next,velocity_command);

		// Need to reset the counter for intermittent timesteps
		timestep_inter = 0;

		// Only need to run this once every 100ms
		OSTimeDlyHMSM(0,0,0,50);
	}
}

// Equivalent to Pendulum Motion Estimation in System Diagram
// Must process edge time and will output to global variables:
// crossing time (t0), speed, direction, Amax, T

void BeamUpdaterTask(void *pdata){

	int current_time = 0;
	int beam_value;
	for (int i = 0; i < BEAM_N; i++){
		beam_times[i] = 0;
		beam_values[i] = 0;
	}

	int hits = 0;
	
	while (true){
		//printf("Waiting for ISR\n");
		
		//case 1: pend starts at big angle and fully passes sensor

		//case 2: 
		OSSemPend(SemProcReady[BEAM_UPDATE_TASK],0,&err); CE(err); // Wait for task to be resumed by ISR - meaning new encoder data available
		//OSSemPend(SemEncoderAngleMutex,0,&err); CE(err);
		// Read new beam values and compute bitwise
		current_time = READ(usecs_dev);
		beam_value = READ(beam_dev);

		// Update the queue that stores recent n sensor readings
		for (int i = 1; i < BEAM_N ; i++){
			beam_times[BEAM_N - i] = beam_times[BEAM_N - i - 1];
			beam_values[BEAM_N - i] = beam_values[BEAM_N - i - 1];
		}

		beam_values[0] = beam_value;
		beam_times[0] = current_time;

		// Wait til we reach 5 beam hits
		if (hits > BEAM_N - 1) {

			// Don't run if latest beam is 0 because it means it is still in transit
			if (beam_values[0] == 0) {
				continue;
			}

			int T1 = beam_times[0] - beam_times[1];
			int T2 = beam_times[1] - beam_times[2];
			int T3 = beam_times[2] - beam_times[3];
			int T4 = beam_times[0] - beam_times[4];

//			printf("%d %d %d %d ", T1, T2, T3, T4);
//			printf("%d %d %d %d %d\n", beam_times[0],beam_times[1],beam_times[2],beam_times[3],beam_times[4]);
			int adaptive_threshold = BEAM_THRESHOLD;

//			if (desired_vel != 0) {
//				adaptive_threshold +=  -THRESHOLD_SCALING * (desired_vel - 25);
////				printf("ADAPT %d ", adaptive_threshold);
//			}

			if (abs(T3 - T2) < adaptive_threshold && abs(T1 - 2*T3) < adaptive_threshold) {
				dir = -1;
				t0 = beam_times[2];

//				printf("GOING FORWARD\n");
			} else if (abs(T2 - T1) < adaptive_threshold && abs(T3 - 2*T1) < adaptive_threshold) {
				dir = 1;
				t0 = beam_times[1];
//				printf("GOING BACKWARD\n");
			}
			 else {
//				printf("INVALID\n");
				continue;
			}

			// Calculate crossing_vel . Multiply by 10e4 to convert uSecs to secs and MASK_DIST (cm) to (m)
			crossing_vel = 1000000 * (MASK_DIST) / (beam_times[0] - beam_times[3]);
			int crossing_vel_counts = 10e6 * (MASK_DIST) / (beam_times[0] - beam_times[3]);
			desired_count = 0;

			T = T4 * 2 / 1000; //ms
//			A_max = 256 * T / (2*PI_SQUARE) * crossing_vel_counts / 1000000;

			// T * vL / (2pi * L)
//			A_max =  crossing_vel * 256 / A_MAX_CONST) * T / 100000;
			A_max = crossing_vel * T / (A_MAX_CONST) * 256 * 360 / 100000 * (10/3);
			printf("%d %d %d %d\n", crossing_vel ,t0,T, A_max);

			//OSSemPost(SemEncoderAngleMutex);

		} else {
			hits++;
			continue;
		}

		

	}
}

void EncoderAngleHexOutputTask(void *pdata) {
	int is_negative = 0;
	int negative_bit_pattern = 0xC0;
	while (true) {
		// Sleep periodically to not load system
		OSTimeDlyHMSM(0,0,0,10);
		// Acquire updated encoder angle
		OSSemPend(SemEncoderAngleMutex,0,&err); CE(err);
		int encoder_angle_copy = READ(encoder_angle_dev);
//		printf("EncoderAngle: %d\n", encoder_angle_copy);
		OSSemPost(SemEncoderAngleMutex);
		// Need to check switches to see what unit we represent angle in
		int sliders_read = READ(sliders_dev);
		int angle_switch = (0x1 << 17 & sliders_read);
		// Then we use degrees
		if (angle_switch){
			encoder_angle_copy = encoder_angle_copy * 3 / 10;
		}

		// IF negative, must output sign to hex and negate to get positive numbers after
		if (encoder_angle_copy < 0){
			is_negative = 1;
			// Negate so our division gives us the magnitude of the digits
			encoder_angle_copy = -encoder_angle_copy;
		}
		else is_negative = 0;
		// Need to work out if using 3 numbers with sign or 4 unsigned
		int digits[4] = {encoder_angle_copy % 10, (encoder_angle_copy / 10) % 10,
		(encoder_angle_copy / 100) % 10, (encoder_angle_copy / 1000) % 10};
		//printf("Encoder Angle: %d\n", encoder_angle_copy);
		int segments[4] = {0x0, 0x0, 0x0, 0x0};
		for (int i = 0; i < 4; i++) {
			switch (digits[i]) {
				case 0:
					segments[i] = 0x03F;
					break;
				case 1:
					segments[i] = 0x06;
					break;
				case 2:
					segments[i] = 0x5B;
					break;
				case 3:
					segments[i] = 0x4F;
					break;
				case 4:
					segments[i] = 0x66;
					break;
				case 5:
					segments[i] = 0x6D;
					break;
				case 6:
					segments[i] = 0x7D;
					break;
				case 7:
					segments[i] = 0x07;
					break;
				case 8:
					segments[i] = 0x7F;
					break;
				case 9:
					segments[i] = 0x6F;
					break;
			}
		}

		alt_up_parallel_port_write_data(hex0_dev, 0);
		alt_up_parallel_port_write_data(hex1_dev, 0); // Might not need
		alt_up_parallel_port_write_data(hex0_dev, segments[3] << 24 | segments[2] << 16 \
			| segments[1]<<8 | segments[0]);
		if (is_negative) alt_up_parallel_port_write_data(hex1_dev, negative_bit_pattern << 24);
	}
}

void InitScheduler(void){
  // initialise datastructures for all processes
  // set up scheduling problem periods and runtimes here

  // initialise semaphore for mutex
	SemProcMutex =  OSSemCreate(1);  CHECK_NULL(SemProcMutex);
	SemEncoderAngleMutex = OSSemCreate(1);  CHECK_NULL(SemEncoderAngleMutex);

    // initialise datastructures
    for (int i=0; i<NUM_PROCS; i++){
		// initialise datastructures for proc i
		param[i] = i;  // storage space to pass parameter to task
		priority[i] = MIN_PRIO_PERIOD + i; // fix priorities using RMS
		// init semaphore regulating period end of proc i
		SemProcReady[i] = OSSemCreate(0); CHECK_NULL(SemProcReady[i]);
    }

    // create processes
    CE(OSTaskCreateExt(BeamUpdaterTask,
    			  (void *)&param[BEAM_UPDATE_TASK], // parameter = process index
    			  (void *)&task_procs_stk[BEAM_UPDATE_TASK][TASK_STACKSIZE -1],
    			  priority[BEAM_UPDATE_TASK], priority[BEAM_UPDATE_TASK],
    			  &task_procs_stk[BEAM_UPDATE_TASK][0],
    			  TASK_STACKSIZE,
    			  NULL,
    			  0));

	CE(OSTaskCreateExt(PIDControlTask,
		  (void *)&param[PID_CONTROL_TASK], // parameter = process index
		  (void *)&task_procs_stk[PID_CONTROL_TASK][TASK_STACKSIZE -1],
		  priority[PID_CONTROL_TASK], priority[PID_CONTROL_TASK],
		  &task_procs_stk[PID_CONTROL_TASK][0],
		  TASK_STACKSIZE,
		  NULL,
		  0));

	CE(OSTaskCreateExt(EncoderAngleHexOutputTask,
			  (void *)&param[ENCODER_ANGLE_HEX_OUTPUT_TASK], // parameter = process index
			  (void *)&task_procs_stk[ENCODER_ANGLE_HEX_OUTPUT_TASK][TASK_STACKSIZE -1],
			  priority[ENCODER_ANGLE_HEX_OUTPUT_TASK], priority[ENCODER_ANGLE_HEX_OUTPUT_TASK],
			  &task_procs_stk[ENCODER_ANGLE_HEX_OUTPUT_TASK][0],
			  TASK_STACKSIZE,
			  NULL,
			  0));
			  
	CE(OSTaskCreateExt(TrajectoryGeneratorTask,
			  (void *)&param[TRAJ_UPDATE_TASK], // parameter = process index
			  (void *)&task_procs_stk[TRAJ_UPDATE_TASK][TASK_STACKSIZE -1],
			  priority[TRAJ_UPDATE_TASK], priority[TRAJ_UPDATE_TASK],
			  &task_procs_stk[TRAJ_UPDATE_TASK][0],
			  TASK_STACKSIZE,
			  NULL,
			  0));

	CE(OSTaskCreateExt(MotionGeneratorTask,
			  (void *)&param[MOTION_UPDATE_TASK], // parameter = process index
			  (void *)&task_procs_stk[MOTION_UPDATE_TASK][TASK_STACKSIZE -1],
			  priority[MOTION_UPDATE_TASK], priority[MOTION_UPDATE_TASK],
			  &task_procs_stk[MOTION_UPDATE_TASK][0],
			  TASK_STACKSIZE,
			  NULL,
			  0));

	// Assume the parallel port ready to be read
	// Set the previous encoder values here from first read
	OSSemPend(SemEncoderAngleMutex,0,&err); CE(err);
	int encoder_read = READ(encoder_dev);
	if (encoder_read == 0) {enAin = 0; enBin = 0;}
	else if (encoder_read == 1) {enAin = 0; enBin = 1;}
	else if (encoder_read == 2) {enAin = 1; enBin = 0;}
	else if (encoder_read == 3) {enAin = 1; enBin = 1;}
	enAin_prev = enAin;
	enBin_prev = enBin;
	OSSemPost(SemEncoderAngleMutex);

}

void  SchedulerTask(void *pdata)
{
//	INT8U  err;

	InitScheduler();

	while (true) {  // one tick at a time
		OSTimeDly(1);
		}
}


static void BeamISR(void * isr_context, alt_u32 id){

	// Clear interrupt
	alt_up_parallel_port_clear_edge_capture(beam_dev);

	// Signal response time

	// Post semaphore to go to computation
	OSSemPost(SemProcReady[BEAM_UPDATE_TASK]);

	// Read the latency from since A,B changed
	//int latency_us = READ(usecs_dev);
	//Signal that we have finished ISR for timing purposes
	//WRITE(timer_dev, 0x2); // Write 1 to rst_interrupt_timer_in
	//WRITE(timer_dev, 0x0); // Write 10 to rst_interrupt_timer_in
	// printf("Latency: %d\n", latency_us);
}

void RegisterBeamInterrupt(void){
// attaches an ISR to IRQ line.
//   If your ISR interacts with RTOS tasks, eg via semaphores, make sure all semaphores are
//  initialised and the OS started BEFORE running this code!  This is because
// an ISR may be instigated immediately after this and attempt to access
// unititialised semaphores and data structures crashing the OS.

	alt_up_parallel_port_clear_edge_capture(beam_dev); // clears interrupts
	alt_irq_register(BEAM_IRQ, context, BeamISR);
	alt_up_parallel_port_set_interrupt_mask(beam_dev, BEAM_INTERRUPT_MASK); // enables interrupts on bits where ?? is 1 - eg use 0b11 for two interrupts.
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
	// Initialise device drivers here.
	OPENDEV(usecs_dev, "/dev/Microseconds");
	OPENDEV(sliders_dev, "/dev/SliderSW");
	OPENDEV(hex0_dev, "/dev/HexDisplays3to0");
	OPENDEV(hex1_dev, "/dev/HexDisplays7to4");
	OPENDEV(ledgreen_dev, "/dev/LEDGreen");
	OPENDEV(ledred_dev, "/dev/LEDRed");
	OPENDEV(keys_dev, "/dev/KEYButtons");
	OPENDEV(encoder_dev, "/dev/Encoder");
	OPENDEV(timer_dev, "/dev/TimerControl");
	OPENDEV(encoder_angle_dev, "/dev/EncoderAngle");
	OPENDEV(hbridge_pwm_dev, "/dev/HbridgePWM");
	OPENDEV(beam_dev, "/dev/Beam");

	// Before registering ISR to IRQ, we must initialise our timing module
	// WRITE(timer_dev, 0x2); // Write 1 to rst_interrupt_timer_in
	// WRITE(timer_dev, 0x0); // Write 10 to rst_interrupt_timer_in

	// RegisterEncoderInterrupt();
	RegisterBeamInterrupt();
	OSTaskCreateExt(SchedulerTask,
					NULL,
					(void *) &start_task_stk[TASK_STACKSIZE-1],
					TASK_START_PRIO,
					TASK_START_PRIO,
					start_task_stk,
					TASK_STACKSIZE,
					NULL,
					0);

	OSStart();
	return 0;
}
