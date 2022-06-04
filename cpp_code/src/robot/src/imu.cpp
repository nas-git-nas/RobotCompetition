#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

#include "imu.h"

// Needed when mixing C and C++ code/libraries
#ifdef __cplusplus
    extern "C" {
#endif
        #include "mpu9150.h"
        #include "linux_glue.h"
        #include "local_defaults.h"

#ifdef __cplusplus
    }
#endif

void register_sig_handler();
void sigint_handler(int sig);

int done;

void IMU::initIMU(void)
{
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;


		
    // Initialize the MPU-9150
	register_sig_handler();
	mpu9150_set_debug(verbose);
	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);
	/*set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);
	if (accel_cal_file)
		free(accel_cal_file);
	if (mag_cal_file)
		free(mag_cal_file);
	memset(&mpu, 0, sizeof(mpudata_t));
	if (sample_rate == 0)
		return -1;*/

}


void IMU::calcGyro(void)
{
	if (mpu9150_read(&mpu) == 0) {
		//print_fused_euler_angles(&mpu);
	    //ss << "\rX: %0.0f Y: %0.0f Z: %0.0f        ",
	    /*ss << "\rX: " << mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE <<
            " Y: " << mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE <<
			" Z: " << mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE << count;*/

		// printf_fused_quaternions(&mpu);
	    // print_calibrated_accel(&mpu);
	    //print_calibrated_mag(&mpu);
	    //print_raw_mag(&mpu);
	    //print_fused_euler_angles(&mpu);
	    printGyro(&mpu);
	}
}

void IMU::printGyro(mpudata_t *mpu)
{
	static short x_max = -1000;
	static short x_min = 1000;
	if(mpu->rawGyro[VEC3_X]>x_max) {
		x_max = mpu->rawGyro[VEC3_X];
	}
	if(mpu->rawGyro[VEC3_X]<x_min) {
		x_min = mpu->rawGyro[VEC3_X];
	}
	
	static short y_max = -1000;
	static short y_min = 1000;
	if(mpu->rawGyro[VEC3_Y]>y_max) {
		y_max = mpu->rawGyro[VEC3_Y];
	}
	if(mpu->rawGyro[VEC3_Y]<y_min) {
		y_min = mpu->rawGyro[VEC3_Y];
	}
	
	static short z_max = -1000;
	static short z_min = 1000;
	if(mpu->rawGyro[VEC3_Z]>z_max) {
		z_max = mpu->rawGyro[VEC3_Z];
	}
	if(mpu->rawGyro[VEC3_Z]<z_min) {
		z_min = mpu->rawGyro[VEC3_Z];
	}
	printf("\rgyro> X: %03d (%02d,%02d), Y: %03d (%02d,%02d), Z: %03d (%02d,%02d)  ",
			mpu->rawGyro[VEC3_X], x_min, x_max,
			mpu->rawGyro[VEC3_Y], y_min, y_max,
			mpu->rawGyro[VEC3_Z], z_min, z_max);
			
			
	int raw_max_z = 32768;
	int raw_min_z = -32767;
	int val_max_z = 2000; //250 degrees/s
	int val_min_z = -2000; //250 degrees/s
	
	float offset_z = (raw_max_z + raw_min_z) / 2;
	float val_rel_z = mpu->rawGyro[VEC3_Z]*(val_max_z-val_min_z)/(raw_max_z-raw_min_z);
	short val_abs_z = short(offset_z + val_rel_z);
	
	
	
   static ros::Time last_update_time = ros::Time::now();
   ros::Time current_time = ros::Time::now();
   ros::Duration delta_time = current_time-last_update_time;
   static float angle = 0;
   angle += delta_time.toSec()*val_abs_z;
   last_update_time = current_time;
   
   printf("acc: %03d, ang: %0.3f\n", val_abs_z, angle);


	fflush(stdout);
}


void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
    
