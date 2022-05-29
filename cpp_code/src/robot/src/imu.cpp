#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

void register_sig_handler();
void sigint_handler(int sig);
int mpu9150_init(int i2c_bus, int sample_rate, int mix_factor);
int mpu9150_read_dmp(mpudata_t *mpu);
int mpu9150_read_mag(mpudata_t *mpu);
int mpu9150_read(mpudata_t *mpu);


void IMU::initIMU(void)
{
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;


	register_sig_handler();

	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);

}


void IMU::calcGyro(void)
{
	if (mpu9150_read(&mpu) == 0) {
		//print_fused_euler_angles(&mpu);
	    //ss << "\rX: %0.0f Y: %0.0f Z: %0.0f        ",
	    ss << "\rX: " << mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE <<
            " Y: " << mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE <<
			" Z: " << mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE << count;

		// printf_fused_quaternions(&mpu);
	    // print_calibrated_accel(&mpu);
	    //print_calibrated_mag(&mpu);
	    //print_raw_mag(&mpu);
	    //print_fused_euler_angles(&mpu);
	    print_raw_gyro(&mpu);
}

void print_raw_gyro(mpudata_t *mpu)
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

int mpu9150_init(int i2c_bus, int sample_rate, int mix_factor)
{
	signed char gyro_orientation[9] = { 1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1 };

	if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS) {
		printf("Invalid I2C bus %d\n", i2c_bus);
		return -1;
	}

	if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE) {
		printf("Invalid sample rate %d\n", sample_rate);
		return -1;
	}

	if (mix_factor < 0 || mix_factor > 100) {
		printf("Invalid mag mixing factor %d\n", mix_factor);
		return -1;
	}

	yaw_mixing_factor = mix_factor;

	linux_set_i2c_bus(i2c_bus);

	printf("\nInitializing IMU .");
	fflush(stdout);

	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		printf("\nmpu_configure_fifo() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
	
	if (mpu_set_sample_rate(sample_rate)) {
		printf("\nmpu_set_sample_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_compass_sample_rate(sample_rate)) {
		printf("\nmpu_set_compass_sample_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_load_motion_driver_firmware()) {
		printf("\ndmp_load_motion_driver_firmware() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		printf("\ndmp_set_orientation() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

  	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		printf("\ndmp_enable_feature() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);
 
	if (dmp_set_fifo_rate(sample_rate)) {
		printf("\ndmp_set_fifo_rate() failed\n");
		return -1;
	}

	printf(".");
	fflush(stdout);

	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}

	printf(" done\n\n");

	return 0;
}

int mpu9150_read_dmp(mpudata_t *mpu)
{
	short sensors;
	unsigned char more;

	if (!data_ready())
		return -1;

	if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
		printf("dmp_read_fifo() failed\n");
		return -1;
	}

	while (more) {
		// Fell behind, reading again
		if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
			printf("dmp_read_fifo() failed\n");
			return -1;
		}
	}

	return 0;
}

int mpu9150_read_mag(mpudata_t *mpu)
{
	if (mpu_get_compass_reg(mpu->rawMag, &mpu->magTimestamp) < 0) {
		printf("mpu_get_compass_reg() failed\n");
		return -1;
	}

	return 0;
}

int mpu9150_read(mpudata_t *mpu)
{
	if (mpu9150_read_dmp(mpu) != 0)
		return -1;

	if (mpu9150_read_mag(mpu) != 0)
		return -1;

	calibrate_data(mpu);

	return data_fusion(mpu);
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
    
