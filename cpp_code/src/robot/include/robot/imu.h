

#ifndef IMU_H
#define IMU_H

typedef struct {
	short rawGyro[3];
	short rawAccel[3];
	long rawQuat[4];
	unsigned long dmpTimestamp;

	short rawMag[3];
	unsigned long magTimestamp;

	short calibratedAccel[3];
	short calibratedMag[3];

	quaternion_t fusedQuat;
	vector3d_t fusedEuler;

	float lastDMPYaw;
	float lastYaw;
} mpudata_t;



#endif
