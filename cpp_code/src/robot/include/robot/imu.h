

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
    extern "C" {
#endif
        #include "mpu9150.h"
        #include "linux_glue.h"
        #include "local_defaults.h"

#ifdef __cplusplus
    }
#endif


class IMU
{
	public:
		/*** VARIABLES ***/

		
		/*** FUNCTIONS ***/
		void initIMU(void);
		void calcGyro(void);

	private:
		/*** VARIABLES ***/

		int verbose = 0;
		mpudata_t mpu;
		int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
		
		/*** FUNCTIONS ***/
		void printGyro(mpudata_t *mpu);

};
#endif
