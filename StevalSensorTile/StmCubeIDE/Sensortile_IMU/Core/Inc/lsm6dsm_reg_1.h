/*
 * lsm6dsm_reg.h
 *
 *  Created on: Nov 26 2021
 *      Author: Felbermayr Simon
 */

#ifndef INC_LSM6DSM_REG_H_
#define INC_LSM6DSM_REG_H_

typedef enum {
        // =0x00 reserved

        LSM6DS3H_REG_FUNC_CFG_ACCESS               = 0x01,

        // =0x02-=0x03 reserved

        LSM6DS3H_REG_SENSOR_SYNC_TIME_FRAME        =0x04,

		LSM6DS3H_REG_SENSOR_SYNC_RES_RATIO         =0x05,

       LSM6DS3H_REG_FIFO_CTRL1                    =0x06,
        LSM6DS3H_REG_FIFO_CTRL2                    =0x07,
        LSM6DS3H_REG_FIFO_CTRL3                    =0x08,
        LSM6DS3H_REG_FIFO_CTRL4                    =0x09,
        LSM6DS3H_REG_FIFO_CTRL5                    =0x0a,

        LSM6DS3H_REG_DRDY_PULSE_CFG                 =0x0b,

        // =0x0c reserved

        LSM6DS3H_REG_INT1_CTRL                     =0x0d,
        LSM6DS3H_REG_INT2_CTRL                     =0x0e,

        LSM6DS3H_REG_WHO_AM_I                      =0x0f,

        LSM6DS3H_REG_CTRL1_XL                      =0x10,
        LSM6DS3H_REG_CTRL2_G                       =0x11,
        LSM6DS3H_REG_CTRL3_C                       =0x12,
        LSM6DS3H_REG_CTRL4_C                       =0x13,
        LSM6DS3H_REG_CTRL5_C                       =0x14,
        LSM6DS3H_REG_CTRL6_C                       =0x15,
        LSM6DS3H_REG_CTRL7_G                       =0x16,
        LSM6DS3H_REG_CTRL8_XL                      =0x17,
        LSM6DS3H_REG_CTRL9_XL                      =0x18,
        LSM6DS3H_REG_CTRL10_C                      =0x19,

        LSM6DS3H_REG_MASTER_CFG                    =0x1a,
        LSM6DS3H_REG_WAKE_UP_SRC                   =0x1b,
        LSM6DS3H_REG_TAP_SRC                       =0x1c,
        LSM6DS3H_REG_TAP_D6D                       =0x1d,

        // also STATUS_SPIAux
        LSM6DS3H_REG_STATUS                        =0x1e,

        // =0x1f reserved

        LSM6DS3H_REG_OUT_TEMP_L                    =0x20,
       LSM6DS3H_REG_OUT_TEMP_H                    =0x21,

        LSM6DS3H_REG_OUTX_L_G                      =0x22,
        LSM6DS3H_REG_OUTX_H_G                      =0x23,
        LSM6DS3H_REG_OUTY_L_G                      =0x24,
        LSM6DS3H_REG_OUTY_H_G                      =0x25,
        LSM6DS3H_REG_OUTZ_L_G                      =0x26,
        LSM6DS3H_REG_OUTZ_H_G                      =0x27,

        LSM6DS3H_REG_OUTX_L_XL                     =0x28,
        LSM6DS3H_REG_OUTX_H_XL                     =0x29,
        LSM6DS3H_REG_OUTY_L_XL                     =0x2a,
        LSM6DS3H_REG_OUTY_H_XL                     =0x2b,
        LSM6DS3H_REG_OUTZ_L_XL                     =0x2c,
        LSM6DS3H_REG_OUTZ_H_XL                     =0x2d,

        LSM6DS3H_REG_SENSORHUB1_REG                =0x2e,
        LSM6DS3H_REG_SENSORHUB2_REG                =0x2f,
        LSM6DS3H_REG_SENSORHUB3_REG                =0x30,
        LSM6DS3H_REG_SENSORHUB4_REG                =0x31,
        LSM6DS3H_REG_SENSORHUB5_REG                =0x32,
        LSM6DS3H_REG_SENSORHUB6_REG                =0x33,
        LSM6DS3H_REG_SENSORHUB7_REG                =0x34,
        LSM6DS3H_REG_SENSORHUB8_REG                =0x35,
        LSM6DS3H_REG_SENSORHUB9_REG                =0x36,
        LSM6DS3H_REG_SENSORHUB10_REG               =0x37,
 		LSM6DS3H_REG_SENSORHUB11_REG               =0x38,
        LSM6DS3H_REG_SENSORHUB12_REG               =0x39,

        LSM6DS3H_REG_FIFO_STATUS1                  =0x3a,
        LSM6DS3H_REG_FIFO_STATUS2                  =0x3b,
        LSM6DS3H_REG_FIFO_STATUS3                  =0x3c,
        LSM6DS3H_REG_FIFO_STATUS4                  =0x3d,

        LSM6DS3H_REG_FIFO_DATA_OUT_L               =0x3e,
        LSM6DS3H_REG_FIFO_DATA_OUT_H               =0x3f,

        LSM6DS3H_REG_TIMESTAMP0_REG                =0x40,
        LSM6DS3H_REG_TIMESTAMP1_REG                =0x41,
        LSM6DS3H_REG_TIMESTAMP2_REG                =0x42,

        // =0x43-=0x48 reserved

        LSM6DS3H_REG_STEP_TIMESTAMP_L              =0x49,
        LSM6DS3H_REG_STEP_TIMESTAMP_H              =0x4a,

        LSM6DS3H_REG_STEP_COUNTER_L                =0x4b,
        LSM6DS3H_REG_STEP_COUNTER_H                =0x4c,

        LSM6DS3H_REG_SENSORHUB13_REG               =0x4d,
        LSM6DS3H_REG_SENSORHUB14_REG               =0x4e,
        LSM6DS3H_REG_SENSORHUB15_REG               =0x4f,
        LSM6DS3H_REG_SENSORHUB16_REG               =0x50,
        LSM6DS3H_REG_SENSORHUB17_REG               =0x51,
        LSM6DS3H_REG_SENSORHUB18_REG               =0x52,

        LSM6DS3H_REG_FUNC_SRC1                     =0x53,
		LSM6DS3H_REG_FUNC_SRC2                     =0x54,
		LSM6DS3H_REG_WRIST_TILT_IA                 =0x55,

        // =0x56-=0x57 reserved

        LSM6DS3H_REG_TAP_CFG                       =0x58,
        LSM6DS3H_REG_TAP_THS_6D                    =0x59,

        // where is int_dur1?
        LSM6DS3H_REG_INT_DUR2                      =0x5a,

        LSM6DS3H_REG_WAKE_UP_THS                   =0x5b,
        LSM6DS3H_REG_WAKE_UP_DUR                   =0x5c,

        LSM6DS3H_REG_FREE_FALL                     =0x5d,

       LSM6DS3H_REG_MD1_CFG                       =0x5e,
        LSM6DS3H_REG_MD2_CFG                       =0x5f,
		LSM6DS3H_REG_MASTER_CMD_CODE			   =0x60,
		LSM6DS3H_REG_SENS_SYNC_SPI_ERROR_CODE	   =0x61,

        // =0x62-=0x65 reserved

        LSM6DS3H_REG_OUT_MAG_RAW_X_L               =0x66,
        LSM6DS3H_REG_OUT_MAG_RAW_X_H               =0x67,
        LSM6DS3H_REG_OUT_MAG_RAW_Y_L               =0x68,
        LSM6DS3H_REG_OUT_MAG_RAW_Y_H               =0x69,
        LSM6DS3H_REG_OUT_MAG_RAW_Z_L               =0x6a,
        LSM6DS3H_REG_OUT_MAG_RAW_Z_H               =0x6b,

        // =0x6c-=0x6e assume reserved but not listed in DS

		LSM6DS3H_REG_INT_OIS						=0x6f,
		LSM6DS3H_REG_CTRL1_OIS						=0x70,
		LSM6DS3H_REG_CTRL2_OIS						=0x71,
		LSM6DS3H_REG_CTRL3_OIS						=0x72,
		LSM6DS3H_REG_X_OFS_USR						=0x73,
		LSM6DS3H_REG_Y_OFS_USR						=0x73,
		LSM6DS3H_REG_Z_OFS_USR						=0x73

		// =0x76-=0x7f reserved6
} LSM6DS3H_REGS_T;

typedef enum {
	BOOT 		= 0x80,
	BDU  		= 0x40,
	H_LACTIVE 	= 0x20,
	PP_OD		= 0x10,
	SIM			= 0x08,
	IF_INC		= 0x04,
	BLE			= 0x02,
	SW_RESET	= 0x01
}CTRL3_C_REG;

typedef enum {
	ODR_XL_3 		= 0x80,
	ODR_XL_2  		= 0x40,
	ODR_XL_1 		= 0x20,
	ODR_XL_0		= 0x10,
	FS_XL_1			= 0x08,
	FS_XL_0			= 0x04,
	LPF1_BW_SEL		= 0x02,
	BW0_XL			= 0x01
}CTRL1_XL;

typedef enum {
	ODR_6660Hz 		= 0xa0,
	ODR_3330Hz  	= 0x90,
	ODR_1660Hz 		= 0x80,
	ODR_833Hz		= 0x70,
}ODR_ACCEL;

typedef enum {
	FS_2g 		= 0x00,
	FS_4g  		= 0x08,
	FS_16g 		= 0x04,
	FS_8g		= 0x0c,
}FS_ACCEL;

typedef enum {
	DRDY_PULSE		= 0x80,
	DRDY_LATCHED	= 0x00,
}DRDY_PULSE_CFG;





#endif /* INC_LSM6DSM_REG_H_ */
