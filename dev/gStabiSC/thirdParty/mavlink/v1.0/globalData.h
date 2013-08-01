#ifndef  GLOBALDATA_H 
#define  GLOBALDATA_H

#include <math.h>

#define THROTTLE 2
#define ELEVATOR 1
#define AILERON 0
#define RUDDER 3
#define GEAR 4
#define VPP 5
#define AUX1   5
#define AUX2   6
#define AUX3   8
#define AUX4   7
#define PI  3.141592654
#define GRAVITY 4166
#define SYSTEM_ID 10
#define MAV_RX_BUFF_SIZE 100
#define ONBOARD_PARAM_COUNT    57
#define ONBOARD_PARAM_NAME_LENGTH 16

					 
 


  enum gMAV_ENUM
  {gMAV_SEND_MSG,
   gMAV_SEND_PARAM,
   gMAV_SEND_NONE,
  };
enum
{ 
	PARAM_VERSION,
	PARAM_SERIAL_NUMBER,
	PARAM_PITCH_P,
	PARAM_PITCH_I,
	PARAM_PITCH_D,
	
	PARAM_ROLL_P,
	PARAM_ROLL_I,
	PARAM_ROLL_D,

	PARAM_YAW_P,
	PARAM_YAW_I,
	PARAM_YAW_D,

	PARAM_PITCH_POWER,
	PARAM_ROLL_POWER,
	PARAM_YAW_POWER,

	PARAM_PITCH_FOLLOW,
	PARAM_ROLL_FOLLOW,
	PARAM_YAW_FOLLOW,

	PARAM_PITCH_FILTER,
	PARAM_ROLL_FILTER,
	PARAM_YAW_FILTER,
	PARAM_GYRO_TRUST,

	PARAM_NPOLES_PITCH,
	PARAM_NPOLES_ROLL,
	PARAM_NPOLES_YAW,

	PARAM_DIR_MOTOR_PITCH,
	PARAM_DIR_MOTOR_ROLL,
	PARAM_DIR_MOTOR_YAW,
	PARAM_MOTOR_FREQ,
	PARAM_RADIO_TYPE,
	PARAM_GYRO_LPF,
	PARAM_TRAVEL_MIN_PITCH,
	PARAM_TRAVEL_MAX_PITCH,
	PARAM_TRAVEL_MIN_ROLL,
	PARAM_TRAVEL_MAX_ROLL,
	PARAM_TRAVEL_MIN_YAW,
	PARAM_TRAVEL_MAX_YAW,

	PARAM_RC_PITCH_LPF,
	PARAM_RC_ROLL_LPF,
	PARAM_RC_YAW_LPF,
	PARAM_SBUS_PITCH_CHAN,
	PARAM_SBUS_ROLL_CHAN,
	PARAM_SBUS_YAW_CHAN,
	PARAM_SBUS_MODE_CHAN,
	PARAM_ACCX_OFFSET,
	PARAM_ACCY_OFFSET,
	PARAM_ACCZ_OFFSET,
	PARAM_GYROX_OFFSET,
	PARAM_GYROY_OFFSET,
	PARAM_GYROZ_OFFSET,
	PARAM_USE_GPS,
	PARAM_SKIP_GYRO_CALIB,
	PARAM_RC_PITCH_TRIM,
	PARAM_RC_ROLL_TRIM,
	PARAM_RC_YAW_TRIM,
	PARAM_RC_PITCH_MODE,
	PARAM_RC_ROLL_MODE,
	PARAM_RC_YAW_MODE,

};
enum
{ gMODE_NORMAL,
  gMODE_FOLLOW_PAN,
  gMODE_FOLLOW_PAN_TILT,

}  ; 
enum
{ gSTATE_CALIB_ACC,
  gSTATE_CALIB_GYRO,
  gSTATE_SENSOR_ERROR,
  gSTATE_NORMAL,
} ;
enum
{ I2C_DMA_RX_WAITING,
  I2C_BUS_AVAILABLE,
  }	;
enum
{ CONFIG_RADIO_TYPE_PPM=0,
  CONFIG_RADIO_TYPE_SBUS=1,
};
enum
{ CONFIG_MOTOR_FREQ_36KHZ=0,
  CONFIG_MOTOR_FREQ_18KHZ=1,
  CONFIG_MOTOR_FREQ_9KHZ=2,
};
enum
{ CONFIG_MOTOR_DIR_NORMAL=1,
  CONFIG_MOTOR_DIR_REVERSE=0,
};
enum
{ CONFIG_SKIP_GYRO_CALIB=1,
  CONFIG_NOT_SKIP_GYRO_CALIB=0,
};
enum
{ CONFIG_RC_MODE_ANGLE=0,
  CONFIG_RC_MODE_SPEED=1,
};
typedef struct __global_struct
{
    float param[ONBOARD_PARAM_COUNT];
    char * param_name[ONBOARD_PARAM_COUNT]; 
    uint16_t parameter_i; // parameter index

}global_struct;

typedef struct __gConfig
{ uint8_t version;
 uint16_t serialNumber;
  float pitchKp;
  float pitchKi;
  float pitchKd;
  float rollKp;
  float rollKi;
  float rollKd;
  float yawKp;
  float yawKi;
  float yawKd;
  float pitchPower;
  float rollPower;
  float yawPower;
  float gyroTrust;
  float pitchFollow;
  float rollFollow;
  float yawFollow;
  float tiltFilter;
  float rollFilter;
  float panFilter;
  uint8_t motorFreq;
  uint8_t radioType;
  int8_t  dirMotorPitch;
  int8_t  dirMotorRoll;
  int8_t  dirMotorYaw;
  uint8_t nPolesPitch;
  uint8_t nPolesRoll;
  uint8_t nPolesYaw;
 
  uint8_t 	gyroLPF;
  
  
  int16_t travelMinRoll;
  int16_t travelMaxRoll;
  int16_t travelMinPitch;
  int16_t travelMaxPitch;
  int16_t travelMinYaw;
  int16_t travelMaxYaw;
  uint8_t 	rcPitchLPF;
  uint8_t 	rcRollLPF;
  uint8_t   rcYawLPF;
  uint8_t sbusPitchChan;
  uint8_t sbusRollChan;
  uint8_t sbusYawChan;
  uint8_t sbusModeChan;
  int16_t accXOffset;
  int16_t accYOffset;
  int16_t accZOffset;
  int16_t gyroXOffset;
  int16_t gyroYOffset;
  int16_t gyroZOffset;
  uint8_t useGPS;
  uint8_t skipGyroCalib;   //calibration gyro at startup or not
  int16_t rcPitchTrim;
  int16_t rcRollTrim;
  int16_t rcYawTrim;
  uint8_t rcPitchMode;
  uint8_t rcRollMode;
  uint8_t rcYawMode;
}	gConfig_t;

typedef struct __gSensor
{ 
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t accXOffset;
  int16_t accYOffset;
  int16_t accZOffset;
  int16_t gyroXOffset;
  int16_t gyroYOffset;
  int16_t gyroZOffset;
  uint8_t newGyroData;
  uint8_t newAccelData;
  uint8_t gyroCalibrated;
  uint8_t INT;
  uint8_t i2cBus;
  uint8_t i2cError;
 

}  gSensor_t;
 
 typedef struct __gAtti
{ float	theta;
  float phi;
  float psi;
  float thetaDot;
  float phiDot;
  float psiDot;
  float time;
}	gAtti_t;
 

typedef struct __gMotor
{ float pitchPos;
  
  int32_t pitchDamp;
  int32_t pitchDrive;
  float pitchPosLock;
  
  float rollPos;
  int32_t rollDamp;
  int32_t rollDrive;
  float rollPosLock;
 
  float yawPos;
  int32_t yawDamp;
  int32_t yawDrive;
  float yawPosLock;
  uint8_t enable;
  uint8_t update;
  uint8_t yawFollow;
  uint8_t pitchFollow;
  uint8_t rollFollow;
}	 gMotor_t;

typedef struct __gRadio
{ int16_t pitch;
  int16_t roll;
  int16_t yaw;
  int16_t mode;
  uint8_t noSbusCount;
  int16_t sbusChan[18];
  uint8_t update;

}	gRadio_t;

typedef struct __gState
{ uint8_t calibAcc;
  uint8_t gyroCalibrated;
  uint8_t sensorError;
  uint8_t normal;
  uint8_t tick;					// tick =1 every 1ms
}	gState_t;

typedef struct __gMode
{ uint8_t normal;
  uint8_t followPan;
  uint8_t followPanTilt;
} gMode_t;

typedef struct __gMav
{ uint8_t sendType;
  uint8_t sendFinish;
  uint8_t msgIndex;
  uint8_t update;
} gMav_t ;

#endif	  
