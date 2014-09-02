/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
    This file is part of VRGimbal by VirtualRobotix Italia s.c.a.r.l..

    VRGimbal is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VRGimbal is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with VRGimbal.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * VRGimbal code is based on the project:
 *        Brushless Gimbal Controller Software by Christian Winkler and Alois Hahn (C) 2013
 *        and
 *        APM:Copter HTTP:///www.ardupilot.com
 *
 * 	Project lead developer and creator: Matteo Murtas - LaserNavigation s.r.l.
 *
 * 	Developers:
 * 	Roberto Navoni
 * 	Emile Castelnuovo
 *
 * 	Please refer to http://vrgimbal.wordpress.com for more information
 *
 * 	Thanks to:
 * 	LeafLabs (www.leaflabs.com) for Wirish Libraries
 * 	APM dev team
 *
*/


//#define DEBUG_IMU_DELAY

/*************************/
/* Include Header Files  */
/*************************/

#include <main.h>
#include "EEPROMAnything.h"
#include "LowPassFilter2p.h"

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
AP_Param param_loader; //(var_info, BOARD_EEPROM_SIZE, &EEPROM);

FastSerial  SerialDBG;
#ifdef GIMBAL_ENABLE_USB
USBSerial SerialUSB;
#endif
FastSerial *cliSerial = &SerialDBG;

HardwareSPI spi2(2);
HardwareI2C i2c2(2);
SerialCommand sCmd(cliSerial);     // Create SerialCommand object

#ifdef GIMBAL_ENABLE_COMPASS
Compass_HMC5843      	compass(&i2c2, cliSerial);
#endif
#if defined ( IMU_BRUGI )
MPU6050 mpu(&i2c2);            // Create MPU object
MPU6050 mpu_yaw(&i2c2);        // Create MPU object
bool mpu_yaw_present = false;


LowPassFilter2p * driveLPF[3] = {NULL, NULL, NULL};

void updateDriveLPF()
{
	float sampleFreq = 1.0f / DT_FLOAT;
	for (int i = 0; i < 3; i++)
	{
		float cutFreq = (float) config.profiles[0].axisConfig[i].stepsLimit;
		if (driveLPF[i] == NULL)
		{
			driveLPF[i] = new LowPassFilter2p(sampleFreq, cutFreq);
		} else {
			driveLPF[i]->set_cutoff_frequency(sampleFreq, cutFreq);
		}
	}
}

void initIMU_LPF()
{
	initMPUlpf(&mpu);
	if (mpu_yaw_present)
		initMPUlpf(&mpu_yaw);

	//float tauIMU2 = (float) config.profiles[0].mpu2LPF / 1e4;  //decimi di millisec
	setIMU2LPF(); //tauIMU2);
}

void initMPUlpf(MPU6050 * p_mpu) {
  // Set Gyro Low Pass Filter(0..6, 0=fastest, 6=slowest)
  switch (config.profiles[0].mpuLPF) {
    case 0:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_256);  break;
    case 1:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_188);  break;
    case 2:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_98);   break;
    case 3:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_42);   break;
    case 4:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_20);   break;
    case 5:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_10);   break;
    case 6:  p_mpu->setDLPFMode(MPU6050_DLPF_BW_5);    break;
    default: p_mpu->setDLPFMode(MPU6050_DLPF_BW_256);  break;
  }
}
#endif

#if defined ( IMU_AP )


#ifdef GIMBAL_ENABLE_COMPASS
AP_Compass_HMC5843      	compass(&i2c2, cliSerial);
#endif

Arduino_Mega_ISR_Registry 	isr_registry;
AP_TimerProcess  scheduler;
static GPS							*g_gps_null = NULL;
#ifdef GIMBAL_IMU_SPI
AP_InertialSensor_MPU6000 	ins(  &spi1,BOARD_CS_IMU_SPI, false, BOARD_INT_IMU_SPI, &Serial);
#else
AP_InertialSensor_MPU6000_I2C	ins(&i2c2, BOARD_INT_IMU, cliSerial, MPU6050_ADDRESS_AD0_LOW);
#endif
AP_AHRS_DCM					ahrs(&ins, g_gps_null);





void initMPUlpf() {

}
#endif


#ifdef GIMBAL_ENABLE_RC
#ifdef RC_USE_LIBS
APM_RC_MP32V3 				APM_RC;
#endif
#endif

/**********************************************/
/* Initialization                             */
/**********************************************/
void clearEEPROM()
{
	int ee = BRUGI_CONFIG_EEPROM_POS;
    unsigned int i;
    char b = 0;
    for (i = BRUGI_CONFIG_EEPROM_POS; i < EEPROM_PAGE_SIZE; i++)
    {
          EEPROM.write(ee++, b);
    }

}

bool read_config()
{
	//EEPROM_readAnything(BRUGI_CONFIG_EEPROM_POS, config);
	byte crc_read = 0;
	EEPROM_readAnything(BRUGI_CONFIG_EEPROM_POS, config, &crc_read);

	crc calculated = crcSlow((crc *)&config, sizeof(config));
	cliSerial->printf(F("Save Config. CRC: %02x RD: %02x CALC: %02x  Size: %d\r\n"), 0, crc_read, calculated, sizeof(config));

	//if (config.crc8 == calculated)
	if (crc_read == calculated)
	{
		return true;
	} else {
		// crc failed intialize directly here, as readEEPROM is void
		cliSerial->println(F("EEPROM CRC failed, initialize EEPROM"));
		setDefaultParameters();
		write_config();
	}
	return false;
}
void write_config()
{
	//EEPROM_writeAnything(BRUGI_CONFIG_EEPROM_POS, config);

	bool old = g_accOutput;
	g_accOutput = false; // do not save enabled OAC output mode
	crc new_crc = crcSlow((crc *)&config, sizeof(config)); // set proper CRC

	cliSerial->printf(F("Save Config. CRC: %02x\r\n"), new_crc);

	clearEEPROM();

	EEPROM_writeAnything(BRUGI_CONFIG_EEPROM_POS, config, &new_crc);
	g_accOutput = old;

}

void dummy(void)
{}

void setup() 
{

	// just for debugging
	#ifdef STACKHEAPCHECK_ENABLE
	stackCheck();
	heapCheck();
	#endif

	LEDPIN_PINMODE
	LEDGREPIN_PINMODE

	LEDGREPIN_ON


	//CH2_PINMODE
	//CH3_PINMODE

	// Start Serial Port
	SerialDBG.configure(SERIAL_CLI_PORT);
	SerialDBG.begin(SERIAL_CLI_BAUD, 0x01, 0x07);

#ifdef GIMBAL_ENABLE_USB
	SerialUSB.begin();

	if (SerialUSB.isConnected())
		cliSerial = (FastSerial*)&SerialUSB;
#endif

	cliSerial->print("\r\n\r\nLASER NAVIGATION SRL\r\n\r\n");
	cliSerial->printf("\r\nInit %s|%s %s\r\n", THISFIRMWARE, __DATE__, __TIME__);
	cliSerial->printf("System Clock: %lu MHz\r\n", SystemCoreClock/1000000);

	// Set Serial Protocol Commands
	setSerialProtocol();

	// Start I2C and Configure Frequency
	cliSerial->print("Init I2C\r\n");
	i2c2.begin();


	cliSerial->print("Init SPI2\r\n");
	spi2.begin(SPI_1_125MHZ, MSBFIRST, 0);
	EEPROM.init(&spi2, BOARD_CS_EEPROM); //, true, 9, 512, 512);

#ifdef ENABLE_AP_PARAM

	param_loader.setup(var_info, BRUGI_CONFIG_EEPROM_POS, &EEPROM);
	param_loader.load_defaults();
	load_parameters();
#endif

  // Read Config, fill with default settings if versions do not match or CRC fails
	  read_config();
  //if ((config.vers != VERSION) || (config.versEEPROM != VERSION_EEPROM))
  if  (config.versEEPROM != VERSION_EEPROM)
  {
    cliSerial->print(F("EEPROM version mismatch, initialize EEPROM"));
    setDefaultParameters();
    write_config();
  }
  
  // Init Sinus Arrays and Motor Stuff
  //superfluo, lo faccio in initmotors
  //recalcMotorStuff();
  
  // Init PIDs to reduce floating point operations.
  initPIDs();
  updateDriveLPF();

#ifdef GIMBAL_ENABLE_RC
  // init RC variables
  initRC();
#endif

 
   // Init BL Controller
  //initBlController();
  // Initialize MPU 
  initResolutionDevider();
  
  // Init IMU variables
  initIMU();
  
#if defined ( IMU_BRUGI )


//  // Auto detect MPU address
//  mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
//  mpu.initialize();
//  if(mpu.testConnection()) {
//	  cliSerial->println(F("MPU6050 ok (HIGH)"));
//  } else {
//    mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
//    mpu.initialize();
//    if(mpu.testConnection()) {
//    	cliSerial->println(F("MPU6050 ok (LOW)"));
//    } else {
//    	cliSerial->println(F("MPU6050 failed"));
//    }
//  }

  bool bMPU_OK = false;
  bool bHighOk = false;
  bool bLowOk = false;
  mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
  mpu.initialize();
  if(mpu.testConnection())
	  bHighOk = true;

  mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
    mpu.initialize();
    if(mpu.testConnection())
  	  bLowOk = true;

    if (bHighOk && bLowOk)
    {
    	mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
    	mpu.initialize();

    	mpu_yaw.setAddr(MPU6050_ADDRESS_AD0_HIGH);
    	mpu_yaw.initialize();
    	mpu_yaw_present = true;

    	bMPU_OK = true;
    	cliSerial->println(F("MPU6050 ok (DOUBLE SENSOR)"));

    } else if (bHighOk) {
    	mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
    	mpu.initialize();
    	bMPU_OK = true;
    	cliSerial->println(F("MPU6050 ok (HIGH)"));
    } else if (bLowOk) {
		mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
		mpu.initialize();
		bMPU_OK = true;
		cliSerial->println(F("MPU6050 ok (LOW)"));
	} else {
		bMPU_OK = false;
		cliSerial->println(F("MPU6050 failed"));
	}

#endif

  //CH2_ON
  
  // set sensor orientation (from config)
  initSensorOrientation();
  
  gyroReadCalibration();
  gyroReadDeadBand();
  accReadCalibration();

#if defined ( IMU_BRUGI )
  // Init MPU Stuff
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G  //TODO: verificare quale scala funziona meglio
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);       //+- 8G
  initMPUlpf(&mpu);                                         // Set Gyro Low Pass Filter
  mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  mpu.setSleepEnabled(false); 
  mpu.setClockOutputEnabled(false); //TODO verificare (tratto da EvvGC)
  mpu.setIntEnabled(0);         //TODO verificare (tratto da EvvGC)
  
  //mpu.setIntDataReadyEnabled()

  if (config.recalibrateOnStartup)
  {
	  // Gyro Offset calibration
	  cliSerial->println(F("Gyro calibration: do not move"));
	  mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
	  gyroOffsetCalibration(&mpu, gyroOffset);
	  initMPUlpf(&mpu);
	  cliSerial->println(F("Gyro calibration: done"));
  }
  if (mpu_yaw_present)
  {
	mpu_yaw.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
	mpu_yaw.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
	mpu_yaw.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G  //TODO: verificare quale scala funziona meglio
	//mpu_yaw.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);       //+- 8G
	initMPUlpf(&mpu_yaw);                                         // Set Gyro Low Pass Filter
	mpu_yaw.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
	mpu_yaw.setSleepEnabled(false);
	  mpu_yaw.setClockOutputEnabled(false); //TODO verificare (tratto da EvvGC)
	  mpu_yaw.setIntEnabled(0);         //TODO verificare (tratto da EvvGC)

	  if (config.recalibrateOnStartup)
	  {
		// Gyro Offset calibration
		cliSerial->println(F("Second Gyro calibration: do not move"));
		mpu_yaw.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
		gyroOffsetCalibration(&mpu_yaw, gyroOffset2);
		initMPUlpf(&mpu_yaw);
		cliSerial->println(F("Second Gyro calibration: done"));
	  }
  }

#endif
  
#ifdef IMU_EVV
  cliSerial->println(F("Initing position: do not move."));
  EVV_Init_Orientation();
#endif

  LEDPIN_ON
  
   // Init BL Controller
  initBlController();

  if (config.profiles[0].axisConfig[axisYAW].mode != 0)
  {
	  cliSerial->println(F("Positioning Yaw"));
	  switchOffAndMoveToPosition(config.profiles[0].axisConfig[axisYAW].motorNumber , config.profiles[0].axisConfig[axisYAW].maxPWM, config.profiles[0].axisConfig[axisYAW].offsetMotor, 3000);
  }

  if (config.profiles[0].axisConfig[axisROLL].mode != 0)
  {
	  cliSerial->println(F("Positioning Roll"));
	  switchOffAndMoveToPosition(config.profiles[0].axisConfig[axisROLL].motorNumber , config.profiles[0].axisConfig[axisROLL].maxPWM, config.profiles[0].axisConfig[axisROLL].offsetMotor, 3000);
  }

  if (config.profiles[0].axisConfig[axisPITCH].mode != 0)
  {
	  cliSerial->println(F("Positioning Pitch"));
	  switchOffAndMoveToPosition(config.profiles[0].axisConfig[axisPITCH].motorNumber , config.profiles[0].axisConfig[axisPITCH].maxPWM, config.profiles[0].axisConfig[axisPITCH].offsetMotor, 3000);
  }

  // motorTest();

#ifdef GIMBAL_ENABLE_RC
  // Init RC-Input
  initRCPins();
#endif

#ifdef MANUAL_INPUT_COUNT
  initManualControllers();
#endif

  LEDGREPIN_OFF
  
  cliSerial->println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));

  //CH2_OFF
  //CH3_OFF

  //TEO 20130607
  if (bMPU_OK)
	  gimState = GIM_IDLE;
  else
	  gimState = GIM_ERROR;
  stateStart = millis();
 
}



int32_t filterMotorDrive(int32_t oldDrive, int32_t newDrive, int32_t limit)
{


	/*
	float n = (float) newDrive;
	float o = (float) oldDrive;
	//utilLP_float(&o, n, 1.0f / (float) (g_outputFilter + 1));

	float coeff = 1.0f / (float) (g_outputFilter + 1);

	 n = o * (1.0f-coeff) + n * coeff;

	return (int) n;*/




	/*
	int n = newDrive;
	if (limit > 0)
	{
		if (newDrive > (oldDrive + limit))
		{
			n = oldDrive + limit;
		} else if (newDrive < (oldDrive - limit))
		{
			n = oldDrive - limit;
		}
	}*/

//  int n = newDrive;
//	if (limit > 0)
//	{
//		//filtro low pass
//		n = (  oldDrive + limit * newDrive ) / limit;
//	}

	float n = newDrive;
	if (limit > 0)
	{
		float CUTOFF = (float) limit;
		float RC = 1.0f/(CUTOFF*2.0f*3.14f);
		float dt = DT_FLOAT; // 1/SAMPLE_RATE;
		float alpha = dt/(RC+dt);
		float n = (float) newDrive;
		float o = (float) oldDrive;
		n = o * (1.0f-alpha) + n * alpha;
	}
	return (int) n;
}


/************************/
/* PID Controller       */
/************************/

int32_t ComputePID(int32_t DTus, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd, int32_t * Ddelta1, int32_t * Ddelta2)
{
  int32_t error = setPoint - in;

  //normalize the error angle between -180..180 to avoid "multiple turn" correction, in particular on yaw
  error = error % 360000;
  if (error > 180000)
	  error = error - 360000;

  //INTEGRAL
  int32_t Ierr;
  Ierr = error * (Ki * DTus / 1000) / 1000; //Ki scaling
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);  //I term saturation (could be good add an anti-windup formula)
  *errorSum += Ierr;


  //DERIVATIVE
#ifdef GIMBAL_DTERM_LOWPASS
#ifdef pt1cut
    float dt = DTms / 1000;
#endif
    // D Term
    int32_t delta = error - *errorOld;
    *errorOld = error;
    int32_t deltaSum = *Ddelta1 + *Ddelta2 + delta;
    *Ddelta2   = *Ddelta1;
    *Ddelta1   = delta;
#ifdef pt1cut
    deltaSum        = LastDterm[axis] + (dt / (pt1cut + dt)) * (deltaSum - LastDterm[axis]); // pt1 element like shown by BRM here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=2624
    LastDterm[axis] = deltaSum;
#endif
    int32_t Dterm = 1000 * (deltaSum * Kd) / DTus;
#else
    int32_t Dterm = Kd * (error - *errorOld) * DTms;
#endif
    *errorOld = error;


  //Compute PID Output
  int32_t out = (Kp * error) + *errorSum + Dterm;


  return out / MOTOR_DRIVE_RESCALER;
}

int32_t ComputePIDF(int32_t DTms, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd, int32_t KdLPF, int32_t * DTermOld)
{
  int32_t error = setPoint - in;

  //normalize the error angle between -180..180 to avoid "multiple turn" correction, in particular on yaw
  error = error % 360000;
  if (error > 180000)
	  error = error - 360000;

  //INTEGRAL
  int32_t Ierr;
  Ierr = error * Ki * DTms / 1000; //Ki scaling
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);  //I term saturation (could be good add an anti-windup formula)
  *errorSum += Ierr;


  //DERIVATIVE
   // D Term
    int32_t delta = error - *errorOld;
    *errorOld = error;

    //Derivative filter
    int32_t DtermNew = (delta * Kd) / DTms;
    int32_t Dterm = filterMotorDrive(*DTermOld, DtermNew, KdLPF);
    *DTermOld = Dterm;


  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Dterm;


  return out / MOTOR_DRIVE_RESCALER;
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/

int getMotorDriveSet(uint8_t axis)
{
	switch (axis)
	{
	case axisROLL:
		return  (int) ( (rollAngleSet / 360.0f) * (float) config.profiles[0].axisConfig[axisROLL].stepsMotor ) + config.profiles[0].axisConfig[axisROLL].offsetMotor;
		break;
	case axisPITCH:
		return  (int) ( (pitchAngleSet / 360.0f) * (float) config.profiles[0].axisConfig[axisPITCH].stepsMotor ) + config.profiles[0].axisConfig[axisPITCH].offsetMotor;
		break;
	case axisYAW:
		return  (int) ( (yawAngleSet / 360.0f) * (float) config.profiles[0].axisConfig[axisYAW].stepsMotor ) + config.profiles[0].axisConfig[axisYAW].offsetMotor;
		break;
	}
	return 0;
}

uint32 superfast_loopTimer = 0;
uint32 fast_loopTimer = 0;
uint32 superfastloop_speed = 1; // SUPER_FAST_MICROSECONDS;
uint32 fastloop_speed = 1; // 5; //FAST_MICROSECONDS;


uint32 motor_loopTimer = 0;

#ifdef DEBUG_IMU_DELAY
//debug velocit di lettura i2c
uint32_t lap_gyros = 0;
uint32_t cnt_gyros = 0;
#endif


int32_t rollAngleSet_OLD = 0;
int32_t pitchAngleSet_OLD = 0;
int32_t yawAngleSet_OLD = 0;


void measure_loop(bool bAppend = true)
{
	//misuro intervallo tra le chiamate
	uint32_t unewloop = micros();
	static uint32_t ulast_motor_loop = 0;
	if (ulast_motor_loop != 0)
	{
		uint32_t lap2 = measure_micro_delay(ulast_motor_loop, unewloop);
		if (bAppend)
			loop_mean_lap.append((float) lap2);
	}
	ulast_motor_loop = unewloop;
}


realtimeStatistics statistic_pitch;
realtimeStatistics statistic_roll;
realtimeStatistics statistic_yaw;
bool bLedOn = false;

realtimeStatistics main_loop_duration[19];


void loop()
{

	int32_t pitchPIDVal;
	int32_t rollPIDVal;
	int32_t yawPIDVal;
	static int32_t pitchErrorSum;
	static int32_t rollErrorSum;
	static int32_t yawErrorSum;
	static int32_t pitchErrorOld;
	static int32_t rollErrorOld;
	static int32_t yawErrorOld;


	static int32_t pitchDelta1 = 0;
	static int32_t rollDelta1 = 0;
	static int32_t yawDelta1 = 0;
	static int32_t pitchDelta2 = 0;
	static int32_t rollDelta2 = 0;
	static int32_t yawDelta2 = 0;

	int deltaDrive[3] = {0, 0, 0};


	int yawDiffDrive = 0;
	float gyroDesiredYaw = 0;
	float diffDriveAngleYaw = 0;
	int driveSetYaw = 0;


	static char pOutCnt = 0;
	static uint32_t output_time = 0;



	uint32_t now = millis();
	static uint32_t ms_last_motor_update = 0;
	int32_t ms_lap = now - ms_last_motor_update;
//		float pid_lap = (float) ms_lap;

	uint32_t unow = micros();
	static uint32_t u_last_motor_update = 0;
	uint32_t pid_lap_us = measure_micro_delay( u_last_motor_update, unow);

	uint32_t pid_lap = ms_lap;

	bool bEnterPID = false;
	bool bOutputDebug = false;


	if ((now - output_time) > (1000 / POUT_FREQ))
	{
		output_time = now;
		bOutputDebug = true;

	}

	//misuro la durata dei loop ma non aggiungo alla statistica quelli con output su seriale
	measure_loop(!bOutputDebug);

	uint32_t unowlap = micros();
	uint32_t unowlaptot = unowlap;

	//if ( pid_lap >= DT_FLOAT) //DT_LOOP_MS)
	if ( pid_lap_us >= DT_LOOP_US)
	{

		main_loop_duration[18].append( measure_micro_delay(u_last_motor_update, unow) );


		u_last_motor_update = unow;
		ms_last_motor_update = now;
		bEnterPID = true;


		if (pid_lap > 10 * DT_LOOP_MS)
			pid_lap = DT_LOOP_MS;
		if (pid_lap_us > 10 * DT_LOOP_US)
			pid_lap_us = DT_LOOP_US;

		motor_loopTimer = now;
		motorUpdate = false;
    
		bool bGyroSaturated = readGyros();

		main_loop_duration[0].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();

		if (config.profiles[0].enableGyro) updateGyroAttitude();

		main_loop_duration[1].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();


		if (config.profiles[0].enableACC) updateACCAttitude();

		main_loop_duration[2].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();


		getAttiduteAngles();
    
		main_loop_duration[3].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();


		pid_lap_us = get_gyro_lap();

#ifdef GIMBAL_ENABLE_RC



		// Evaluate RC-Signals
		evaluateRC();
		//RC pitch
		if (config.profiles[0].rcConfig[axisPITCH].absolute) {
			utilLP_float(&pitchAngleSet, PitchPhiSet, rcLPF_tc[axisPITCH]);
		} else if (PitchResetting) {
			utilLP_float(&pitchAngleSet, 0, rcLPF_tc[axisPITCH]);
			PitchPhiSet = pitchAngleSet;
		} else {
			utilLP_float(&pitchAngleSet, PitchPhiSet, 0.01);
		}
		//RC roll
		if (config.profiles[0].rcConfig[axisROLL].absolute) {
			utilLP_float(&rollAngleSet, RollPhiSet,  rcLPF_tc[axisROLL]);
		} else if  (RollResetting) {
			utilLP_float(&rollAngleSet, 0, rcLPF_tc[axisROLL]);
			RollPhiSet = rollAngleSet;
		} else {
			utilLP_float(&rollAngleSet, RollPhiSet, 0.01f);
		}
		//RC yaw
		if (config.profiles[0].rcConfig[axisYAW].absolute) {
			utilLP_float(&yawAngleSet, YawPhiSet, rcLPF_tc[axisYAW]);
		} else if (YawResetting) {
			utilLP_float(&yawAngleSet, 0, rcLPF_tc[axisYAW]);
			YawPhiSet = yawAngleSet;
		} else {
			utilLP_float(&yawAngleSet, YawPhiSet, 0.01);
		}
		if (YawLocked)
		{
			//inganno la stabilizzazione
			yawAngleSet = angle[axisYAW] / 1000.0;
			YawPhiSet = yawAngleSet;
		}

		main_loop_duration[4].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();
#endif



		int new_pitchMotorDrive = pitchMotorDrive;
		int new_rollMotorDrive = rollMotorDrive;
		int new_yawMotorDrive = yawMotorDrive;


		//verifico la protezione da saturazione gyro
		if ((config.profiles[0].saturationLock > 0) && (bGyroSaturated))
		{
			if ((gimState == GIM_LOCKED) || (gimState == GIM_SATURATION))  //--> quando riattivo la stabilizzazione potrei ricevere delle piccole scosse
			{
				g_saturationLockStart = millis();
				enableMotorUpdates = false;
				LEDGREPIN_OFF
				setACCFastMode(true);
				gimState = GIM_SATURATION;
				stateStart = g_saturationLockStart;
			}
		}

		//****************************
		// pitch PID
		//****************************
		float gyroDesiredPitch = 0;
		if (config.profiles[0].axisConfig[axisPITCH].mode == 0)
		{
			pitchPIDVal = ComputePID(pid_lap_us, angle[axisPITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd, &pitchDelta1, &pitchDelta2);
			//pitchPIDVal = ComputePIDF(pid_lap, angle[axisPITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd, config.profiles[0].axisConfig[axisPITCH].stepsLimit,  &pitchDelta1);
			// motor control
			new_pitchMotorDrive = pitchPIDVal * config.profiles[0].axisConfig[axisPITCH].motorDirection;
		}
		else if (config.profiles[0].axisConfig[axisPITCH].mode == 1)
		{
			float dAngle = normalize_yaw( pitchAngleSet - pitchAngleSet_OLD );
			gyroDesiredPitch =  dAngle / ((float) pid_lap_us / 1000000.0f);  // /s

			pitchPIDVal = ComputePID(pid_lap_us,  (float) gyroADC[axisPITCH] / resolutionDevider, gyroDesiredPitch , &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd, &pitchDelta1, &pitchDelta2);

			// motor control
			int drive = pitchPIDVal * (pid_lap_us / 1000) * config.profiles[0].axisConfig[axisPITCH].motorDirection;
			new_pitchMotorDrive = pitchMotorDrive + drive;
			if (drive != 0)
				pitchAngleSet_OLD = normalize_yaw(pitchAngleSet_OLD + (gyroDesiredPitch * pid_lap_us / 1000000.0) );
		} else {
			new_pitchMotorDrive = (int) ((float) config.profiles[0].axisConfig[axisPITCH].stepsMotor * ((float)pitchAngleSet) / 360.0  + config.profiles[0].axisConfig[axisPITCH].offsetMotor);
		}

		//****************************
		// roll PID
		//****************************
		float gyroDesiredRoll= 0;
		if (config.profiles[0].axisConfig[axisROLL].mode == 0)
		{
			rollPIDVal = ComputePID(pid_lap_us, angle[axisROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd, &rollDelta1, &rollDelta2);
			// rollPIDVal = ComputePIDF(pid_lap, angle[axisROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd,config.profiles[0].axisConfig[axisROLL].stepsLimit, &rollDelta1);
			// motor control
			new_rollMotorDrive = rollPIDVal * config.profiles[0].axisConfig[axisROLL].motorDirection;
		}
		else if (config.profiles[0].axisConfig[axisROLL].mode == 1)
		{
			float dAngle = normalize_yaw( rollAngleSet - rollAngleSet_OLD );
			gyroDesiredRoll =  dAngle / ((float) pid_lap_us / 1000000.0);  // /s

			rollPIDVal = ComputePID(pid_lap_us,  (float) gyroADC[axisROLL] / resolutionDevider, gyroDesiredRoll , &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd, &rollDelta1, &rollDelta2);

			// motor control
			int drive = rollPIDVal * (pid_lap_us/1000) * config.profiles[0].axisConfig[axisROLL].motorDirection;
			new_rollMotorDrive = rollMotorDrive + drive;
			if (drive != 0)
				rollAngleSet_OLD = normalize_yaw(rollAngleSet_OLD + (gyroDesiredRoll * pid_lap_us / 1000000.0) );
		} else {
			new_rollMotorDrive = (int) ((float) config.profiles[0].axisConfig[axisROLL].stepsMotor * ((float)rollAngleSet ) / 360.0 + config.profiles[0].axisConfig[axisROLL].offsetMotor);
		}

		//****************************
		// yaw PID
		//****************************
		//gestisco il cambio di modo in runtime azzerando lo storico degli errori
		static int8_t modeYaw_old = 0;
		if (modeYaw_old != config.profiles[0].axisConfig[axisYAW].mode)
		{
			yawErrorOld = 0;
			yawErrorSum = 0;
			yawDelta1 = 0;
			yawDelta2 = 0;

			modeYaw_old = config.profiles[0].axisConfig[axisYAW].mode;
		}


		if (config.profiles[0].axisConfig[axisYAW].mode == 0)
		{
			yawPIDVal = ComputePID(pid_lap_us, angle[axisYAW], yawAngleSet * 1000 , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd, &yawDelta1, &yawDelta2);
			//yawPIDVal = ComputePIDF(pid_lap, angle[axisYAW], yawAngleSet * 1000 , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd,config.profiles[0].axisConfig[axisYAW].stepsLimit, &yawDelta1);

			int yawMotorDrive_OLD = yawMotorDrive;
			// motor control
			new_yawMotorDrive = yawPIDVal * config.profiles[0].axisConfig[axisYAW].motorDirection;

			yawDiffDrive = new_yawMotorDrive - yawMotorDrive_OLD;
			yawAngleSet_OLD = yawAngleSet;
		}
		else if (config.profiles[0].axisConfig[axisYAW].mode == 1)
		{
//			gyroDesiredYaw = yawAngleSet / 10;
//			yawPIDVal = ComputePID(pid_lap_us, 1000.0f * (float) gyroADC[axisYAW] / resolutionDevider, 1000.0f * gyroDesiredYaw , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd, &yawDelta1, &yawDelta2);
//			// motor control
//			int drive = yawPIDVal * config.profiles[0].axisConfig[axisYAW].motorDirection;
//			new_yawMotorDrive = yawMotorDrive + drive;
			yawPIDVal = ComputePIDF(pid_lap, angle[axisYAW], yawAngleSet * 1000 , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd,config.profiles[0].axisConfig[axisYAW].stepsLimit, &yawDelta1);

			int yawMotorDrive_OLD = yawMotorDrive;
			// motor control
			new_yawMotorDrive = yawPIDVal * config.profiles[0].axisConfig[axisYAW].motorDirection;

			yawDiffDrive = new_yawMotorDrive - yawMotorDrive_OLD;
			yawAngleSet_OLD = yawAngleSet;
		}
		else if ( (config.profiles[0].axisConfig[axisYAW].mode == 2) ) //|| (config.profiles[0].axisConfig[axisYAW].mode == 3))
		{

			driveSetYaw = getMotorDriveSet(axisYAW);//(int) ( (yawAngleSet / 360.0f) * (float) config.profiles[0].stepsMotorYaw ) + config.profiles[0].offsetMotorYaw;
			driveSetYaw = inormalize_val(driveSetYaw, config.profiles[0].axisConfig[axisYAW].stepsMotor);

			int diff_drive = driveSetYaw - yawMotorDrive;
			diff_drive = inormalize_val(diff_drive, config.profiles[0].axisConfig[axisYAW].stepsMotor);

			diffDriveAngleYaw = 360.0f * (float) diff_drive / (float) config.profiles[0].axisConfig[axisYAW].stepsMotor;


			int driveLimit1 = ((float) config.profiles[0].axisConfig[axisYAW].driveLimit1Angle / 360.0f) * config.profiles[0].axisConfig[axisYAW].stepsMotor;
			int driveLimit2 = ((float) config.profiles[0].axisConfig[axisYAW].driveLimit2Angle / 360.0f) * config.profiles[0].axisConfig[axisYAW].stepsMotor;
			float maxGyro = (float) config.profiles[0].axisConfig[axisYAW].maxGyroDrive;  //in realt sarebbe da legare a rcGain e rcLPF



			//3. formula lineare con tolleranza intorno allo zero
			//
			//    	           maxGyro
			//    	\    |    /
			//    	 \   |   /
			//    	  \--+--/
			//    	     0 L1 L2

			//if (config.profiles[0].axisConfig[axisYAW].mode == 3)
			//{
				if (diff_drive > driveLimit2)
					gyroDesiredYaw = maxGyro ;
				else if (diff_drive > driveLimit1)
					gyroDesiredYaw = maxGyro * (float)(diff_drive - driveLimit1) / (float) (driveLimit2 - driveLimit1);
				else if (diff_drive < -driveLimit2)
					gyroDesiredYaw = - maxGyro;
				else if (diff_drive < -driveLimit1)
					gyroDesiredYaw = - maxGyro * (float)(abs(diff_drive) - driveLimit1) / (float) (driveLimit2 - driveLimit1);
				else
					gyroDesiredYaw = 0;
			//}


			//TODO: TEO: verificare quale moltiplicazione di config.profiles[0].dirMotorYaw *  genera l'inverzione di segno
			gyroDesiredYaw = config.profiles[0].axisConfig[axisYAW].motorDirection * gyroDesiredYaw;

			yawPIDVal = ComputePID(pid_lap_us,  (float) gyroADC[axisYAW] / resolutionDevider, gyroDesiredYaw , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd, &yawDelta1, &yawDelta2);
			// motor control
			int drive = yawPIDVal * (pid_lap_us/1000) * config.profiles[0].axisConfig[axisYAW].motorDirection;

//				//TEO 20130917
//				//anzich pilotare la velocit angolare provo  settare l'angolo obiettivo come per la stabilize
//				float angle_to_reach = angle[axisYAW] + gyroDesiredYaw * pid_lap_us / 1000;
//				yawPIDVal = ComputePID(pid_lap_us,  angle[axisYAW], angle_to_reach , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd, &yawDelta1, &yawDelta2);
//				// motor control
//				int drive = yawPIDVal * config.profiles[0].dirMotorYaw;


			if (config.profiles[0].axisConfig[axisYAW].mode == 3)
			{
				if ((abs(diff_drive) < driveLimit1) && (yawAngleSet == YawPhiSet))
				{
					if (fabs((float) gyroADC[axisYAW] / resolutionDevider) < maxGyro)
					{
						//ho gi raggiunto l'obiettivo: tengo fermo
						drive = 0;
					} else {
						//applico la stabilizzazione perch sto ruotando troppo veloce (quindi qualcuno sta ruotando bruscamente il supporto)
					}
				}
			}

			new_yawMotorDrive = yawMotorDrive + drive;
			new_yawMotorDrive = inormalize_val(new_yawMotorDrive, config.profiles[0].axisConfig[axisYAW].stepsMotor);

		}
		else if ( config.profiles[0].axisConfig[axisYAW].mode == 3)
		{
			driveSetYaw = getMotorDriveSet(axisYAW);//(int) ( (yawAngleSet / 360.0f) * (float) config.profiles[0].stepsMotorYaw ) + config.profiles[0].offsetMotorYaw;
			driveSetYaw = inormalize_val(driveSetYaw, config.profiles[0].axisConfig[axisYAW].stepsMotor);

			int diff_drive = driveSetYaw - yawMotorDrive;
			diff_drive = inormalize_val(diff_drive, config.profiles[0].axisConfig[axisYAW].stepsMotor);

			diffDriveAngleYaw = 360.0f * (float) diff_drive / (float) config.profiles[0].axisConfig[axisYAW].stepsMotor;

			float yaw_target = angle[axisYAW] + diffDriveAngleYaw * 1000;

			yawPIDVal = ComputePID(pid_lap_us, angle[axisYAW], yaw_target , &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd, &yawDelta1, &yawDelta2);

			// motor control
			new_yawMotorDrive = yawPIDVal * config.profiles[0].axisConfig[axisYAW].motorDirection;

		} else {
			//new_yawMotorDrive = getMotorDriveSet(axisYAW);
			new_yawMotorDrive = yawMotorDrive + config.profiles[0].axisConfig[axisYAW].motorDirection * config.profiles[0].axisConfig[axisYAW].Kp / 1000;
		}

		//aggiorno i valori per il motor interrupt
		pitchMotorDrive_PREV = pitchMotorDrive;
		//pitchMotorDrive = new_pitchMotorDrive;
		//pitchMotorDrive = filterMotorDrive(pitchMotorDrive, new_pitchMotorDrive, config.profiles[0].axisConfig[axisPITCH].stepsLimit);
		pitchMotorDrive = (int32_t) driveLPF[axisPITCH]->apply((float) new_pitchMotorDrive);
		deltaDrive[axisPITCH] = pitchMotorDrive - pitchMotorDrive_PREV;
		pitchMotorDrive_INT_step = deltaDrive[axisPITCH] / LOOPUPDATE_FACTOR;
		statistic_pitch.append((float) deltaDrive[axisPITCH]);
//		if ((g_driveAlert[axisPITCH] > 0) && (abs(deltaDrive[axisPITCH]) > g_driveAlert[axisPITCH]))
//		{
//			cliSerial->print("DRV!! ");cliSerial->print(deltaDrive[axisPITCH]);
//			cliSerial->print(" P ");cliSerial->print(angle[axisPITCH]); cliSerial->println();
//		}

		rollMotorDrive_PREV = rollMotorDrive;
		//rollMotorDrive = new_rollMotorDrive;
		//rollMotorDrive = filterMotorDrive(rollMotorDrive, new_rollMotorDrive, config.profiles[0].axisConfig[axisROLL].stepsLimit);
		rollMotorDrive = (int32_t) driveLPF[axisROLL]->apply((float) new_rollMotorDrive);
		deltaDrive[axisROLL] = rollMotorDrive - rollMotorDrive_PREV;
		rollMotorDrive_INT_step = deltaDrive[axisROLL] / LOOPUPDATE_FACTOR;
		statistic_roll.append((float) deltaDrive[axisROLL]);
//		if ((g_driveAlert[axisROLL] > 0) && (abs(deltaDrive[axisROLL]) > g_driveAlert[axisROLL]))
//		{
//			cliSerial->print("DRV!! ");cliSerial->print(deltaDrive[axisROLL]);
//			cliSerial->print(" R ");cliSerial->print(angle[axisROLL]); cliSerial->println();
//		}

		yawMotorDrive_PREV = yawMotorDrive;
		//yawMotorDrive = new_yawMotorDrive;
		//yawMotorDrive = filterMotorDrive(yawMotorDrive, new_yawMotorDrive, config.profiles[0].axisConfig[axisYAW].stepsLimit);
		yawMotorDrive = (int32_t) driveLPF[axisYAW]->apply((float) new_yawMotorDrive);
		deltaDrive[axisYAW] = yawMotorDrive - yawMotorDrive_PREV;

		yawMotorDrive_INT_step = deltaDrive[axisYAW] / LOOPUPDATE_FACTOR;
		statistic_yaw.append((float) deltaDrive[axisYAW]);
//		if ((g_driveAlert[axisYAW] > 0) && (abs(deltaDrive[axisYAW]) > g_driveAlert[axisYAW]))
//		{
//			cliSerial->print("DRV!! ");cliSerial->print(deltaDrive[axisYAW]);
//			cliSerial->print(" Y ");cliSerial->print(angle[axisYAW]); cliSerial->println();
//		}
		motor_update_values = true;

		//misuro durata funzione
		main_loop_duration[5].append( measure_micro_delay(unowlap, micros()) );
		unowlap = micros();

	}


#ifndef BRUGI_USE_INTERRUPT_TIMER
	//attuazione (spostata qui sotto in modo che sia più immediata e allineata con i LAP calcolati qui sopra)
	motorInterrupt();

	//stampo debug per analisi funzione di trasferimento
	if (g_bTest[1])
	{
		int a = -1;
		int gyro = gyroADC2[axisYAW];
		int gyroL = (int) gyroADC2_lfp[axisYAW];
		int drv = yawMotorDrive;
		if (g_driveAlert[axisROLL] > 0)
		{
			a = axisROLL;
			gyro = gyroADC[a];
			gyroL = 0;
			drv = rollMotorDrive;
		}
		else if (g_driveAlert[axisPITCH] > 0)
		{
			a = axisPITCH;
			gyro = gyroADC[a];
			gyroL = 0;
			drv = pitchMotorDrive;
		}
		else if (g_driveAlert[axisYAW] > 0)
		{
			a = axisYAW;
			gyro = gyroADC[a];
			gyroL = 0;
			drv = yawMotorDrive;
		}

		cliSerial->print(F("# "));
		cliSerial->print(gyro);cliSerial->print(F(" "));
		cliSerial->print(drv);cliSerial->print(F(" "));
		cliSerial->print(gyroL);
		cliSerial->println();
	}

#endif

	main_loop_duration[6].append( measure_micro_delay(unowlap, micros()) );
	unowlap = micros();

	//****************************
	// debug and telemetry outputs
	//****************************
	if (bOutputDebug)
	{
		bLedOn = !bLedOn;
		if (bLedOn)
			LEDGREPIN_ON
		else
			LEDGREPIN_OFF

		if (gimState == GIM_SATURATION)
		{
			if (bLedOn)
				LEDPIN_OFF
			else
				LEDPIN_ON
		}

		if (g_bTest[7])
		{
			cliSerial->print(F("DRV "));
			cliSerial->print(deltaDrive[axisROLL]);cliSerial->print(F(" "));
			cliSerial->print(deltaDrive[axisPITCH]);cliSerial->print(F(" "));
			cliSerial->print(deltaDrive[axisYAW]);cliSerial->print(F(" "));
			//cliSerial->print(pwm_val[axisROLL]);cliSerial->print(F(" "));
			//cliSerial->print(pwm_val[axisPITCH]);cliSerial->print(F(" "));
			//cliSerial->print(pwm_val[axisYAW]);cliSerial->print(F(" "));
			cliSerial->println();
		}

		//debug stabilizzazione yaw
		if (g_bSendYawOutput)
		{

			cliSerial->print(F("YAW "));
			cliSerial->print(pid_lap);cliSerial->print(F(" "));
			cliSerial->print(YawPhiSet);cliSerial->print(F(" "));
			cliSerial->print(yawAngleSet);cliSerial->print(F(" "));
			cliSerial->print(yawAngleSet_OLD);cliSerial->print(F(" "));
			cliSerial->print(diffDriveAngleYaw);cliSerial->print(F(" "));
			cliSerial->print(gyroADC[axisYAW] / resolutionDevider);cliSerial->print(F(" "));
			cliSerial->print(gyroDesiredYaw);cliSerial->print(F(" "));
			cliSerial->print(driveSetYaw);cliSerial->print(F(" "));
			cliSerial->print(yawMotorDrive);cliSerial->print(F(" "));
			cliSerial->print(yawDiffDrive);cliSerial->print(F(" "));
#ifdef BOARD_MOT1_ISENSE
			cliSerial->print(getMotorCurrentRaw(axisYAW));cliSerial->print(F(" "));
			float I = 1000.0f * getMotorCurrent(axisYAW);
			cliSerial->print(I);cliSerial->print(F(" "));
#endif
			cliSerial->println();

		}
		// 600 us
		if(g_accOutput){ cliSerial->print(angle[axisPITCH]);
									cliSerial->print(F(" ACC "));cliSerial->print(angle[axisROLL]);
									cliSerial->print(F(" "));cliSerial->print(angle[axisYAW]);
									cliSerial->print(F(" "));cliSerial->print(interrupt_mean_lap.mean());
									cliSerial->print(F(" "));cliSerial->print(interrupt_mean_duration.mean());
									cliSerial->print(F(" "));cliSerial->print(loop_mean_lap.mean());

									//cliSerial->print(F(" "));cliSerial->print(imu_mean_duration.mean());
									//cliSerial->print(F(" "));cliSerial->print(rc_mean_duration.mean());
									//cliSerial->print(F(" "));cliSerial->print(pid_mean_duration.mean());   //loop_mean_lap.vmax());

#ifdef DEBUG_LOOP_DURATION  //TEO 20140902 evito di sporcare troppo l'output
									for (int i = 0; i < 19; i++) {
										cliSerial->print(F(" ["));
										cliSerial->print(i);
										cliSerial->print(F("]"));
										cliSerial->print(main_loop_duration[i].mean());
									}
#endif
									cliSerial->println();
		}

		if (g_bSendDebugOutput){
			cliSerial->print(F("DBG "));
			cliSerial->print(millis());cliSerial->print(F(" "));
			cliSerial->print(angle[axisROLL]);cliSerial->print(F(" "));
			cliSerial->print(angle[axisPITCH]);cliSerial->print(F(" "));
			cliSerial->print(angle[axisYAW]);cliSerial->print(F(" "));

			Vector3f dummy;
			dummy.x =0;
			dummy.y =0;
			dummy.z =0;

#if ((defined ( IMU_AP )) || ( defined (IMU_MIXED)))
			print_vector(ins.get_accel());
			print_vector(ins.get_gyro());
#ifdef GIMBAL_ENABLE_COMPASS
			cliSerial->print(compass.mag_x);cliSerial->print(F(" "));
			cliSerial->print(compass.mag_y);cliSerial->print(F(" "));
			cliSerial->print(compass.mag_z);cliSerial->print(F(" "));

#else
			print_vector(dummy);
#endif
#else

			Vector3f a;
			a.x = accADC[axisROLL];
			a.y = accADC[axisPITCH];
			a.z = accADC[axisYAW];
			print_vector(a);

			Vector3f g;
			g.x = gyroADC[axisROLL];
			g.y = gyroADC[axisPITCH];
			g.z = gyroADC[axisYAW];
			//if (mpu_yaw_present)
			//	g.z = gyroADC2[axisYAW];
			print_vector(g);
#ifdef GIMBAL_ENABLE_COMPASS
#ifdef COMPASS_AP
			cliSerial->print(compass.mag_x);cliSerial->print(F(" "));
			cliSerial->print(compass.mag_y);cliSerial->print(F(" "));
			cliSerial->print(compass.mag_z);cliSerial->print(F(" "));
#else
			Vector3i m = compass.get_raw();
			print_vector(m);
#endif
#else
			Vector3f g2;
			g2.x = 0; //gyroADC2_lfp[axisROLL];
			g2.y = gyroADC2[axisYAW]; //gyroADC2_lfp[axisPITCH];
			g2.z = gyroADC2_lfp[axisYAW];
			print_vector(g2);

			//print_vector(dummy);
#endif

#endif
			cliSerial->println();
		}

		if (g_bSendRCOutput)
		{
			cliSerial->print("RC  "); cliSerial->print(now); cliSerial->print("\t");
			for (int i = 0; i < RC_DATA_SIZE; i++)
			{
				cliSerial->print(rcData[i].rx); cliSerial->print("\t");
			}

			cliSerial->print(rollAngleSet); cliSerial->print("\t");
			cliSerial->print(pitchAngleSet); cliSerial->print("\t");
			cliSerial->print(yawAngleSet); cliSerial->print("\t");

			cliSerial->print(RollPhiSet); cliSerial->print("\t");
			cliSerial->print(PitchPhiSet); cliSerial->print("\t");
			cliSerial->print(YawPhiSet); cliSerial->print("\t");



			cliSerial->println();
		}

#ifdef MANUAL_INPUT_COUNT
		if (g_bSendJoyOutput)
		{
			cliSerial->print("JOY "); cliSerial->print(now); cliSerial->print("\t");
			for (int i = 0; i < MANUAL_INPUT_COUNT; i++)
			{
				uint16_t val = 0;
				getManCmdAxisValue((uint8) i, &val);
				cliSerial->print(val); cliSerial->print("\t");
			}
			cliSerial->println();
		}
#endif
	}

	main_loop_duration[7].append( measure_micro_delay(unowlap, micros()) );
	unowlap = micros();

	//****************************
	// slow rate actions
	//****************************
	if (bEnterPID)
	{

		bool tmp = false;
		switch (count) {
			case 1:
				readACC(axisROLL);
				main_loop_duration[8].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
				break;
			case 2:
				readACC(axisPITCH);
				main_loop_duration[9].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
				break;
			case 3:
				readACC(axisYAW);
				main_loop_duration[10].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
				break;
			case 4:
				updateACC();
				main_loop_duration[11].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
				break;
			case 5:
#ifdef GIMBAL_ENABLE_COMPASS
				readMAG();
#endif
				break;
			case 6:
				// gimbal state transitions
				switch (gimState)
				{
					case GIM_IDLE :
						enableMotorUpdates = false;
						LEDGREPIN_OFF
						setACCFastMode(true);
						// wait 2 sec to settle ACC, before PID controlerbecomes active
						if (now - stateStart >= IDLE_TIME_SEC)
						{
							gimState = GIM_UNLOCKED;
							stateStart = now;
						}
						break;

					case GIM_SATURATION :
						enableMotorUpdates = false;
						LEDGREPIN_OFF
						setACCFastMode(true);
						// wait 2 sec to settle ACC, before PID controlerbecomes active
						if (now - stateStart >= config.profiles[0].saturationLock)
						{
							gimState = GIM_UNLOCKED;
							stateStart = now;
							g_saturationLockStart = 0;
						}
						break;


					case GIM_UNLOCKED :
						enableMotorUpdates = true;
						LEDGREPIN_ON
						setACCFastMode(true);
						// allow PID controller to settle on ACC position
						if (now - stateStart >= LOCK_TIME_SEC)
						{
							gimState = GIM_LOCKED;
							stateStart = now;

							enableMotorUpdates = true;
							LEDGREPIN_ON
							setACCFastMode(false);
						}
						break;
					case GIM_LOCKED :
						// normal operation
						break;
					case GIM_ERROR:
						enableMotorUpdates = false;
						LEDGREPIN_OFF
						break;
				}
				main_loop_duration[12].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
				break;

#ifdef GIMBAL_ENABLE_RC
			case 7:
				// RC Pitch function
				if (rcData[RC_DATA_PITCH].valid) {
					if(config.profiles[0].rcConfig[axisPITCH].absolute) {
						PitchPhiSet = rcData[RC_DATA_PITCH].setpoint;
					} else {
						if(fabs(rcData[RC_DATA_PITCH].rcSpeed)>0.01) {
							PitchPhiSet += rcData[RC_DATA_PITCH].rcSpeed * 0.01;
						}
					}
				} else {
					PitchPhiSet = 0;
				}
				//verifico se  stato richiesto il reset
				tmp = false;
				if (rcData[RC_DATA_RESET_PITCH].valid) {
					//il reset mi interessa solo se sto pilotando in relativo
					if(!config.profiles[0].rcConfig[axisPITCH].absolute) {
						//if (rcData[RC_DATA_RESET_PITCH].rcSpeed > 0)
						if (rcData[RC_DATA_RESET_PITCH].setpoint > 0.5f)
						{
							//PitchPhiSet = 0;
							tmp = true;
						}
					}
				}
				PitchResetting = tmp;


				tmp = false;
				if (rcData[RC_DATA_MODE_PITCH].valid) {
					if (rcData[RC_DATA_MODE_PITCH].setpoint > 0.5f)
					{
						tmp = true;
					}
				}
				PitchLocked = tmp;

				if (config.profiles[0].rcConfig[axisPITCH].minOutput < config.profiles[0].rcConfig[axisPITCH].maxOutput) {
					PitchPhiSet = constrain(PitchPhiSet, config.profiles[0].rcConfig[axisPITCH].minOutput, config.profiles[0].rcConfig[axisPITCH].maxOutput);
				} else {
					PitchPhiSet = constrain(PitchPhiSet, config.profiles[0].rcConfig[axisPITCH].maxOutput, config.profiles[0].rcConfig[axisPITCH].minOutput);
				}

				main_loop_duration[13].append( measure_micro_delay(unowlap, micros()) );
				unowlap = micros();
			break;
		case 8:
			// RC roll function
			if (rcData[RC_DATA_ROLL].valid){
				if(config.profiles[0].rcConfig[axisROLL].absolute){
					RollPhiSet = rcData[RC_DATA_ROLL].setpoint;
				} else {
					if(fabs(rcData[RC_DATA_ROLL].rcSpeed)>0.01) {
						RollPhiSet += rcData[RC_DATA_ROLL].rcSpeed * 0.01;
					}
				}
			} else {
				RollPhiSet = 0;
			}

			//verifico se  stato richiesto il reset
			tmp = false;
			if (rcData[RC_DATA_RESET_ROLL].valid) {
				//il reset mi interessa solo se sto pilotando in relativo
				if(!config.profiles[0].rcConfig[axisROLL].absolute) {
					//if (rcData[RC_DATA_RESET_ROLL].rcSpeed > 0)
					if (rcData[RC_DATA_RESET_ROLL].setpoint > 0.5f)
					{
						//RollPhiSet = 0;
						tmp = true;
					}
				}
			}
			RollResetting = tmp;

			tmp = false;
			if (rcData[RC_DATA_MODE_ROLL].valid) {
				if (rcData[RC_DATA_MODE_ROLL].setpoint > 0.5f)
				{
					tmp = true;
				}
			}
			RollLocked = tmp;

			if (config.profiles[0].rcConfig[axisROLL].minOutput < config.profiles[0].rcConfig[axisROLL].maxOutput) {
				RollPhiSet = constrain(RollPhiSet, config.profiles[0].rcConfig[axisROLL].minOutput, config.profiles[0].rcConfig[axisROLL].maxOutput);
			} else {
				RollPhiSet = constrain(RollPhiSet, config.profiles[0].rcConfig[axisROLL].maxOutput, config.profiles[0].rcConfig[axisROLL].minOutput);
			}
			main_loop_duration[14].append( measure_micro_delay(unowlap, micros()) );
			unowlap = micros();
			break;
		case 9:
			// RC yaw function
			if (rcData[RC_DATA_YAW].valid){
				if(config.profiles[0].rcConfig[axisYAW].absolute){
					YawPhiSet = rcData[RC_DATA_YAW].setpoint;
				} else {
					if(fabs(rcData[RC_DATA_YAW].rcSpeed)>0.01) {
						YawPhiSet += rcData[RC_DATA_YAW].rcSpeed * 0.01;
					}
				}
			} else {
				YawPhiSet = 0;
			}

			//verifico se  stato richiesto il reset
			tmp = false;
			if (rcData[RC_DATA_RESET_YAW].valid) {
				//il reset mi interessa solo se sto pilotando in relativo
				if(!config.profiles[0].rcConfig[axisYAW].absolute) {
					//if (rcData[RC_DATA_RESET_YAW].rcSpeed > 0)
					if (rcData[RC_DATA_RESET_YAW].setpoint > 0.5f)
					{
						//YawPhiSet = 0;
						tmp = true;
					}
				}
			}
			YawResetting = tmp;

			tmp = false;
			if (rcData[RC_DATA_MODE_YAW].valid) {
				if (rcData[RC_DATA_MODE_YAW].setpoint > 0.5f)
				{
					tmp = true;
				}
			}
			YawLocked = tmp;

			if ((config.profiles[0].rcConfig[axisYAW].minOutput == config.profiles[0].rcConfig[axisYAW].maxOutput) && (config.profiles[0].rcConfig[axisYAW].maxOutput == 0)) {
				//normalizzare il phiset genera dei casini micidiali quando si salta da -180 a 180 e viceversa
				//quindi lo evito
				//YawPhiSet = normalize_yaw(YawPhiSet);
			} else if (config.profiles[0].rcConfig[axisYAW].minOutput < config.profiles[0].rcConfig[axisYAW].maxOutput) {
				YawPhiSet = constrain(YawPhiSet, config.profiles[0].rcConfig[axisYAW].minOutput, config.profiles[0].rcConfig[axisYAW].maxOutput);
			} else {
				YawPhiSet = constrain(YawPhiSet, config.profiles[0].rcConfig[axisYAW].maxOutput, config.profiles[0].rcConfig[axisYAW].minOutput);
			}
			main_loop_duration[15].append( measure_micro_delay(unowlap, micros()) );
			unowlap = micros();
			break;
#endif
		case 10:
#ifdef STACKHEAPCHECK_ENABLE
			stackHeapEval(false);
#endif
			count=0;
			break;
		default:
			break;
	}
	count++;

	//****************************
	// check RC channel timeouts
	//****************************
#ifdef GIMBAL_ENABLE_RC
	checkRcTimeouts();
#endif
	main_loop_duration[16].append( measure_micro_delay(unowlap, micros()) );
	unowlap = micros();

	//****************************
	// Evaluate Serial inputs
	//****************************
	//verifico se devo cambiare seriale da UART a USB
#ifdef GIMBAL_ENABLE_USB
	if (SerialUSB.isConnected())
		cliSerial = (FastSerial*)&SerialUSB;
	else
		cliSerial = &SerialDBG;
	sCmd.setSerialPort(cliSerial);
#endif
	sCmd.readSerial();

	main_loop_duration[17].append( measure_micro_delay(unowlap, micros()) );




	}

}


