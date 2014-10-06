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

    Please refer to http://vrgimbal.wordpress.com for more information
*/

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#include "main.h"

#include "LowPassFilter2p.h"

#if defined ( IMU_EVV )

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 40
#endif

#define NUMAXIS 3
#define EULAR 3

float AccAngleSmooth[EULAR];
float CameraOrient[EULAR];

float CameraOrient2[EULAR];


LowPassFilter2p lpfIMU2(DT_FLOAT, 0);
float lpfIMU2_constant = 0;

void EVV_Init_Orientation();

// set default sensor orientation (sensor upside)
void initSensorOrientationDefault() {

	//versione con chip in alto e connettore a vaschetta verso dietro
	// channel assignment
	sensorDef.Gyro[axisROLL].idx = 0;
	sensorDef.Gyro[axisPITCH].idx = 1;
	sensorDef.Gyro[axisYAW].idx = 2;

	sensorDef.Acc[axisROLL].idx = 0;
	sensorDef.Acc[axisPITCH].idx = 1;
	sensorDef.Acc[axisYAW].idx = 2;

	sensorDef.Mag[axisROLL].idx = 0;
	sensorDef.Mag[axisPITCH].idx = 1;
	sensorDef.Mag[axisYAW].idx = 2;

	// direction
	sensorDef.Gyro[axisROLL].dir = 1;
	sensorDef.Gyro[axisPITCH].dir = 1;
	sensorDef.Gyro[axisYAW].dir = 1;

	sensorDef.Acc[axisROLL].dir = 1;
	sensorDef.Acc[axisPITCH].dir = 1;
	sensorDef.Acc[axisYAW].dir = 1;

	sensorDef.Mag[axisROLL].dir = -1;
	sensorDef.Mag[axisPITCH].dir = -1;
	sensorDef.Mag[axisYAW].dir = 1;

  
}

// swap two char items
void swap_char(char * a, char * b) {
	char tmp = *a;
	*a = *b;
	*b = tmp;
}
// swap two int items
void swap_int(int * a, int * b) {
	int tmp = *a;
	*a = *b;
	*b = tmp;
}

// set sensor orientation according config
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void initSensorOrientation() {
  
	initSensorOrientationDefault();

	if (config.axisReverseZ) {
		// flip over roll

		sensorDef.Acc[axisYAW].dir *= -1;
		sensorDef.Acc[axisROLL].dir *= -1;
		//sensorDef.Acc[axisPITCH].dir *= -1;

		//sensorDef.Gyro[axisPITCH].dir *= -1;
		sensorDef.Gyro[axisROLL].dir *= -1;
		sensorDef.Gyro[axisYAW].dir *= -1;


		sensorDef.Mag[axisYAW].dir *= -1;
		//sensorDef.Mag[axisROLL].dir *= -1;
		sensorDef.Mag[axisPITCH].dir *= -1;
	}
	if (config.axisSwapXY) {
		// swap gyro axis
		swap_char(&sensorDef.Gyro[axisROLL].idx, &sensorDef.Gyro[axisPITCH].idx);
		swap_int(&sensorDef.Gyro[axisROLL].dir, &sensorDef.Gyro[axisPITCH].dir);

		//sensorDef.Gyro[axisPITCH].dir *= -1;   // try and error ;-)
		sensorDef.Gyro[axisROLL].dir *= -1;


		// swap acc axis
		swap_char(&sensorDef.Acc[axisROLL].idx, &sensorDef.Acc[axisPITCH].idx);
		swap_int(&sensorDef.Acc[axisROLL].dir, &sensorDef.Acc[axisPITCH].dir); sensorDef.Acc[axisROLL].dir *= -1;


		swap_char(&sensorDef.Mag[axisROLL].idx, &sensorDef.Mag[axisPITCH].idx);
		swap_int(&sensorDef.Mag[axisROLL].dir, &sensorDef.Mag[axisPITCH].dir); sensorDef.Mag[axisROLL].dir *= -1;
	}

	//compass.set_transformation
}

void setACCFastMode (bool fastMode) {
	if (fastMode) {
		AccComplFilterConst = (float)DT_FLOAT/(1.0 + DT_FLOAT); // 2 sec
		AccComplFilterConst2 = AccComplFilterConst;
	} else {
		AccComplFilterConst = (float)DT_FLOAT/(config.profiles[0].accTimeConstant + DT_FLOAT);
		AccComplFilterConst2 = (float)DT_FLOAT/(config.profiles[0].accTimeConstantSat + DT_FLOAT);
		if (AccComplFilterConst < AccComplFilterConst2)
			AccComplFilterConst2 = AccComplFilterConst;
	}
}

void initIMU() {
 
	// resolutionDevider=131, scale = 0.000133
	// 102us
	gyroScale =  1.0 / resolutionDevider / 180.0 * PI;  // convert to radians

	setACCFastMode(false);

}


void setIMU2LPF() //float decayTime)
{
//	float f = 0.0f;
//	if (decayTime > 0)
//		f = 1.0f / (2.0f * PI * decayTime);
//	float sf = LOOPUPDATE_FREQ;  //sampling freq
//
//	lpfIMU2.set_cutoff_frequency(sf, f);

	//lpfIMU2_constant = (float)DT_FLOAT/((float) config.profiles[0].mpu2LPF / 10000.0f + DT_FLOAT);

	if (config.profiles[0].mpu2LPF > 0)
	{
		float CUTOFF = (float) config.profiles[0].mpu2LPF;
		float RC = 1.0f/(CUTOFF*2.0f*3.14f);
		float dt = DT_FLOAT; // 1/SAMPLE_RATE;
		float alpha = dt/(RC+dt);
		lpfIMU2_constant = alpha;

		cliSerial->print("IMU2_LPF=");
		cliSerial->print((int) ( 1000.0f * lpfIMU2_constant ));
		cliSerial->println();

	} else {
		lpfIMU2_constant = 1.0f;
		cliSerial->println("IMU2_LPF NO FILTER");
	}


}

float _readGyros_lap = 0.0f;
uint32_t _readGyros_lap_us = 0;

bool readGyros() {
	int16_t axisRot[3];
	uint8_t idx;
	int i;

	bool bSaturated = false;


	//misuro il tempo dall'ultima lettura
	static uint32_t last_gyro_update = 0;
	uint32_t now = micros();

	//_readGyros_lap = ((float) measure_micro_delay( last_gyro_update, now) / 1000000.0f);
	_readGyros_lap_us = measure_micro_delay( last_gyro_update, now);
	_readGyros_lap = ((float) _readGyros_lap_us / 1000000.0f);
	last_gyro_update = now;

	if (_readGyros_lap > 10 * DT_FLOAT)
		_readGyros_lap = DT_FLOAT;



	// read gyros
	mpu.getRotation(&axisRot[0], &axisRot[1], &axisRot[2]);

		for (i = 0; i < 3; i++)
		{
			idx = sensorDef.Gyro[i].idx;
			gyroADC[i] = axisRot[idx]-gyroOffset[idx];
			if ((gyroADC[i] < gyroDeadBand[idx]) && (gyroADC[i] > -gyroDeadBand[idx]))
				gyroADC[i] = 0;
			if (gyroSaturation[idx] > 0)
			{
				if (gyroADC[i] < -gyroSaturation[idx])
				{
					gyroADC[i] = -gyroSaturation[idx];
					bSaturated = true;
				}
				if (gyroADC[i] > gyroSaturation[idx])
				{
					gyroADC[i] = gyroSaturation[idx];
					bSaturated = true;
				}
			}
			gyroADC[i] *= sensorDef.Gyro[i].dir;
		}

	if (!g_bTest[0])
	{
		if (mpu_yaw_present)
		{
#ifdef IMU_SECONDARY_ON_ROLL
		mpu2.getRotation(&axisRot[0], &axisRot[1], &axisRot[2]);

		for (i = 0; i < 3; i++)
		{
			idx = sensorDef.Gyro[i].idx;
			gyroADC2[i] = axisRot[idx]-gyroOffset2[idx];
			if ((gyroADC2[i] < gyroDeadBand2[idx]) && (gyroADC2[i] > -gyroDeadBand2[idx]))
				gyroADC2[i] = 0;
			if (gyroSaturation2[idx] > 0)
			{
				if (gyroADC2[i] < -gyroSaturation2[idx])
				{
					gyroADC2[i] = -gyroSaturation2[idx];
					bSaturated = true;
				}
				if (gyroADC2[i] > gyroSaturation2[idx])
				{
					gyroADC2[i] = gyroSaturation2[idx];
					bSaturated = true;
				}
			}
			gyroADC2[i] *= sensorDef.Gyro[i].dir;
		}
#else

		//LETTURA SEMPLIFICATA
		i = 2;
		idx = sensorDef.Gyro[i].idx;
		int16_t rotYaw = 0;
		switch (idx)
		{
		case 0:
			rotYaw = mpu_yaw.getRotationX();
			break;
		case 1:
			rotYaw = mpu_yaw.getRotationY();
			break;
		case 2:
			rotYaw = mpu_yaw.getRotationZ();
			break;

		}
		gyroADC2[i] = rotYaw - gyroOffset2[idx];
		if ((gyroADC2[i] < gyroDeadBand2[idx]) && (gyroADC2[i] > -gyroDeadBand2[idx]))
			gyroADC2[i] = 0;
		if (gyroSaturation2[idx] > 0)
		{
			if (gyroADC2[i] < -gyroSaturation2[idx])
			{
				gyroADC2[i] = -gyroSaturation2[idx];
				bSaturated = true;
			}
			if (gyroADC2[i] > gyroSaturation2[idx])
			{
				gyroADC2[i] = gyroSaturation2[idx];
				bSaturated = true;
			}
		}
		gyroADC2[i] *= sensorDef.Gyro[i].dir;

#endif
		}
	}
	return bSaturated;
}

void readACC(axisDef axis) {
	// get acceleration

	if (g_bTest[0])
		return;


	char idx;
	int16_t val;
	idx = sensorDef.Acc[axis].idx;
	val = mpu.getAccelerationN(idx);  // TODO: 370us

	//float v = accScale[idx] * MPU6000_ACCEL_SCALE_1G * (float) val - accOffset[idx] ;

	float v = accScale[idx] * (float) val - accOffset[idx] / MPU6000_ACCEL_SCALE_1G ;
	v *= (float) sensorDef.Acc[axis].dir;
	accADC[axis] = v;

	//accADC[axis] = val;
	//accADC[axis] = MPU6000_ACCEL_SCALE_1G *(float) val;
	//accADC[axis] = accScale[axis] * val - accOffset[axis] ;

}


Vector3f meanGyro;
int meanCount = 0;

void print_raw(float x1,float y1,float z1,
		float x2 = 0.0f,float y2 = 0.0f,float z2 = 0.0f,
		float x3 = 0.0f,float y3 = 0.0f,float z3 = 0.0f,
		float temp = 0.0f)
{

	//media giroscopio
	meanCount++;
	meanGyro.x = (meanGyro.x * (meanCount - 1) + x2) / meanCount;
	meanGyro.y = (meanGyro.y * (meanCount - 1) + y2) / meanCount;
	meanGyro.z = (meanGyro.z * (meanCount - 1) + z2) / meanCount;


	//debug
	//****************************
	// slow rate actions
	//****************************
	static uint32_t output_time = 0;
	uint32_t now = millis();
	if ((now - output_time) > (1000 / POUT_FREQ))
	{

		output_time = now;

		cliSerial->print(F("RAW "));
		cliSerial->print(now);cliSerial->print(F(" "));

		Vector3f dummy;
		dummy.x =x1;
		dummy.y =y1;
		dummy.z =z1;

		print_vector(dummy); //(acc);

		dummy.x = x2;
		dummy.y = y2;
		dummy.z = z2;

		print_vector(dummy); //(gyro);

		dummy.x =x3;
		dummy.y =y3;
		dummy.z =z3;

		print_vector(dummy); //(mag);
		cliSerial->print(temp);cliSerial->print(F(" "));

		print_vector(meanGyro);

		cliSerial->println();

		meanGyro.x = 0.0f;
		meanGyro.y = 0.0f;
		meanGyro.z = 0.0f;
		meanCount = 0;
	}

}

uint32_t  get_gyro_lap()
{
	return _readGyros_lap_us;
}

float getGyroDeg(int eulerAxis)
{
	if (eulerAxis == axisYAW)
	{
		if (!g_bTest[0])
		{
			//SECONDA IMU
			if (mpu_yaw_present)
			{
				return (float) gyroADC2[axisYAW] / resolutionDevider;
			}
		}
	}
	return (float) gyroADC[eulerAxis] / resolutionDevider;
}

void updateGyroAttitude(){
	uint8_t axis;

	float GyroData[3];
	float GyroRate[EULAR];

	for (axis = 0; axis < 3; axis++) {
		GyroData[axis] = (float) gyroADC[axis]  * gyroScale;
	}

	//applico qui solo il deltaGyro, la parte acc la applico nella funzione dedicata
	GyroRate[axisPITCH] = GyroData[axisX];
	CameraOrient[axisPITCH] = CameraOrient[axisPITCH] + GyroRate[axisPITCH] * _readGyros_lap;

	GyroRate[axisROLL] = -GyroData[axisZ] * sin(CameraOrient[axisPITCH]) + GyroData[axisY] * cos(fabs(CameraOrient[axisPITCH]));
	CameraOrient[axisROLL] = CameraOrient[axisROLL] + GyroRate[axisROLL] * _readGyros_lap;

	//yaw provvisorio
	GyroRate[axisYAW] = -GyroData[axisZ] * cos(fabs(CameraOrient[axisPITCH])) - GyroData[axisY] * sin(CameraOrient[axisPITCH]); //presuming Roll is horizontal
	CameraOrient[axisYAW] = CameraOrient[axisYAW] + GyroRate[axisYAW] * _readGyros_lap;

	if (!g_bTest[0])
	{
		//SECONDA IMU
		if (mpu_yaw_present)
		{

		float GyroData2[3];
		float GyroRate2[EULAR];

		for (axis = 0; axis < 3; axis++) {
			if (config.profiles[0].mpu2LPF == 0)
			{
				//no filtro
				gyroADC2_lfp[axis] = (float) gyroADC2[axis];
			} else {
				//filtro 2 ordine
				//gyroADC2_lfp[axis] = lpfIMU2.apply((float) gyroADC2[axis]);
				//filtro LPF semplice
				gyroADC2_lfp[axis] = (1.0 - lpfIMU2_constant) * gyroADC2_lfp[axis] + lpfIMU2_constant * (float) gyroADC2[axis];
			}
			GyroData2[axis] = -  gyroADC2_lfp[axis] * gyroScale;
		}

#ifdef IMU_SECONDARY_ON_ROLL
#else

		GyroRate2[axisYAW] = -GyroData2[axisZ];
		CameraOrient2[axisYAW] = CameraOrient2[axisYAW] + GyroRate2[axisYAW] * _readGyros_lap;
#endif
		//hack per il pid
		CameraOrient[axisYAW] = CameraOrient2[axisYAW];
	}

	}

//	if (mpu_yaw_present)
//	{
//		#ifdef IMU_SECONDARY_ON_ROLL
//			TODO: gestire la seconda imu sul roll
//		#else
//			//gyroADC2_lfp[axisZ] = lpfIMU2.apply((float) gyroADC2[axisZ]);
//			//filtro LPF semplice
//			gyroADC2_lfp[axisZ] = (1.0 - lpfIMU2_constant) * gyroADC2_lfp[axisZ] + lpfIMU2_constant * (float) gyroADC2[axisZ];
//			GyroRate[axisYAW] = -  gyroADC2_lfp[axisZ] * gyroScale;
//
//			//GyroRate[axisYAW] = - gyroADC2[axisZ] * gyroScale;
//		#endif
//	} else {
//		GyroRate[axisYAW] = -GyroData[axisZ] * cos(fabs(CameraOrient[axisPITCH])) - GyroData[axisY] * sin(CameraOrient[axisPITCH]); //presuming Roll is horizontal
//	}
//	CameraOrient[axisYAW] = CameraOrient[axisYAW] + GyroRate[axisYAW] * _readGyros_lap;
////	//filtro passa basso lo yaw
////	if (mpu_yaw_present)
////		CameraOrient[axisYAW] = lpfIMU2.apply(CameraOrient[axisYAW]);
//
////	//constraint -PI .. PI
////	if (CameraOrient[axisYAW] > PI)
////		CameraOrient[axisYAW] = CameraOrient[axisYAW] - 2 * PI;
////	if (CameraOrient[axisYAW] < -PI)
////			CameraOrient[axisYAW] = CameraOrient[axisYAW] + 2 * PI;

}

void updateACC(){
	float AccAngle[EULAR];
	if (accADC[axisZ] != 0)
	{
		//AccAngle[axisROLL] = -(atan2_substitute(accADC[axisX], accADC[axisZ])); //Calculating pitch ACC angle
		AccAngle[axisROLL] = -sgn(atan2_substitute(accADC[axisX], accADC[axisZ])) * fabs( atan2_substitute(accADC[axisX], sqrt(accADC[axisZ] * accADC[axisZ] + accADC[axisY] * accADC[axisY]))); //Calculating pitch ACC angle
		AccAngle[axisPITCH] = +(atan2_substitute(accADC[axisY], accADC[axisZ])); //Calculating roll ACC angle

		//LPF on Acc
		utilLP_float(&(AccAngleSmooth[axisROLL]), AccAngle[axisROLL], (1.0f/ACC_LPF_FACTOR));
		utilLP_float(&(AccAngleSmooth[axisPITCH]), AccAngle[axisPITCH], (1.0f/ACC_LPF_FACTOR));

		accMag = sqrt(accADC[axisX] * accADC[axisX] + accADC[axisY] * accADC[axisY]+ accADC[axisZ] * accADC[axisZ]);

	}

}


void updateACCAttitude(){
//TODO: testare questa modifica e verificare che il range di valori sia corretto
	float accExtra = fabs(accMag * MPU6000_ACCEL_SCALE_1G - GRAVITY);
	if (accExtra < 0.4 * GRAVITY) {

		//TEO 20141006
		//indurisco il filtro passabasso al salire dell'accelerazione
		//(più alta è l'accelerazione più forte è la probabilità che siano intervenute accelerazioni laterali che mi fanno sbagliare la stima)
		float filter = AccComplFilterConst;
		filter = (accExtra / (0.4 * GRAVITY)) * (AccComplFilterConst2 - AccComplFilterConst) + AccComplFilterConst;


		CameraOrient[axisPITCH] = (1.0 - filter) * CameraOrient[axisPITCH] + filter * AccAngleSmooth[axisPITCH]; //Pitch Horizon
		CameraOrient[axisROLL] =  (1.0 - filter) * CameraOrient[axisROLL]  + filter * AccAngleSmooth[axisROLL]; //Roll Horizon
	}

}






void getAttiduteAngles() {

  // attitude of the estimated vector
	angle[axisROLL]  = ((int32_t)config.profiles[0].axisConfig[axisROLL].angleOffset * 10) +  CameraOrient[axisROLL] * 1000.0f * 180.0f / PI;
	angle[axisPITCH] = ((int32_t)config.profiles[0].axisConfig[axisPITCH].angleOffset * 10) + CameraOrient[axisPITCH] * 1000.0f * 180.0f / PI;
	angle[axisYAW] = ((int32_t)config.profiles[0].axisConfig[axisYAW].angleOffset * 10) + CameraOrient[axisYAW] * 1000.0f * 180.0f / PI;
}




void EVV_Init_Orientation()
{

	AccAngleSmooth[axisROLL] = 0.0f;
	AccAngleSmooth[axisPITCH] = 0.0f;
	AccAngleSmooth[axisYAW] = 0.0f;

	CameraOrient[axisROLL] = 0.0f;
	CameraOrient[axisPITCH] = 0.0f;
	CameraOrient[axisYAW] = 0.0f;

	CameraOrient2[axisROLL] = 0.0f;
	CameraOrient2[axisPITCH] = 0.0f;
	CameraOrient2[axisYAW] = 0.0f;

    int init_loops = 150;
    float AccAngle[NUMAXIS];
    int i;

    for (i = 0; i < init_loops; i++)
    {
    	readACC(axisROLL);
    	readACC(axisPITCH);
    	readACC(axisYAW);

        //AccAngle[axisROLL] = -(atan2_substitute(accADC[axisX], accADC[axisZ])); //Calculating pitch ACC angle
    	AccAngle[axisROLL] = -sgn(atan2_substitute(accADC[axisX], accADC[axisZ])) * fabs( atan2_substitute(accADC[axisX], sqrt(accADC[axisZ] * accADC[axisZ] + accADC[axisY] * accADC[axisY]))); //Calculating pitch ACC angle
        AccAngle[axisPITCH] = +(atan2_substitute(accADC[axisY], accADC[axisZ])); //Calculating roll ACC angle

        if (i == 0)
        {
        	AccAngleSmooth[axisROLL] = AccAngle[axisROLL];
        	AccAngleSmooth[axisPITCH] = AccAngle[axisPITCH];
        } else {

			//AccAngleSmooth[axisROLL] = ((AccAngleSmooth[axisROLL] * (float)(init_loops - 1)) + AccAngle[axisROLL]) / (float)init_loops; //Averaging pitch ACC values
			//AccAngleSmooth[axisPITCH] = ((AccAngleSmooth[axisPITCH] * (float)(init_loops - 1)) + AccAngle[axisPITCH]) / (float)init_loops; //Averaging roll ACC values

			AccAngleSmooth[axisROLL] = ((AccAngleSmooth[axisROLL] * (float)(i - 1)) + AccAngle[axisROLL]) / (float)i; //Averaging pitch ACC values
			AccAngleSmooth[axisPITCH] = ((AccAngleSmooth[axisPITCH] * (float)(i - 1)) + AccAngle[axisPITCH]) / (float)i; //Averaging roll ACC values
        }
        delay(1);
    }

    CameraOrient[axisPITCH] = AccAngleSmooth[axisPITCH];
    CameraOrient[axisROLL] = AccAngleSmooth[axisROLL];
    CameraOrient[axisYAW] = 0.0;





}


#endif
