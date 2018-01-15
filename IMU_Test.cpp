//============================================================================
// Name        : IMU_Test.cpp
// Author      : Brendan Martin
// Version     : 1.0
// Copyright   : Written by Brendan Martin, January 2018
//				 Redistribution and reuse of this code is permitted provided that the following is acknowledged:
//				 1) This software is provided "as-is" with no warranties.
//				 2) Under no circumstances am I liable for any direct or indirect damage(s) to any
//					hardware, nor for any loss or corruption of data.
// Description : Testing I2C comms with 9DOF LSM9DS1 sensor stick
//============================================================================

#include <iomanip>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <fstream>

using namespace std;
//============================================================================

// Map relevant registers and addresses for the LSM9DS1 iNEMO Inertial Module

#define MAGDRES   0x1E        // Magnetometer address
#define AGDRES    0x6B	      // Accelerometer and Gyro address

void SetupIMU()
{
	// Used to open and setup the I2C Bus 2 on the MicroController.
	// Use 'sudo i2cdetec -y -r x' in linux to confirm the bus you
	// are connected to, where 'x' is typically 0, 1, 2, or 3.

	int getAG = open("/dev/i2c-2", O_RDWR);   		// The -2 indicates the 2nd bus on the microcontroller
	ioctl(getAG, I2C_SLAVE, AGDRES);

	// Select control register CNTRL_REG6_XL (0x20)
    // X, Y and Z Axis enable, power on mode, data rate 952 Hz
	// Scale selection, +/- 8g
	// Default Band Width Selection, Default Ant-Aliasing filter bandwidth
	// Write 110 11 0 00 = 0xD8
	char config[2] = {0};
	config[0] = 0x20;
    config[1] = 0xD8;
    write(getAG, config, 2);

	// Select control register CNTRL_REG1_G (0x10)
    // X, Y and Z data rate 952 Hz (match XL)
    // Set scale to 2000 dps
    // Default bandwidth selection
    // Write 110 11 0 00 = 0xD8
    config[0] = 0x10;
    config[1] = 0xD8;
    write(getAG, config, 2);

    // Set the IMU to bypass mode using FIFO_CTRL register (0x2E). Should be in bypass by default
    // Write 000 0 0000
    config[0] = 0x2E;
    config[1] = 0x00;
    write(getAG, config, 2);
    close(getAG); // Setup is done, close the bus. THIS IS CRITICAL!!!!!!
}

double * GetXL_M()
{
    static double XL_G[6] = {0, 0, 0, 0, 0, 0};
    int getAG = open("/dev/i2c-2", O_RDWR);   		// define getAG again for this function
	ioctl(getAG, I2C_SLAVE, AGDRES);

	// =========================== Get XL Data =====================================
    // Read data -- output is 16 bit unsigned for XL and Gyro
    // least significant bit (lsb), then most significant bit (msb) for each Accelerometer Axis
    // Read xAccl lsb data from register(0x28)
    char reg[1] = {0x28};
    char data[1] = {0};
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_1 = data[0];

    // Read xAccl msb data from register(0x29)
    reg[0] = 0x29;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_2 = data[0];

    // Read yAccl lsb data from register(0x2A)
    reg[0] = 0x2A;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_3 = data[0];

    // Read yAccl msb data from register(0x2B)
    reg[0] = 0x2B;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_4 = data[0];

    // Read zAccl lsb data from register(0x2C)
    reg[0] = 0x2C;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_5 = data[0];

    // Read zAccl msb data from register(0x2D)
    reg[0] = 0x2D;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char XL_6 = data[0];

    // Convert the lsb and msb to usable data in m/s^2 (65536 is full range, above half represents neg. values
    int xAccl = (XL_2 * 256 + XL_1);
    if(xAccl > 32767)
    {
        xAccl -= 65536;
    }
    double xXL = (xAccl * .024414435 / 9.80665);

    int yAccl = (XL_4 * 256 + XL_3);
    if(yAccl > 32767)
    {
        yAccl -= 65536;
    }
    double yXL = (yAccl * .024414435 / 9.80665);

    int zAccl = (XL_6 * 256 + XL_5);
    if(zAccl > 32767)
    {
        zAccl -= 65536;
    }
    double zXL = (zAccl * .024414435 / 9.80665);

    // =========================== Get Gyro Data =====================================
    // Read data
    // least significant bit (lsb), then most significant bit (msb) for each Accelerometer Axis
    // Read xGy lsb data from register(0x18)
    reg[0] = 0x18;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_1 = data[0];

    // Read xGy msb data from register(0x19)
    reg[0] = 0x19;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_2 = data[0];

    // Read yGy lsb data from register(0x1A)
    reg[0] = 0x1A;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_3 = data[0];

    // Read yGy msb data from register(0x1B)
    reg[0] = 0x1B;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_4 = data[0];

    // Read zGy lsb data from register(0x1C)
    reg[0] = 0x1C;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_5 = data[0];

    // Read zGy msb data from register(0x1D)
    reg[0] = 0x1D;
    write(getAG, reg, 1);
    read(getAG, data, 1);
    char Gy_6 = data[0];

    close(getAG); // Reads are done, close the bus. THIS IS CRITICAL!!!!!!

    // Convert the lsb and msb to usable data in dps using conversion factor *.07 (See data sheet for each rate scale)
    double xGy = (Gy_2 * 256 + Gy_1);
    if(xGy > 32767)
    {
        xGy -= 65536;
    }
    xGy = (xGy * .06104) - 1.5; // added offset based off average reads at 0 rotation

    double yGy = (Gy_4 * 256 + Gy_3);
    if(yGy > 32767)
    {
        yGy -= 65536;
    }
    yGy = (yGy * .06104) - 0.7; // added offset based off average reads at 0 rotation

    double zGy = (Gy_6 * 256 + Gy_5);
    if(zGy > 32767)
    {
        zGy -= 65536;
    }
    zGy = (zGy * .06104) + 0.28; // added offset based off average reads at 0 rotation

    // ========= Populate Output Array -- Grab with pointer and call to GetXL_M =============

    XL_G[0] = xXL;
    XL_G[1] = yXL;
    XL_G[2] = zXL;
    XL_G[3] = xGy;
    XL_G[4] = yGy;
    XL_G[5] = zGy;

    return XL_G;
}

double GetTime()
{
	struct timeval t;
	gettimeofday(&t, NULL);
	double time = (t.tv_sec + (t.tv_usec / 1000000.0)); // returns the time of day in seconds
	return time;
}

double * GetAttitude (double timeval, double Last_Pitch, double Last_Roll)
{
	double R2D = 57.2958;    // radians to degrees
	double time;
	double dt;
	double A_Pitch;
	double A_Roll;
	double G_Pitch;
	double G_Roll;
	double Pitch;
	double Roll;
	double Norm;
	double temp_val;
	static double Angles[5] = {0, 0, 0, 0, 0};
	double *IMU1; 		// setup pointer to the Get_XL output array
	IMU1 = GetXL_M();	// get most recent Accel and Gyro Data
	double Acc[3] = {*(IMU1 + 0), *(IMU1 + 1), *(IMU1 + 2)};
	double Gyr[3] = {*(IMU1 + 3), *(IMU1 + 4), *(IMU1 + 5)};

	// Calculate accelerometer angle vectors
	Norm = sqrt(Acc[0]*Acc[0] + Acc[1]*Acc[1] + Acc[2]*Acc[2]);
	temp_val = Acc[0]/Norm;
	if (temp_val > 1)			// Make sure the value being passed to asin is within the bounds of the function
	{
		temp_val = 1.0;
	}
	if (temp_val < -1)
	{
		temp_val = -1.0;
	}
	A_Pitch = asin(temp_val)*R2D;

	temp_val = Acc[1]/Norm;
	if (temp_val > 1)
	{
		temp_val = 1.0;
	}
	if (temp_val < -1)
	{
		temp_val = -1.0;
	}
	A_Roll = asin(temp_val)*R2D;

	time = GetTime();		// Get the current time
	dt = time - timeval;	// Calculate elapsed time since last gyro integration
	cout << setprecision(12) << "Loop Time (seconds): " << dt << endl;		// For Debug and/or to see loop speed

	// Calculate new angle from Gyro Data -- integrate Gyro over dt and add to last known angle
	G_Pitch = Last_Pitch + Gyr[0]*dt;
	G_Roll = Last_Roll + Gyr[1]*dt;

	// Merge Accelerometer Angle and Gyro Angle using 'complementary filer'
	Pitch = .85*G_Pitch + .15*A_Pitch;
	Roll = .85*G_Roll + .15*A_Roll;

	// Create output array
	Angles[0] = Pitch;
	Angles[1] = Roll;
	Angles[2] = A_Pitch;
	Angles[3] = A_Roll;
	Angles[4] = 0;

	if (isnan(Angles[0]) == true)			// Debug Tool to identify divide by 0 caused by lack of accelerometer data
	{
		Angles[4] = 1;						// Debug indicator passed to the while loop to exit when a NaN is found
		cout << "Encountered Fatal Computation Error!" << endl;
		cout << "Last known values: " << endl;
		cout << "dt: " << dt << " Normal Vector: " << Norm << " IMU A_X: " << Acc[0] << " IMU G_X: " << Gyr[0] << endl;

	}

	// Log Data to Text File. Useful for plotting/analysis in Matlab or Excel
	ofstream Attitude_File;
	Attitude_File.open("Attitude_Log.txt", std::ofstream::out | std::ofstream::app); // open and go to end of file each write
	Attitude_File << dt << "\t" << A_Pitch << "\t" << A_Roll << "\t" << G_Pitch << "\t" << G_Roll << "\t" << Pitch << "\t" << Roll << "\n";
	Attitude_File.close();

	return Angles;
}

int main()
{
	double CurrentTime;
	double NewTime;
	double Elaps;
	double *Attitude;
	double Pitch;
	double Roll;
	int run = 0;

	SetupIMU();								   			// Initialize the IMU LSM9DS1 with some output to terminal
	cout << "Initializing IMU " << endl;
	sleep(1);
	cout << "Device Acquired." << endl;
	sleep(1);
    cout << "Accelerometer Initialized." << endl;
    sleep(1);
    cout << "Gyro Initialized." << endl;
    sleep(1);
    cout << "Bypass Mode Enabled." << endl;
    cout << "IMU Successfully Initialized." << endl;
    sleep(2);

	CurrentTime = GetTime();
	Attitude = GetAttitude(CurrentTime, 0, 0); // Current attitude, only use accelerometer data to start, assumes stationary IMU
	Pitch = *(Attitude + 2);
	Roll = *(Attitude + 3);

	while(run == 0)
	{
		Attitude = GetAttitude(CurrentTime, Pitch, Roll);
		CurrentTime = GetTime();
		Pitch = *(Attitude+0);
		Roll = *(Attitude+1);

		if (*(Attitude+4) == 1)				// Looks for the indicator created in GetAttitude
		{
			run = 1;   // Kills Loop
		}
		else
		{
			cout << "Pitch: " << Pitch << " Roll: " << Roll << endl;
		}
		// usleep(10000); uncomment to add delay in the loop speed. Will run as fast as possible while commented.
	}

	return 0;
}
