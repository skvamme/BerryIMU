/*
        A simple program that demonstrates how to program a magnetometer
	on the Raspberry Pi and includes tilt compensation.
	http://ozzmaker.com/2014/12/01/compass1


    Copyright (C) 2014  Mark Williams

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Library General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    Library General Public License for more details.
    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free
    Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
    MA 02111-1307, USA
*/
#include <string.h>
#include <stdint.h>
#include "LSM9DS0.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include "erl_comm.c"

int file;

void writeMagReg(uint8_t reg, uint8_t value);
void writeAccReg(uint8_t reg, uint8_t value);
void readBlock(uint8_t command, uint8_t size, uint8_t *data);
void readMAG(int * m);
void readACC(int * a);

typedef unsigned char byte;

void signal_callback_handler(int signum) {
    exit(signum);
}


int main(int argc, char *argv[])

{
	char filename[20];
	int magRaw[3];
	int accRaw[3];
	float accXnorm,accYnorm,pitch,roll,magXcomp,magYcomp;
    double heading;
    byte buf[32];

    signal(SIGPIPE, signal_callback_handler);


	//Open the i2c bus
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
        exit(1);
	}


	//Select the magnetomoter
	if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0) {
        exit(1);
    }



	// Enable accelerometer.
        writeAccReg(CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
        writeAccReg(CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

        //Enable the magnetometer
        writeMagReg( CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
        writeMagReg( CTRL_REG6_XM, 0b01100000);   // +/-12gauss
        writeMagReg( CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode



	while(1)
	{
		readMAG(magRaw);
		readACC(accRaw);

		//If your IMU is upside down, comment out the two lines below which we correct the tilt calculation
		//accRaw[0] = -accRaw[0];
		//accRaw[1] = -accRaw[1];

		//Normalize accelerometer raw values.
        accXnorm = accRaw[0]/sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);
        accYnorm = accRaw[1]/sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);

		//Calculate pitch and roll
		pitch = asin(accXnorm);
		roll = -asin(accYnorm/cos(pitch));

		//Calculate the new tilt compensated values
		magXcomp = magRaw[0]*cos(pitch)+magRaw[02]*sin(pitch);
		magYcomp = magRaw[0]*sin(roll)*sin(pitch)+magRaw[1]*cos(roll)-magRaw[2]*sin(roll)*cos(pitch);


		//Calculate heading
		heading = 180*atan2(magYcomp,magXcomp)/M_PI;

                //Convert heading to 0 - 360
		if(heading < 0)
		      heading += 360;


        memcpy(buf, (unsigned char*) (&heading), 8);
        write_cmd(buf, 8);

		//Sleep for 25ms
		usleep(25000);

	}

}

void selectDevice(int file, int addr)
{
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
        exit(1);
        }
}


void writeAccReg(uint8_t reg, uint8_t value)
{
  selectDevice(file,ACC_ADDRESS);

  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        exit(1);
    }
}



void writeMagReg(uint8_t reg, uint8_t value)
{
  selectDevice(file,MAG_ADDRESS);
  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        exit(1);
    }
}

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    if (result != size)
    {
        exit(1);
    }
}


void readACC(int  *a)
{
        uint8_t block[6];
        selectDevice(file,ACC_ADDRESS);
                readBlock(0x80 | OUT_X_L_A, sizeof(block), block);

        *a = (int16_t)(block[0] | block[1] << 8);
        *(a+1) = (int16_t)(block[2] | block[3] << 8);
        *(a+2) = (int16_t)(block[4] | block[5] << 8);

}

void readMAG(int  *m)
{
        uint8_t block[6];

        readBlock(0x80 | OUT_X_L_M, sizeof(block), block);

        *m = (int16_t)(block[0] | block[1] << 8);
        *(m+1) = (int16_t)(block[2] | block[3] << 8);
        *(m+2) = (int16_t)(block[4] | block[5] << 8);

}


