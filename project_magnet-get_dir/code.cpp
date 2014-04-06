/* (c) 2014 - Erik Regla Torres
 *
 *  This file is part of LSM303DLHC.
 *
 *  LSM303DLHC is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  LSM303DLHC is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with LSM303DLHC. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>

#include "LSM303DLHC.h"
#include "LSM303DLHC.cpp"

using namespace std;

#define RAD2DEG 57.2957795

int main(int argc, char **argv) {
    cout << "Reading magnetometer data... press [ENTER] to read" << endl;
    LSM303DLHC sensor("/dev/i2c-1");
    sensor.init_magnetometer();
    sensor.init_accelerometer();
    lsm303_t data;
    while (1) {
        sensor.read_magnetometer(&data);
        cout << "MAG: x:" << (data.x*RAD2DEG) << " y:" << (data.y*RAD2DEG) << " z:" << (data.z*RAD2DEG) << endl;
        sensor.read_accelerometer(&data);
        cout << "ACC: x:" << (data.x*RAD2DEG)  << " y:" << (data.y*RAD2DEG)  << " z:" << (data.z*RAD2DEG)  << endl;
        cin.ignore();
    }
    return 0;
}
