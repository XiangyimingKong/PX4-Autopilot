/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "hx711.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace hx711_loadcell
{

HX711 *g_dev{nullptr};



static int calibrate(float known_mass, uint8_t calibration_num_samples)
{
	if (g_dev == nullptr) {
		PX4_WARN("must be running to calibrate");
		return -1;
	}

	if( g_dev->start_calibration(known_mass, calibration_num_samples) != PX4_OK){
		PX4_WARN("failed to start calibration");
		return -1;
	}

	return 0;

}
static int tare(uint8_t calibration_num_samples)
{
	if (g_dev == nullptr) {
		PX4_WARN("must be running to calibrate");
		return -1;
	}

	if( g_dev->start_tare(calibration_num_samples) != PX4_OK){
		PX4_WARN("failed to start calibration");
		return -1;
	}

	return 0;

}

static int start(uint8_t gain, uint8_t num_samples)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	/* create the driver */
	g_dev = new HX711(gain, num_samples);
	if (g_dev == nullptr) {
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the FT Technologies Digital Wind Sensor FT742. This driver is required to operate alongside
a RS485 to UART signal transfer module.

Most boards are configured to enable/start the driver on a specified UART using the SENS_FTX_CFG parameter.

### Examples

Attempt to start driver on a specified serial device.
$ hx711 start -g 128 -a 10
Stop driver
$ hx711 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hx711", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Calibrate the loadcell");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int hx711_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;
	uint8_t gain = 128;
	uint8_t num_samples = 10;
	float known_mass = 0.0f;
	uint8_t calibration_num_samples = 10;

	while ((ch = px4_getopt(argc, argv, "g:a:m:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'g':
			gain = (uint8_t)atoi(myoptarg);
			break;
		case 'a':
			num_samples = (uint8_t)atoi(myoptarg);
			break;
		case 'm':
			calibration_num_samples = (uint8_t)atoi(myoptarg);
			break;
		case 'c':
			known_mass = (float)atof(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option");
			return hx711_loadcell::usage();
		}
	}

	if (myoptind >= argc) {
		hx711_loadcell::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return hx711_loadcell::start(gain, num_samples);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return hx711_loadcell::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return hx711_loadcell::status();
	}
	else if (!strcmp(argv[myoptind], "calibrate")) {
		return hx711_loadcell::calibrate(known_mass, calibration_num_samples);
	}
	else if (!strcmp(argv[myoptind], "tare")) {
		return hx711_loadcell::tare(calibration_num_samples);
	}

	hx711_loadcell::usage();
	return -1;
}
