/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/**
 * HX711 Calibration offset
 *
 * @reboot_required true
 * @volatile true
 */
PARAM_DEFINE_FLOAT(HX711_FACTOR, 1.0);

/**
 * HX711 Max Load
 *
 * @min 0
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(HX711_OFFSET, 0);

/**
 * HX711 Output Topic
 *
 * Selects which uORB topic the HX711 publishes to.
 * 0 = payload_mass (default)
 * 1 = winch_mass
 *
 * @value 0 Payload Mass
 * @value 1 Winch Mass
 * @min 0
 * @max 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(HX711_TOPIC, 0);

/**
 * HX711 Max Load
 *
 * Max load the sensor can measure
 *
 * @min 0
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(HX711_MAX_LOAD, 20.0);
