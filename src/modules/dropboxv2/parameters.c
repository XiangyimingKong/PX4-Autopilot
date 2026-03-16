/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * Winch full speed in meters per second
 *
 * This parameter defines the full speed of the winch when operating at 100% PWM.
 * Used for calculating the distance traveled by the winch.
 *
 * @unit m/s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_SPD, 0.5f);

/**
 * Winch target descent distance
 *
 * The target distance the winch will descend during automatic deployment.
 * The winch will slow down at (target - 2m) and reach full extension at this distance.
 *
 * @unit m
 * @min 2.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_DIST, 15.0f);

/**
 * Enable door functionality
 *
 * Enables or disables door control and limit switch handling.
 * Set to 1 if doors are present, 0 if winch-only configuration.
 *
 * @value 0 Disabled (winch-only)
 * @value 1 Enabled (doors present)
 * @group Dropbox
 * @reboot_required true
 */
PARAM_DEFINE_INT32(DROPBOX_DOOR_EN, 0);

/**
 * Auto winch release mass threshold
 *
 * The winch mass must drop below this value (kg) to detect payload release.
 *
 * @unit kg
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_REL_THR, 0.5f);

/**
 * Auto winch retry mass fraction
 *
 * After ascending, if the winch mass exceeds this fraction of the saved payload mass,
 * the sequence retries with another descent.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_RTY_FRC, 1.0f);

/**
 * Auto winch post-release continue time
 *
 * How many seconds to continue descending after payload release is detected.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_POST_T, 0.5f);

/**
 * Auto winch retry descent time
 *
 * How many seconds to descend when retrying after a failed release check.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 1
 * @group Dropbox
 */
PARAM_DEFINE_FLOAT(WINCH_V2_RETRY_T, 1.5f);
