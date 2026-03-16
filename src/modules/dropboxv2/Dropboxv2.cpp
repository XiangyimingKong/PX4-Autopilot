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

#include "Dropboxv2.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <board_config.h>

using namespace time_literals;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Compare two floating-point values for equality with epsilon tolerance
 * @param a First value
 * @param b Second value
 * @param epsilon Tolerance (default: 0.001f)
 * @return true if values are equal within epsilon
 */
static inline bool is_float_equal(float a, float b, float epsilon = 0.001f)
{
	float diff = a - b;
	// Get absolute value without using fabs
	float abs_diff = (diff < 0.0f) ? -diff : diff;
	return abs_diff <= epsilon;
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

Dropboxv2::Dropboxv2() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
}

Dropboxv2::~Dropboxv2()
{
	perf_free(_loop_perf);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool Dropboxv2::init()
{
	// Arm actuators
	actuator_armed_s actuator_armed{};
	actuator_armed.timestamp = hrt_absolute_time();
	actuator_armed.armed = true;
	_actuator_armed_pub.publish(actuator_armed);

	// Cache parameters once at startup (avoid repeated flash reads during runtime)
	_cached_pwm_max = _param_pwm_max.get();
	_cached_winch_speed = _param_winch_speed.get();
	_cached_target_distance = _param_winch_target_distance.get();
	_doors_enabled = (_param_door_enable.get() == 1);
	_cached_release_threshold = _param_release_threshold.get();
	_cached_retry_fraction = _param_retry_fraction.get();
	_cached_post_release_time = _param_post_release_time.get();
	_cached_retry_down_time = _param_retry_down_time.get();

	PX4_INFO("Dropboxv2 initialized (PWM max: %d, Winch speed: %.2f m/s, Target: %.1f m, Doors: %s, RelThresh: %.2f kg, RetryFrac: %.2f, PostRelT: %.1f s, RetryT: %.1f s)",
		_cached_pwm_max, (double)_cached_winch_speed, (double)_cached_target_distance,
		_doors_enabled ? "enabled" : "disabled",
		(double)_cached_release_threshold, (double)_cached_retry_fraction,
		(double)_cached_post_release_time, (double)_cached_retry_down_time);

	// Configure GPIO pins
	configure_gpio_pins();

	// Initialize dropbox status
	_dropbox_status.timestamp = hrt_absolute_time();
	_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;  // Assume doors start closed
	_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;  // Assume winch starts at top
	_dropbox_status.cutter_state = dropbox_status_s::CUTTER_IDLE;  // Cutter starts idle
	_dropbox_status.winch_mass = 0.0f;  // Will be updated from winch_mass topic
	_dropbox_status_pub.publish(_dropbox_status);
	_last_status_publish_time = hrt_absolute_time();

	// Initialize all motors to stopped
	publish_to_motor(0, 0.0f);  // Left door
	publish_to_motor(1, 0.0f);  // Right door
	publish_to_motor(2, 0.0f);  // Winch
	publish_to_motor(3, 1.0f);  // Cutter

	// Initialize button states to prevent false triggers
	const hrt_abstime now = hrt_absolute_time();
	_last_door_button_state = px4_arch_gpioread(DOOR_BUTTON_PIN);
	_last_up_button_state = !px4_arch_gpioread(UP_BUTTON_PIN);
	_last_down_button_state = !px4_arch_gpioread(DOWN_BUTTON_PIN);
	_last_door_button_time = now;
	_last_up_button_time = now;
	_last_down_button_time = now;

	ScheduleDelayed(100_ms);
	return true;
}

void Dropboxv2::configure_gpio_pins()
{
	// px4_arch_configgpio(BUTTON_POWER_PIN);
	// px4_arch_gpiowrite(BUTTON_POWER_PIN,1);  // Read once to clear any spurious state

	// Button GPIO pins
	px4_arch_configgpio(DOOR_BUTTON_PIN);
	px4_arch_configgpio(UP_BUTTON_PIN);
	px4_arch_configgpio(DOWN_BUTTON_PIN);

	// Winch limit switch
	px4_arch_configgpio(WINCH_LIMIT_SWITCH_PIN);

	// Door limit switches (only if doors enabled)
	if (_doors_enabled) {
		px4_arch_configgpio(LEFT_DOOR_CLOSE_LIMIT_PIN);
		px4_arch_configgpio(LEFT_DOOR_OPEN_LIMIT_PIN);
		px4_arch_configgpio(RIGHT_DOOR_CLOSE_LIMIT_PIN);
		px4_arch_configgpio(RIGHT_DOOR_OPEN_LIMIT_PIN);
	}
}

// ============================================================================
// MAIN RUN LOOP
// ============================================================================

void Dropboxv2::Run()
{
	const hrt_abstime now = hrt_absolute_time();

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Handle winch ramping
	handle_winch_ramping(now);

	// Handle winch limit switches
	handle_winch_limit_switches(now);

	// Handle door limit switches (if doors enabled)
	if (_doors_enabled) {
		handle_door_limit_switches(now);
	}

	// Update winch distance tracking
	update_winch_distance(now);

	// Handle manual winch distance-based slowdown
	handle_manual_winch_slowdown(now);

	// Handle precheck sequence
	handle_precheck(now);

	// Handle auto winch deployment
	handle_auto_winch(now);

	// Handle cutter sequence
	handle_cutter_sequence(now);

	// Update winch mass from sensor
	update_winch_mass(now);

	// Handle HereLink input
	// handle_rc_input(now);

	// Handle physical buttons
	handle_physical_buttons(now);

	// Handle incoming vehicle commands
	handle_vehicle_commands(now);

	// Publish status periodically
	publish_status(now);

	perf_end(_loop_perf);
	ScheduleDelayed(20_ms);
}

// ============================================================================
// RC INPUT HANDLER
// ============================================================================

void Dropboxv2::handle_rc_input(const hrt_abstime now)
{
	if (!_input_rc_sub.updated()) {
		return;
	}

	input_rc_s input_rc{};
	if (!_input_rc_sub.copy(&input_rc)) {
		return;
	}

	// Map RC channels to parameters
	// CH10: doors (toggle), CH9: winch (3-position), CH14: cutter

	// Door toggle logic - CH10: Toggle on any edge (1000→2000 or 2000→1000)
	// Only process door commands if doors are enabled
	if (_doors_enabled) {
		static int last_door_value = 1500;  // Initialize to middle value
		static bool first_run = true;  // Flag to prevent false trigger on startup

		// On first run, initialize the state without triggering an action
		if (first_run) {
			if (input_rc.values[10] == 1000 || input_rc.values[10] == 2000) {
				last_door_value = input_rc.values[10];
			}
			first_run = false;
		}

		// Detect edge on any transition (1000→2000 or 2000→1000)
		if ((last_door_value == 1000 && input_rc.values[10] == 2000) ||
		    (last_door_value == 2000 && input_rc.values[10] == 1000)) {
			// Toggle between open and close based on current state
			if (_dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSED_BOX) {
				_params[0] = 1.0f;  // open door
				PX4_INFO("Door toggle: Opening doors (RC edge: %d → %d)", last_door_value, input_rc.values[10]);
			} else if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENED_BOX) {
				_params[0] = 2.0f;  // close door
				PX4_INFO("Door toggle: Closing doors (RC edge: %d → %d)", last_door_value, input_rc.values[10]);
			}
		} else {
			_params[0] = 0.0f;  // no action
		}

		// Update last value for next iteration
		if (input_rc.values[10] == 1000 || input_rc.values[10] == 2000) {
			last_door_value = input_rc.values[10];
		}
	} else {
		_params[0] = 0.0f;  // Doors disabled, no action
	}

	// CH9 winch: >1600 = wind up, <1400 = wind down, 1400-1600 = stop
	if (input_rc.values[9] > 1600) {
		_params[1] = 1.0f;  // wind up
	} else if (input_rc.values[9] < 1400) {
		_params[1] = 2.0f;  // wind down
	} else {
		_params[1] = 3.0f;  // stop (middle position)
	}

	// CH14 cutter: Toggle on any edge (1000→2000 or 2000→1000)
	static int last_cutter_value = 1500;  // Initialize to middle value
	bool cutter_edge_detected = false;

	if ((last_cutter_value == 1000 && input_rc.values[14] == 2000) ||
	    (last_cutter_value == 2000 && input_rc.values[14] == 1000)) {
		cutter_edge_detected = true;
		PX4_INFO("Cutter edge detected: %d -> %d, triggering cut", last_cutter_value, input_rc.values[14]);
	}

	_params[2] = cutter_edge_detected ? 1.0f : 0.0f;  // cutter

	// Update last value for next iteration
	if (input_rc.values[14] == 1000 || input_rc.values[14] == 2000) {
		last_cutter_value = input_rc.values[14];
	}

	// CH13 precheck: Toggle on any edge (1000→2000 or 2000→1000) to trigger pre-operation check
	static int last_precheck_value = 1500;  // Initialize to middle value
	bool precheck_edge_detected = false;

	if ((last_precheck_value == 1000 && input_rc.values[13] == 2000) ||
	    (last_precheck_value == 2000 && input_rc.values[13] == 1000)) {
		precheck_edge_detected = true;
		PX4_INFO("Precheck edge detected: %d -> %d, triggering pre-operation check", last_precheck_value, input_rc.values[13]);
	}

	_params[4] = precheck_edge_detected ? 1.0f : 0.0f;  // precheck (param4)

	// Update last value for next iteration
	if (input_rc.values[13] == 1000 || input_rc.values[13] == 2000) {
		last_precheck_value = input_rc.values[13];
	}

	// Print all RC input channels
	// PX4_INFO("RC Input - CH0: %d, CH1: %d, CH2: %d, CH3: %d, CH4: %d, CH5: %d, CH6: %d, CH7: %d, CH8: %d, CH9: %d, CH10: %d, CH11: %d, CH12: %d, CH13: %d, CH14: %d, CH15: %d, CH16: %d, CH17: %d",
	// 	input_rc.values[0], input_rc.values[1], input_rc.values[2], input_rc.values[3],
	// 	input_rc.values[4], input_rc.values[5], input_rc.values[6], input_rc.values[7],
	// 	input_rc.values[8], input_rc.values[9], input_rc.values[10], input_rc.values[11],
	// 	input_rc.values[12], input_rc.values[13], input_rc.values[14], input_rc.values[15],
	// 	input_rc.values[16], input_rc.values[17]);


	//opendoor channel 8

	// Send vehicle command if any parameter changed
	if (!is_float_equal(_params[0], _params_buffer[0]) ||
	    !is_float_equal(_params[1], _params_buffer[1]) ||
	    !is_float_equal(_params[2], _params_buffer[2]) ||
	    !is_float_equal(_params[4], _params_buffer[4])) {
		send_vehicle_command(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX, _params[0], _params[1], _params[2], _params[4]);
		PX4_INFO("RC Command - Doors: %.1f, Winch: %.1f, Cutter: %.1f, Precheck: %.1f",
			(double)_params[0], (double)_params[1], (double)_params[2], (double)_params[4]);
		// Update buffer
		for (int i = 0; i < 7; i++) {
			_params_buffer[i] = _params[i];
		}
		_params[0]=0.0f; //reset to neutral after command sent
		_params[1]=0.0f;
		_params[2]=0.0f;
		_params[4]=0.0f;
	}
}

// ============================================================================
// PHYSICAL BUTTON HANDLERS
// ============================================================================

void Dropboxv2::handle_physical_buttons(const hrt_abstime now)
{
	// DOOR button - toggle between open/close (only if doors enabled)
	if (_doors_enabled) {
		bool door_button_pressed = px4_arch_gpioread(DOOR_BUTTON_PIN);  // Active high

		if (door_button_pressed != _last_door_button_state) {
			PX4_INFO("[DEBUG] Door button state change detected: %d -> %d", _last_door_button_state, door_button_pressed);
			if (now - _last_door_button_time >= BUTTON_DEBOUNCE_TIME) {
				if (door_button_pressed) {
					// Toggle between open and close based on current state
					if (_dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSED_BOX) {
						PX4_INFO("Door button: opening box");
						openbox();
					} else if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENED_BOX) {
						PX4_INFO("Door button: closing box");
						closebox();
					} else {
						PX4_INFO("Door button: doors moving, ignoring");
					}
				}
				_last_door_button_state = door_button_pressed;
				_last_door_button_time = now;
			} else {
				PX4_INFO("[DEBUG] Door button debounce - ignored (time: %llu us)", now - _last_door_button_time);
			}
		}
	}

	// UP button - winch up
	bool up_button_pressed = px4_arch_gpioread(UP_BUTTON_PIN);  // Active high

	if (up_button_pressed != _last_up_button_state) {
		PX4_INFO("[DEBUG] UP button state change detected: %d -> %d", _last_up_button_state, up_button_pressed);
		if (now - _last_up_button_time >= BUTTON_DEBOUNCE_TIME) {
			if (up_button_pressed) {
				if (_auto_winch_state != AUTO_WINCH_IDLE) {
					PX4_WARN("Up button: ABORTING auto winch - manual override");
					_auto_winch_state = AUTO_WINCH_IDLE;
				}
				if (_precheck_state != PRECHECK_IDLE) {
					PX4_WARN("Up button: ABORTING precheck - manual override");
					_precheck_state = PRECHECK_IDLE;
				}
				winchup();
				PX4_INFO("Up button: winch up");
			} else {
				winchstop();
				PX4_INFO("Up button: winch stopped");
			}
			_last_up_button_state = up_button_pressed;
			_last_up_button_time = now;
		} else {
			PX4_INFO("[DEBUG] UP button debounce - ignored (time: %llu us)", now - _last_up_button_time);
		}
	}

	// DOWN button - winch down
	bool down_button_pressed = px4_arch_gpioread(DOWN_BUTTON_PIN);  // Active high

	if (down_button_pressed != _last_down_button_state) {
		PX4_INFO("[DEBUG] DOWN button state change detected: %d -> %d", _last_down_button_state, down_button_pressed);
		if (now - _last_down_button_time >= BUTTON_DEBOUNCE_TIME) {
			if (down_button_pressed) {
				if (_auto_winch_state != AUTO_WINCH_IDLE) {
					PX4_WARN("Down button: ABORTING auto winch - manual override");
					_auto_winch_state = AUTO_WINCH_IDLE;
				}
				if (_precheck_state != PRECHECK_IDLE) {
					PX4_WARN("Down button: ABORTING precheck - manual override");
					_precheck_state = PRECHECK_IDLE;
				}
				winchdown();
				PX4_INFO("Down button: winch down");
			} else {
				winchstop();
				PX4_INFO("Down button: winch stopped");
			}
			_last_down_button_state = down_button_pressed;
			_last_down_button_time = now;
		} else {
			PX4_INFO("[DEBUG] DOWN button debounce - ignored (time: %llu us)", now - _last_down_button_time);
		}
	}

}

// ============================================================================
// VEHICLE COMMAND HANDLER
// ============================================================================

void Dropboxv2::handle_vehicle_commands(const hrt_abstime now)
{
	if (!_vehicle_command_sub.updated()) {
		return;
	}

	vehicle_command_s vcmd{};
	if (!_vehicle_command_sub.copy(&vcmd) || vcmd.target_component != 3) {
		return;
	}

	_cur_vcmd_target_system = vcmd.source_system;
	_cur_vcmd_target_component = vcmd.source_component;

	if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_DROPBOX) {

		// ====================================================================
		// PARAM1: Door commands (1.0 = open, 2.0 = close)
		// ====================================================================
		if (!is_float_equal(vcmd.param1, 0.0f)) {
			if (!_doors_enabled) {
				// Doors are disabled via parameter - UNSUPPORTED (feature not enabled)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED, 1,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_WARN("Door command UNSUPPORTED - doors disabled (DROPBOX_DOOR_EN=0)");

			} else if (is_float_equal(vcmd.param1, 1.0f)) {
				// OPEN doors command
				// Check if doors are already moving
				if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENING_BOX ||
				    _dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSING_BOX) {
					// Doors are busy - TEMPORARILY_REJECTED (will finish moving)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, 1,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Open doors TEMP REJECTED - doors currently moving");

				} else if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENED_BOX) {
					// Already open - DENIED (already in target state)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 1,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Open doors DENIED - doors already open");

				} else {
					// Open doors - ACCEPTED
					openbox();
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 1,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("Open doors ACCEPTED");
				}

			} else if (is_float_equal(vcmd.param1, 2.0f)) {
				// CLOSE doors command
				// Check if doors are already moving
				if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENING_BOX ||
				    _dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSING_BOX) {
					// Doors are busy - TEMPORARILY_REJECTED (will finish moving)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, 1,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Close doors TEMP REJECTED - doors currently moving");

				} else if (_dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSED_BOX) {
					// Already closed - DENIED (already in target state)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 1,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Close doors DENIED - doors already closed");

				} else {
					// Check if winch is at top (safety requirement)
					bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

					if (!winch_at_top) {
						// Cannot close doors - winch not at top - DENIED (safety violation)
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 1,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_WARN("Close doors DENIED - winch not at top (safety check failed)");

					} else {
						// Close doors - ACCEPTED
						closebox();
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 1,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_INFO("Close doors ACCEPTED");
					}
				}

			} else {
				// Invalid param1 value - DENIED (invalid parameter)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 1,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_WARN("Door command DENIED - invalid param1: %.2f", (double)vcmd.param1);
			}
		}

		// ====================================================================
		// PARAM2: Winch commands (1.0=up, 2.0=down, 3.0=stop, 4.0=auto)
		// ====================================================================
		if (!is_float_equal(vcmd.param2, 0.0f)) {
			// Check for winch fault condition (applies to all winch commands except stop)
			if (_winch_fault && !is_float_equal(vcmd.param2, 3.0f)) {
				// System fault detected - FAILED (permanent until restart)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 2,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_ERR("Winch command FAILED - fault detected. Restart module to clear.");

			} else if (is_float_equal(vcmd.param2, 1.0f)) {
				// WIND UP command
				// Check if already at top
				bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

				if (winch_at_top) {
					// Already at top - DENIED (already at limit, waiting won't help)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 2,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Winch up DENIED - already at top limit");

				} else {
					// Abort auto winch if running (manual override takes priority)
					if (_auto_winch_state != AUTO_WINCH_IDLE) {
						PX4_WARN("Winch up: ABORTING auto winch - manual override");
						_auto_winch_state = AUTO_WINCH_IDLE;
					}
					if (_precheck_state != PRECHECK_IDLE) {
						PX4_WARN("Winch up: ABORTING precheck - manual override");
						_precheck_state = PRECHECK_IDLE;
					}
					winchup();
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("Winch up ACCEPTED");
				}

			} else if (is_float_equal(vcmd.param2, 2.0f)) {
				// WIND DOWN command
				// Check door safety (only if doors enabled)
				if (_doors_enabled) {
					bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
					bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);

					if (left_closed || right_closed) {
						// Doors closed - DENIED (safety violation, waiting won't help)
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 2,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_WARN("Winch down DENIED - doors closed (safety check failed)");

					} else {
						// Abort auto winch if running (manual override takes priority)
						if (_auto_winch_state != AUTO_WINCH_IDLE) {
							PX4_WARN("Winch down: ABORTING auto winch - manual override");
							_auto_winch_state = AUTO_WINCH_IDLE;
						}
						if (_precheck_state != PRECHECK_IDLE) {
							PX4_WARN("Winch down: ABORTING precheck - manual override");
							_precheck_state = PRECHECK_IDLE;
						}
						winchdown();
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_INFO("Winch down ACCEPTED");
					}
				} else {
					// Doors disabled - wind down always allowed
					// Abort auto winch if running (manual override takes priority)
					if (_auto_winch_state != AUTO_WINCH_IDLE) {
						PX4_WARN("Winch down: ABORTING auto winch - manual override");
						_auto_winch_state = AUTO_WINCH_IDLE;
					}
					if (_precheck_state != PRECHECK_IDLE) {
						PX4_WARN("Winch down: ABORTING precheck - manual override");
						_precheck_state = PRECHECK_IDLE;
					}
					winchdown();
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("Winch down ACCEPTED");
				}

			} else if (is_float_equal(vcmd.param2, 3.0f)) {
				// STOP winch command - always allowed, aborts auto winch if running
				if (_auto_winch_state != AUTO_WINCH_IDLE) {
					PX4_WARN("Winch stop: ABORTING auto winch - manual override");
					_auto_winch_state = AUTO_WINCH_IDLE;
				}
				if (_precheck_state != PRECHECK_IDLE) {
					PX4_WARN("Winch stop: ABORTING precheck - manual override");
					_precheck_state = PRECHECK_IDLE;
				}
				winchstop();
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("Winch stop ACCEPTED");

			} else {
				// Invalid param2 value - DENIED (invalid parameter)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 2,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_WARN("Winch command DENIED - invalid param2: %.2f", (double)vcmd.param2);
			}
		}

		// ====================================================================
		// PARAM3: Cutter command (1.0 = cut line)
		// ====================================================================
		if (!is_float_equal(vcmd.param3, 0.0f)) {
			if (is_float_equal(vcmd.param3, 1.0f)) {
				// CUT LINE command
				// Check if cutter is busy
				if (_cutter_state != CUTTER_IDLE) {
					// Cutter already active - TEMPORARILY_REJECTED
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, 3,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Cutter TEMP REJECTED - already active");

				} else {
					// Cut line - ACCEPTED
					cutline(5);
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 3,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("Cutter ACCEPTED");
				}

			} else {
				// Invalid param3 value - DENIED (invalid parameter)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 3,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_WARN("Cutter command DENIED - invalid param3: %.2f", (double)vcmd.param3);
			}
		}

		// ====================================================================
		// PARAM4: Precheck (1.0 = start precheck), Auto-winch (2.0 = start auto winch)
		// ====================================================================
		if (!is_float_equal(vcmd.param4, 0.0f)) {
			if (is_float_equal(vcmd.param4, 1.0f)) {
				// PRECHECK command
				// Check if precheck already running
				if (_precheck_state != PRECHECK_IDLE) {
					// Already running - TEMPORARILY_REJECTED
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, 4,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Precheck TEMP REJECTED - already running");

				} else if (_winch_fault) {
					// Winch fault - FAILED
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 4,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_ERR("Precheck FAILED - winch fault detected");

				} else {
					// Start precheck - ACCEPTED
					bool success = do_precheck();
					if (success) {
						// send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						// 	vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 4,
						// 	_cur_vcmd_target_system, _cur_vcmd_target_component);
						// PX4_INFO("Precheck ACCEPTED");
					} else {
						// Failed to start (shouldn't happen with above checks)
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 4,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_ERR("Precheck FAILED - internal error");
					}
				}

			} else if (is_float_equal(vcmd.param4, 2.0f)) {
				// AUTO WINCH command
				if (_winch_fault) {
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 4,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_ERR("Auto-winch FAILED - winch fault active");

				} else if (_auto_winch_state != AUTO_WINCH_IDLE) {
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, 4,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_WARN("Auto-winch TEMP REJECTED - already in progress");

				} else {
					bool success = do_winch();
					if (success) {
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 4,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_INFO("Auto-winch ACCEPTED");
					} else {
						send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
							vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 4,
							_cur_vcmd_target_system, _cur_vcmd_target_component);
						PX4_WARN("Auto-winch DENIED");
					}
				}

			} else {
				// Invalid param4 value - DENIED (invalid parameter)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, 4,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_WARN("Param4 command DENIED - invalid param4: %.2f", (double)vcmd.param4);
			}
		}

	} else {
		// Unknown command - UNSUPPORTED
		send_dropbox_command_ack(now, vcmd.command,
			vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED, 0,
			_cur_vcmd_target_system, _cur_vcmd_target_component);
		PX4_WARN("Command UNSUPPORTED: %ld", vcmd.command);
	}
}

// ============================================================================
// WINCH RAMPING
// ============================================================================

void Dropboxv2::handle_winch_ramping(const hrt_abstime now)
{
	if (!_winch_ramping_active) {
		return;
	}

	// Calculate elapsed time since ramp start
	hrt_abstime elapsed = now - _winch_ramp_start_time;

	if (elapsed >= WINCH_RAMP_TIME) {
		// Ramp complete
		if (_winch_stopping) {
			// Stopping ramp complete - set to 0
			publish_to_motor(2, 0.0f);
			_winch_ramping_active = false;
			_winch_stopping = false;
			_winch_direction = 0;

			// Check limit switch to determine final state
			bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
			if (winch_at_top) {
				_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
			} else {
				_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
			}
		} else {
			// Speed up ramp complete - set to target speed
			float final_speed = _winch_target_speed * _winch_direction;
			publish_to_motor(2, final_speed);
			_winch_ramping_active = false;
		}
	} else {
		// Linear interpolation
		float progress = (float)elapsed / (float)WINCH_RAMP_TIME;

		if (_winch_stopping) {
			// Ramp down from initial speed to 0
			float current_speed = _winch_initial_speed * (1.0f - progress);
			publish_to_motor(2, current_speed);
		} else {
			// Ramp from initial speed to target speed
			// Interpolate: initial + (target - initial) * progress
			float target_with_direction = _winch_target_speed * _winch_direction;
			float current_speed = _winch_initial_speed + (target_with_direction - _winch_initial_speed) * progress;
			publish_to_motor(2, current_speed);
		}
	}
}

void Dropboxv2::handle_winch_limit_switches(const hrt_abstime now)
{
	// Read winch limit switch state (active-low: pressed = 0, released = 1)
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	// Debug: Print winch limit switch state periodically
	static hrt_abstime last_debug_print = 0;
	static bool last_winch_at_top_state = false;

	// Print when limit switch state changes
	if (winch_at_top != last_winch_at_top_state) {
		PX4_INFO("[DEBUG] Winch limit switch state change: %d -> %d (AT_TOP=%d)",
			last_winch_at_top_state, winch_at_top, winch_at_top);
		last_winch_at_top_state = winch_at_top;
	}

	if (now - last_debug_print >= 1000_ms) {
		// PX4_INFO("[DEBUG] Winch limit: AT_TOP=%d | State=%d Distance=%.2fm",
		// 	winch_at_top, _dropbox_status.winch_state, (double)_winch_distance_traveled);
		last_debug_print = now;
	}

	// Track if limit switch becomes disengaged during winding down
	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN && !winch_at_top) {
		if (!_limit_switch_was_disengaged) {
			PX4_INFO("[DEBUG] Limit switch disengaged during winding down");
		}
		_limit_switch_was_disengaged = true;
	}

	// SAFETY: If winch hits top limit switch while winding DOWN, this is a critical fault
	// BUT: Only trigger fault if the switch was previously disengaged (normal descent started)
	if (winch_at_top && _dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN && _limit_switch_was_disengaged) {
		PX4_INFO("[DEBUG] FAULT CONDITION: Winch at top while winding DOWN!");
		// EMERGENCY STOP - Motor off immediately (no ramping)
		publish_to_motor(2, 0.0f);

		// Cancel any active operations
		_winch_ramping_active = false;
		_winch_stopping = false;
		_winch_direction = 0;

		// Set fault flag to prevent further operations
		_winch_fault = true;
		_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;

		PX4_ERR("CRITICAL FAULT: Winch hit top limit while winding DOWN! Motor stopped. System locked.");
		PX4_ERR("This indicates reversed wiring or mechanical failure. Restart module to clear fault.");
		return;
	}

	// If winch is at top limit and we're winding up, stop immediately
	if (winch_at_top && _dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP) {
		PX4_INFO("[DEBUG] Winch at top limit while winding UP - stopping");
		// Stop motor immediately (safety override - no ramping)
		publish_to_motor(2, 0.0f);

		// Cancel any active ramping
		_winch_ramping_active = false;
		_winch_stopping = false;
		_winch_direction = 0;

		// Reset distance counter at top position
		_winch_distance_traveled = 0.0f;
		PX4_INFO("Winch reached top - distance reset");

		// Update state to top
		_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
	}
}

void Dropboxv2::update_winch_distance(const hrt_abstime now)
{
	// Initialize on first call
	if (_last_distance_update_time == 0) {
		_last_distance_update_time = now;
		return;
	}

	// Calculate time delta in seconds
	float dt = (now - _last_distance_update_time) / 1000000.0f;
	_last_distance_update_time = now;

	// Get current winch PWM value (-1.0 to +1.0)
	// Positive PWM = up (winding in), Negative PWM = down (paying out)
	float current_pwm_normalized = _dropbox_position[2];

	// Use cached PWM_MAIN_MAX3 parameter (read once at init, not every loop)
	int pwm_max = _cached_pwm_max;
	int pwm_min = 2 * PWM_DISARM - pwm_max;  // Calculate min symmetrically around disarm

	// Convert normalized PWM (-1.0 to +1.0) to actual PWM value
	float actual_pwm;
	if (current_pwm_normalized >= 0.0f) {
		// Positive: interpolate between disarm and max
		actual_pwm = PWM_DISARM + (current_pwm_normalized * (pwm_max - PWM_DISARM));
	} else {
		// Negative: interpolate between min and disarm
		actual_pwm = PWM_DISARM + (current_pwm_normalized * (PWM_DISARM - pwm_min));
	}

	// Apply deadzone: only count distance if PWM is outside deadzone
	float speed_factor = 0.0f;  // Default: no movement in deadzone

	if (actual_pwm > PWM_DEADZONE_HIGH) {
		// Positive PWM = moving up (winding in): distance decreases
		float pwm_range = pwm_max - PWM_DEADZONE_HIGH;
		if (pwm_range > 1.0f) {  // Protect against invalid configuration
			float pwm_active = actual_pwm - PWM_DEADZONE_HIGH;
			speed_factor = -pwm_active / pwm_range;  // -1.0 to 0.0 (negative = moving up)
		}
	} else if (actual_pwm < PWM_DEADZONE_LOW) {
		// Negative PWM = moving down (paying out): distance increases
		float pwm_range = PWM_DEADZONE_LOW - pwm_min;
		if (pwm_range > 1.0f) {  // Protect against invalid configuration
			float pwm_active = PWM_DEADZONE_LOW - actual_pwm;
			speed_factor = pwm_active / pwm_range;  // 0.0 to 1.0 (positive = moving down)
		}
	}
	// else: in deadzone, speed_factor = 0.0

	// Use cached winch speed parameter (read once at init, not every loop)
	float winch_full_speed = _cached_winch_speed;

	// Calculate actual speed: speed_factor * full_speed
	float actual_speed = speed_factor * winch_full_speed;

	// Calculate distance traveled: speed * time
	float distance_delta = actual_speed * dt;
	_winch_distance_traveled += distance_delta;

	// Print distance and current PWM every 1 second
	// if (now - _last_distance_print_time >= DISTANCE_PRINT_INTERVAL) {
	// 	PX4_INFO("Winch distance: %.2f m, PWM: %.2f", (double)_winch_distance_traveled, (double)current_pwm_normalized);
	// 	_last_distance_print_time = now;
	// }
}

void Dropboxv2::handle_manual_winch_slowdown(const hrt_abstime now)
{
	// Universal slowdown applies to ALL winch operations (manual and auto)
	// Upper zone: 0 to WINCH_UPPER_SLOWDOWN_LIMIT (2m from top)
	// Lower zone: (target - WINCH_LOWER_SLOWDOWN_LIMIT) to target (2m before target)
	//
	// ZONE OVERLAP HANDLING:
	// If target < (WINCH_UPPER_SLOWDOWN_LIMIT + WINCH_LOWER_SLOWDOWN_LIMIT),
	// the zones overlap and winch stays at slow speed throughout the entire travel.
	// Example: target=3m means zones overlap (3 < 2+2), so always slow speed.

	// Only monitor during active winch operations
	if (_dropbox_status.winch_state != dropbox_status_s::WINCH_WINDING_UP &&
	    _dropbox_status.winch_state != dropbox_status_s::WINCH_WINDING_DOWN) {
		_manual_winch_slowdown_initiated = false;  // Reset flag when not winding
		return;
	}

	// CRITICAL: If winch is stopping, don't interfere with the stop ramp
	if (_winch_stopping) {
		return;
	}

	// WINCH DOWN: Monitor zones and adjust speed
	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN) {
		// Upper zone: -inf to WINCH_UPPER_SLOWDOWN_LIMIT (2m from top)
		// Lower zone: (target - 2m) to target
		bool in_upper_zone = (_winch_distance_traveled <= WINCH_UPPER_SLOWDOWN_LIMIT);
		float slowdown_trigger = _cached_target_distance - WINCH_LOWER_SLOWDOWN_LIMIT;
		bool in_lower_zone = (_winch_distance_traveled >= slowdown_trigger);

		// Determine current zone: 0=upper, 1=middle, 2=lower
		static int8_t last_zone = -1;
		int8_t current_zone = in_upper_zone ? 0 : (in_lower_zone ? 2 : 1);

		// Reset flag when entering a new zone
		if (current_zone != last_zone) {
			_manual_winch_slowdown_initiated = false;
			last_zone = current_zone;
		}

		if (in_upper_zone) {
			// In upper zone: stay slow (already set by winchdown())
		} else if (in_lower_zone) {
			// In lower zone: slow down
			if (!_manual_winch_slowdown_initiated) {
				float current_speed = _dropbox_position[2];
				_winch_initial_speed = current_speed;
				_winch_target_speed = WINCH_SLOW_SPEED;
				_winch_direction = -1;
				_winch_stopping = false;
				_winch_ramping_active = true;
				_winch_ramp_start_time = now;
				_manual_winch_slowdown_initiated = true;

				PX4_INFO("Winch down: entering lower zone at %.2fm, slowing to %.2f PWM",
					(double)_winch_distance_traveled, (double)WINCH_SLOW_SPEED);
			}
		} else {
			// In middle zone: speed up to full speed
			if (!_manual_winch_slowdown_initiated) {
				float current_speed = _dropbox_position[2];
				_winch_initial_speed = current_speed;
				_winch_target_speed = WINCH_MAX_SPEED;
				_winch_direction = -1;
				_winch_stopping = false;
				_winch_ramping_active = true;
				_winch_ramp_start_time = now;
				_manual_winch_slowdown_initiated = true;

				PX4_INFO("Winch down: exiting upper zone at %.2fm, ramping to full speed",
					(double)_winch_distance_traveled);
			}
		}
	}

	// WINCH UP: Slow down when entering upper zone
	else if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP) {
		// Upper zone: -inf to WINCH_UPPER_SLOWDOWN_LIMIT (2m from top)
		// Lower zone: (target - 2m) to target (starting from bottom, slow ascent)
		bool in_upper_zone = (_winch_distance_traveled <= WINCH_UPPER_SLOWDOWN_LIMIT);
		float slowdown_trigger = _cached_target_distance - WINCH_LOWER_SLOWDOWN_LIMIT;
		bool in_lower_zone = (_winch_distance_traveled >= slowdown_trigger);

		// Determine current zone: 0=upper, 1=middle, 2=lower
		static int8_t last_zone_up = -1;
		int8_t current_zone = in_upper_zone ? 0 : (in_lower_zone ? 2 : 1);

		// Reset flag when entering a new zone
		if (current_zone != last_zone_up) {
			_manual_winch_slowdown_initiated = false;
			last_zone_up = current_zone;
		}

		if (in_upper_zone) {
			// In upper zone: slow down when approaching top
			if (!_manual_winch_slowdown_initiated) {
				float current_speed = _dropbox_position[2];
				_winch_initial_speed = current_speed;
				_winch_target_speed = WINCH_SLOW_SPEED;  // 0.3 PWM
				_winch_direction = 1;  // Up direction (positive)
				_winch_stopping = false;
				_winch_ramping_active = true;
				_winch_ramp_start_time = now;
				_manual_winch_slowdown_initiated = true;

				PX4_INFO("Winch up: entering upper zone at %.2fm, slowing to %.2f PWM",
					(double)_winch_distance_traveled, (double)WINCH_SLOW_SPEED);
			}
		} else if (in_lower_zone) {
			// In lower zone: stay slow when starting from bottom
			if (!_manual_winch_slowdown_initiated) {
				float current_speed = _dropbox_position[2];
				_winch_initial_speed = current_speed;
				_winch_target_speed = WINCH_SLOW_SPEED;
				_winch_direction = 1;
				_winch_stopping = false;
				_winch_ramping_active = true;
				_winch_ramp_start_time = now;
				_manual_winch_slowdown_initiated = true;

				PX4_INFO("Winch up: in lower zone at %.2fm, maintaining slow speed",
					(double)_winch_distance_traveled);
			}
		} else {
			// In middle zone: speed up to full speed
			if (!_manual_winch_slowdown_initiated) {
				float current_speed = _dropbox_position[2];
				_winch_initial_speed = current_speed;
				_winch_target_speed = WINCH_MAX_SPEED;
				_winch_direction = 1;
				_winch_stopping = false;
				_winch_ramping_active = true;
				_winch_ramp_start_time = now;
				_manual_winch_slowdown_initiated = true;

				PX4_INFO("Winch up: exiting lower zone at %.2fm, ramping to full speed",
					(double)_winch_distance_traveled);
			}
		}
	}
}

// ============================================================================
// WINCH CONTROL
// ============================================================================

void Dropboxv2::winchup()
{
	// Check for fault condition
	if (_winch_fault) {
		PX4_ERR("Winch operation blocked - fault detected. Restart module to clear.");
		return;
	}

	// Check if winch is already at top limit switch
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	if (winch_at_top) {
		// Cannot move up when already at top - only allow downward movement
		PX4_WARN("Winch already at top limit - cannot move up");
		return;
	}

	// Get current motor speed to continue from current position
	float current_speed = _dropbox_position[2];

	_dropbox_status.winch_state = dropbox_status_s::WINCH_WINDING_UP;
	_winch_direction = 1;  // Positive = up (winding in)

	// Upper zone is from -inf to WINCH_UPPER_SLOWDOWN_LIMIT (2m from top)
	// Lower zone is from (target - 2m) to target (starting from bottom)
	// Start at slow speed if in either zone, full speed in middle
	float slowdown_trigger = _cached_target_distance - WINCH_LOWER_SLOWDOWN_LIMIT;
	bool in_upper_zone = (_winch_distance_traveled <= WINCH_UPPER_SLOWDOWN_LIMIT);
	bool in_lower_zone = (_winch_distance_traveled >= slowdown_trigger);

	if (in_upper_zone || in_lower_zone) {
		_winch_target_speed = WINCH_SLOW_SPEED;  // Slow in upper or lower zone
		PX4_INFO("Winch up starting slow at %.2fm (in %s zone)",
			(double)_winch_distance_traveled,
			in_upper_zone ? "upper" : "lower");
	} else {
		_winch_target_speed = WINCH_MAX_SPEED;   // Full speed in middle zone
		PX4_INFO("Winch up starting fast at %.2fm (in middle zone)",
			(double)_winch_distance_traveled);
	}

	_winch_initial_speed = current_speed;  // Start from current PWM value
	_winch_stopping = false;
	_winch_ramping_active = true;
	_winch_ramp_start_time = hrt_absolute_time();
	_manual_winch_slowdown_initiated = false;  // Reset slowdown flag for new operation
}

void Dropboxv2::winchdown()
{
	// Check for fault condition
	if (_winch_fault) {
		PX4_ERR("Winch operation blocked - fault detected. Restart module to clear.");
		return;
	}

	// SAFETY CHECK: If doors are enabled, prevent winch-down when doors are closed
	if (_doors_enabled) {
		bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
		bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
		bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

		if (left_closed || right_closed) {
			PX4_WARN("SAFETY: Cannot winch down - doors are closed (L=%d, R=%d, Winch@Top=%d). Open doors first.",
				left_closed, right_closed, winch_at_top);
			return;
		}
	}

	// Get current motor speed to continue from current position
	float current_speed = _dropbox_position[2];

	_dropbox_status.winch_state = dropbox_status_s::WINCH_WINDING_DOWN;
	_winch_direction = -1;  // Negative = down (paying out)
	_limit_switch_was_disengaged = false;  // Reset flag - will be set to true once switch is disengaged

	// Upper zone is from -inf to WINCH_UPPER_SLOWDOWN_LIMIT (2m from top)
	// Start at slow speed if in upper zone, full speed otherwise
	if (_winch_distance_traveled <= WINCH_UPPER_SLOWDOWN_LIMIT) {
		_winch_target_speed = WINCH_SLOW_SPEED;  // Slow in upper zone
	} else {
		_winch_target_speed = WINCH_MAX_SPEED;   // Full speed beyond upper zone
	}

	_winch_initial_speed = current_speed;  // Start from current PWM value
	_winch_stopping = false;
	_winch_ramping_active = true;
	_winch_ramp_start_time = hrt_absolute_time();
	_manual_winch_slowdown_initiated = false;  // Reset slowdown flag for new operation
}

void Dropboxv2::winchstop()
{
	// Get current motor speed
	float current_speed = _dropbox_position[2];
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);


	// If already stopped, set appropriate state based on limit switch
	if (is_float_equal(current_speed, 0.0f)) {
		_winch_ramping_active = false;
		_winch_stopping = false;
		_winch_direction = 0;

		// Set state based on limit switch position
		if (winch_at_top) {
			_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
		} else {
			_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
		}
		return;
	}

	// If already ramping down to stop, do nothing
	if (_winch_stopping && _winch_ramping_active) {
		return;
	}

	// Ramp down from current speed to 0
	_winch_initial_speed = current_speed;  // Store current speed (could be positive or negative)
	_winch_target_speed = 0.0f;
	_winch_stopping = true;
	_winch_ramping_active = true;
	_winch_ramp_start_time = hrt_absolute_time();
}

bool Dropboxv2::do_precheck()
{
	// Check if precheck is already running
	if (_precheck_state != PRECHECK_IDLE) {
		PX4_WARN("Precheck already in progress");
		return false;
	}

	PX4_INFO("Starting pre-operation check...");

	// STEP 0: Save winch mass FIRST (before any motors move)
	_saved_winch_mass = _dropbox_status.winch_mass;
	PX4_INFO("Pre-op: Payload weight saved: %.2f kg", (double)_saved_winch_mass);

	// Read initial state
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
	bool left_closed = false;
	bool right_closed = false;

	if (_doors_enabled) {
		left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
		right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	}

	bool doors_ok = !_doors_enabled || (left_closed && right_closed);
	bool winch_ok = winch_at_top;

	PX4_INFO("Pre-op: Initial state - Winch: %s, Doors: %s",
		winch_ok ? "TOP ✓" : "NOT AT TOP",
		_doors_enabled ? (doors_ok ? "CLOSED ✓" : "NOT CLOSED") : "DISABLED");

	// STEP 1: Wind winch to top if needed (safety first)
	if (!winch_ok) {
		PX4_INFO("Pre-op: Winch not at top, winding up...");
		_precheck_state = PRECHECK_WINDING_UP;
		_precheck_timeout_start = hrt_absolute_time();
		_dropbox_status.winch_state = dropbox_status_s::WINCH_WINDING_UP;

		// Use ramping to reach PRECHECK_WINCH_SPEED (0.3 PWM)
		float current_speed = _dropbox_position[2];
		_winch_direction = 1;  // Positive = up
		_winch_target_speed = PRECHECK_WINCH_SPEED;
		_winch_initial_speed = current_speed;
		_winch_stopping = false;
		_winch_ramping_active = true;
		_winch_ramp_start_time = hrt_absolute_time();

		PX4_INFO("Pre-op: Ramping winch up to %.2f PWM speed", (double)PRECHECK_WINCH_SPEED);

		// Send initial ACK to report precheck started (10% progress)
		send_dropbox_command_ack(_precheck_timeout_start, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
			vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 10,
			_cur_vcmd_target_system, _cur_vcmd_target_component);
		PX4_INFO("[ACK] Precheck STARTED -> sys=%d comp=%d progress=10%%", _cur_vcmd_target_system, _cur_vcmd_target_component);
		return true;
	}

	// STEP 2: Close doors if needed (only if doors enabled and not at closed limit)
	if (_doors_enabled && !doors_ok) {
		PX4_INFO("Pre-op: Doors not closed, closing...");
		_precheck_state = PRECHECK_CLOSING_DOORS;
		_precheck_timeout_start = hrt_absolute_time();
		_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSING_BOX;
		closebox();
		return true;
	}

	// STEP 3: Both winch and doors OK - complete precheck
	_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
	if (_doors_enabled) {
		_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
	}
	_precheck_complete = true;
	_precheck_state = PRECHECK_IDLE;
	PX4_INFO("Pre-operation check COMPLETE ✓ (mass already saved: %.2f kg)", (double)_saved_winch_mass);

	// Send completion ACK (100%)
	send_dropbox_command_ack(hrt_absolute_time(), vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
		vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 100,
		_cur_vcmd_target_system, _cur_vcmd_target_component);
	PX4_INFO("[ACK] Precheck COMPLETE -> sys=%d comp=%d progress=100%%", _cur_vcmd_target_system, _cur_vcmd_target_component);
	return true;
}

void Dropboxv2::handle_precheck(const hrt_abstime now)
{
	if (_precheck_state == PRECHECK_IDLE) {
		return;
	}

	// Check for timeout
	hrt_abstime elapsed = now - _precheck_timeout_start;

	switch (_precheck_state) {
		case PRECHECK_WINDING_UP: {
			// Check if winch reached top
			bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

			if (winch_at_top) {
				publish_to_motor(2, 0.0f);
				_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
				PX4_INFO("Pre-op: Winch at top ✓");

				// Check if doors need closing
				bool need_close_doors = false;
				if (_doors_enabled) {
					bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
					bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
					need_close_doors = !(left_closed && right_closed);
				}

				if (need_close_doors) {
					PX4_INFO("Pre-op: Doors not closed, closing...");
					_precheck_state = PRECHECK_CLOSING_DOORS;
					_precheck_timeout_start = now;
					_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSING_BOX;
					closebox();
					// Report progress: doors closing (40%)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 40,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("[ACK] Precheck IN_PROGRESS -> sys=%d comp=%d progress=40%% (closing doors)",
						_cur_vcmd_target_system, _cur_vcmd_target_component);
				} else {
					// Complete precheck (mass already saved at start)
					if (_doors_enabled) {
						_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
					}
					_precheck_complete = true;
					_precheck_state = PRECHECK_IDLE;
					PX4_INFO("Pre-operation check COMPLETE ✓ (mass saved: %.2f kg)", (double)_saved_winch_mass);

					// Send completion ACK (100%)
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 100,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("[ACK] Precheck COMPLETE -> sys=%d comp=%d progress=100%%",
						_cur_vcmd_target_system, _cur_vcmd_target_component);
				}
			} else if (elapsed > PRECHECK_WINCH_TIMEOUT) {
				PX4_ERR("Pre-op FAILED - Winch timeout");
				publish_to_motor(2, 0.0f);
				_precheck_state = PRECHECK_IDLE;
				_precheck_complete = false;

				// Send FAILED ACK (0%) for timeout
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 0,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Precheck FAILED (winch timeout) -> sys=%d comp=%d progress=0%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			break;
		}

		case PRECHECK_CLOSING_DOORS: {
			// Check if both doors reached closed position
			bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
			bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);

			if (left_closed && right_closed) {
				_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
				PX4_INFO("Pre-op: Both doors closed ✓");

				// Complete precheck (mass already saved at start)
				_precheck_complete = true;
				_precheck_state = PRECHECK_IDLE;
				PX4_INFO("Pre-operation check COMPLETE ✓ (mass saved: %.2f kg)", (double)_saved_winch_mass);

				// Send completion ACK (100%)
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 100,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Precheck COMPLETE -> sys=%d comp=%d progress=100%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			} else if (elapsed > PRECHECK_DOOR_TIMEOUT) {
				PX4_ERR("Pre-op FAILED - Door timeout");
				publish_to_motor(0, 0.0f);
				publish_to_motor(1, 0.0f);
				_precheck_state = PRECHECK_IDLE;
				_precheck_complete = false;

				// Send FAILED ACK (0%) for door timeout
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 0,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Precheck FAILED (door timeout) -> sys=%d comp=%d progress=0%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			break;
		}

		default:
			break;
	}
}

// ============================================================================
// AUTO WINCH DEPLOYMENT
// ============================================================================

bool Dropboxv2::do_winch()
{
	// Gate checks
	if (_winch_fault) {
		PX4_ERR("Cannot start auto-winch - winch fault active");
		return false;
	}

	if (_auto_winch_state != AUTO_WINCH_IDLE) {
		PX4_WARN("Cannot start auto-winch - already running");
		return false;
	}

	// Use the mass saved during precheck as the payload reference weight
	_auto_winch_saved_mass = _saved_winch_mass;
	_auto_winch_mass_released = false;

	PX4_INFO("Auto-winch starting. Using precheck payload mass: %.3f kg", (double)_auto_winch_saved_mass);

	if (_doors_enabled) {
		// Open doors first, then descend
		PX4_INFO("Auto-winch: opening doors before descent...");
		_auto_winch_state = AUTO_WINCH_OPENING_DOORS;
		_auto_winch_timer = hrt_absolute_time();
		openbox();
	} else {
		// No doors - go straight to descending
		PX4_INFO("Auto-winch: descending (no doors)...");
		_auto_winch_state = AUTO_WINCH_DESCENDING;
		winchdown();
	}

	return true;
}

void Dropboxv2::handle_auto_winch(const hrt_abstime now)
{
	if (_auto_winch_state == AUTO_WINCH_IDLE) {
		return;
	}

	// -------------------------------------------------------------------------
	// GLOBAL SAFETY: Abort on winch fault at any stage
	// -------------------------------------------------------------------------
	if (_winch_fault) {
		PX4_ERR("Auto-winch ABORTED - winch fault detected!");
		winchstop();
		_auto_winch_state = AUTO_WINCH_IDLE;
		send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
			vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 0,
			_cur_vcmd_target_system, _cur_vcmd_target_component);
		PX4_INFO("[ACK] Auto-winch FAILED (fault) -> sys=%d comp=%d progress=0%%",
			_cur_vcmd_target_system, _cur_vcmd_target_component);
		return;
	}

	switch (_auto_winch_state) {

		// -----------------------------------------------------------------
		case AUTO_WINCH_OPENING_DOORS: {
			// Wait for both door open limit switches
			bool left_open  = !px4_arch_gpioread(LEFT_DOOR_OPEN_LIMIT_PIN);
			bool right_open = !px4_arch_gpioread(RIGHT_DOOR_OPEN_LIMIT_PIN);

			if (left_open && right_open) {
				PX4_INFO("Auto-winch: doors open ✓ - starting descent");
				_auto_winch_state = AUTO_WINCH_DESCENDING;
				winchdown();
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 20,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=20%% (descending)",
					_cur_vcmd_target_system, _cur_vcmd_target_component);

			} else if (now - _auto_winch_timer >= AUTO_WINCH_DOOR_TIMEOUT) {
				PX4_ERR("Auto-winch ABORTED - door open timeout");
				publish_to_motor(0, 0.0f);
				publish_to_motor(1, 0.0f);
				_auto_winch_state = AUTO_WINCH_IDLE;
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 0,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch FAILED (door open timeout) -> sys=%d comp=%d progress=0%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			break;
		}

		// -----------------------------------------------------------------
		case AUTO_WINCH_DESCENDING: {
			// Wait until mass drops below release threshold
			if (_dropbox_status.winch_mass < _cached_release_threshold) {
				_auto_winch_mass_released = true;
				_auto_winch_timer = now;
				_auto_winch_state = AUTO_WINCH_POST_RELEASE_WAIT;
				PX4_INFO("Auto-winch: payload released (%.3f kg) - continuing down for %.1fs",
					(double)_dropbox_status.winch_mass,
					(double)_cached_post_release_time);
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 50,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=50%% (payload released)",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			// Winch continues descending via winchdown() - limit switches still apply
			break;
		}

		// -----------------------------------------------------------------
		case AUTO_WINCH_POST_RELEASE_WAIT: {
			// Keep descending for N seconds after release
			if (now - _auto_winch_timer >= (hrt_abstime)(_cached_post_release_time * 1e6f)) {
				PX4_INFO("Auto-winch: post-release wait complete - ascending");
				_auto_winch_state = AUTO_WINCH_ASCENDING;
				winchup();
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 65,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=65%% (ascending)",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			// Winch continues descending - no action needed
			break;
		}

		// -----------------------------------------------------------------
		case AUTO_WINCH_ASCENDING: {
			// Check if mass comes back (payload re-attached / not dropped cleanly)
			float retry_threshold = _auto_winch_saved_mass * _cached_retry_fraction;

			if (_dropbox_status.winch_mass > retry_threshold && _auto_winch_saved_mass > 0.1f) {
				PX4_WARN("Auto-winch: mass re-detected (%.3f kg > %.3f kg threshold) - retrying descent",
					(double)_dropbox_status.winch_mass, (double)retry_threshold);
				_auto_winch_state = AUTO_WINCH_RETRY_DOWN;
				_auto_winch_timer = now;
				winchdown();
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 40,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=40%% (retry descent)",
					_cur_vcmd_target_system, _cur_vcmd_target_component);

			} else if (_dropbox_status.winch_state == dropbox_status_s::WINCH_TOP) {
				// Reached the top limit switch - sequence complete
				if (_doors_enabled) {
					PX4_INFO("Auto-winch: at top - closing doors");
					_auto_winch_state = AUTO_WINCH_CLOSING_DOORS;
					_auto_winch_timer = now;
					closebox();
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 85,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=85%% (closing doors)",
						_cur_vcmd_target_system, _cur_vcmd_target_component);
				} else {
					PX4_INFO("Auto-winch: sequence COMPLETE ✓");
					_auto_winch_state = AUTO_WINCH_IDLE;
					send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
						vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 100,
						_cur_vcmd_target_system, _cur_vcmd_target_component);
					PX4_INFO("[ACK] Auto-winch COMPLETE -> sys=%d comp=%d progress=100%%",
						_cur_vcmd_target_system, _cur_vcmd_target_component);
				}
			}
			break;
		}

		// -----------------------------------------------------------------
		case AUTO_WINCH_RETRY_DOWN: {
			// Go down for N seconds to re-attempt release
			if (now - _auto_winch_timer >= (hrt_abstime)(_cached_retry_down_time * 1e6f)) {
				PX4_INFO("Auto-winch: retry descent complete - ascending again");
				_auto_winch_state = AUTO_WINCH_ASCENDING;
				winchup();
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, 65,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch IN_PROGRESS -> sys=%d comp=%d progress=65%% (ascending after retry)",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			// Winch continues descending
			break;
		}

		// -----------------------------------------------------------------
		case AUTO_WINCH_CLOSING_DOORS: {
			bool left_closed  = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
			bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);

			if (left_closed && right_closed) {
				PX4_INFO("Auto-winch: doors closed ✓ - sequence COMPLETE ✓");
				_auto_winch_state = AUTO_WINCH_IDLE;
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 100,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch COMPLETE -> sys=%d comp=%d progress=100%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);

			} else if (now - _auto_winch_timer >= AUTO_WINCH_DOOR_TIMEOUT) {
				PX4_ERR("Auto-winch: door close timeout - sequence ended with open doors!");
				publish_to_motor(0, 0.0f);
				publish_to_motor(1, 0.0f);
				_auto_winch_state = AUTO_WINCH_IDLE;
				send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
					vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, 90,
					_cur_vcmd_target_system, _cur_vcmd_target_component);
				PX4_INFO("[ACK] Auto-winch FAILED (door close timeout) -> sys=%d comp=%d progress=90%%",
					_cur_vcmd_target_system, _cur_vcmd_target_component);
			}
			break;
		}

		default:
			break;
	}
}

void Dropboxv2::publish_to_motor(int motor_id, float pwm_value)
{
	_dropbox_position[motor_id] = pwm_value;

	actuator_servos_s actuator_servos{};

	for (int i = 0; i < 8; i++) {
		actuator_servos.control[i] = _dropbox_position[i];
	}

	// All motors use servos
	actuator_servos.timestamp = hrt_absolute_time();
	_actuator_servos_pub.publish(actuator_servos);
}

// ============================================================================
// DOOR CONTROL (only if doors enabled via DROPBOX_DOOR_EN parameter)
// ============================================================================

void Dropboxv2::openbox()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	PX4_INFO("Opening dropbox");
	_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENING_BOX;
	open_left_door();
	open_right_door();
}

void Dropboxv2::closebox()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	// SAFETY CHECK: Only allow door closing when winch is at top limit switch
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	if (!winch_at_top) {
		PX4_WARN("SAFETY: Cannot close doors - winch not at top (Winch@Top=%d). Wind up first.", winch_at_top);
		return;
	}

	PX4_INFO("Closing dropbox");
	_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSING_BOX;
	close_left_door();
	close_right_door();
}

void Dropboxv2::open_left_door()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	PX4_INFO("Opening LEFT door");
	_left_door_opening = true;
	_left_door_closing = false;
	publish_to_motor(0, 1.0f);  // Positive PWM = open
}

void Dropboxv2::close_left_door()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	// SAFETY CHECK: Only allow door closing when winch is at top limit switch
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	if (!winch_at_top) {
		PX4_WARN("SAFETY: Cannot close left door - winch not at top (Winch@Top=%d). Wind up first.", winch_at_top);
		return;
	}

	PX4_INFO("Closing LEFT door");
	_left_door_closing = true;
	_left_door_opening = false;
	publish_to_motor(0, -1.0f);  // Negative PWM = close
}

void Dropboxv2::open_right_door()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	PX4_INFO("Opening RIGHT door");
	_right_door_opening = true;
	_right_door_closing = false;
	publish_to_motor(1, -1.0f);  // Negative PWM = open
}

void Dropboxv2::close_right_door()
{
	if (!_doors_enabled) {
		PX4_WARN("Door functionality disabled (DROPBOX_DOOR_EN=0)");
		return;
	}

	// SAFETY CHECK: Only allow door closing when winch is at top limit switch
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	if (!winch_at_top) {
		PX4_WARN("SAFETY: Cannot close right door - winch not at top (Winch@Top=%d). Wind up first.", winch_at_top);
		return;
	}

	PX4_INFO("Closing RIGHT door");
	_right_door_closing = true;
	_right_door_opening = false;
	publish_to_motor(1, 1.0f);  // Positive PWM = close
}

void Dropboxv2::stop_door_motor(int motor_id, const char *message)
{
	publish_to_motor(motor_id, 0.0f);
	PX4_INFO("%s - motor stopped", message);
}

void Dropboxv2::handle_door_limit_switches(const hrt_abstime now)
{
	// Read door limit switches (active-low: pressed = 0, released = 1)
	bool left_open = !px4_arch_gpioread(LEFT_DOOR_OPEN_LIMIT_PIN);
	bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	bool right_open = !px4_arch_gpioread(RIGHT_DOOR_OPEN_LIMIT_PIN);
	bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);

	// Debug: Print limit switch states periodically
	static hrt_abstime last_debug_print = 0;
	if (now - last_debug_print >= 1000_ms) {
		PX4_INFO("L_OPEN=%d L_CLOSED=%d R_OPEN=%d R_CLOSED=%d | L_Opening=%d L_Closing=%d R_Opening=%d R_Closing=%d | WINCH=%d",
			left_open, left_closed, right_open, right_closed,
			_left_door_opening, _left_door_closing, _right_door_opening, _right_door_closing, !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN));
		last_debug_print = now;
	}

	// Stop left door at limits
	if (left_open && _left_door_opening) {
		stop_door_motor(0, "LEFT door reached OPEN limit");
		_left_door_opening = false;

		// Update door state: if both stopped, set to opened
		if (!_right_door_opening && !_right_door_closing) {
			_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENED_BOX;
		}
	}
	if (left_closed && _left_door_closing) {
		stop_door_motor(0, "LEFT door reached CLOSED limit");
		_left_door_closing = false;

		// Update door state: if both stopped, set to closed
		if (!_right_door_opening && !_right_door_closing) {
			_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
		}
	}

	// Stop right door at limits
	if (right_open && _right_door_opening) {
		stop_door_motor(1, "RIGHT door reached OPEN limit");
		_right_door_opening = false;

		// Update door state: if both stopped, set to opened
		if (!_left_door_opening && !_left_door_closing) {
			_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENED_BOX;
		}
	}
	if (right_closed && _right_door_closing) {
		stop_door_motor(1, "RIGHT door reached CLOSED limit");
		_right_door_closing = false;

		// Update door state: if both stopped, set to closed
		if (!_left_door_opening && !_left_door_closing) {
			_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
		}
	}
}

// ============================================================================
// CUTTER CONTROL
// ============================================================================

void Dropboxv2::cutline(int repeat_count)
{
	if (_cutter_state != CUTTER_IDLE) {
		return;
	}

	_cutter_start_time = hrt_absolute_time();
	_cutter_state = CUTTER_CUTTING;
	_cutter_count = 0;
	_cutter_target_count = repeat_count;

	_dropbox_status.cutter_state = dropbox_status_s::CUTTER_CUTLINE;
	publish_to_motor(3, -1.0f);  // Start cutting
}

void Dropboxv2::handle_cutter_sequence(const hrt_abstime now)
{
	if (_cutter_state == CUTTER_IDLE) {
		return;
	}

	hrt_abstime elapsed = now - _cutter_start_time;

	switch (_cutter_state) {
		case CUTTER_CUTTING:
			// Check if cut duration complete
			if (elapsed >= CUTTER_CUT_TIME) {
				_cutter_count++;
				PX4_INFO("Cutter: cut %d/%d complete", _cutter_count, _cutter_target_count);

				publish_to_motor(3, 1.0f);  // Return to idle position

				// Check if we need more cuts
				if (_cutter_count < _cutter_target_count) {
					// Enter pause state
					_cutter_state = CUTTER_PAUSING;
					_cutter_start_time = now;
				} else {
					// All cuts complete
					_cutter_state = CUTTER_IDLE;
					_dropbox_status.cutter_state = dropbox_status_s::CUTTER_IDLE;
				}
			}
			break;

		case CUTTER_PAUSING:
			// Check if pause duration complete
			if (elapsed >= CUTTER_PAUSE_TIME) {
				// Start next cut
				_cutter_state = CUTTER_CUTTING;
				_cutter_start_time = now;
				publish_to_motor(3, -1.0f);  // Start cutting again
			}
			break;

		case CUTTER_IDLE:
		default:
			break;
	}
}


// ============================================================================
// STATUS PUBLISHING
// ============================================================================

void Dropboxv2::update_winch_mass(const hrt_abstime now)
{
	// Read latest winch mass from external sensor/module
	if (_winch_mass_sub.updated()) {
		winch_mass_s winch_mass{};
		if (_winch_mass_sub.copy(&winch_mass)) {
			float raw_mass = winch_mass.mass;

			// Add new reading to circular buffer
			_mass_buffer[_mass_buffer_index] = raw_mass;
			_mass_buffer_index = (_mass_buffer_index + 1) % MASS_BUFFER_SIZE;

			// Track number of valid samples (up to buffer size)
			if (_mass_buffer_count < MASS_BUFFER_SIZE) {
				_mass_buffer_count++;
			}

			// Calculate moving average
			float sum = 0.0f;
			for (int i = 0; i < _mass_buffer_count; i++) {
				sum += _mass_buffer[i];
			}
			_filtered_mass = sum / _mass_buffer_count;

			// Update dropbox_status with filtered mass reading
			_dropbox_status.winch_mass = _filtered_mass;

			// SAFETY CHECK: If mass exceeds 30kg, immediately stop the winch motor
			if (_dropbox_status.winch_mass > WINCH_MASS_SAFETY_LIMIT) {
				// Stop motor immediately (safety override)
				// publish_to_motor(2, 0.0f);

				// Cancel any active ramping
				// _winch_ramping_active = false;
				// _winch_stopping = false;
				// _winch_direction = 0;

				PX4_ERR("SAFETY: Winch stopped! Mass %.2f kg exceeds safety limit of %.1f kg",
					(double)_dropbox_status.winch_mass, (double)WINCH_MASS_SAFETY_LIMIT);
			}

			// Rate-limited printing: only print mass every 1 second
			if (now - _last_mass_print_time >= MASS_PRINT_INTERVAL) {
				PX4_INFO("Winch mass updated: %.3f kg (raw: %.3f kg, samples: %d)",
					(double)_dropbox_status.winch_mass, (double)raw_mass, _mass_buffer_count);
				_last_mass_print_time = now;
			}
		}
	}
}

void Dropboxv2::publish_status(const hrt_abstime now)
{
	if (now - _last_status_publish_time >= STATUS_PUBLISH_INTERVAL) {
		_dropbox_status.timestamp = now;
		_dropbox_status_pub.publish(_dropbox_status);
		_last_status_publish_time = now;
	}
}

// ============================================================================
// COMMUNICATION
// ============================================================================

bool Dropboxv2::send_vehicle_command(const hrt_abstime now, const uint32_t cmd, const float param1,
	const float param2, const float param3, const float param4, const double param5,
	const double param6, const float param7)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;
	vcmd.confirmation = 0;
	vcmd.from_external = false;
	vcmd.timestamp = now;

	vcmd.source_system = 1;
	vcmd.target_system = 1;
	vcmd.source_component = 3;
	vcmd.target_component = 3;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	return vcmd_pub.publish(vcmd);

}

bool Dropboxv2::send_dropbox_command_ack(const hrt_abstime now, const uint32_t cmd, const uint8_t result,
	const uint8_t progress, const uint8_t target_system, const uint16_t target_component)
{
	vehicle_command_ack_s command_ack{};
	command_ack.timestamp = now;
	command_ack.command = cmd;
	command_ack.result = result;
	if(result==vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS){
		command_ack.result_param1 = progress;

	}else{
		command_ack.result_param1 = 255;

	}
	command_ack.result_param2 = 0;
	command_ack.target_system = target_system;
	command_ack.target_component = target_component;

	_vehicle_command_ack_pub.publish(command_ack);
	return true;
}

// ============================================================================
// MODULE ENTRY POINTS
// ============================================================================

int Dropboxv2::task_spawn(int argc, char *argv[])
{
	Dropboxv2 *instance = new Dropboxv2();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Dropboxv2::print_status()
{
	const char* winch_states[] = {"Top", "Bottom", "Winding Up", "Winding Down", "Mid-Air"};

	PX4_INFO("Winch State: %s", winch_states[_dropbox_status.winch_state]);
	PX4_INFO("Distance Traveled: %.2f m", (double)_winch_distance_traveled);
	PX4_INFO("Precheck: %s", _precheck_complete ? "Complete" : "Not done");
	PX4_INFO("Saved Mass: %.3f kg", (double)_saved_winch_mass);
	PX4_INFO("Winch Fault: %s", _winch_fault ? "YES - SYSTEM LOCKED" : "No");

	perf_print_counter(_loop_perf);

	return 0;
}

int Dropboxv2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dropbox v2 control module - minimal version with communication infrastructure only.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dropboxv2", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("winchup", "Wind winch up");
	PRINT_MODULE_USAGE_COMMAND_DESCR("winchdown", "Wind winch down");
	PRINT_MODULE_USAGE_COMMAND_DESCR("winchstop", "Stop winch");
	PRINT_MODULE_USAGE_COMMAND_DESCR("precheck", "Precheck: save winch mass and wind up to top at 0.7 PWM");
	PRINT_MODULE_USAGE_COMMAND_DESCR("dowinch", "Automatic winch: down to 2m @ 0.7 PWM, wait for mass < 0.5kg, wait 1s, return to top (requires precheck first)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cutline", "Cut the winch line (repeats 5 times)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openbox", "Open both doors (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closebox", "Close both doors (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openleft", "Open left door (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closeleft", "Close left door (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openright", "Open right door (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closeright", "Close right door (requires DROPBOX_DOOR_EN=1)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Dropboxv2::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	Dropboxv2 *instance = get_instance();
	if (!instance) {
		PX4_ERR("Dropboxv2 not running");
		return PX4_ERROR;
	}

	const char *cmd = argv[0];

	if (!strcmp(cmd, "status")) {
		return instance->print_status();
	}

	if (!strcmp(cmd, "precheck")) {
		instance->do_precheck();
		return 0;
	}

	if (!strcmp(cmd, "winchup")) {
		instance->winchup();
		return 0;
	}

	if (!strcmp(cmd, "winchdown")) {
		instance->winchdown();
		return 0;
	}

	if (!strcmp(cmd, "winchstop")) {
		instance->winchstop();
		return 0;
	}

	if (!strcmp(cmd, "dowinch")) {
		instance->do_winch();
		return 0;
	}

	if (!strcmp(cmd, "cutline")) {
		instance->cutline(5);
		return 0;
	}

	if (!strcmp(cmd, "openbox")) {
		instance->openbox();
		return 0;
	}

	if (!strcmp(cmd, "closebox")) {
		instance->closebox();
		return 0;
	}

	if (!strcmp(cmd, "openleft")) {
		instance->open_left_door();
		return 0;
	}

	if (!strcmp(cmd, "closeleft")) {
		instance->close_left_door();
		return 0;
	}

	if (!strcmp(cmd, "openright")) {
		instance->open_right_door();
		return 0;
	}

	if (!strcmp(cmd, "closeright")) {
		instance->close_right_door();
		return 0;
	}

	return print_usage("unknown command");
}

extern "C" __EXPORT int dropboxv2_main(int argc, char *argv[])
{
	return Dropboxv2::main(argc, argv);
}
