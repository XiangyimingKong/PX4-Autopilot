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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/input_rc.h>

using namespace time_literals;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static bool is_float_equal(float a, float b, float epsilon = 0.001f)
{
	float diff = a - b;
	float abs_diff = (diff < 0) ? -diff : diff;
	return abs_diff <= epsilon;
}

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

Dropboxv2::Dropboxv2() :
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
	PX4_INFO("Dropbox module started and actuator is armed");

	// Initialize motor positions (all stopped except cutter at idle)
	publish_to_motor(0, 0.0f);  // Left door
	publish_to_motor(1, 0.0f);  // Right door
	publish_to_motor(2, 0.0f);  // Winch
	publish_to_motor(3, 1.0f);  // Cutter

	// Configure GPIO pins first (needed to read limit switches)
	configure_gpio_pins();

	// Read actual hardware state from limit switches to initialize states correctly
	bool left_door_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	bool right_door_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	bool left_door_open = !px4_arch_gpioread(LEFT_DOOR_OPEN_LIMIT_PIN);
	bool right_door_open = !px4_arch_gpioread(RIGHT_DOOR_OPEN_LIMIT_PIN);
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
	px4_arch_gpiowrite(BUTTON_POWER_PIN, 1);  // Power button GPIO high (active low)

	// Initialize dropbox status based on actual hardware state
	_dropbox_status.timestamp = hrt_absolute_time();

	// Determine door state from limit switches
	if (left_door_closed && right_door_closed) {
		_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSED_BOX;
		PX4_INFO("Init: Doors are CLOSED");
	} else if (left_door_open && right_door_open) {
		_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENED_BOX;
		PX4_INFO("Init: Doors are OPEN");
	} else {
		// Doors are in between - assume opening state
		_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENED_BOX;
		PX4_INFO("Init: Doors are neither fully open nor closed, nexgt button press will close them");
	}

	// Determine winch state from limit switch
	if (winch_at_top) {
		_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
		_winch_cumulative_time = 0;  // Reset timer since at top
		PX4_INFO("Init: Winch is at TOP");
	} else {
		_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
		PX4_INFO("Init: Winch is in MID-AIR (not at top)");
	}

	// Cutter always starts at idle
	_dropbox_status.cutter_state = dropbox_status_s::CUTTER_IDLE;

	_dropbox_status_pub.publish(_dropbox_status);  // Publish initial state
	_last_status_publish_time = hrt_absolute_time();  // Initialize timer
	PX4_INFO("Status publishing initialized - will report every 1 second");

	// Initialize payload mass samples (collect 20 samples over 2 seconds)
	PX4_INFO("Collecting initial payload mass samples...");
	payload_mass_s payload_mass_msg;
	for (int i = 0; i < PAYLOAD_MASS_SAMPLES; i++) {
		if (_payload_mass_sub.update(&payload_mass_msg)) {
			update_payload_mass_samples(payload_mass_msg.mass);
		} else {
			// If no message available, use 0.0
			update_payload_mass_samples(0.0f);
		}
		usleep(100000);  // Wait 100ms between samples (2 seconds total)
	}
	PX4_INFO("Payload mass initialization complete");

	// Initialize button states to prevent false triggers
	const hrt_abstime now = hrt_absolute_time();
	_last_door_button_state = !px4_arch_gpioread(DOOR_BUTTON_PIN);
	_last_up_button_state = !px4_arch_gpioread(UP_BUTTON_PIN);
	_last_down_button_state = !px4_arch_gpioread(DOWN_BUTTON_PIN);
	_last_door_button_time = now;
	_last_up_button_time = now;
	_last_down_button_time = now;

	ScheduleDelayed(1_s);
	return true;
}

void Dropboxv2::configure_gpio_pins()
{
	// Door GPIO pins
	px4_arch_configgpio(LEFT_DOOR_CLOSE_LIMIT_PIN);
	px4_arch_configgpio(LEFT_DOOR_OPEN_LIMIT_PIN);

	px4_arch_configgpio(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	px4_arch_configgpio(RIGHT_DOOR_OPEN_LIMIT_PIN);

	// Winch and cutter GPIO pins
	px4_arch_configgpio(WINCH_LIMIT_SWITCH_PIN);
	px4_arch_configgpio(WINCH_STEP_PIN);
	px4_arch_configgpio(CUTTER_STEP_PIN);

	// Button GPIO pins
	px4_arch_configgpio(DOOR_BUTTON_PIN);
	px4_arch_configgpio(UP_BUTTON_PIN);
	px4_arch_configgpio(DOWN_BUTTON_PIN);
	px4_arch_configgpio(BUTTON_POWER_PIN);
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

	// Update payload mass subscription
	update_payload_mass();

	// Update winch mass subscription
	update_winch_mass();

	// Process inputs
	handle_rc_input(now);
	handle_physical_buttons(now);
	handle_vehicle_commands(now);

	// Process sequences
	handle_dowinch_sequence(now);
	handle_do_dropbox_sequence(now);
	handle_cutter_sequence(now);
	handle_winch_ramping(now);
	update_winch_cumulative_timer(now);

	// Monitor limit switches and enforce safety
	handle_dropbox_limit_switches(now);
	handle_winch_limit_switches(now);
	handle_safety_cases();

	// Publish status every second
	publish_status(now);

	ScheduleDelayed(50_ms);
	perf_end(_loop_perf);
}

// ============================================================================
// INPUT HANDLERS
// ============================================================================

void Dropboxv2::update_payload_mass()
{
	if (_payload_mass_sub.updated()) {
		payload_mass_s payload_mass_msg{};
		if (_payload_mass_sub.copy(&payload_mass_msg)) {
			_payload_mass = payload_mass_msg.mass;
			// Add to FIFO array for averaging
			update_payload_mass_samples(payload_mass_msg.mass);
			// PX4_INFO("Updated payload mass: %.1f g", (double)_payload_mass);
		}
	}
}

void Dropboxv2::update_winch_mass()
{
	if (_winch_mass_sub.updated()) {
		winch_mass_s winch_mass_msg{};
		if (_winch_mass_sub.copy(&winch_mass_msg)) {
			_dropbox_status.winch_mass = winch_mass_msg.mass;
			// PX4_INFO("Updated winch mass: %.1f g", (double)_dropbox_status.winch_mass);
		}
	}
}

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

	// Door toggle logic - CH10 = 2000 toggles between open and close
	static bool last_door_toggle_state = false;  // Track previous RC state to detect edge
	static bool first_run = true;  // Flag to prevent false trigger on startup
	bool door_toggle_active = (input_rc.values[10] == 2000);

	// On first run, initialize the state without triggering an action
	if (first_run) {
		last_door_toggle_state = door_toggle_active;
		first_run = false;
		// Removed automatic door position check - init sequence handles this now
	}

	// Detect rising edge (toggle switch goes from low to high)
	if (door_toggle_active && !last_door_toggle_state) {
		// Toggle between open and close based on current state
		if (_dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSED_BOX) {
			_params[0] = 1.0f;  // open door
			PX4_INFO("Door toggle: Opening doors");
		} else if (_dropbox_status.dropbox_state == dropbox_status_s::DP_OPENED_BOX) {
			_params[0] = 2.0f;  // close door
			PX4_INFO("Door toggle: Closing doors");
		}
	} else {
		_params[0] = 0.0f;  // no action
	}
	last_door_toggle_state = door_toggle_active;

	// CH9 winch: >1600 = wind up, <1400 = wind down, 1400-1600 = stop
	if (input_rc.values[9] > 1600) {
		_params[1] = 1.0f;  // wind up
	} else if (input_rc.values[9] < 1400) {
		_params[1] = 2.0f;  // wind down
	} else {
		_params[1] = 0.0f;  // stop (middle position)
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
	}	// Print all RC input channels
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
	    !is_float_equal(_params[2], _params_buffer[2])) {
		send_vehicle_command(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX, _params[0], _params[1], _params[2]);
		PX4_INFO("RC Command - Doors: %.1f, Winch: %.1f, Cutter: %.1f",
			(double)_params[0], (double)_params[1], (double)_params[2]);
		// Update buffer
		for (int i = 0; i < 3; i++) {
			_params_buffer[i] = _params[i];
		}
		_params[0]=0.0f; //reset to neutral after command sent
		_params[1]=0.0f;
		_params[2]=0.0f;
	}
}

void Dropboxv2::handle_physical_buttons(const hrt_abstime now)
{
	static constexpr hrt_abstime DEBOUNCE_TIME = 100_ms;

	// Read button states (active-low: 0 = pressed, 1 = released)
	bool door_button = px4_arch_gpioread(DOOR_BUTTON_PIN);
	bool up_button = px4_arch_gpioread(UP_BUTTON_PIN);
	bool down_button = px4_arch_gpioread(DOWN_BUTTON_PIN);

	// Door button: toggle between open/close
	if (door_button && !_last_door_button_state && (now - _last_door_button_time) > DEBOUNCE_TIME) {
		_last_door_button_time = now;

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
	_last_door_button_state = door_button;

	// Up button: winch up on press, stop winch on release
	if (up_button && !_last_up_button_state && (now - _last_up_button_time) > DEBOUNCE_TIME) {
		_last_up_button_time = now;
		PX4_INFO("Up button pressed: winching up");
		winchup(true);  // Override lock
	}
	// Stop winch when up button is released (only if debounce time has passed since last press)
	else if (!up_button && _last_up_button_state && (now - _last_up_button_time) > DEBOUNCE_TIME) {
		PX4_INFO("Up button released: stopping winch");
		_winch_ramping_active = false;  // Cancel any ramping
		publish_to_motor(2, 0.0f);

		// Check if winch is at top limit switch
		bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
		if (winch_at_top) {
			_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
			PX4_INFO("Winch stopped at top");
		} else {
			_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
			PX4_INFO("Winch stopped in mid-air");
		}
	}
	_last_up_button_state = up_button;

	// Down button: winch down on press, stop winch on release
	if (down_button && !_last_down_button_state && (now - _last_down_button_time) > DEBOUNCE_TIME) {
		_last_down_button_time = now;
		PX4_INFO("Down button pressed: winching down");
		winchdown(true);  // Override lock
	}
	// Stop winch when down button is released (only if debounce time has passed since last press)
	else if (!down_button && _last_down_button_state && (now - _last_down_button_time) > DEBOUNCE_TIME) {
		PX4_INFO("Down button released: stopping winch");
		_winch_ramping_active = false;  // Cancel any ramping
		publish_to_motor(2, 0.0f);

		// TODO: Check payload mass to determine if at WINCH_BOTTOM
		// For now, set to MID_AIR when stopping during descent
		_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
		PX4_INFO("Winch stopped in mid-air");
	}
	_last_down_button_state = down_button;
}

// ============================================================================
// VEHICLE COMMAND HANDLER
// ============================================================================
/**
 * Handle vehicle commands for dropbox control
 *
 * Command: VEHICLE_CMD_DO_DROPBOX
 * Target Component: 3 (MAV_COMP_ID_GIMBAL)
 *
 * Parameters:
 * -----------
 * param1 - Door Control:
 *   1.0 = Open both doors (openbox)
 *   2.0 = Close both doors (closebox)
 *          Safety: Cannot close unless winch at top
 *          Returns: ACCEPTED if successful, TEMPORARILY_REJECTED if safety check fails
 *
 * param2 - Winch Control:
 *   1.0 = Wind up (winchup with override)
 *   2.0 = Wind down (winchdown with override)
 *          Safety: Cannot lower unless doors fully open
 *          Returns: ACCEPTED if successful, TEMPORARILY_REJECTED if safety check fails
 *   3.0 = Stop winch immediately
 *
 * param3 - Cutter Control:
 *   1.0 = Cut line (cutline, repeats 5 times automatically)
 *          Each cut: 800ms active + 200ms pause between cuts
 *          Total time: ~5 seconds
 *
 * param4 - System Commands:
 *   1.0 = Run pre-operation check (pre_operation_check)
 *          - Winds winch to top if needed
 *          - Closes doors if needed
 *          - Saves current winch_mass as payload weight
 *   2.0 = Run complete dropbox delivery sequence (do_dropbox)
 *          Safety: Requires winch at top, doors closed, payload weight set
 *          Sequence: Open doors → dowinch → close doors
 *
 * All commands send acknowledgments:
 * - VEHICLE_CMD_RESULT_ACCEPTED: Command executed successfully
 * - VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: Safety check prevented execution
 */
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

	PX4_INFO("Got command: %u, params: %.2f, %.2f, %.2f, %.2f, %.2f",
		vcmd.command, (double)vcmd.param1, (double)vcmd.param2, (double)vcmd.param3, (double)vcmd.param4, (double)vcmd.param5);

	if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_DROPBOX) {
		// Process door commands (param1)
		if (is_float_equal(vcmd.param1, 1.0f)) {
			openbox();
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 1, _cur_vcmd_target_system, _cur_vcmd_target_component);
		} else if (is_float_equal(vcmd.param1, 2.0f)) {
			bool success = closebox();
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				success ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
				1, _cur_vcmd_target_system, _cur_vcmd_target_component);
		}

		// Process winch commands (param2)
		if(is_float_equal(vcmd.param2, 3.0f)) {
			_winch_ramping_active = false;  // Cancel any ramping
			publish_to_motor(2, 0.0f);  // Stop winch

			// Update winch state to stop cumulative timer
			bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
			if (winch_at_top) {
				_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
			} else {
				_dropbox_status.winch_state = dropbox_status_s::WINCH_MID_AIR;
			}

			PX4_INFO("Winch stopped");
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2, _cur_vcmd_target_system, _cur_vcmd_target_component);
		} else if (is_float_equal(vcmd.param2, 1.0f)) {
			winchup(true);  // Override lock
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 2, _cur_vcmd_target_system, _cur_vcmd_target_component);
		} else if (is_float_equal(vcmd.param2, 2.0f)) {
			bool success = winchdown(true);  // Override lock
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				success ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
				2, _cur_vcmd_target_system, _cur_vcmd_target_component);
		}

		// Process cutter command (param3)
		if (is_float_equal(vcmd.param3, 1.0f)) {
			// Call cutline with repeat count of 5
			cutline(5);
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 3, _cur_vcmd_target_system, _cur_vcmd_target_component);
		}

		// Process pre-operation check command (param4)
		// 1 = run pre-operation check
		if (is_float_equal(vcmd.param4, 1.0f)) {
			PX4_INFO("Running pre-operation check");
			pre_operation_check();
		}
		// 2 = run complete dropbox delivery sequence
		else if (is_float_equal(vcmd.param4, 2.0f)) {
			PX4_INFO("Running complete dropbox delivery sequence");
			do_dropbox();
			send_dropbox_command_ack(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, 4, _cur_vcmd_target_system, _cur_vcmd_target_component);
		}
	}
}

// ============================================================================
// LIMIT SWITCH HANDLERS
// ============================================================================

void Dropboxv2::handle_dropbox_limit_switches(const hrt_abstime now)
{
	// Read all door limit switches (active-low: pressed = 0)
	bool left_open = !px4_arch_gpioread(LEFT_DOOR_OPEN_LIMIT_PIN);
	bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	bool right_open = !px4_arch_gpioread(RIGHT_DOOR_OPEN_LIMIT_PIN);
	bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);

	// Stop left door at limits
	if (left_open && _left_door_opening) {
		stop_door_motor(0, "LEFT door reached OPEN limit");
		_left_door_opening = false;
	}
	if (left_closed && _left_door_closing) {
		stop_door_motor(0, "LEFT door reached CLOSED limit");
		_left_door_closing = false;
	}

	// Stop right door at limits
	if (right_open && _right_door_opening) {
		stop_door_motor(1, "RIGHT door reached OPEN limit");
		_right_door_opening = false;
	}
	if (right_closed && _right_door_closing) {
		stop_door_motor(1, "RIGHT door reached CLOSED limit");
		_right_door_closing = false;
	}

	// Update overall dropbox state
	update_dropbox_state(left_closed, right_closed, left_open, right_open, now);
}

void Dropboxv2::handle_winch_limit_switches(const hrt_abstime now)
{
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	if (winch_at_top && _dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP) {
		_winch_ramping_active = false;  // Cancel any ramping
		publish_to_motor(2, 0.0f);
		_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;

		// Reset cumulative timer when reaching top
		_winch_cumulative_time = 0;
		_winch_timer_active = false;
		PX4_INFO("Winch reached top - motor stopped, timer reset");
	}
}

void Dropboxv2::update_dropbox_state(bool left_closed, bool right_closed, bool left_open, bool right_open, const hrt_abstime now)
{
	uint8_t old_state = _dropbox_status.dropbox_state;
	uint8_t new_state = old_state;

	// Determine target state based on limit switches
	if (left_closed && right_closed) {
		// Both doors are at closed position
		// Only transition to CLOSED if we're currently CLOSING, or already CLOSED
		if (old_state == dropbox_status_s::DP_CLOSING_BOX || old_state == dropbox_status_s::DP_CLOSED_BOX) {
			new_state = dropbox_status_s::DP_CLOSED_BOX;
		}
	} else if (left_open && right_open) {
		// Both doors are at open position
		// Only transition to OPENED if we're currently OPENING, or already OPENED
		if (old_state == dropbox_status_s::DP_OPENING_BOX || old_state == dropbox_status_s::DP_OPENED_BOX) {
			new_state = dropbox_status_s::DP_OPENED_BOX;
		}
	}

	// Only update if state changed
	if (new_state != old_state) {
		_dropbox_status.dropbox_state = new_state;
	}
}

// ============================================================================
// SAFETY HANDLERS
// ============================================================================

void Dropboxv2::handle_safety_cases()
{
	// Safety Rule 1: Winch cannot lower unless doors are fully open
	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN &&
	    _dropbox_status.dropbox_state != dropbox_status_s::DP_OPENED_BOX) {
		emergency_stop_winch("not all doors opened");
	}

	// Safety Rule 2: Doors cannot close unless winch is at top
	if (is_closing_doors() && _dropbox_status.winch_state != dropbox_status_s::WINCH_TOP) {
		emergency_stop_doors("winch not at top");
	}
}

bool Dropboxv2::is_closing_doors() const
{
	return (_dropbox_status.dropbox_state == dropbox_status_s::DP_CLOSING_BOX ||
	        _left_door_closing || _right_door_closing);
}

bool Dropboxv2::is_winch_safe_check() const
{
	return _dropbox_status.winch_state == dropbox_status_s::WINCH_TOP;
}

bool Dropboxv2::is_doors_safe_check() const
{
	return _dropbox_status.dropbox_state == dropbox_status_s::DP_OPENED_BOX;
}

void Dropboxv2::emergency_stop_winch(const char *reason)
{
	_winch_ramping_active = false;  // Cancel any ramping
	publish_to_motor(2, 0.0f);
	_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
	PX4_WARN("Emergency: Winch stopped - %s", reason);
}

void Dropboxv2::emergency_stop_doors(const char *reason)
{
	publish_to_motor(0, 0.0f);
	publish_to_motor(1, 0.0f);
	_left_door_closing = false;
	_right_door_closing = false;
	_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENED_BOX;
	PX4_WARN("Emergency: Doors stopped - %s", reason);
}

void Dropboxv2::stop_door_motor(int motor_id, const char *message)
{
	publish_to_motor(motor_id, 0.0f);
	PX4_INFO("%s - motor stopped", message);
}

// ============================================================================
// STARTUP PROCEDURE
// ============================================================================

// ============================================================================
// PRE-OPERATION CHECK
// ============================================================================

void Dropboxv2::pre_operation_check()
{
	PX4_INFO("Starting pre-operation check...");

	bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	// Check and report initial status
	bool doors_ok = left_closed && right_closed;
	bool winch_ok = winch_at_top;

	PX4_INFO("Pre-op: Initial state - Winch: %s, Doors: %s",
		winch_ok ? "TOP ✓" : "NOT AT TOP",
		doors_ok ? "CLOSED ✓" : "NOT CLOSED");

	// STEP 1: Wind winch to top if needed (must be done first for safety)
	if (!winch_ok) {
		PX4_INFO("Pre-op: Winch not at top, winding up...");
		if (!wait_for_winch_to_wind_up()) {
			PX4_ERR("Pre-operation check FAILED - Could not wind winch to top");
			return;
		}
		PX4_INFO("Pre-op: Winch now at top ✓");
	}

	// STEP 2: Close doors if needed (only after winch is at top)
	// Re-check door status in case it changed
	left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	doors_ok = left_closed && right_closed;

	if (!doors_ok) {
		PX4_INFO("Pre-op: Doors not closed, closing...");
		if (!wait_for_doors_to_close()) {
			PX4_ERR("Pre-operation check FAILED - Could not close doors");
			return;
		}
		PX4_INFO("Pre-op: Doors now closed ✓");
	}

	// STEP 3: Save current winch mass as payload weight for dowinch sequence
	_payload_weight = _dropbox_status.winch_mass;
	PX4_INFO("Pre-op: Payload weight saved: %.2f kg (from winch mass)", (double)_payload_weight);

	PX4_INFO("Pre-operation check COMPLETE ✓ - System ready for operation");
}

bool Dropboxv2::wait_for_doors_to_close()
{
	PX4_INFO("Pre-op: Closing doors...");
	closebox();

	const hrt_abstime start_time = hrt_absolute_time();
	const hrt_abstime timeout = 10_s;

	bool left_closed = false;
	bool right_closed = false;

	while (hrt_absolute_time() - start_time < timeout) {
		// Check left door
		if (!left_closed) {
			left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
			if (left_closed) {
				publish_to_motor(0, 0.0f);
				PX4_INFO("Pre-op: Left door closed ✓");
			}
		}

		// Check right door
		if (!right_closed) {
			right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
			if (right_closed) {
				publish_to_motor(1, 0.0f);
				PX4_INFO("Pre-op: Right door closed ✓");
			}
		}

		// Both doors closed - success
		if (left_closed && right_closed) {
			PX4_INFO("Pre-op: Both doors closed ✓");
			return true;
		}

		usleep(100000); // 100ms delay
	}

	// Timeout - stop any remaining motors
	PX4_ERR("Pre-op: FAILED - Door timeout");
	publish_to_motor(0, 0.0f);
	publish_to_motor(1, 0.0f);
	return false;
}

bool Dropboxv2::wait_for_winch_to_wind_up()
{
	PX4_INFO("Pre-op: Winding winch up...");

	// Force cumulative timer to 0 to use slower speed (0.55) during precheck
	_winch_cumulative_time = 0;

	// Use high-level function to start winding
	winchup();

	const hrt_abstime start_time = hrt_absolute_time();
	const hrt_abstime timeout = 30_s;

	while (hrt_absolute_time() - start_time < timeout) {
		hrt_abstime now = hrt_absolute_time();

		// Keep cumulative time at 0 to maintain slow speed
		_winch_cumulative_time = 0;

		// Manually call ramping handler since main loop isn't running
		handle_winch_ramping(now);

		// Check if reached top
		if (!px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN)) {
			_winch_ramping_active = false;  // Cancel ramping
			publish_to_motor(2, 0.0f);
			_dropbox_status.winch_state = dropbox_status_s::WINCH_TOP;
			_winch_cumulative_time = 0;  // Reset timer at top (stays at 0)
			PX4_INFO("Pre-op: Winch at top ✓");
			return true;
		}

		usleep(50000);  // 50ms delay (same as main loop)
	}

	PX4_ERR("Pre-op: FAILED - Winch timeout");
	_winch_ramping_active = false;  // Cancel ramping on timeout
	publish_to_motor(2, 0.0f);
	_winch_cumulative_time = 0;  // Reset on failure
	return false;
}

// ============================================================================
// DOOR CONTROL
// ============================================================================

void Dropboxv2::openbox()
{
	PX4_INFO("Opening both doors");
	_dropbox_status.dropbox_state = dropbox_status_s::DP_OPENING_BOX;

	open_left_door();
	open_right_door();
}

bool Dropboxv2::closebox()
{
	if (!is_winch_safe_check()) {
		PX4_WARN("Cannot close doors - winch not at top");
		return false;
	}

	PX4_INFO("Closing both doors");
	_dropbox_status.dropbox_state = dropbox_status_s::DP_CLOSING_BOX;

	close_left_door();
	close_right_door();
	return true;
}

void Dropboxv2::open_left_door()
{
	PX4_INFO("Opening LEFT door");
	_left_door_opening = true;
	_left_door_closing = false;
	publish_to_motor(0, 1.0f);
}

void Dropboxv2::close_left_door()
{
	if (!is_winch_safe_check()) {
		PX4_WARN("Cannot close left door - winch not at top");
		return;
	}

	PX4_INFO("Closing LEFT door");
	_left_door_closing = true;
	_left_door_opening = false;
	publish_to_motor(0, -1.0f);
}

void Dropboxv2::open_right_door()
{
	PX4_INFO("Opening RIGHT door");
	_right_door_opening = true;
	_right_door_closing = false;
	publish_to_motor(1, 1.0f);
}

void Dropboxv2::close_right_door()
{
	if (!is_winch_safe_check()) {
		PX4_WARN("Cannot close right door - winch not at top");
		return;
	}

	PX4_INFO("Closing RIGHT door");
	_right_door_closing = true;
	_right_door_opening = false;
	publish_to_motor(1, -1.0f);
}

// ============================================================================
// WINCH CONTROL
// ============================================================================

void Dropboxv2::winchup(bool override_lock)
{
	PX4_INFO("Winching up");
	_dropbox_status.winch_state = dropbox_status_s::WINCH_WINDING_UP;

	// Start ramping
	_winch_ramping_active = true;
	_winch_ramp_start_time = hrt_absolute_time();
	_winch_target_speed = WINCH_MAX_SPEED;  // Positive for up
	publish_to_motor(2, 0.0f);  // Start from 0
}

bool Dropboxv2::winchdown(bool override_lock)
{
	if (!is_doors_safe_check()) {
		PX4_WARN("Cannot lower winch - doors not fully open");
		return false;
	}

	PX4_INFO("Winching down");
	_dropbox_status.winch_state = dropbox_status_s::WINCH_WINDING_DOWN;

	// Start ramping
	_winch_ramping_active = true;
	_winch_ramp_start_time = hrt_absolute_time();
	_winch_target_speed = -WINCH_MAX_SPEED;  // Negative for down
	publish_to_motor(2, 0.0f);  // Start from 0
	return true;
}

void Dropboxv2::dowinch()
{
	if (_dropbox_status.winch_state != dropbox_status_s::WINCH_TOP) {
		PX4_WARN("Winch must be at top before starting sequence");
		return;
	}

	if (_payload_weight <= 0.0f) {
		PX4_WARN("Payload weight not set. Run pre-operation check first!");
		return;
	}

	PX4_INFO("Starting automatic winch sequence (payload weight: %.2f kg)", (double)_payload_weight);
	_dowinch_active = true;
	_dowinch_waiting_for_release = false;
	winchdown();

	//To do: this needs to work with the winch mass subscription to automate the release detection
}

void Dropboxv2::handle_dowinch_sequence(const hrt_abstime now)
{
	if (!_dowinch_active) {
		return;
	}

	// Detect payload release by checking if winch mass dropped significantly below saved payload weight
	// Use 10% threshold to detect release (winch mass should drop close to zero when payload releases)
	float release_threshold = _payload_weight * 0.1f;  // 10% of original weight

	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN &&
	    _dropbox_status.winch_mass < release_threshold && !_dowinch_waiting_for_release) {

		PX4_INFO("Payload released (winch mass %.2f < threshold %.2f), waiting 2.0s",
			(double)_dropbox_status.winch_mass, (double)release_threshold);
		_dowinch_waiting_for_release = true;
		_dowinch_wait_start_time = now;

		publish_to_motor(2, 0.0f);
		_winch_ramping_active = false;  // Cancel any ramping
		_dropbox_status.winch_state = dropbox_status_s::WINCH_BOTTOM;
	}

	// Wind back up after wait
	if (_dowinch_waiting_for_release && (now - _dowinch_wait_start_time) > 1_s) {
		PX4_INFO("Wait complete, winding up");
		_dowinch_waiting_for_release = false;
		winchup();
	}

	// Sequence complete
	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_TOP &&
	    _dowinch_active && !_dowinch_waiting_for_release) {
		PX4_INFO("Winch sequence complete");
		_dowinch_active = false;
	}
}

void Dropboxv2::do_dropbox()
{
	// Safety checks before starting sequence
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);
	bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
	bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
	bool doors_closed = left_closed && right_closed;

	if (!winch_at_top) {
		PX4_ERR("Cannot start dropbox sequence - winch not at top");
		return;
	}

	if (!doors_closed) {
		PX4_ERR("Cannot start dropbox sequence - doors not closed");
		return;
	}


	if (_payload_weight <= 0.0f) {
		PX4_WARN("Payload weight not set. Run pre-operation check first!");
		return;
	}

	PX4_INFO("Starting complete dropbox delivery sequence");
	_do_dropbox_active = true;
	_do_dropbox_state = DropboxSequenceState::OPENING_DOORS;
	openbox();
}

void Dropboxv2::handle_do_dropbox_sequence(const hrt_abstime now)
{
	if (!_do_dropbox_active) {
		return;
	}

	bool left_open = !px4_arch_gpioread(LEFT_DOOR_OPEN_LIMIT_PIN);
	bool right_open = !px4_arch_gpioread(RIGHT_DOOR_OPEN_LIMIT_PIN);
	bool doors_fully_open = left_open && right_open;
	bool winch_at_top = !px4_arch_gpioread(WINCH_LIMIT_SWITCH_PIN);

	switch (_do_dropbox_state) {
		case DropboxSequenceState::OPENING_DOORS:
			PX4_INFO("Dropbox sequence: Opening doors...");
			_do_dropbox_state = DropboxSequenceState::WAITING_FOR_DOORS_OPEN;
			break;

		case DropboxSequenceState::WAITING_FOR_DOORS_OPEN:
			if (doors_fully_open) {
				PX4_INFO("Dropbox sequence: Doors fully open, starting dowinch");
				_do_dropbox_state = DropboxSequenceState::RUNNING_DOWINCH;
				dowinch();
			}
			break;

		case DropboxSequenceState::RUNNING_DOWINCH:
			// Wait for dowinch sequence to complete (dowinch will set _dowinch_active to false when done)
			if (!_dowinch_active) {
				PX4_INFO("Dropbox sequence: Dowinch complete, waiting for winch at top");
				_do_dropbox_state = DropboxSequenceState::WAITING_FOR_WINCH_TOP;
			}
			break;

		case DropboxSequenceState::WAITING_FOR_WINCH_TOP:
			if (winch_at_top) {
				PX4_INFO("Dropbox sequence: Winch at top, closing doors");
				_do_dropbox_state = DropboxSequenceState::CLOSING_DOORS;
				closebox();
			}
			break;

		case DropboxSequenceState::CLOSING_DOORS:
		{
			// Check if doors are closed
			bool left_closed = !px4_arch_gpioread(LEFT_DOOR_CLOSE_LIMIT_PIN);
			bool right_closed = !px4_arch_gpioread(RIGHT_DOOR_CLOSE_LIMIT_PIN);
			bool doors_closed = left_closed && right_closed;

			if (doors_closed) {
				PX4_INFO("Dropbox sequence: Doors closed, sequence COMPLETE!");
				_do_dropbox_state = DropboxSequenceState::COMPLETE;
				_do_dropbox_active = false;
			}
			break;
		}

		case DropboxSequenceState::COMPLETE:
		case DropboxSequenceState::IDLE:
			// Nothing to do
			break;
	}
}

// ============================================================================
// CUTTER CONTROL
// ============================================================================

void Dropboxv2::cutline(int repeat_count)
{
	if (_cutter_active) {
		PX4_WARN("Cutter already active, ignoring command");
		return;
	}

	_cutter_start_time = hrt_absolute_time();
	_cutter_active = true;
	_cutter_count = 0;
	_cutter_target_count = repeat_count;

	PX4_INFO("Starting cutter sequence (will repeat %d times) at time: %llu us", repeat_count, (unsigned long long)_cutter_start_time);

	_dropbox_status.cutter_state = dropbox_status_s::CUTTER_CUTLINE;

	publish_to_motor(3, -1.0f);  // Start cutting
}

void Dropboxv2::handle_cutter_sequence(const hrt_abstime now)
{
	if (!_cutter_active) {
		return;
	}

	// Get current time (now might be stale from start of Run() loop)
	hrt_abstime current_time = hrt_absolute_time();
	hrt_abstime elapsed = current_time - _cutter_start_time;

	// Check if 800ms (800000 microseconds) has elapsed
	if (elapsed >= 800000) {
		_cutter_count++;
		PX4_INFO("Cutter cut %d of %d complete, elapsed: %llu us", _cutter_count, _cutter_target_count, (unsigned long long)elapsed);

		publish_to_motor(3, 1.0f);  // Return to idle position

		// Check if we need to repeat
		if (_cutter_count < _cutter_target_count) {
			// Wait a bit before next cut (500ms)
			px4_usleep(500000);

			// Start next cut
			_cutter_start_time = hrt_absolute_time();
			_dropbox_status.cutter_state = dropbox_status_s::CUTTER_CUTLINE;
			publish_to_motor(3, -1.0f);  // Start cutting again
			PX4_INFO("Starting cut %d of %d", _cutter_count + 1, _cutter_target_count);
		} else {
			// All cuts complete
			_dropbox_status.cutter_state = dropbox_status_s::CUTTER_IDLE;
			_cutter_active = false;
			PX4_INFO("All %d cuts complete", _cutter_target_count);
		}
	}
}

// ============================================================================
// WINCH RAMPING
// ============================================================================

void Dropboxv2::handle_winch_ramping(const hrt_abstime now)
{
	// Determine target speed based on cumulative time
	float cumulative_seconds = (float)_winch_cumulative_time / 1000000.0f;
	float target_base_speed = (cumulative_seconds >= 10.0f) ? 1.0f : 0.55f;

	// Check if winch is actively moving
	bool winch_moving = (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP ||
	                     _dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN);

	if (!winch_moving) {
		// Winch not moving - reset ramping states
		_winch_ramping_active = false;
		_winch_speed_transition_ramping = false;
		_winch_last_speed = 0.0f;
		return;
	}

	// Apply direction to target speed
	float target_speed = target_base_speed;
	if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN) {
		target_speed = -target_speed;
	}

	// Handle initial ramp from 0 to initial speed
	if (_winch_ramping_active) {
		hrt_abstime elapsed = now - _winch_ramp_start_time;

		if (elapsed >= WINCH_RAMP_DURATION) {
			// Initial ramp complete
			publish_to_motor(2, target_speed);
			_winch_last_speed = target_speed;
			_winch_ramping_active = false;
			PX4_INFO("Winch initial ramp complete, speed: %.2f", (double)target_speed);
		} else {
			// Ramping from 0 to target
			float ramp_progress = (float)elapsed / (float)WINCH_RAMP_DURATION;
			float current_speed = target_speed * ramp_progress;
			publish_to_motor(2, current_speed);
			_winch_last_speed = current_speed;
		}
		return;
	}

	// Check if we need to transition speed (0.5 <-> 1.0)
	float abs_last_speed = (_winch_last_speed < 0) ? -_winch_last_speed : _winch_last_speed;
	float abs_target_speed = (target_speed < 0) ? -target_speed : target_speed;

	if (!is_float_equal(abs_last_speed, abs_target_speed) && !is_float_equal(abs_last_speed, 0.0f)) {
		// Speed change detected - start transition ramp if not already ramping
		if (!_winch_speed_transition_ramping) {
			_winch_speed_transition_ramping = true;
			_winch_speed_transition_start_time = now;
			PX4_INFO("Starting speed transition from %.2f to %.2f",
				(double)_winch_last_speed, (double)target_speed);
		}
	}

	// Handle speed transition ramping
	if (_winch_speed_transition_ramping) {
		hrt_abstime elapsed = now - _winch_speed_transition_start_time;

		if (elapsed >= WINCH_RAMP_DURATION) {
			// Transition complete
			publish_to_motor(2, target_speed);
			_winch_last_speed = target_speed;
			_winch_speed_transition_ramping = false;
			PX4_INFO("Speed transition complete, speed: %.2f (cumulative time: %.2fs)",
				(double)target_speed, (double)cumulative_seconds);
		} else {
			// Interpolate between last speed and target speed
			float ramp_progress = (float)elapsed / (float)WINCH_RAMP_DURATION;
			float current_speed = _winch_last_speed + (target_speed - _winch_last_speed) * ramp_progress;
			publish_to_motor(2, current_speed);
			_winch_last_speed = current_speed;
		}
	} else {
		// No ramping needed, maintain target speed
		if (!is_float_equal(target_speed, _winch_last_speed)) {
			publish_to_motor(2, target_speed);
			_winch_last_speed = target_speed;
		}
	}
}

void Dropboxv2::update_winch_cumulative_timer(const hrt_abstime now)
{
	// Determine if winch is currently moving
	bool winch_moving = (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP ||
	                     _dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN);

	if (winch_moving) {
		if (!_winch_timer_active) {
			// Winch just started moving - initialize timer
			_winch_last_update_time = now;
			_winch_timer_active = true;
		} else {
			// Winch is moving - update cumulative time
			hrt_abstime delta_time = now - _winch_last_update_time;

			if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_DOWN) {
				// Add time for winding down (positive)
				_winch_cumulative_time += delta_time;
			} else if (_dropbox_status.winch_state == dropbox_status_s::WINCH_WINDING_UP) {
				// Subtract time for winding up (negative)
				if (_winch_cumulative_time > delta_time) {
					_winch_cumulative_time -= delta_time;
				} else {
					_winch_cumulative_time = 0;  // Don't go negative
				}
			}

			_winch_last_update_time = now;
		}
	} else {
		// Winch stopped - deactivate timer but keep cumulative time
		_winch_timer_active = false;
	}
}
// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void Dropboxv2::publish_to_motor(int motor_id, float pwm_value)
{
	_dropbox_position[motor_id] = pwm_value;

	actuator_servos_s actuator_servos{};

	for (int i = 0; i < 8; i++) {
		actuator_servos.control[i] = _dropbox_position[i];
	}

	// All motors (doors, winch, cutter) use servos
	actuator_servos.timestamp = hrt_absolute_time();
	_actuator_servos_pub.publish(actuator_servos);
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
	vcmd.timestamp = now;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = 1;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = 3;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	return vcmd_pub.publish(vcmd);
}

bool Dropboxv2::send_dropbox_command_ack(const hrt_abstime now, const uint32_t cmd, const uint8_t result,
	const uint8_t progress, const uint8_t target_system, const uint16_t target_component)
{
	vehicle_command_ack_s vcmd_ack{};
	vcmd_ack.command = cmd;
	vcmd_ack.result = result;
	vcmd_ack.result_param1 = progress;
	vcmd_ack.result_param2 = 0;
	vcmd_ack.target_system = target_system;
	vcmd_ack.target_component = target_component;

	return _vehicle_command_ack_pub.publish(vcmd_ack);
}

// ============================================================================
// STATUS PUBLISHING
// ============================================================================

void Dropboxv2::publish_status(const hrt_abstime now)
{
	// Publish status every second
	hrt_abstime elapsed = now - _last_status_publish_time;

	if (elapsed >= STATUS_PUBLISH_INTERVAL) {
		_dropbox_status.timestamp = now;
		_dropbox_status_pub.publish(_dropbox_status);
		_last_status_publish_time = now;

		// // Convert cumulative time to seconds for readability
		// float cumulative_seconds = (float)_winch_cumulative_time / 1000000.0f;

		// // State names for easier reading
		// const char* door_states[] = {"Closed", "Opened", "Closing", "Opening"};
		// const char* winch_states[] = {"Top", "Bottom", "Winding Up", "Winding Down", "Mid-Air"};
		// const char* cutter_states[] = {"Idle", "Cutting"};

		// PX4_INFO("Status: Door=%s, Winch=%s, Cutter=%s, WinchMass=%.2fkg, WinchTime=%.2fs",
		// 	door_states[_dropbox_status.dropbox_state],
		// 	winch_states[_dropbox_status.winch_state],
		// 	cutter_states[_dropbox_status.cutter_state],
		// 	(double)_dropbox_status.winch_mass,
		// 	(double)cumulative_seconds);
	}
}

// ============================================================================
// PAYLOAD MASS TRACKING
// ============================================================================

void Dropboxv2::update_payload_mass_samples(float new_mass)
{
	// Add new sample to FIFO array
	_payload_mass_samples[_payload_mass_sample_index] = new_mass;
	_payload_mass_sample_index = (_payload_mass_sample_index + 1) % PAYLOAD_MASS_SAMPLES;

	// Mark array as full once we've wrapped around
	if (_payload_mass_sample_index == 0 && !_payload_mass_array_full) {
		_payload_mass_array_full = true;
		_tared_payload_mass = get_average_payload_mass();
		PX4_INFO("Payload mass tared (no load baseline): %.3f kg", (double)_tared_payload_mass);
	}
}

float Dropboxv2::get_average_payload_mass() const
{
	if (!_payload_mass_array_full) {
		return 0.0f;  // Not enough samples yet
	}

	float sum = 0.0f;
	for (int i = 0; i < PAYLOAD_MASS_SAMPLES; i++) {
		sum += _payload_mass_samples[i];
	}
	return sum / PAYLOAD_MASS_SAMPLES;
}

// ============================================================================
// MODULE INTERFACE
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

int Dropboxv2::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dropbox payload delivery system controller.
Controls doors, winch, and line cutter with safety interlocks.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Dropboxv2", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openbox", "Open both doors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closebox", "Close both doors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openleft", "Open left door");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closeleft", "Close left door");
	PRINT_MODULE_USAGE_COMMAND_DESCR("openright", "Open right door");
	PRINT_MODULE_USAGE_COMMAND_DESCR("closeright", "Close right door");
	PRINT_MODULE_USAGE_COMMAND_DESCR("winchup", "Wind winch up");
	PRINT_MODULE_USAGE_COMMAND_DESCR("winchdown", "Wind winch down");
	PRINT_MODULE_USAGE_COMMAND_DESCR("dowinch", "Auto sequence: lower, release, return");
	PRINT_MODULE_USAGE_COMMAND_DESCR("dodropbox", "Complete delivery: open doors, dowinch, close doors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cutline", "Cut the winch line");
	PRINT_MODULE_USAGE_COMMAND_DESCR("precheck", "Pre-operation safety check");
	PRINT_MODULE_USAGE_COMMAND_DESCR("state", "Print current system state");
	PRINT_MODULE_USAGE_COMMAND_DESCR("kill", "Emergency stop");
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
		PX4_ERR("Dropbox not running");
		return PX4_ERROR;
	}

	const hrt_abstime now = hrt_absolute_time();
	const char *cmd = argv[0];

	// Direct action commands
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
	if (!strcmp(cmd, "cuttertest") || !strcmp(cmd, "cutline")) {
		instance->cutline(5);
		return 0;
	}
	if (!strcmp(cmd, "winchstop")) {
		instance->publish_to_motor(2, 0.0f);
		return 0;
	}
	if (!strcmp(cmd, "dowinch")) {
		instance->dowinch();
		return 0;
	}
	if (!strcmp(cmd, "dodropbox")) {
		instance->do_dropbox();
		return 0;
	}
	if (!strcmp(cmd, "kill")) {
		actuator_armed_s actuator_armed{};
		actuator_armed.armed = false;
		instance->_actuator_armed_pub.publish(actuator_armed);
		PX4_WARN("EMERGENCY STOP - System disarmed. Reboot required.");
		return 0;
	}
	if (!strcmp(cmd, "state")) {
		const char* dp_states[] = {"Closed", "Opened", "Closing", "Opening"};
		const char* winch_states[] = {"Top", "Bottom", "Winding Up", "Winding Down", "Mid-Air"};
		const char* cutter_states[] = {"Idle", "Cutting"};

		// Convert cumulative time to seconds
		float cumulative_seconds = (float)instance->_winch_cumulative_time / 1000000.0f;

		PX4_INFO("Door: %s | Winch: %s | Cutter: %s | Winch Time: %.2fs",
			dp_states[instance->_dropbox_status.dropbox_state],
			winch_states[instance->_dropbox_status.winch_state],
			cutter_states[instance->_dropbox_status.cutter_state],
			(double)cumulative_seconds);
		return 0;
	}

	// Commands that trigger via vehicle_command
	if (!strcmp(cmd, "openbox")) {
		instance->_params[0] = 1.0f;
	} else if (!strcmp(cmd, "closebox")) {
		instance->_params[0] = 2.0f;
	} else if (!strcmp(cmd, "winchup")) {
		instance->_params[1] = 1.0f;
	} else if (!strcmp(cmd, "winchdown")) {
		instance->_params[1] = 2.0f;
	} else if (!strcmp(cmd, "cutline")) {
		instance->_params[2] = 1.0f;
	} else if (!strcmp(cmd, "precheck")) {
		instance->_params[3] = 1.0f;
	} else {
		return print_usage("unknown command");
	}

	// Send command and reset params
	instance->send_vehicle_command(now, vehicle_command_s::VEHICLE_CMD_DO_DROPBOX,
		instance->_params[0], instance->_params[1], instance->_params[2],
		instance->_params[3], instance->_params[4]);
	instance->_params[0] = 0.0f;
	instance->_params[1] = 0.0f;
	instance->_params[2] = 0.0f;
	instance->_params[3] = 0.0f;
	instance->_params[4] = 0.0f;

	return 0;
}

extern "C" __EXPORT int dropboxv2_main(int argc, char *argv[])
{
	return Dropboxv2::main(argc, argv);
}
