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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/dropbox_status.h>
#include <uORB/topics/winch_mass.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/input_rc.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

class Dropboxv2 : public ModuleBase<Dropboxv2>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Dropboxv2();
	~Dropboxv2() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	// ========================================================================
	// MODULE BASE INTERFACE
	// ========================================================================
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);

	bool init();

	// ========================================================================
	// PUBLIC API - WINCH CONTROL
	// ========================================================================
	void winchup();      // Start winding winch up (with ramping)
	void winchdown();    // Start winding winch down (with ramping)
	void winchstop();    // Stop winch (with ramping down)

	/**
	 * Pre-operation check sequence:
	 * 1. Save current winch_mass as payload weight
	 * 2. Wind winch to top (if not already there)
	 * 3. Close doors (if enabled and not closed)
	 * Returns true if precheck started successfully
	 */
	bool do_precheck();

	/**
	 * Automatic winch deployment sequence:
	 * - If doors enabled: open doors first, then deploy, then close doors
	 * - Winch down until mass < 0.5kg (payload released)
	 * - Continue down for 3 seconds post-release
	 * - Winch up; if >60% of saved mass detected again, winch down 5s and retry
	 * - When successful, ascend to top limit switch
	 */
	bool do_winch();

	void publish_to_motor(int motor_id, float pwm_value);  // Low-level motor control

	// ========================================================================
	// PUBLIC API - CUTTER CONTROL
	// ========================================================================
	/**
	 * Cut the winch line with repeated cuts
	 * @param repeat_count Number of cut cycles (default: 5)
	 * Each cycle: 800ms cutting + 500ms pause
	 */
	void cutline(int repeat_count = 5);

	// ========================================================================
	// PUBLIC API - DOOR CONTROL (only if DROPBOX_DOOR_EN=1)
	// ========================================================================
	void openbox();          // Open both doors (coordinated control)
	void closebox();         // Close both doors (coordinated control)
	void open_left_door();   // Open left door only
	void close_left_door();  // Close left door only
	void open_right_door();  // Open right door only
	void close_right_door(); // Close right door only
	void stop_door_motor(int motor_id, const char *message);

private:
	// ========================================================================
	// MAIN LOOP
	// ========================================================================
	void Run() override;

	// ========================================================================
	// INITIALIZATION
	// ========================================================================
	void configure_gpio_pins();  // Configure GPIO pins for limit switches and buttons

	// ========================================================================
	// WINCH HANDLERS
	// ========================================================================
	void handle_winch_ramping(const hrt_abstime now);         // Smooth speed ramping up/down
	void handle_winch_limit_switches(const hrt_abstime now);  // Stop at top limit
	void update_winch_distance(const hrt_abstime now);        // Track winch travel distance
	void handle_manual_winch_slowdown(const hrt_abstime now); // Distance-based slowdown for manual control
	void handle_precheck(const hrt_abstime now);              // Precheck state machine
	void handle_auto_winch(const hrt_abstime now);            // Auto winch deployment state machine

	// ========================================================================
	// CUTTER HANDLERS
	// ========================================================================
	void handle_cutter_sequence(const hrt_abstime now);  // Cutter repeat sequence state machine

	// ========================================================================
	// DOOR HANDLERS
	// ========================================================================
	void handle_door_limit_switches(const hrt_abstime now);  // Stop doors at open/close limits

	// ========================================================================
	// INPUT HANDLERS
	// ========================================================================
	void handle_physical_buttons(const hrt_abstime now);  // Physical up/down buttons
	void handle_rc_input(const hrt_abstime now);          // RC radio control input

	// ========================================================================
	// VEHICLE COMMAND HANDLING
	// ========================================================================
	void handle_vehicle_commands(const hrt_abstime now);  // Process MAVLink vehicle commands

	// ========================================================================
	// STATUS PUBLISHING
	// ========================================================================
	void publish_status(const hrt_abstime now);    // Publish dropbox_status to uORB
	void update_winch_mass(const hrt_abstime now); // Update mass from external sensor

	// ========================================================================
	// COMMUNICATION UTILITIES
	// ========================================================================
	bool send_vehicle_command(const hrt_abstime now, const uint32_t cmd, const float param1 = NAN,
		const float param2 = NAN, const float param3 = NAN, const float param4 = NAN,
		const double param5 = static_cast<double>(NAN), const double param6 = static_cast<double>(NAN),
		const float param7 = NAN);
	bool send_dropbox_command_ack(const hrt_abstime now, const uint32_t cmd, const uint8_t result,
		const uint8_t progress, const uint8_t target_system, const uint16_t target_component);

	// ========================================================================
	// STATE VARIABLES - GENERAL
	// ========================================================================
	dropbox_status_s _dropbox_status{};                     // Main status structure
	float _params_buffer[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // Command parameter buffer
	float _params[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};         // Current command parameters
	float _dropbox_position[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // Position array
	uint8_t _cur_vcmd_target_system{0};      // Current vehicle command target system ID
	uint8_t _cur_vcmd_target_component{0};   // Current vehicle command target component ID

	// ========================================================================
	// STATE VARIABLES - PHYSICAL BUTTON INPUTS
	// ========================================================================
	bool _last_door_button_state{false};                     // Previous state of door button
	bool _last_up_button_state{false};                       // Previous state of up button
	bool _last_down_button_state{false};                     // Previous state of down button
	hrt_abstime _last_door_button_time{0};                   // Last time door button changed
	hrt_abstime _last_up_button_time{0};                     // Last time up button changed
	hrt_abstime _last_down_button_time{0};                   // Last time down button changed
	static constexpr hrt_abstime BUTTON_DEBOUNCE_TIME = 200_ms;  // Debounce time for buttons

	// ========================================================================
	// STATE VARIABLES - WINCH RAMPING
	// ========================================================================
	bool _winch_ramping_active{false};       // True when ramping is in progress
	hrt_abstime _winch_ramp_start_time{0};   // Time when current ramp started
	float _winch_target_speed{0.0f};         // Target speed for ramping
	float _winch_initial_speed{0.0f};        // Speed at start of ramp (for ramp down)
	int8_t _winch_direction{0};              // -1 = up, 0 = stop, 1 = down
	bool _winch_stopping{false};             // True when ramping down to stop
	bool _winch_fault{false};                // True when winch hit limit switch while winding down (safety lock)
	bool _limit_switch_was_disengaged{false}; // Track if limit switch was disengaged during winding down

	// ========================================================================
	// STATE VARIABLES - WINCH DISTANCE TRACKING
	// ========================================================================
	float _winch_distance_traveled{0.0f};                    // Total distance traveled in meters
	hrt_abstime _last_distance_update_time{0};               // Last time distance was updated
	hrt_abstime _last_distance_print_time{0};                // Last time distance was printed
	static constexpr hrt_abstime DISTANCE_PRINT_INTERVAL = 500_ms;  // Print every 1 second
	bool _manual_winch_slowdown_initiated{false};            // True when manual slowdown zone entered

	// ========================================================================
	// STATE VARIABLES - DOOR TRACKING (only used if DROPBOX_DOOR_EN=1)
	// ========================================================================
	bool _left_door_opening{false};   // True when left door is opening
	bool _left_door_closing{false};   // True when left door is closing
	bool _right_door_opening{false};  // True when right door is opening
	bool _right_door_closing{false};  // True when right door is closing

	// ========================================================================
	// STATE MACHINE - PRECHECK SEQUENCE
	// ========================================================================
	enum PrecheckState {
		PRECHECK_IDLE,                    // No precheck in progress
		PRECHECK_CHECKING_INITIAL_STATE,  // Checking initial winch/door positions
		PRECHECK_WINDING_UP,              // Winding winch to top
		PRECHECK_CLOSING_DOORS,           // Closing doors (if enabled)
		PRECHECK_SAVING_MASS              // Saving mass measurement
	};
	PrecheckState _precheck_state{PRECHECK_IDLE};
	float _saved_winch_mass{0.0f};                           // Saved payload mass from precheck
	bool _precheck_complete{false};                          // True when precheck completed successfully
	hrt_abstime _precheck_timeout_start{0};                  // Timeout tracking
	static constexpr float PRECHECK_WINCH_SPEED = 0.3f;      // Fixed 0.4 PWM speed for precheck windup
	static constexpr hrt_abstime PRECHECK_WINCH_TIMEOUT = 30_s;  // 30 second timeout for winch
	static constexpr hrt_abstime PRECHECK_DOOR_TIMEOUT = 10_s;   // 10 second timeout for doors

	// ========================================================================
	// MASS FILTERING
	// ========================================================================
	static constexpr int MASS_BUFFER_SIZE = 5;               // Number of samples to average
	float _mass_buffer[MASS_BUFFER_SIZE]{};                  // Circular buffer for mass readings
	int _mass_buffer_index{0};                               // Current index in circular buffer
	int _mass_buffer_count{0};                               // Number of valid samples in buffer
	float _filtered_mass{0.0f};                              // Current filtered mass value

	// ========================================================================
	// STATE MACHINE - AUTO WINCH DEPLOYMENT
	// ========================================================================
	enum AutoWinchState {
		AUTO_WINCH_IDLE,                    // Not running
		AUTO_WINCH_OPENING_DOORS,           // Waiting for doors to fully open (doors mode only)
		AUTO_WINCH_DESCENDING,              // Descending, waiting for mass < threshold
		AUTO_WINCH_POST_RELEASE_WAIT,       // Mass released, continue down for 3 seconds
		AUTO_WINCH_ASCENDING,               // Ascending back to top
		AUTO_WINCH_RETRY_DOWN,              // Mass re-detected on ascent, going back down 5s
		AUTO_WINCH_CLOSING_DOORS            // Waiting for doors to close (doors mode only)
	};
	AutoWinchState _auto_winch_state{AUTO_WINCH_IDLE};
	hrt_abstime _auto_winch_timer{0};                        // General purpose timer for waits
	float _auto_winch_saved_mass{0.0f};                      // Mass at start of sequence (payload weight)
	bool _auto_winch_mass_released{false};                   // True once mass drop detected
	float _cached_release_threshold{0.5f};                   // Cached WINCH_V2_REL_THR: mass below this = payload released (kg)
	float _cached_retry_fraction{1.0f};                      // Cached WINCH_V2_RTY_FRC: fraction of saved mass triggering retry
	float _cached_post_release_time{0.5f};                   // Cached WINCH_V2_POST_T: seconds to continue down after release
	float _cached_retry_down_time{1.5f};                     // Cached WINCH_V2_RETRY_T: seconds to descend on retry
	static constexpr hrt_abstime AUTO_WINCH_DOOR_TIMEOUT = 10_s;          // Timeout waiting for doors

	// ========================================================================
	// WINCH SLOWDOWN ZONES
	// ========================================================================
	hrt_abstime _last_slowdown_print_time{0};                // Rate limit slowdown debug messages
	static constexpr float WINCH_MASS_SAFETY_LIMIT = 30.0f;          // Safety limit - stop winch if mass exceeds this (kg)

	// Universal winch slowdown zones (apply to all winch operations - manual and auto)
	// Upper zone: 0 to WINCH_UPPER_SLOWDOWN_LIMIT meters from top
	// Lower zone: (target - WINCH_LOWER_SLOWDOWN_LIMIT) to target
	static constexpr float WINCH_UPPER_SLOWDOWN_LIMIT = 2.0f;        // Distance from top where slowdown begins (meters)
	static constexpr float WINCH_LOWER_SLOWDOWN_LIMIT = 2.0f;        // Distance from target where slowdown begins (meters)
	static constexpr float WINCH_SLOW_SPEED = 0.6f;                  // Speed in slowdown zones (PWM)

	// ========================================================================
	// WINCH CONFIGURATION CONSTANTS
	// ========================================================================
	static constexpr hrt_abstime WINCH_RAMP_TIME = 1000_ms;  // 1 second ramp time (adjustable)
	static constexpr float WINCH_MAX_SPEED = 1.0f;           // Maximum winch speed

	// ========================================================================
	// CACHED PARAMETERS (to avoid repeated flash reads - critical for ESP32)
	// ========================================================================
	int _cached_pwm_max{2000};              // Cached PWM_MAIN_MAX3 parameter
	float _cached_winch_speed{0.6f};        // Cached WINCH_V2_SPD parameter
	float _cached_target_distance{6.0f};    // Cached WINCH_V2_DIST parameter
	bool _doors_enabled{false};             // Cached DROPBOX_DOOR_EN parameter

	// ========================================================================
	// STATE MACHINE - CUTTER SEQUENCE
	// ========================================================================
	enum CutterState {
		CUTTER_IDLE,     // No cutting in progress
		CUTTER_CUTTING,  // Currently cutting (motor on)
		CUTTER_PAUSING   // Pausing between cuts (motor off)
	};
	CutterState _cutter_state{CUTTER_IDLE};
	hrt_abstime _cutter_start_time{0};                       // Time when current cut/pause started
	int _cutter_count{0};                                    // Number of cuts completed
	int _cutter_target_count{5};                             // Target number of cuts
	static constexpr hrt_abstime CUTTER_CUT_TIME = 800_ms;   // Time to cut (motor on)
	static constexpr hrt_abstime CUTTER_PAUSE_TIME = 500_ms; // Pause between cuts (motor off)

	// ========================================================================
	// PWM CONFIGURATION
	// ========================================================================
	static constexpr int PWM_DISARM = 1500;          // Center/disarm PWM value (µs)
	static constexpr int PWM_DEADZONE_LOW = 1450;    // Lower deadzone threshold (µs)
	static constexpr int PWM_DEADZONE_HIGH = 1550;   // Upper deadzone threshold (µs)

	// ========================================================================
	// FLASH PARAMETERS (PX4 parameter system)
	// ========================================================================
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WINCH_V2_SPD>) _param_winch_speed,      // Winch speed in m/s
		(ParamFloat<px4::params::WINCH_V2_DIST>) _param_winch_target_distance, // Target descent distance in meters
		(ParamInt<px4::params::PWM_MAIN_MAX3>) _param_pwm_max,           // Maximum PWM for motor 3
		(ParamInt<px4::params::DROPBOX_DOOR_EN>) _param_door_enable,     // Door enable flag (0=disabled, 1=enabled)
		(ParamFloat<px4::params::WINCH_V2_REL_THR>) _param_release_threshold,  // Mass below this = payload released (kg)
		(ParamFloat<px4::params::WINCH_V2_RTY_FRC>) _param_retry_fraction,     // Fraction of saved mass that triggers retry
		(ParamFloat<px4::params::WINCH_V2_POST_T>) _param_post_release_time,   // Seconds to continue down after release
		(ParamFloat<px4::params::WINCH_V2_RETRY_T>) _param_retry_down_time     // Seconds to descend on retry
	)

	// ========================================================================
	// STATUS PUBLISHING
	// ========================================================================
	hrt_abstime _last_status_publish_time{0};                // Last time status was published
	static constexpr hrt_abstime STATUS_PUBLISH_INTERVAL = 100_ms;  // Publish every 100ms
	hrt_abstime _last_mass_print_time{0};                    // Last time mass was printed
	static constexpr hrt_abstime MASS_PRINT_INTERVAL = 1000_ms;  // Print mass every 1 second

	// ========================================================================
	// uORB SUBSCRIPTIONS
	// ========================================================================
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};  // Vehicle commands from MAVLink
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};                // RC radio input
	uORB::Subscription _winch_mass_sub{ORB_ID(winch_mass)};            // Winch mass from external sensor

	// ========================================================================
	// uORB PUBLICATIONS
	// ========================================================================
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};  // Command acknowledgements
	uORB::Publication<dropbox_status_s> _dropbox_status_pub{ORB_ID(dropbox_status)};                // Dropbox status
	uORB::Publication<actuator_armed_s> _actuator_armed_pub{ORB_ID(actuator_armed)};                // Actuator armed status
	uORB::PublicationMulti<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};        // Servo/motor outputs

	// ========================================================================
	// PERFORMANCE COUNTERS
	// ========================================================================
	perf_counter_t _loop_perf;  // Main loop performance counter
};
