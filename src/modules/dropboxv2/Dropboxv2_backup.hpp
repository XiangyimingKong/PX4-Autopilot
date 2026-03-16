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
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <uORB/topics/dropbox_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_motors.h>

#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/payload_mass.h>
#include <uORB/topics/winch_mass.h>
#include <lib/perf/perf_counter.h>

#include <drivers/drv_hrt.h>


using namespace time_literals;

class Dropboxv2 : public ModuleBase<Dropboxv2>, public px4::ScheduledWorkItem
{
public:
	Dropboxv2();
	~Dropboxv2() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);


	bool init();
	void openbox();
	bool closebox();
	void open_left_door();
	void close_left_door();
	void open_right_door();
	void close_right_door();
	void winchup(bool override_lock = false);
	bool winchdown(bool override_lock = false);
	void cutline(int repeat_count = 1);
	void dowinch();
	void do_dropbox();
	void publish_to_motor(int motor_id, float pwm_value);



private:
	void Run() override;

	// Initialization
	void configure_gpio_pins();

	// Input handlers
	void update_payload_mass();
	void update_winch_mass();
	void handle_rc_input(const hrt_abstime now);
	void handle_physical_buttons(const hrt_abstime now);
	void handle_vehicle_commands(const hrt_abstime now);

	// Limit switch handlers
	void handle_dropbox_limit_switches(const hrt_abstime now);
	void handle_winch_limit_switches(const hrt_abstime now);
	void update_dropbox_state(bool left_closed, bool right_closed, bool left_open, bool right_open, const hrt_abstime now);

	// Safety handlers
	void handle_safety_cases();
	bool is_closing_doors() const;
	bool is_winch_safe_check() const;
	bool is_doors_safe_check() const;
	void emergency_stop_winch(const char *reason);
	void emergency_stop_doors(const char *reason);
	void stop_door_motor(int motor_id, const char *message);

	// Startup and pre-operation
	void pre_operation_check();
	bool wait_for_doors_to_close();
	bool wait_for_winch_to_wind_up();

	// Sequences
	void handle_dowinch_sequence(const hrt_abstime now);
	void handle_do_dropbox_sequence(const hrt_abstime now);
	void handle_cutter_sequence(const hrt_abstime now);
	void handle_winch_ramping(const hrt_abstime now);
	void update_winch_cumulative_timer(const hrt_abstime now);

	// Status publishing
	void publish_status(const hrt_abstime now);

	// Payload mass tracking
	void update_payload_mass_samples(float new_mass);
	float get_average_payload_mass() const;

	// Utility
	bool send_vehicle_command(const hrt_abstime now, const uint32_t cmd, const float param1 = NAN,
		const float param2 = NAN, const float param3 = NAN, const float param4 = NAN,
		const double param5 = static_cast<double>(NAN), const double param6 = static_cast<double>(NAN),
		const float param7 = NAN);
	bool send_dropbox_command_ack(const hrt_abstime now, const uint32_t cmd, const uint8_t result,
		const uint8_t progress, const uint8_t target_system, const uint16_t target_component);

	dropbox_status_s _dropbox_status{};
	float _dropbox_position[8]={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float _params[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float _params_buffer[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	int _is_winch_down = 0;
	int _is_sudden_load = 0;
	float _payload_mass = 0;
	float _payload_weight{0.0f};  // Payload weight captured during preflight check

	// Dowinch sequence state
	bool _dowinch_active{false};
	hrt_abstime _dowinch_wait_start_time{0};
	bool _dowinch_waiting_for_release{false};

	// Do_dropbox sequence state
	bool _do_dropbox_active{false};
	enum class DropboxSequenceState {
		IDLE,
		OPENING_DOORS,
		WAITING_FOR_DOORS_OPEN,
		RUNNING_DOWINCH,
		WAITING_FOR_WINCH_TOP,
		CLOSING_DOORS,
		COMPLETE
	};
	DropboxSequenceState _do_dropbox_state{DropboxSequenceState::IDLE};

	hrt_abstime _single_switch_start_time{0};
	bool _single_switch_detected{false};
	uint32_t _last_switch_state{0};
	uint8_t _cur_vcmd_target_system{0};
	uint8_t _cur_vcmd_target_component{0};
	bool _fully_closed_msg_printed{false};
	bool _fully_opened_msg_printed{false};

	// Physical button state tracking
	bool _last_door_button_state{false};
	bool _last_up_button_state{false};
	bool _last_down_button_state{false};
	hrt_abstime _last_door_button_time{0};
	hrt_abstime _last_up_button_time{0};
	hrt_abstime _last_down_button_time{0};

	// Door movement state tracking
	bool _left_door_opening{false};
	bool _left_door_closing{false};
	bool _right_door_opening{false};
	bool _right_door_closing{false};

	// Cutter sequence state
	bool _cutter_active{false};
	hrt_abstime _cutter_start_time{0};
	int _cutter_count{0};           // Current cut number
	int _cutter_target_count{5};    // Target number of cuts (default 5);

	// Winch ramping state
	bool _winch_ramping_active{false};
	hrt_abstime _winch_ramp_start_time{0};
	float _winch_target_speed{0.0f};
	static constexpr hrt_abstime WINCH_RAMP_DURATION = 1_s;  // 1 second
	static constexpr float WINCH_MAX_SPEED = 0.5f;  // Not used anymore - speed determined by cumulative time

	// Speed transition ramping (0.5 to 1.0)
	bool _winch_speed_transition_ramping{false};
	hrt_abstime _winch_speed_transition_start_time{0};
	float _winch_last_speed{0.0f};  // Track last speed for smooth transitions

	// Winch cumulative timer (tracks net downward time)
	hrt_abstime _winch_cumulative_time{0};  // Microseconds of net downward movement
	hrt_abstime _winch_last_update_time{0};  // Last time we updated the cumulative timer
	bool _winch_timer_active{false};  // Whether winch is currently moving
	static constexpr hrt_abstime WINCH_SPEED_TRANSITION_TIME = 3000_ms;  // 3 seconds threshold

	// Status publishing timer
	hrt_abstime _last_status_publish_time{0};
	static constexpr hrt_abstime STATUS_PUBLISH_INTERVAL = 100_ms;

	// Payload mass tracking (FIFO with averaging)
	static constexpr int PAYLOAD_MASS_SAMPLES = 20;
	float _payload_mass_samples[PAYLOAD_MASS_SAMPLES]{};
	int _payload_mass_sample_index{0};
	bool _payload_mass_array_full{false};
	float _tared_payload_mass{0.0f};  // Tared value (no load) from initial samples

	// uORB Subscriptions
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _payload_mass_sub{ORB_ID(payload_mass)};
	uORB::Subscription _winch_mass_sub{ORB_ID(winch_mass)};

	// uORB Publications
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<dropbox_status_s> _dropbox_status_pub{ORB_ID(dropbox_status)};
	uORB::Publication<actuator_armed_s> _actuator_armed_pub{ORB_ID(actuator_armed)};
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::PublicationMulti<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	perf_counter_t _loop_perf;
};
