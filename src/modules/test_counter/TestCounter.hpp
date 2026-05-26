/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 *
 * TestCounter module
 *
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/test_counter.h>

using namespace time_literals;

class TestCounter : public ModuleBase<TestCounter>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TestCounter();
	~TestCounter() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	uORB::Publication<test_counter_s> _test_counter_pub{ORB_ID(test_counter_tx)};
	uORB::Subscription                _test_counter_sub{ORB_ID(test_counter_rx)};

	bool        _is_sender{true};         ///< cached from TCNT_MODE at init: true=Sender, false=Receiver
	float       _freq_hz{20.0f};          ///< cached from TCNT_FREQ_HZ at init

	uint32_t    _counter{0};
	hrt_abstime _last_run_us{0};
	hrt_abstime _last_rx_timestamp{0};  ///< timestamp of last received message, used to compute delta

	static constexpr uint8_t RX_RATE_WINDOW_S{5};
	static constexpr uint16_t RX_TS_BUFFER_SIZE{256};
	hrt_abstime _rx_timestamps[RX_TS_BUFFER_SIZE]{};
	uint16_t _rx_ts_head{0};
	uint16_t _rx_ts_count{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TCNT_MODE>)    _param_tc_mode,    ///< 0.0=Receiver, 1.0=Sender
		(ParamFloat<px4::params::TCNT_FREQ_HZ>) _param_tc_freq_hz  ///< Update rate in Hz
	)
};
