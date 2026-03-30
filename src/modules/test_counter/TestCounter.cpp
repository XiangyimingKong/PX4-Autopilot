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

// param set TCNT_MODE 1.0 (Sender)
// param set TCNT_MODE 0.0 (Receiver)


#include "TestCounter.hpp"

using namespace time_literals;

TestCounter::TestCounter() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool
TestCounter::init()
{
	// Cache parameters once — both are marked reboot_required so they won't change at runtime
	_is_sender = (_param_tc_mode.get() >= 0.5f);
	_freq_hz   = _param_tc_freq_hz.get();

	const uint32_t interval_us = (_freq_hz > 0.0f) ? (uint32_t)(1e6f / _freq_hz) : 50000U;

	ScheduleOnInterval(interval_us);
	return true;
}

void
TestCounter::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}


	const hrt_abstime now = hrt_absolute_time();

	float update_freq_hz = _freq_hz;

	if (_last_run_us > 0) {
		const float dt_s = (float)(now - _last_run_us) * 1e-6f;
		update_freq_hz = (dt_s > 0.0f) ? (1.0f / dt_s) : _freq_hz;
	}


	if (!_is_sender) {

		// --- RECEIVER mode: read from uORB and print with offset timestamp ---

		if(_test_counter_sub.updated()) {
			test_counter_s received{};
			_test_counter_sub.copy(&received);
		// if (_test_counter_sub.update(&received)) {

			// Compute delta between consecutive received messages
			double delta_ms = 0.0;

			if (_last_rx_timestamp > 0) {
				delta_ms = (double)(received.timestamp - _last_rx_timestamp) * 1e-3;
			}

			_last_rx_timestamp = received.timestamp;

			PX4_INFO("[RX] counter: %lu  delta: %.3f ms  freq: %.2f Hz, update freq: %.2f Hz",
				 (unsigned long)received.counter,
				 delta_ms,
				 (double)received.update_freq_hz, (double)update_freq_hz);
		}
		_last_run_us = now;
		return;
	}

	// --- SENDER mode: compute frequency, publish and print ---

	_last_run_us = now;

	test_counter_s msg{};
	msg.timestamp      = now;
	msg.counter        = _counter;
	msg.update_freq_hz = update_freq_hz;
	_test_counter_pub.publish(msg);

	if (_counter % 10 == 0) {
		PX4_INFO("[TX] counter: %ld  freq: %.2f Hz", _counter, (double)update_freq_hz);
	}
	_counter++;
}

int
TestCounter::task_spawn(int argc, char *argv[])
{
	TestCounter *instance = new TestCounter();

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

int
TestCounter::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
TestCounter::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

TestCounter module. Publishes a test_counter uORB topic at 20Hz
containing a monotonically increasing counter value and the
actual measured update frequency.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("test_counter", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int test_counter_main(int argc, char *argv[])
{
	return TestCounter::main(argc, argv);
}
