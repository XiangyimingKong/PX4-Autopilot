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
	// Schedule to run at 20Hz
	ScheduleOnInterval(TEST_COUNTER_INTERVAL_US);
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

	// Compute actual update frequency
	float update_freq_hz = TEST_COUNTER_UPDATE_RATE_HZ;

	if (_last_run_us > 0) {
		const float dt_s = (float)(now - _last_run_us) * 1e-6f;
		update_freq_hz = (dt_s > 0.0f) ? (1.0f / dt_s) : TEST_COUNTER_UPDATE_RATE_HZ;
	}

	_last_run_us = now;

	// Publish the message
	test_counter_s msg{};
	msg.timestamp      = now;
	msg.counter        = _counter;
	msg.update_freq_hz = update_freq_hz;
	_test_counter_pub.publish(msg);

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
