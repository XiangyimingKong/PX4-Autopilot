/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef TEST_COUNTER_HPP
#define TEST_COUNTER_HPP

#include <uORB/Subscription.hpp>
#include <uORB/topics/test_counter.h>

class MavlinkStreamTestCounter : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTestCounter(mavlink); }

	static constexpr const char *get_name_static() { return "TEST_COUNTER"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TEST_COUNTER; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _test_counter_sub.advertised() ?
		       (MAVLINK_MSG_ID_TEST_COUNTER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamTestCounter(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _test_counter_sub{ORB_ID(test_counter)};

	bool send() override
	{
		test_counter_s uorb_msg;

		if (_test_counter_sub.update(&uorb_msg)) {
			mavlink_test_counter_t mavlink_msg{};
			mavlink_msg.timestamp      = uorb_msg.timestamp;
			mavlink_msg.counter        = uorb_msg.counter;
			mavlink_msg.update_freq_hz = uorb_msg.update_freq_hz;

			mavlink_msg_test_counter_send_struct(_mavlink->get_channel(), &mavlink_msg);

			return true;
		}

		return false;
	}
};

#endif // TEST_COUNTER_HPP
