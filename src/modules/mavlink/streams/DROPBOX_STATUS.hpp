/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#ifndef DROPBOX_STATUS_HPP
#define DROPBOX_STATUS_HPP

#include <uORB/topics/dropbox_status.h>

class MavlinkStreamDropboxStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDropboxStatus(mavlink); }

	static constexpr const char *get_name_static() { return "DROPBOX_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DROPBOX_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _dropbox_status_sub.advertised() ? MAVLINK_MSG_ID_DROPBOX_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamDropboxStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _dropbox_status_sub{ORB_ID(dropbox_status)};

	bool send() override
	{
		dropbox_status_s dropbox_status{};
		bool sent = false;

		if (_dropbox_status_sub.update(&dropbox_status)) {
			mavlink_dropbox_status_t mavlink_dropbox_status{};
			mavlink_dropbox_status.dropbox_state = dropbox_status.dropbox_state;
			mavlink_dropbox_status.winch_state = dropbox_status.winch_state;
			mavlink_dropbox_status.cutter_state = dropbox_status.cutter_state;
			mavlink_dropbox_status.winch_mass = dropbox_status.winch_mass;
			mavlink_msg_dropbox_status_send_struct(_mavlink->get_channel(), &mavlink_dropbox_status);
			sent = true;
		}

		return sent;
	}
};

#endif // DROPBOX_STATUS_HPP
