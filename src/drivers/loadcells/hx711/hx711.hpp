/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file hx711.hpp
 * @author Henry Kotze <henry@flycloudline.com>
 *
 * Driver for the HX711 Amplifier.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/payload_mass.h>
#include <uORB/topics/winch_mass.h>

#include <drivers/drv_hrt.h>

using namespace time_literals;


class HX711 : public px4::ScheduledWorkItem
{
public:
	HX711(uint8_t gain, uint8_t num_samples);
	~HX711() override;

	int 			        init();
	void				print_info();
	static int			data_ready(int irq, void *context, void *arg);
	int				start_calibration(float known_mass, uint8_t num_samples);
	int 				start_tare(uint8_t num_samples);

private:

	void				start();
	void				stop();
	void				Run() override;
	void                            calibrate_tare(float measure_mass, uint8_t sample_num);
	void   				calibrate_factor(float measure_mass, uint8_t sample_num);

	uORB::Publication<payload_mass_s> _payload_mass_pub{ORB_ID(payload_mass)};
	uORB::Publication<winch_mass_s> _winch_mass_pub{ORB_ID(winch_mass)};

	payload_mass_s _payload_mass{};
	winch_mass_s _winch_mass{};

	uint8_t _gain{64};
	uint8_t _num_samples{0};
	float    _max_load{0.0f};
	float    _calibration_factor{1.0};
	uint8_t _total_clock_pulses{50};
	uint8_t _cnt{0};

	int32_t _raw_reading{0};
	int32_t _raw_reading_saved{0};
	float    _measurement{0.0f};
	float    _avg_measurement{0.0f};
	float    _measurement_sum{0.0f};
	float    _avg_measurement_sum{0.0f};
	float    _calibration_offset{0.0f};
	float    _calibrated_mass{0.0f};
	float    _sum_error_measurement{0.0f};
	float    _sum_measurement{0.0f};
	uint8_t _clk_state{0};
	px4::atomic<bool> _hx711_rdy{false};

	uint8_t _clk_steps{0};
	uint8_t _avg_sample_num{0};
	uint8_t _sample_num{0};
	uint8_t _calibrated_sample{0};
	bool    _calibrating{false};
	bool    _taring{false};
	bool    _have_calibrated{false};
	uint8_t _num_calibration_samples{0};

	param_t _calibration_offset_param;
	param_t _calibration_factor_param;
	param_t _output_topic_param;
	param_t _max_load_param;
	int32_t _output_topic{0};  // 0 = payload_mass, 1 = winch_mass


	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

};
