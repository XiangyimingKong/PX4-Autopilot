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

#include "hx711.hpp"

#include <inttypes.h>
#include <fcntl.h>
#include <string.h>

HX711::HX711(uint8_t gain, uint8_t num_samples) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_num_samples(num_samples),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	switch (gain)
	{
	case 128:
		_total_clock_pulses = 50;
		break;
	case 64:
		_total_clock_pulses = 54;
		break;

	default:
		_total_clock_pulses = 50;
		break;
	}

	_calibration_factor_param = param_find("HX711_FACTOR");
	_calibration_offset_param = param_find("HX711_OFFSET");
	_output_topic_param = param_find("HX711_TOPIC");
	_max_load_param = param_find("HX711_MAX_LOAD");


	param_get(_calibration_offset_param, &_calibration_offset);
	param_get(_calibration_factor_param, &_calibration_factor);
	param_get(_output_topic_param, &_output_topic);
	param_get(_max_load_param, &_max_load);

	_payload_mass_pub.advertise();
	_winch_mass_pub.advertise();


}

HX711::~HX711()
{
	px4_arch_unconfiggpio(HX711_CLK);
	px4_arch_unconfiggpio(HX711_DO);
	px4_arch_gpiosetevent(HX711_DO, false, false, false, nullptr, nullptr);

	stop();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
HX711::data_ready(int irq, void *context, void *arg)
{
	px4_arch_gpiosetevent(HX711_DO, false, false, false, nullptr, nullptr);
	HX711 *instance = static_cast<HX711 *>(arg);

	if (instance->_hx711_rdy.load()) {
		perf_count(instance->_comms_errors);
	}

	// dont generate an event when we pulsing the clock

	instance->_hx711_rdy.store(true);
	instance->ScheduleNow();

	return PX4_OK;
}

int
HX711::start_calibration(float known_mass, uint8_t num_samples)
{
	_calibrated_mass = known_mass;
	_num_calibration_samples = num_samples;
	_calibrating = true;
	_taring = false;
	_calibrated_sample = 0;
	_calibration_factor = 1.0f;
	_sum_measurement = 0.0f;


	return PX4_OK;

}

int
HX711::start_tare(uint8_t num_samples)
{
	_num_calibration_samples = num_samples;
	_calibration_offset = 0.0f;
	_calibrating = false;
	_taring = true;
	_calibration_factor = 1.0f;
	_sum_measurement = 0.0f;
	_calibrated_sample = 0;
	_sum_error_measurement = 0.0f;

	return PX4_OK;

}

int
HX711::init()
{

	/* we are configuring the gpio pins.
	 * This pins are defined within the board config file of the esp32
	*/
	px4_arch_configgpio(HX711_CLK);
	px4_arch_configgpio(HX711_DO);

	// we want to run the interrupt when the Data line goes from high to low when we
	// are waiting for the hx711 te become ready
	px4_arch_gpiosetevent(HX711_DO, false, true, true, &HX711::data_ready, this);

	start();
	PX4_INFO("HX711 init succefully");

	return PX4_OK;
}

void HX711::start()
{

	/* schedule a cycle to start things */
	ScheduleNow();
}

void HX711::stop()
{
	ScheduleClear();
}

void HX711::Run()
{

	perf_begin(_sample_perf);
	if(_hx711_rdy.load()){

		irqstate_t flags = px4_enter_critical_section();

		for(_clk_steps = 0; _clk_steps < _total_clock_pulses;){
			_clk_state = !_clk_state;

			if(_clk_state == 0 && _cnt < 24){

				uint32_t bit = static_cast<uint32_t>(px4_arch_gpioread(HX711_DO));
				_raw_reading = _raw_reading | ( bit << (23 -_cnt));
				_cnt += 1;
			}

			px4_arch_gpiowrite(HX711_CLK, _clk_state);
			_clk_steps +=1;
			px4_udelay(1);

		}

		px4_arch_gpiowrite(HX711_CLK, 0);
		px4_arch_gpiosetevent(HX711_DO, false, true, true, &HX711::data_ready, this);
		_hx711_rdy.store(false);

		px4_leave_critical_section(flags);

		_clk_steps = 0;
		_cnt = 0;
		_clk_state = 0;

		// Handling if the measurement is negative
		if(_raw_reading & 0x800000){
			_raw_reading |= 0xff000000;
		}

		if(!(_raw_reading == 8388607 || _raw_reading == 0 || _raw_reading == 1))
		{

			_measurement = ((static_cast<float>(_raw_reading) / 8388607.0f ) - _calibration_offset) * _calibration_factor;
			_raw_reading_saved = _raw_reading;

			_avg_measurement_sum = _avg_measurement_sum + _measurement;

			_avg_sample_num += 1;
			_raw_reading = 0;

			if(_calibrating)
			{
				_calibrated_sample += 1;
				calibrate_factor(_measurement, _calibrated_sample);
			}
			if(_taring)
			{
				_calibrated_sample += 1;
				calibrate_tare(_measurement, _calibrated_sample);
			}
			if(!_calibrating && !_taring){

				hrt_abstime timestamp = hrt_absolute_time();

				if(static_cast<float>(fabs(_measurement)) < _max_load) {

					if (_output_topic == 0) {
						// Publish to payload_mass topic
						_payload_mass.timestamp = timestamp;
						_payload_mass.mass = _measurement;
						_payload_mass.average_mass = _avg_measurement;
						_payload_mass_pub.publish(_payload_mass);

					} else if (_output_topic == 1) {
						// Publish to winch_mass topic
						_winch_mass.timestamp = timestamp;
						_winch_mass.mass = _measurement;
						_winch_mass.average_mass = _avg_measurement;
						_winch_mass_pub.publish(_winch_mass);

					}else{
						PX4_INFO("HX711_TOPIC param invalid, must be 0 (payload_mass) or 1 (winch_mass)");
					}
				}
				else {
					PX4_INFO("HX711 measurement out of range: %f kg", (double)_measurement);
				}

			}

			if(_avg_sample_num == _num_samples && !_calibrating && !_taring){
				_avg_measurement = (_avg_measurement_sum / static_cast<float>(_avg_sample_num));
				_avg_measurement_sum = 0.0f;
				_avg_sample_num = 0;

			}
		} else {
			_raw_reading = 0;
			_measurement = 0;
			_clk_state = 0;
		}
	}

	perf_end(_sample_perf);
}

void HX711::print_info()
{
	PX4_INFO("Raw Reading: %ld",_raw_reading_saved);
	PX4_INFO("Average Reading: %f kg", (double)_avg_measurement);
	PX4_INFO("Number of sample to average: %d", _num_samples);
	PX4_INFO("Calibrated offset: %f", (double)_calibration_offset);
	PX4_INFO("Calibration factor %f", (double)_calibration_factor);
	PX4_INFO("Output topic: %s", _output_topic == 0 ? "payload_mass" : "winch_mass");
	PX4_INFO("Calibrating:  %d", _calibrating);
	PX4_INFO("Taring %d", _taring);

	// if you have calibrated this power cycle we will displat it only then
	if(_have_calibrated && !_calibrating){
		PX4_INFO("Calibrated with %f kg mass", (double)_calibrated_mass);
		PX4_INFO("Calibrated with %d samples", _num_calibration_samples);
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void HX711::calibrate_factor(float measure_mass, uint8_t sample_num)
{

	// expected ADC output given the mass applied to the loadcell
	_sum_measurement += measure_mass;

	if(sample_num == _num_calibration_samples){
		 _calibration_factor = _calibrated_mass / (_sum_measurement) * static_cast<float>(sample_num);
		// we are saving the calibrated offset to memory;
		param_set_no_notification(_calibration_factor_param, &_calibration_factor);
		_calibrating = false;
		_have_calibrated = true;
		_sum_error_measurement = 0.0f;
		_sum_measurement = 0.0f;
		_calibrated_sample = 0;
		_avg_sample_num = 0;

	}

}

void HX711::calibrate_tare(float measure_mass, uint8_t sample_num)
{

	// expected ADC output given the mass applied to the loadcell
	float error_measurement = measure_mass;

	_sum_error_measurement += error_measurement;

	if(sample_num == _num_calibration_samples){
		_calibration_offset = _sum_error_measurement / static_cast<float>(sample_num);
		// we are saving the calibrated offset to memory;
		param_set_no_notification(_calibration_offset_param, &_calibration_offset);
		_taring = false;
		_have_calibrated = true;
		_sum_error_measurement = 0.0f;
		_calibrated_sample = 0;
		_avg_sample_num = 0;

	}

}
