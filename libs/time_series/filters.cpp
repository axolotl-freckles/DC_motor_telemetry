/**
 * @file filters.cpp
 * @author ACMAX (aavaloscorales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-14
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "filters.hpp"

float LowPassRC::operator() (float value) {
	float new_val = _alpha*value + (1-_alpha)*_prev_val;
	_prev_val = new_val;
	return new_val;
}
LowPassRC::LowPassRC(const LowPassRC &other)
: Filter()
	, _sample_time_s(other.sample_time())
	, _alpha(other.alpha())
	, _prev_val(other.prevVal())
{}
LowPassRC::LowPassRC(float rc, float sampleTime_s)
: Filter()
	, _sample_time_s(sampleTime_s)
	, _alpha(sampleTime_s/(sampleTime_s+rc))
	, _prev_val(0.0)
{}

template<int av_window_size>
float SlidingAverage<av_window_size>::operator() (float value) {
	_samples[_idx] = value;
	_curr_av += (_samples[_idx] - _samples[(_idx+av_window_size)%av_window_size])
	            /av_window_size;
	_idx = (_idx + 1)%av_window_size;
	return _curr_av;
}
template<int av_window_size>
SlidingAverage<av_window_size>::SlidingAverage(const SlidingAverage<av_window_size> &other)
: Filter()
	, _curr_av(other._curr_av)
	, _idx(0)
{
	for (int i=0; i<av_window_size; i++) _samples[i] = other.samples()[i];
}
template<int av_window_size> SlidingAverage<av_window_size>::SlidingAverage()
: Filter()
	, _samples{0}
	, _curr_av(0)
	, _idx(0)
{}
template<int av_window_size> SlidingAverage<av_window_size>::SlidingAverage(float starting_average)
: Filter()
	, _samples{0}
	, _curr_av(starting_average)
	, _idx(0)
{}
