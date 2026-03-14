/**
 * @file filters.hpp
 * @author ACMAX (aavaloscorales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-14
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

class Filter {
public:
	/**
	 * @brief Receives the next value of a secuence and returns the next value
	 * on the filtered secuence.
	 * Passing values of other or multiple secuences MUST not be done.
	 *
	 * @param value Next sample on the signal to filter
	 * @return Sample of the filtered signal
	 */
	virtual float operator() (float value) = 0;
};

template <int av_window_size = 2>
class SlidingAverage : public Filter {
public:
	explicit SlidingAverage(const SlidingAverage<av_window_size> &other);
	SlidingAverage();
	SlidingAverage(float starting_average);

	inline const float* samples()  const { return _samples; }
	inline float current_average() const { return _curr_av; }

	float operator() (float value) override;

private:
	float _samples[av_window_size];
	float _curr_av;
	int   _idx;
};

class LowPassRC : public Filter {
public:
	explicit LowPassRC(const LowPassRC &other);
	LowPassRC(float rc, float sampleTime_s);

	inline void set_rc(float rc) { _alpha = _sample_time_s/(_sample_time_s+rc);}
	inline float alpha()       const { return _alpha;         }
	inline float prevVal()     const { return _prev_val;      }
	inline float sample_time() const { return _sample_time_s; }

	float operator() (float value) override;

private:
	const float _sample_time_s;
	float _alpha;
	float _prev_val;
};
