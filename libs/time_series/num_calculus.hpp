/**
 * @file num_calculus.hpp
 * @author ACMAX (aavaloscorales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <limits>

class Integrator {
public:
	Integrator(
		const float SAMPLE_TIME_s,
		const float starting_value = 0.0f,
		const float saturator_max =  std::numeric_limits<float>::infinity(),
		const float saturator_min = -std::numeric_limits<float>::infinity()
	);
	explicit Integrator(const Integrator &_other);

	inline float  sampleTime()         const { return _SAMPLE_TIME_s; }
	inline float  integralAcumulator() const { return _integral_acum; }
	inline float  saturatorMax() const { return _saturator_max; }
	inline float  saturatorMin() const { return _saturator_min; }
	inline float& saturatorMax() { return _saturator_max; }
	inline float& saturatorMin() { return _saturator_min; }
	inline void   setIntegralAcumulator(float integral_acum) {
		_integral_acum = integral_acum;
	}

	/**
	 * @brief Calculates the integral of the given value
	 *
	 * @param value Next value to integrate
	 * @return float Integral of the history of values
	 */
	float operator() (float value);
private:
	const float _SAMPLE_TIME_s;
	float       _saturator_max;
	float       _saturator_min;
	float       _integral_acum;
};

class Derivator {
public:
	Derivator(const float SAMPLE_TIME_S, const float starting_value = 0.0f);
	explicit Derivator(const Derivator &_other);

	inline float sampleTime()    const { return _SAMPLE_TIME_s; }
	inline float previousValue() const { return _prev_val; }

	/**
	 * @brief Calculates the derivative of the given value
	 *
	 * @param value Next value to calculate the derivative
	 * @return float Derivative of the value
	 */
	float operator() (float value);
private:
	const float _SAMPLE_TIME_s;
	float       _prev_val;
};
