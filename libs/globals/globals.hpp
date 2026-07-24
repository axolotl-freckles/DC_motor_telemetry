/**
 * @file globals.hpp
 * @author ACMAX (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2026-06-25
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>

constexpr uint32_t SAMPLE_TIME_ms    = 10;
constexpr uint64_t MODEL_SIM_TIME_us = 1000;
constexpr uint32_t MODEL_SIM_TIME_ms = MODEL_SIM_TIME_us/1000U;
constexpr float    SAMPLE_TIME_s     = SAMPLE_TIME_ms*1e-3;
constexpr float    MODEL_SIM_TIME_s  = MODEL_SIM_TIME_ms*1e-3f;
