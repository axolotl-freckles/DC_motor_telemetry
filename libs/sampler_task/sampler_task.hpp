/**
 * @file sampler_task.hpp
 * @author ACMAX (aavaloscorrales@gmail.com)
 * @brief
 * @version 0.1
 * @date 2026-03-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "tasks.hpp"

#include "encoder.hpp"

namespace task{

namespace sampler {
	enum SamplerState_e : EventBits_t {
		IDLE     = 0b1 << 0,
		SAMPLING = 0b1 << 1,
		ERROR    = 0b1 << 12
	};

class SamplerTask : public StateTask {
public:
	struct config_params {
		QueueHandle_t speed_qh;
	};

	static SamplerTask& get_instance();

	void set_params(const config_params& params);

	esp_err_t start() override;
	esp_err_t stop()  override;

	EventBits_t get_state() override;

	float current_w();
	float current_TL();
	float current_TI();
	float current_Volt();
	float estimated_load();
	const Encoder &get_encoder() const;

	void set_applied_voltage(float applied_voltage);

	virtual ~SamplerTask();
private:

	SamplerTask();
};

} // namespace sampler

} // namespace task
