/**
 * @file encoder_task.hpp
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

namespace encoder {
	enum EncoderState_e : EventBits_t {
		IDLE     = 0b1 << 0,
		SAMPLING = 0b1 << 1,
		ERROR    = 0b1 << 12
	};

class EncoderTask : public StateTask {
public:
	struct config_params {
		QueueHandle_t speed_qh;
	};

	static EncoderTask& get_instance();

	void set_params(const config_params& params);

	esp_err_t start() override;
	esp_err_t stop()  override;

	EventBits_t get_state() override;

	const Encoder &get_encoder() const;

	virtual ~EncoderTask();
private:

	EncoderTask();
};


}

}
