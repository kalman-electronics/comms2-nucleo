#include "comms.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

#include "Radio/Impl/SX1278/SX1278.hpp"
using namespace radio::sx1278;

PinoutConfig pinout_config = {
	.spi_handle = &hspi1,
	.NSS = {SPI_CS_GPIO_Port, SPI_CS_Pin},
	.RESET = {SPI_RST_GPIO_Port, SPI_RST_Pin},
	.DIO0 = {DIO0_GPIO_Port, DIO0_Pin}
};

SX1278 sx_radio(pinout_config);

void comms_init() {

}

void rx_callback() {
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

void comms_main() {
	auto status = sx_radio.init(
		433UL,
		lora::Power::POWER_17_DB,
		lora::SpreadingFactor::SF_6,
		lora::Bandwidth::BW_250_KHZ,
		lora::CodingRate::CR_4_5,
		lora::HeaderMode::IMPLICIT,
		lora::LNAGain::G1,
		lora::PayloadCRC::ON,
		8U,
		60
	);

	sx_radio.on_rx = rx_callback;

	if (status != Status::OK) {
		while(1) {
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			HAL_Delay(100);
		}
	}

	while(1) {
		constexpr uint8_t len = 102;
		uint8_t data[len];
		memset(data, 0xFA, len);
		
		sx_radio.startTransmit(data, len);

		HAL_Delay(5000);
	}
}

extern "C" __weak void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == DIO0_Pin) {
		sx_radio.on_dio0_irq();
		HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	}
}
