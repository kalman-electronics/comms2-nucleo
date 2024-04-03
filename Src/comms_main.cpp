#include "comms.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

#include "Radio/Impl/SX1278/SX1278.hpp"
using namespace radio::sx1278;

void comms_init() {

}

void comms_main() {
	PinoutConfig pinout_config = {
		.spi_handle = &hspi1,
		.NSS = {SPI_CS_GPIO_Port, SPI_CS_Pin},
		.RESET = {SPI_RST_GPIO_Port, SPI_RST_Pin},
		.DIO0 = {DIO0_GPIO_Port, DIO0_Pin}
	};

	SX1278 radio(pinout_config);

	auto status = radio.init(
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
		
		radio.transmit(data, len);

		HAL_Delay(5000);
	}
}
