#pragma once

#include "main.h"

namespace utils {
    struct GPIO_Pin {
		GPIO_TypeDef* GPIOPort;
		uint16_t GPIOPin;
	}; 
}