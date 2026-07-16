/*
 * DHT.c
 *
 *  Created on: Apr 19, 2025
 *
 */

#include "DHT.h"

#define lineDown() 		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_RESET)
#define lineUp()		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_SET)
#define getLine()		(HAL_GPIO_ReadPin(sensor->DHT_Port, sensor->DHT_Pin) == GPIO_PIN_SET)
#define Delay(d)		HAL_Delay(d)

static void goToOutput(DHT_sensor *sensor) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Set line high by default
	lineUp();

	// Configure pin as output
	GPIO_InitStruct.Pin = sensor->DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 	// Open-drain output
	GPIO_InitStruct.Pull = sensor->pullUp;			// Pull-up option

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	// High speed
	HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

static void goToInput(DHT_sensor *sensor) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configure pin as input
	GPIO_InitStruct.Pin = sensor->DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = sensor->pullUp;			// Pull-up option
	HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

DHT_data DHT_getData(DHT_sensor *sensor) {
	DHT_data data = {-128.0f, -128.0f};

	#if DHT_POLLING_CONTROL == 1
	/* Polling frequency limitation */
	// Determine polling interval based on sensor type
	uint16_t pollingInterval;
	if (sensor->type == DHT11) {
		pollingInterval = DHT_POLLING_INTERVAL_DHT11;
	} else {
		pollingInterval = DHT_POLLING_INTERVAL_DHT22;
	}

	// If polling too soon, return last known good values
	if ((HAL_GetTick() - sensor->lastPollingTime < pollingInterval) && sensor->lastPollingTime != 0) {
		data.hum = sensor->lastHum;
		data.temp = sensor->lastTemp;
		return data;
	}
	sensor->lastPollingTime = HAL_GetTick() + 1;
	#endif

	/* Request data from sensor */
	// Switch pin to output mode
	goToOutput(sensor);
	// Pull data line low for 18 ms
	lineDown();
	Delay(18);
	// Pull data line high and switch to input mode
	lineUp();
	goToInput(sensor);

	#ifdef DHT_IRQ_CONTROL
	// Disable interrupts to avoid timing issues
	__disable_irq();
	#endif

	/* Wait for response from sensor */
	uint16_t timeout = 0;

	// Wait for line to go low
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			// If no response, sensor is likely not connected
			sensor->lastHum = -128.0f;
			sensor->lastTemp = -128.0f;
			return data;
		}
	}

	timeout = 0;
	// Wait for line to go high
	while(!getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			sensor->lastHum = -128.0f;
			sensor->lastTemp = -128.0f;
			return data;
		}
	}

	timeout = 0;
	// Wait for line to go low again
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) {
			#ifdef DHT_IRQ_CONTROL
			__enable_irq();
			#endif
			return data;
		}
	}

	/* Read data from sensor */
	uint8_t rawData[5] = {0, 0, 0, 0, 0};
	for (uint8_t a = 0; a < 5; a++) {
		for (uint8_t b = 7; b != 255; b--) {
			uint16_t hT = 0, lT = 0;

			// Count time line stays low
			while(!getLine() && lT != 65535) lT++;

			// Count time line stays high
			timeout = 0;
			while(getLine() && hT != 65535) hT++;

			// If high time > low time, bit is 1
			if (hT > lT) rawData[a] |= (1 << b);
		}
	}

	#ifdef DHT_IRQ_CONTROL
	// Re-enable interrupts after reading
	__enable_irq();
	#endif

	/* Check data integrity */
	if ((uint8_t)(rawData[0] + rawData[1] + rawData[2] + rawData[3]) == rawData[4]) {
		// If checksum matches, convert values
		if (sensor->type == DHT22) {
			data.hum = (float)(((uint16_t)rawData[0] << 8) | rawData[1]) * 0.1f;

			// Check for negative temperature
			if (!(rawData[2] & (1 << 7))) {
				data.temp = (float)(((uint16_t)rawData[2] << 8) | rawData[3]) * 0.1f;
			} else {
				rawData[2] &= ~(1 << 7);
				data.temp = (float)(((uint16_t)rawData[2] << 8) | rawData[3]) * -0.1f;
			}
		}
		if (sensor->type == DHT11) {
			data.hum = (float)rawData[0];
			data.temp = (float)rawData[2];
		}
	}

	#if DHT_POLLING_CONTROL == 1
	sensor->lastHum = data.hum;
	sensor->lastTemp = data.temp;
	#endif

	return data;
}

