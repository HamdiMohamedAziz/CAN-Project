/*
 * DHT.h
 *
 *  Created on: Apr 19, 2025
 *
 */

#ifndef DHT_H_
#define DHT_H_

#include "main.h"

/* Settings */
#define DHT_TIMEOUT 				10000	// Number of iterations before the function returns empty values
#define DHT_POLLING_CONTROL			1		// Enable polling frequency control for the sensor
#define DHT_POLLING_INTERVAL_DHT11	2000	// Polling interval for DHT11 (0.5 Hz according to datasheet). Can be set to 1500 and it will still work
#define DHT_POLLING_INTERVAL_DHT22	1000	// Polling interval for DHT22 (1 Hz according to datasheet)
#define DHT_IRQ_CONTROL						// Disable interrupts during data exchange with the sensor

/* Structure for the data returned by the sensor */
typedef struct {
	float hum;   // Humidity
	float temp;  // Temperature
} DHT_data;

/* Type of sensor used */
typedef enum {
	DHT11,
	DHT22
} DHT_type;

/* Sensor object structure */
typedef struct {
	GPIO_TypeDef *DHT_Port;	// Sensor GPIO port (GPIOA, GPIOB, etc.)
	uint16_t DHT_Pin;		// Sensor GPIO pin number (GPIO_PIN_0, GPIO_PIN_1, etc.)
	DHT_type type;			// Sensor type (DHT11 or DHT22)
	uint8_t pullUp;			// Whether a pull-up is required for the data line (GPIO_NOPULL = no, GPIO_PULLUP = yes)

	// Sensor polling frequency control. Do not fill these values manually!
	#if DHT_POLLING_CONTROL == 1
	uint32_t lastPollingTime; // Last time the sensor was polled
	float lastTemp;			  // Last temperature value
	float lastHum;			  // Last humidity value
	#endif
} DHT_sensor;

/* Function prototypes */
DHT_data DHT_getData(DHT_sensor *sensor); // Get data from the sensor

#endif

