/*
 * module_cfg.h
 *
 *  Created on: Jan 6, 2024
 *      Author: lucas
 */

#ifndef INC_MODULE_CFG_H_
#define INC_MODULE_CFG_H_

#include "stdbool.h"
#include "stdint.h"


#define SENSOR_NUMBERS 4
#define INPUT_NUMBERS  4
#define OUTPUT_NUMBERS 4

int module_cfg_init(void);

typedef struct _sensorsCfg
{
	bool enable;

}sensorsCfg;

typedef struct _inputsCfg
{
	bool enable;
	bool inverted;
	uint16_t debouncingTime; // problema - existe apenas 3 timers, sendo necessario um pra cada tempo de debouncing


}inputsCfg;

typedef struct _outputsCfg
{

	bool enable;

}outputsCfg;

typedef struct _module_cfg
{
	uint16_t boardID;
	bool enable;
	sensorsCfg sensors[SENSOR_NUMBERS];
	outputsCfg outputs[INPUT_NUMBERS];
	inputsCfg  inputs[OUTPUT_NUMBERS];

}module_cfg;



#endif /* INC_MODULE_CFG_H_ */
