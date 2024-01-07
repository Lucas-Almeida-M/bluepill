/*
 * module_cfg.c
 *
 *  Created on: Jan 6, 2024
 *      Author: lucas
 */


#include "module_cfg.h"

module_cfg configs = {0};
module_cfg read_cfg = {0};

int module_cfg_init(void)
{
	int result = 1;
	//TODO: implementar funcao para ler variaveis salvas na flash
	configs.boardID = 1;
	configs.enable = true;

	configs.sensors[0].enable = true;
	configs.sensors[1].enable = true;
	configs.sensors[2].enable = true;
	configs.sensors[3].enable = true;

	configs.inputs[0].enable = true;
	configs.inputs[0].inverted = false;
	configs.inputs[0].debouncingTime = 5;

	configs.inputs[1].enable = true;
	configs.inputs[1].inverted = false;
	configs.inputs[1].debouncingTime = 5;

	configs.inputs[2].enable = true;
	configs.inputs[2].inverted = false;
	configs.inputs[2].debouncingTime = 5;

	configs.inputs[3].enable = true;
	configs.inputs[3].inverted = false;
	configs.inputs[3].debouncingTime = 5;

	return result;
}
