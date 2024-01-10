
#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_


#include "can.h"
#include <stdio.h>
#include <stdbool.h>


#define STD_ID  0
#define EXT_ID  1
#define FILTER_TYPE_16 0
#define FILTER_TYEPE_32 1
#define FILTER_MAX_NUM 14


#define CAN_HEADER 2
#define CAN_SIZE 8


extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;


typedef union // Ini
{

	enum FilterIds
	{
		BROADCAST = 0x00,
		BOARD_F7 = 0x01,
		DEVICE_1 = 0X02, // HABILITAR DE ACORDO COM DEVICE
//		DEVICE_2 = 0X03,
//		DEVICE_3 = 0X04,
//		DEVICE_4 = 0X05,
//		DEVICE_5 = 0X06,
//		DEVICE_6 = 0X07,
		CANID_COUNT

	} FilterId;

	uint32_t FilterIdList[CANID_COUNT];

} FilterList;

typedef struct control
{
	bool bitControl0 : 1;
	bool bitControl1 : 1;
	bool bitControl2 : 1;
	bool bitControl3 : 1;
	bool bitControl4 : 1;
	bool bitControl5 : 1;
	bool bitControl6 : 1;
	bool bitControl7 : 1;
}controlBit;

typedef union CANPACKET
{
	uint8_t buffer[CAN_SIZE + 1];
	struct
	{
		uint8_t canID;
		uint8_t seq;
		union cont
		{
			controlBit controlBits;
			uint8_t control;
		}ctrl0;

		union cont1
		{
			controlBit controlBits;
			uint8_t control;
		}ctrl1;

		uint8_t data [CAN_SIZE - CAN_HEADER];
	}packet;

} CanPacket;


typedef union UARTPACKET
{
	uint8_t buffer[CAN_SIZE + 1];
	struct
	{
		uint8_t canID;
		uint8_t seq;
		uint8_t crtl;
		uint8_t data [(CAN_SIZE + 1) - CAN_HEADER];
	}packet;

}UartPacket;



bool ValidatePacket(uint8_t canID);


void DecodeCanPacket(uint32_t canID, UartPacket *uartPacket, uint8_t *buffer);
bool CanWritePacket(uint32_t id, uint8_t *buffer, uint8_t can_rtr, uint16_t tamanho);
void ConfigFilterList (uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t filterBank, uint8_t idType, uint8_t filterScale);





void LoadFilterList(FilterList *filterIdList);
void InitFilterList(uint32_t *idList, uint8_t numFilters, uint8_t filterScale);


#endif /* SRC_PARSER_H_ */
