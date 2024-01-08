#include <Parse.h>


uint32_t TxMailbox;

CAN_FilterTypeDef *canFilterConfig1;

bool ValidatePacket(uint8_t canID)
{
    for (int validationId = 0; validationId < CANID_COUNT; validationId++)
    {
        if (canID == validationId)
        {
            return true;
        }
    }

    return false;
}



void DecodeCanPacket(uint32_t canID, UartPacket *uartPacket, uint8_t *buffer)// decodifica o pacote can
{

	uartPacket->packet.canID =  (uint8_t) canID;
	uartPacket->packet.seq =  buffer[0];
	uartPacket->packet.crtl =  buffer[1];

	for (int i = 0; i < 6; i++)
	{
		uartPacket->packet.data[i] = buffer [i + 2];
	}
}

bool CanWritePacket(uint32_t id, uint8_t *buffer, uint8_t can_rtr, uint16_t tamanho)
{

	TxHeader.StdId             = id;             // ID da mensagem
	TxHeader.RTR               = can_rtr;       //(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
	TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended
	TxHeader.DLC               = tamanho;      //Tamanho do pacote 0 - 8 bytes
	TxHeader.TransmitGlobalTime = DISABLE;

	//Data Frame é a mensagem que contém informações a serem transmitidas do emissor ao receptor

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, buffer, &TxMailbox) != HAL_OK)
	{

		return false;
	}

	 while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 3) { }

	return true;
}

void LoadFilterList(FilterList *filterIdList) {

	for (int i = 0; i < CANID_COUNT; i++) {

        filterIdList->FilterIdList[i] = (uint32_t) i;
    }
}

void ConfigFilterList (uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, uint32_t filterBank, uint8_t idType, uint8_t filterScale)
{
    CAN_FilterTypeDef filterConfig = {0};

    filterConfig.FilterActivation = ENABLE;
    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filterConfig.FilterBank = filterBank;

    if (filterScale == FILTER_TYEPE_32) {

        filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;

        if (idType == EXT_ID) {

            filterConfig.FilterIdHigh = (id1 & 0x1FFFFFFF) | (id1 << 31);
            filterConfig.FilterIdLow = (id2 & 0x1FFFFFFF) | (id2 << 31);
            filterConfig.FilterMaskIdHigh = (id3 & 0x1FFFFFFF) | (idType << 31);
            filterConfig.FilterMaskIdLow = (id4 & 0x1FFFFFFF) | (idType << 31);

        } else {
            filterConfig.FilterIdHigh = id1 & 0x1FFFFFFF;
            filterConfig.FilterIdLow = id2 & 0x1FFFFFFF;
            filterConfig.FilterMaskIdHigh = id3 & 0x1FFFFFFF;
            filterConfig.FilterMaskIdLow = id4 & 0x1FFFFFFF;
        }

    } else {

        filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
        filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;

        if (idType == EXT_ID) {
            filterConfig.FilterIdHigh = ((id1 & 0x1FFF) << 5) | (idType << 15);
            filterConfig.FilterIdLow = ((id2 & 0x1FFF) << 5) | (idType << 15);
            filterConfig.FilterMaskIdHigh = ((id3 & 0x1FFF) << 5) | (idType << 15);
            filterConfig.FilterMaskIdLow = ((id4 & 0x1FFF) << 5) | (idType << 15);
        } else {
            filterConfig.FilterIdHigh = (id1 & 0x7FF) << 5;
            filterConfig.FilterIdLow = (id2 & 0x7FF) << 5;
            filterConfig.FilterMaskIdHigh = (id3 & 0x7FF) << 5;
            filterConfig.FilterMaskIdLow = (id4 & 0x7FF) << 5;
        }
    }

    HAL_CAN_ConfigFilter(&hcan, &filterConfig);

   // return (HAL_CAN_ConfigFilter(&hcan, &filterConfig) == HAL_OK) ? false : true;
}

void InitFilterList(uint32_t *idList, uint8_t numFilters, uint8_t filterScale)
{

	//Como o protocolo é simples, não foi adicionada a o possibilidade de utilização com EXTID (id com 29 bits)
	uint16_t filterBankCounter = 0;
	uint8_t maxFilterBanks = FILTER_MAX_NUM; //STM32F0 tem somente uma interface CAN assim tendo apenas 14 filter banks
	uint32_t id, id1, id2, id3, id4 = 0;


	if (numFilters > maxFilterBanks)
	{
		numFilters = maxFilterBanks;
	}


	if (filterScale == FILTER_TYEPE_32) //Utilizando o registrados em 32bits
	{
		for (id = 0; id < numFilters && filterBankCounter < maxFilterBanks; id += 2)
		{
			id1 = idList[id];
			id2 = idList[id + 1];
			ConfigFilterList(id1, id2, 0x0000, 0x0000, filterBankCounter, STD_ID, filterScale); // considerar sómente o ID1 e ID2
			filterBankCounter++;
		}
	}
	else
	{
		for (id = 0; id < numFilters && filterBankCounter < maxFilterBanks;id += 4)
		{
			id1 = idList[id];
			id2 = idList[id + 1];
			id3 = idList[id + 2];
			id4 = idList[id + 3];
			ConfigFilterList(id1, id2, id3, id4, filterBankCounter, STD_ID, filterScale);
			filterBankCounter++;
		}
	}
}






