/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "NMEA.h"
#include "string.h"
#include "stm32f1xx_hal.h"

/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/

#define NMEA_MES_MAX_LEN   82u
#define HAL_TIMEOUT    10u // 10ms
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	IDLE = 0u,
	DATA,
	MESSAGE_CRC1,
	MESSAGE_CRC2,
	EOL_SYMBOL1,
	EOL_SYMBOL2,
}dtNMEAReceiveState;
/*----------------------------------------------------------------------------*/
typedef struct{
	uint8_t NMEA_buf[NMEA_MES_MAX_LEN];
	uint8_t ReceiveDataCnt;
	uint8_t receivedCRC;
	uint8_t isMessageReceivedFlag;
	uint8_t messageCnt;
	dtNMEAReceiveState receiveState;
}dtNMEAReceiver;
/*----------------------------------------------------------------------------*/
typedef struct{
	char cardinalPoint;
	uint8_t degree;
	float minute;
}dtNMEACoordinate;

typedef struct{
	dtNMEACoordinate latitude;
	dtNMEACoordinate longitude;
	float altitude;
	float speed;
	uint8_t satteliteAmount;
	uint8_t isDataReliable;
}dtNMEAData;
/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
dtNMEAReceiver NMEAReceiver;
dtNMEAData NMEAFineData;
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/

/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/

/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/
int8_t atoi_nibble_hex(uint8_t symb){
	uint8_t nibbleDec = 0;
	if((symb >= (uint8_t)'A') && (symb <= (uint8_t)'F')){
		nibbleDec = 10u + (symb - (uint8_t)'A');
	}else if((symb >= (uint8_t)'a') && (symb <= (uint8_t)'f')){
		nibbleDec = 10u + (symb - (uint8_t)'a');
	}else if((symb >= (uint8_t)'0') && (symb <= (uint8_t)'9')){
		nibbleDec = symb - (uint8_t)'0';
	}else{
		return -1;
	}
	return (int8_t)nibbleDec;
}

/*----------------------------------------------------------------------------*/
uint8_t NMEA_CrcCalc(dtNMEAReceiver* receiver){
	uint8_t crc = 0u;
	for(uint8_t i = 0u; i < receiver->ReceiveDataCnt; i++){
		crc ^= receiver->NMEA_buf[i];
	}
	return crc;
}

/*----------------------------------------------------------------------------*/
void receiveNMEAMessage(dtNMEAReceiver* receiver, uint8_t receivedByte){
	int8_t crc1 = 0u, crc2 = 0u;

	switch(receiver->receiveState){
	    case IDLE:
	    	if('$' == (char)receivedByte){
	    		receiver->receiveState = DATA;
	    		receiver->ReceiveDataCnt = 0u;
	    	}
	    	break;
	    case DATA:
	    	if(receiver->ReceiveDataCnt < NMEA_MES_MAX_LEN){
	    		if('*' != (char)receivedByte){
	    			receiver->NMEA_buf[receiver->ReceiveDataCnt++] = receivedByte;
	    		}else{
	    			receiver->receiveState = MESSAGE_CRC1;
	    		}
	    	}else{
	    		receiver->receiveState = IDLE;
	    	}
	    	break;
	    case MESSAGE_CRC1:
	    		crc1 = atoi_nibble_hex(receivedByte);
	    		if(crc1 >= 0u){
	    			receiver->receivedCRC = (crc1* 0x10);
	    			receiver->receiveState = MESSAGE_CRC2;
	    		}else{
	    			receiver->receiveState = IDLE;
	    		}
	    		break;
	    case MESSAGE_CRC2:
	    		crc2 = atoi_nibble_hex(receivedByte);
	    		if(crc2 >= 0u){
	    			receiver->receivedCRC += crc2 ;
	    			if(receiver->receivedCRC == NMEA_CrcCalc(receiver)){
	    				receiver->receiveState = EOL_SYMBOL1;
	    			}else{
	    				receiver->receiveState = IDLE;
	    			}
	    		}else{
	    			receiver->receiveState = IDLE;
	    		}
	    		break;
	    case EOL_SYMBOL1:
	    	if('\r' == (char)receivedByte){
	    		receiver->receiveState = EOL_SYMBOL2;
	    	}else{
	    		receiver->receiveState = IDLE;
	    	}
	    	break;
	    case EOL_SYMBOL2:
	    	if('\n' == (char)receivedByte){
	    		/* Complete message has been received */
	    		receiver->isMessageReceivedFlag = PASS;
	    		receiver->messageCnt++;
	    	}
	    	receiver->receiveState = IDLE;
	    	break;
	    default:
	    	/* defensive programming */
	    	receiver->receiveState = IDLE;
	}
}
/*----------------------------------------------------------------------------*/
uint8_t parseNMEAMessage(dtNMEAData* Data, uint8_t rawMessage, uint8_t rawMessageLen){


}
/*----------------------------------------------------------------------------*/


/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
void GPS_init(void){
	/* Init state struct */
	NMEAReceiver.ReceiveDataCnt = 0u;
	NMEAReceiver.isMessageReceivedFlag = FAIL;
	NMEAReceiver.messageCnt = 0u;
	NMEAReceiver.receivedCRC = 0u;
	NMEAReceiver.receiveState = IDLE;

}
/*----------------------------------------------------------------------------*/
void GPS_UART_IRQHandler(uint8_t receivedByte){
	receiveNMEAMessage(&NMEAReceiver, receivedByte);
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/

/******************************************************************************/
