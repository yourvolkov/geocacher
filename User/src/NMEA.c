/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "NMEA.h"
#include "string.h"
#include "Utils.h"
#include "stm32f1xx_hal.h"

/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/

#define NMEA_MES_MAX_LEN   82u
#define HAL_TIMEOUT    10u // 10ms
#define MSG_TYPE_POS	2u
#define MSG_TYPE_LEN	3u
#define DEGREES_MAX_STRING    4 // DDD + \0 symb
#define MINUTES_MAX_STRING    9 // ММ.МММММ + \0 symb
#define LATITUDE_DEGREES_SYMB_AMNT	2
#define LONGITUDE_DEGREES_SYMB_AMNT	3
#define MINUTES_SYMB_AMNT	8 // ММ.МММММ

#define DATA_FIELD_MAX_LEN	15u

#define GGA_MES_TYPE	"GGA"
#define GLL_MES_TYPE	"GLL"
#define GSA_MES_TYPE	"GSA"
#define GSV_MES_TYPE	"GSV"
#define RMC_MES_TYPE	"RMC"
#define VTG_MES_TYPE	"VTG"
#define DHV_MES_TYPE	"DHV"
#define GST_MES_TYPE	"GST"

#define GLL_LATITUDE_FIELD_POS	1u
#define GLL_LATITUDE_CARDINAL_FIELD_POS	2u
#define GLL_LONGITUDE_FIELD_POS	3u
#define GLL_LONGITUDE_CARDINAL_FIELD_POS	4u
#define GLL_UTC_TIME_FIELD_POS	5u
#define GLL_IS_DATA_RELIABLE_FIELD_POS	6u

#define GSV_SATTELITE_AMNT_FIELD_POS	3u
#define VTG_SPEED_KMH_FIELD_POS	7u
#define GGA_ALTITUDE_FIELD_POS	9u

#define GLL_DATA_RELIABLE_SYMB	'A'
#define GLL_DATA_NOT_RELIABLE_SYMB	'V'
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
typedef enum{
	LATITUDE,
	LONGITUDE,
	SPEED,
	ALTITUDE
}dtNMEADataType;
/*----------------------------------------------------------------------------*/
typedef struct{
	dtNMEACoordinate latitude;
	dtNMEACoordinate longitude;
	float altitude;
	float speed;
	uint8_t satteliteAmount;
	uint8_t isDataReliable;
	uint8_t msgCounters[9u]; // for debug purposes
	uint8_t latitudeRaw[DATA_FIELD_MAX_LEN]; // for debug purposes
	uint8_t longitudeRaw[DATA_FIELD_MAX_LEN]; // for debug purposes
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

/*----------------------------------------------------------------------------*/
uint8_t NMEA_CrcCalc(dtNMEAReceiver* receiver){
	uint8_t crc = 0u;
	for(uint8_t i = 0u; i < receiver->ReceiveDataCnt; i++){
		crc ^= receiver->NMEA_buf[i];
	}
	return crc;
}

/*----------------------------------------------------------------------------*/
/*
 * Example string: GNGLL,5546.95900,N,03740.69200,E,102030.000,A
 * */
uint8_t NMEA_getField(uint8_t* message, uint8_t message_len, uint8_t field_pos, uint8_t* field, uint8_t* field_len, uint8_t field_len_max){
	uint8_t fields_cnt = 0u;
	uint8_t field_start_pos = 0u;
	uint8_t field_end_pos = (message_len - 1u);
	uint8_t mes_len = 0u;

	for(uint8_t i = 0; i < message_len; i++){
		if(*(message + i) == ','){
			fields_cnt++;
			if(fields_cnt == field_pos){
				/* Field start found */
				field_start_pos = i + 1u;
				mes_len = 0u;
			}else if(fields_cnt == (field_pos + 1u)){
				/* Field end found */
				field_end_pos = i - 1u;
				break;
			}else{
				/* Defensive programming */
			}
		}else{
			mes_len++;
		}
	}

	if((field_start_pos <= field_end_pos) && (fields_cnt > 0) && (mes_len < field_len_max) && (mes_len > 0u)){
		/* Not an empty field */
		*field_len = field_end_pos - field_start_pos + 1;
		memcpy(field, (message + field_start_pos), *field_len);
		return PASS;
	}else{
		return FAIL;
	}
}
/*----------------------------------------------------------------------------*/

void parse_float_data(uint8_t* field, dtNMEADataType type){
	uint8_t degreesTmp[DEGREES_MAX_STRING] = {0u};
	uint8_t minutesTmp[MINUTES_MAX_STRING] = {0u};

	if(type == LATITUDE){
		/* two first digits are degeees */
		memcpy(degreesTmp, field, LATITUDE_DEGREES_SYMB_AMNT);
		memcpy(minutesTmp, (field + LATITUDE_DEGREES_SYMB_AMNT), MINUTES_SYMB_AMNT);
		NMEAFineData.latitude.degree = _atoi(degreesTmp);
		NMEAFineData.latitude.minute = _atof(minutesTmp);
		NMEAFineData.latitude.qualifier = NMEAFineData.isDataReliable ? DATA_AVAILABLE : DATA_NOT_RELIABLE;
	}else if(type == LONGITUDE){
		/* three first digits are degeees */
		memcpy(degreesTmp, field, LONGITUDE_DEGREES_SYMB_AMNT);
		memcpy(minutesTmp, (field + LONGITUDE_DEGREES_SYMB_AMNT), MINUTES_SYMB_AMNT);
		NMEAFineData.longitude.degree = _atoi(degreesTmp);
		NMEAFineData.longitude.minute = _atof(minutesTmp);
		NMEAFineData.longitude.qualifier = NMEAFineData.isDataReliable ? DATA_AVAILABLE : DATA_NOT_RELIABLE;
	}else{
		/* Defensive programming */
	}

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
uint8_t parseNMEAMessage(dtNMEAData* Data, uint8_t* rawMessage, uint8_t rawMessageLen){
	uint8_t* msgType = &rawMessage[MSG_TYPE_POS];
	uint8_t field[DATA_FIELD_MAX_LEN] = {0u};
	uint8_t field_len = 0u;
	uint8_t ret = FAIL;

	if(memcmp(msgType, GGA_MES_TYPE, MSG_TYPE_LEN) == 0u){
		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							GGA_ALTITUDE_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);

		if(ret == PASS){
			field[field_len] = '\0';
			NMEAFineData.altitude = _atof(field);
		}

		NMEAFineData.msgCounters[0u]++;
	}else if(memcmp(msgType, GLL_MES_TYPE, MSG_TYPE_LEN) == 0u){
		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							GLL_LATITUDE_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);
		if(ret == PASS){
			memcpy(NMEAFineData.latitudeRaw, field, field_len);
			parse_float_data(field, LATITUDE);
		}

		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							GLL_LONGITUDE_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);
		if(ret == PASS){
			memcpy(NMEAFineData.longitudeRaw, field, field_len);
			parse_float_data(field, LONGITUDE);
		}

		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							GLL_IS_DATA_RELIABLE_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);
		if(ret == PASS){
			if(field[0u] == GLL_DATA_RELIABLE_SYMB){
				NMEAFineData.isDataReliable = PASS;
			}else{
				NMEAFineData.isDataReliable = FAIL;
			}
		}

		NMEAFineData.msgCounters[1u]++;
	}else if(memcmp(msgType, GSA_MES_TYPE, MSG_TYPE_LEN) == 0u){
		NMEAFineData.msgCounters[2u]++;
	}else if(memcmp(msgType, GSV_MES_TYPE, MSG_TYPE_LEN) == 0u){
		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							GSV_SATTELITE_AMNT_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);

		if(ret == PASS){
			field[field_len] = '\0';
			NMEAFineData.satteliteAmount = _atoi(field);
		}

		NMEAFineData.msgCounters[3u]++;
	}else if(memcmp(msgType, RMC_MES_TYPE, MSG_TYPE_LEN) == 0u){
		NMEAFineData.msgCounters[4u]++;
	}else if(memcmp(msgType, VTG_MES_TYPE, MSG_TYPE_LEN) == 0u){
		ret = NMEA_getField(rawMessage,
							rawMessageLen,
							VTG_SPEED_KMH_FIELD_POS,
							field,
							&field_len,
							DATA_FIELD_MAX_LEN);

		if(ret == PASS){
			field[field_len] = '\0';
			NMEAFineData.speed = _atof(field);
		}

		NMEAFineData.msgCounters[5u]++;
	}else if(memcmp(msgType, DHV_MES_TYPE, MSG_TYPE_LEN) == 0u){
		/* Message was never received */
		NMEAFineData.msgCounters[6u]++;
	}else if(memcmp(msgType, GST_MES_TYPE, MSG_TYPE_LEN) == 0u){
		/* Message was never received */
		NMEAFineData.msgCounters[7u]++;
	}else{
		NMEAFineData.msgCounters[8u]++;
		/* unknown or unsupported message type */
	}

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

	NMEAFineData.altitude = 0.0f;
	NMEAFineData.isDataReliable = FAIL;

	NMEAFineData.latitude.qualifier = DATA_NOT_AVAILABLE;
	NMEAFineData.latitude.cardinalPoint = 'N';
	NMEAFineData.latitude.degree = 0u;
	NMEAFineData.latitude.minute = 0.0f;

	NMEAFineData.longitude.qualifier = DATA_NOT_AVAILABLE;
	NMEAFineData.longitude.cardinalPoint = 'E';
	NMEAFineData.longitude.degree = 0u;
	NMEAFineData.longitude.minute = 0.0f;
	NMEAFineData.satteliteAmount = 0u;
	NMEAFineData.speed = 0.0f;

}
/*----------------------------------------------------------------------------*/
void GPS_UART_IRQHandler(uint8_t receivedByte){
	receiveNMEAMessage(&NMEAReceiver, receivedByte);
}
/*----------------------------------------------------------------------------*/
void GPS_handle(void){
	if(NMEAReceiver.isMessageReceivedFlag == PASS){
		parseNMEAMessage(&NMEAFineData, NMEAReceiver.NMEA_buf, NMEAReceiver.ReceiveDataCnt);
		NMEAReceiver.isMessageReceivedFlag = FAIL;
	}
}

/*----------------------------------------------------------------------------*/
void GPS_get_current_latitude(dtNMEACoordinate* currLatitude){
	*currLatitude = NMEAFineData.latitude;

}

/*----------------------------------------------------------------------------*/
void GPS_get_current_longitude(dtNMEACoordinate* currLongitude){
	*currLongitude = NMEAFineData.longitude;

}

/*----------------------------------------------------------------------------*/
void GPS_get_current_altitude(float* currAltitude){
	*currAltitude = NMEAFineData.altitude;
}

/*----------------------------------------------------------------------------*/
void GPS_get_current_speed(float* currSpeed){
	*currSpeed = NMEAFineData.speed;
}

/*----------------------------------------------------------------------------*/
void GPS_get_current_satellite_amount(uint8_t* currSatAmnt){
	*currSatAmnt = NMEAFineData.satteliteAmount;
}
/******************************************************************************/
