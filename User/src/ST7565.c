/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "ST7565.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "font5x7.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define CMD_DISPLAY_OFF    (uint8_t)(0b10101110u) // 0xAEu
#define CMD_DISPLAY_ON     (uint8_t)(0b10101111u) // 0xAFu

#define CMD_DISPLAY_START_LINE(line)     (uint8_t)((0b01000000u) | (line))
#define CMD_DISPLAY_SET_PAGE(page)       (uint8_t)((0b10110000u) | (page))
#define CMD_DISPLAY_SET_COLUMN_L(column) (uint8_t)((0b00000000u) | (column & 0x0Fu))
#define CMD_DISPLAY_SET_COLUMN_H(column) (uint8_t)((0b00010000u) | ((((uint8_t)column & 0xF0u) >> 4u) & 0x0Fu))

#define CMD_DISPLAY_ALL_POINTS_ON        (uint8_t)(0b10100101u)
#define CMD_DISPLAY_ALL_POINTS_OFF       (uint8_t)(0b10100100u)

#define CMD_DISPLAY_BIAS_SET(bias)       (uint8_t)((0b10100000u) | (bias & 0x1u)) //0: 1/9 bias, 1: 1/7 bias
#define CMD_DISPLAY_VOLTAGE_REG_SET(voltage)       (uint8_t)((0b00100000u) | (voltage & 0x7u))

#define CMD_DISPLAY_ADC_NORMAL        (uint8_t)(0b10100000u)
#define CMD_DISPLAY_ADC_REVERSE       (uint8_t)(0b10100001u)

#define CMD_DISPLAY_NORMAL        (uint8_t)(0b10100110u)
#define CMD_DISPLAY_REVERSE       (uint8_t)(0b10100111u)

#define CMD_READ_MOD_WRITE       (uint8_t)(0b11100000u)
#define CMD_READ_MOD_WRITE_END   (uint8_t)(0b11101110u)
#define CMD_DISPLAY_RESET        (uint8_t)(0b11100010u)

#define CMD_DISPLAY_COMMON_OTPUT_MODE_NORMAL       (uint8_t)(0b11000000u)
#define CMD_DISPLAY_COMMON_OTPUT_MODE_REVERSE       (uint8_t)(0b11001000u)

#define CMD_DISPLAY_POWER_CONTROL_SET(mode)       (uint8_t)((0b00101000u) | (mode & 0x7u))

#define CMD_DISPLAY_SET_VOLUME_1         (uint8_t)(0b10000001u)
#define CMD_DISPLAY_SET_VOLUME_2(vol)    (uint8_t)((0b00000000u) | (vol & 0x3Fu))

#define CMD_NOP            (uint8_t)(0b11100011u)



#define HAL_TIMEOUT    10u // 10ms
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	COMMAND = 0u,
	DATA
}dtCmdPinMode;

/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
uint8_t LCD_buf[LCD_PAGES_AMNT][LCD_COLUMN_AMNT];
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/
extern SPI_HandleTypeDef hspi1;
/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/

/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/
void LCD_setCmdPinMode(dtCmdPinMode mode){
	if(mode == COMMAND){
		HAL_GPIO_WritePin(CMD_PORT, CMD_PIN, GPIO_PIN_RESET);
	}else if(mode == DATA){
		HAL_GPIO_WritePin(CMD_PORT, CMD_PIN, GPIO_PIN_SET);
	}else{
		/* Defensive programming */
	}

}
/*----------------------------------------------------------------------------*/
dtReturnValue LCD_sendData(uint8_t* data, uint16_t data_len){
	HAL_StatusTypeDef retVal = 0u;

	LCD_setCmdPinMode(DATA);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	retVal = HAL_SPI_Transmit(&hspi1, data, data_len, HAL_TIMEOUT);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

}
/*----------------------------------------------------------------------------*/
dtReturnValue LCD_sendCmd(uint8_t cmd){
	uint8_t buf = cmd;
	HAL_StatusTypeDef retVal = 0u;

	LCD_setCmdPinMode(COMMAND);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	retVal = HAL_SPI_Transmit(&hspi1, &buf, 1u, HAL_TIMEOUT);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

	return OK;
}

/*----------------------------------------------------------------------------*/
void LCD_clearScreen(void){
	memset(LCD_buf, 0x00u, sizeof(LCD_buf));
	for(uint8_t i = 0; i < LCD_PAGES_AMNT; i++){
		LCD_sendCmd(CMD_DISPLAY_SET_PAGE(i));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(0u));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(0u));
		LCD_sendData(&LCD_buf[i][0u], LCD_COLUMN_AMNT);
		LCD_sendCmd(CMD_DISPLAY_OFF);
		LCD_sendCmd(CMD_DISPLAY_ON);
	}
}

/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
void LCD_init(void){
	/* Init RAM buffer */
	memset(LCD_buf, 0u, sizeof(LCD_buf));

	/* Pull reset pin LOW */
	HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_RESET);
	HAL_Delay(100u); /* 100ms */
	HAL_GPIO_WritePin(RES_PORT, RES_PIN, GPIO_PIN_SET);

	/* Initialization procedure */
	LCD_sendCmd(CMD_DISPLAY_BIAS_SET(0u));
	LCD_sendCmd(CMD_DISPLAY_ADC_NORMAL);
	LCD_sendCmd(CMD_DISPLAY_NORMAL); /* invertion of the pixels */
	LCD_sendCmd(CMD_DISPLAY_COMMON_OTPUT_MODE_REVERSE); /* Normal: addressing starts from bottom left corner; Reverse: top left corner */

	LCD_sendCmd(CMD_DISPLAY_VOLTAGE_REG_SET(0x3u));
	LCD_sendCmd(CMD_DISPLAY_SET_VOLUME_1);
	LCD_sendCmd(CMD_DISPLAY_SET_VOLUME_2(0x3Fu));

	LCD_sendCmd(CMD_DISPLAY_POWER_CONTROL_SET(0x7u));

	LCD_sendCmd(CMD_DISPLAY_ON);
	LCD_clearScreen();
	/* Test */
//	LCD_sendCmd(CMD_DISPLAY_ALL_POINTS_ON);

}

/*----------------------------------------------------------------------------*/
void LCD_test(void){
	static uint8_t currColumn = 0u;
	static uint8_t currLine = 0u;

	/* Calc buffer adding one pixel at the time */
	LCD_buf[currLine / 8][currColumn] = 0xFF >> (8 - ((currLine % 8 ) + 1));

	currColumn++;
	if(currColumn > LCD_COLUMN_AMNT - 1){
		currColumn = 0u;
		currLine++;
		if(currLine > 63){
			currLine = 0u;
			LCD_clearScreen();
		}
	}



	LCD_sendCmd(CMD_DISPLAY_SET_PAGE(currLine / 8));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(0u));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(0u));
	LCD_sendData(&LCD_buf[currLine / 8][0u], LCD_COLUMN_AMNT);

	HAL_Delay(1u);
}

/*----------------------------------------------------------------------------*/
void LCD_print_test(uint8_t cursorX, uint8_t cursorY, char* line, size_t len){
	static char character = 0u;
	uint8_t string_char = character;
	for(uint8_t i = 0; i < 21; i++)
	{
		uint8_t pos = i * (FONT_CHAR_WIDTH + 1);
		memcpy(&LCD_buf[cursorY][pos], &font_data[FONT_CHAR_WIDTH * string_char++], FONT_CHAR_WIDTH);
		LCD_buf[cursorY][pos + FONT_CHAR_WIDTH] = 0u; /* Space between chars */
	}
	character++;
	//memcpy(&LCD_buf[cursorY][0u], &font_data[FONT_CHAR_WIDTH * character++], LCD_COLUMN_AMNT);

	LCD_sendCmd(CMD_DISPLAY_SET_PAGE(cursorY));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(0u));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(0u));
	LCD_sendData(&LCD_buf[cursorY][0u], LCD_COLUMN_AMNT);
}

/*----------------------------------------------------------------------------*/
void LCD_print(uint8_t cursorX, uint8_t cursorY, char* line, size_t len){
	for(uint8_t i = 0; i < len; i++)
	{
		uint8_t pos = cursorX + (i * (FONT_CHAR_WIDTH + 1));
		memcpy(&LCD_buf[cursorY][pos], &font_data[FONT_CHAR_WIDTH * (uint8_t)line[i]], FONT_CHAR_WIDTH);
		LCD_buf[cursorY][pos + FONT_CHAR_WIDTH] = 0u; /* Space between chars */
	}


	LCD_sendCmd(CMD_DISPLAY_SET_PAGE(cursorY));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(0u));
	LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(0u));
	LCD_sendData(&LCD_buf[cursorY][0u], LCD_COLUMN_AMNT);
}
/******************************************************************************/
