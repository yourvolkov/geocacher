/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "ST7565.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "font5x7.h"
#include "math.h"
#include "stdlib.h"
#include "Utils.h"
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


const uint8_t lcd_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE, 0x1F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFE, 0x1F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x41, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x41, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0xDE, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x76, 0x3F, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x72, 0x3F, 0x80, 0x00, 0x80, 0x00, 0x00, 0x03, 0x81, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x03, 0x81, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x07, 0xFC, 0x80, 0x38, 0x00, 0x00, 0x00, 0x0C, 0x07, 0xFC, 0x80, 0x38, 0x00, 0x00, 0x00, 0x03, 0xC7, 0xF8, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x03, 0xC7, 0xF8, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x70, 0x06, 0xE0, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x70, 0x06, 0xE0, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x41, 0x18, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x41, 0x18, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF1, 0xF1, 0xE4, 0x02, 0x00, 0x00, 0x0F, 0xFF, 0xF1, 0xF1, 0xE4, 0x02, 0x00, 0x00, 0x0F, 0xFF, 0xFD, 0xFF, 0x04, 0x0C, 0x00, 0x00, 0x0F, 0xFF, 0xFD, 0xFF, 0x04, 0x0C, 0x00, 0x00, 0x0F, 0xE0, 0x01, 0xF1, 0xF8, 0x10, 0x00, 0x00, 0x0F, 0xE0, 0x01, 0xF1, 0xF8, 0x10, 0x00, 0x00, 0x0F, 0xC0, 0x70, 0x41, 0x00, 0x70, 0x00, 0x00, 0x0F, 0xC0, 0x70, 0x41, 0x00, 0x70, 0x00, 0x00, 0x1F, 0xC0, 0xF0, 0x0E, 0x00, 0xE0, 0x00, 0x00, 0x3F, 0xC0, 0xF0, 0x0E, 0x00, 0xE0, 0x00, 0x00, 0xC7, 0xE0, 0xC0, 0x30, 0x01, 0xC2, 0x00, 0x01, 0xC7, 0xE0, 0x80, 0x30, 0x03, 0x82, 0x00, 0x00, 0x47, 0xFC, 0x80, 0x00, 0x7F, 0x04, 0x00, 0x02, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0x0C, 0x00, 0x02, 0x1F, 0xFE, 0x00, 0x00, 0xFE, 0x18, 0x00, 0x0C, 0x7F, 0xFF, 0xE0, 0x01, 0xE0, 0x10, 0x30, 0x04, 0x7F, 0xFF, 0xE0, 0x03, 0xE0, 0x10, 0x20, 0x03, 0x80, 0x3F, 0x80, 0x07, 0x03, 0xE1, 0xC0, 0x03, 0x80, 0x3F, 0x80, 0x07, 0x03, 0xE1, 0xC0, 0x00, 0x7F, 0xFF, 0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x7E, 0x3C, 0x00, 0x70, 0x00, 0x00, 0x03, 0xE0, 0x7E, 0x3C, 0x00, 0x70, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x03, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x03, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t lcd_bitmap_inv[8][64];

const uint8_t lcd_arrow[] = {
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,
  0x00, 0x10, 0x00,
  0x00, 0x1C, 0x00,
  0x7F, 0xFF, 0x00,
  0x7F, 0xFF, 0xC0,
  0x7F, 0xFF, 0xE0,

  0x7F, 0xFF, 0xC0,
  0x7F, 0xFF, 0x00,
  0x00, 0x1E, 0x00,
  0x00, 0x18, 0x00,
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00
};

uint8_t lcd_arrow_conv[2][24];
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
void convert_horizontal_bitmap(uint8_t* in, uint8_t* out, uint8_t w, uint8_t h){
	uint8_t input_width = w % 8 ? (w / 8) + 1 : (w / 8);
	uint8_t output_height = h % 8 ? (h / 8) + 1 : (h / 8);

	memset(out, 0u, w * output_height);

	for(uint8_t row = 0; row < h; row++)
	for(uint8_t col_byte_cnt = 0; col_byte_cnt < input_width; col_byte_cnt++)
	for(uint8_t k = 0; k < 8; k++){
		uint16_t in_pos = col_byte_cnt + (row * input_width);
		uint16_t out_pos = k + (col_byte_cnt * 8) + ((row / 8) * input_width * 8);

		if(UTIL_READ_BIT(*(in + in_pos), 7 - k)){
					UTIL_SET_BIT(*(out + out_pos), row % 8);
		}
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
	//LCD_sendCmd(CMD_DISPLAY_ALL_POINTS_ON);

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
/*----------------------------------------------------------------------------*/
void LCD_draw_pixel(uint8_t x, uint8_t y){
	/* Calc y position in RAM buf */
	if( (x < LCD_COLUMN_AMNT) && y < ((LCD_PAGES_AMNT * PAGE_LEN))){
		uint8_t page = y / PAGE_LEN;
		uint8_t Ypos_within_page = y % PAGE_LEN;
		uint8_t page_tmp = 0u;


		page_tmp = LCD_buf[page][x];

		/* inverse logic: lower y value is higher bit position ? */
		UTIL_SET_BIT(page_tmp, Ypos_within_page);
		LCD_buf[page][x] = page_tmp;

		/* Let's now draw each pixel to the LCD which is not very effitient but will be updated in the future */
		LCD_sendCmd(CMD_DISPLAY_SET_PAGE(page));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(x));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(x));
		LCD_sendData(&LCD_buf[page][x], 1u); // only one page containing pixel
	}
}
/*----------------------------------------------------------------------------*/
/*
 * According to Bresenham's line algorithm
 * */
void LCD_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
	uint8_t delta_x = abs(x2 - x1);
	uint8_t delta_y = abs(y2 - y1);
	int16_t error = 0u;

	uint8_t y = 0u;
	uint8_t x = 0u;
	uint8_t line_end = 0u;
	int16_t coef = 0u; // / (float)(delta_x + 1);
	int8_t direction = 0u;
	uint8_t error_div = 1u;
	uint8_t main_dir = 0;

	if(delta_x > delta_y){
		coef = (delta_y + 1); // / (float)(delta_x + 1);
		error_div += delta_x;
		direction = y2 - y1;

		if(x2 > x1){
			x = x1;
			y = y1;
			line_end = x2;
		}else{
			x = x2;
			y = y2;
			line_end = x1;
			direction *= -1;
		}
	}else{
		y = x1;
		x = y1;
		coef = (delta_x + 1); // / (float)(delta_x + 1);
		error_div += delta_y;
		direction = x2 - x1;

		if(y2 > y1){
			y = x1;
			x = y1;
			line_end = y2;
		}else{
			y = x2;
			x = y2;
			line_end = y1;
			direction *= -1;
		}

		main_dir = 1;
	}

	if(direction > 0){
		direction = 1u;
	}else{
		direction = -1;
	}

	for(; x <= line_end; x++){
		if(!main_dir){
			LCD_draw_pixel(x, y);
		}else{
			LCD_draw_pixel(y, x);
		}

		error += coef;
		if(error > error_div){
			y += direction;
			error -= error_div;
		}
	}
}

/*----------------------------------------------------------------------------*/
void LCD_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation){
	if(rotation){
		/* Calculate roatation matrix */   /*TODO refactor! */
		uint8_t centerX, centerY;

		centerX = x1 + ((x2 - x1)/2);
		centerY = y1 + ((y2 - y1)/2);

		float _sin = sin(rotation);
		float _cos = cos(rotation);
		int8_t x1_tmp, y1_tmp, x2_tmp, y2_tmp;

		/* 1 side */
		x1_tmp = (x1 - centerX) *_cos - (y1 - centerY)*_sin;
		y1_tmp = (x1 - centerX)*_sin + (y1 - centerY)*_cos;

		x2_tmp = (x2 - centerX)*_cos - (y1 - centerY)*_sin;
		y2_tmp = (x2 - centerX)*_sin + (y1 - centerY)*_cos;

		LCD_draw_line(x1_tmp + centerX, y1_tmp + centerY, x2_tmp + centerX, y2_tmp + centerY);

		/* 2 side */
		x1_tmp = (x2 - centerX) *_cos - (y1 - centerY)*_sin;
		y1_tmp = (x2 - centerX)*_sin + (y1 - centerY)*_cos;

		x2_tmp = (x2 - centerX)*_cos - (y2 - centerY)*_sin;
		y2_tmp = (x2 - centerX)*_sin + (y2 - centerY)*_cos;

		LCD_draw_line(x1_tmp + centerX, y1_tmp + centerY, x2_tmp + centerX, y2_tmp + centerY);


		/* 3 side */
		x1_tmp = (x2 - centerX) *_cos - (y2 - centerY)*_sin;
		y1_tmp = (x2 - centerX)*_sin + (y2 - centerY)*_cos;

		x2_tmp = (x1 - centerX)*_cos - (y2 - centerY)*_sin;
		y2_tmp = (x1 - centerX)*_sin + (y2 - centerY)*_cos;

		LCD_draw_line(x1_tmp + centerX, y1_tmp + centerY, x2_tmp + centerX, y2_tmp + centerY);


		/* 4 side */
		x1_tmp = (x1 - centerX) *_cos - (y2 - centerY)*_sin;
		y1_tmp = (x1 - centerX)*_sin + (y2 - centerY)*_cos;

		x2_tmp = (x1 - centerX)*_cos - (y1 - centerY)*_sin;
		y2_tmp = (x1 - centerX)*_sin + (y1 - centerY)*_cos;

		LCD_draw_line(x1_tmp + centerX, y1_tmp + centerY, x2_tmp + centerX, y2_tmp + centerY);

	}else{
		LCD_draw_line(x1, y1, x2, y1);
		LCD_draw_line(x2, y1, x2, y2);
		LCD_draw_line(x2, y2, x1, y2);
		LCD_draw_line(x1, y2, x1, y1);
	}
}

/*----------------------------------------------------------------------------*/
void LCD_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap){
	uint8_t page_amnt = height / 8;
	if(height % 8)
		page_amnt++;

	for(uint8_t i = 0; i < page_amnt; i++){

		memcpy(&LCD_buf[y + i][x], (bitmap + (i * width)), width);

		LCD_sendCmd(CMD_DISPLAY_SET_PAGE(y + i));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(x));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(x));
		LCD_sendData(&LCD_buf[y + i][x], width); // y -- not a coordinate, but a page: TODO
	}
}
/******************************************************************************/
