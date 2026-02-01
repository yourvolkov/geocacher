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


typedef struct{
	uint8_t x;
	uint8_t y;
}point;
/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
uint8_t LCD_buf[LCD_PAGES_AMNT][LCD_COLUMN_AMNT];

const uint8_t lcd_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x68, 0x00, 0x00, 0x00, 0x40, 0x00, 0x6C, 0x00, 0x00, 0x01, 0xB0, 0x00, 0x82, 0x00, 0x00, 0x01, 0x88, 0x00, 0x92, 0x00, 0x00, 0x01, 0x88, 0x00, 0x92, 0x00, 0x00, 0x02, 0x48, 0x00, 0x91, 0x80, 0x00, 0x02, 0x46, 0x03, 0x11, 0x80, 0x00, 0x02, 0x46, 0x03, 0x11, 0x80, 0x00, 0x02, 0x77, 0xFF, 0x1C, 0x40, 0x00, 0x02, 0x7F, 0xFF, 0xFD, 0xC0, 0x00, 0x02, 0x7F, 0xFF, 0xFD, 0xC0, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x40, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x20, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x30, 0x00, 0x03, 0xF8, 0xFF, 0xFF, 0xB0, 0x00, 0x03, 0xF0, 0x38, 0xFF, 0xF0, 0x00, 0x03, 0xF0, 0x38, 0xFF, 0xF0, 0x00, 0x03, 0xF0, 0x38, 0x7F, 0xF0, 0x00, 0x07, 0xE0, 0x38, 0x3F, 0xC8, 0x00, 0x0F, 0xC0, 0x38, 0x1F, 0xC8, 0x00, 0x0D, 0xCE, 0x38, 0xEF, 0x88, 0x00, 0x0C, 0xC7, 0x3B, 0x80, 0x04, 0x00, 0x0C, 0x47, 0x3B, 0x80, 0x06, 0x00, 0x10, 0x07, 0x3B, 0x80, 0x06, 0x00, 0x10, 0x00, 0x30, 0x00, 0x07, 0x00, 0x10, 0x00, 0x20, 0x00, 0x07, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0xF8, 0x00, 0x01, 0x00, 0x10, 0x00, 0xF8, 0x00, 0x01, 0x00, 0x1C, 0x01, 0xFC, 0x00, 0x07, 0x00, 0x0C, 0x08, 0xF8, 0x60, 0x09, 0x00, 0x0C, 0x08, 0x78, 0x60, 0x09, 0x00, 0x0E, 0x06, 0x20, 0xE0, 0x31, 0x00, 0x0F, 0x86, 0x23, 0x80, 0x41, 0x00, 0x07, 0x82, 0x33, 0x80, 0x41, 0x00, 0x03, 0xC1, 0xFC, 0x80, 0x06, 0x00, 0x02, 0x70, 0xDB, 0x01, 0x86, 0x00, 0x02, 0x70, 0xDB, 0x01, 0x06, 0x00, 0x0E, 0x70, 0xDB, 0x00, 0x06, 0x00, 0x0E, 0x30, 0xC4, 0x00, 0x01, 0x00, 0x0E, 0x30, 0x44, 0x00, 0x01, 0x00, 0x0E, 0x30, 0x38, 0x00, 0x01, 0x00, 0x13, 0xB8, 0x00, 0x00, 0x01, 0x00, 0x13, 0x9C, 0x00, 0x00, 0x01, 0x00, 0x13, 0x8E, 0x00, 0x00, 0x01, 0x00, 0x13, 0x87, 0x00, 0x00, 0x01, 0x00, 0x13, 0x83, 0x80, 0x00, 0x01, 0x00, 0x12, 0x41, 0xC0, 0x00, 0x01, 0x00, 0x12, 0x30, 0x20, 0x00, 0x01, 0x00, 0x12, 0x10, 0x20, 0x00, 0x01, 0x00, 0x12, 0x08, 0x18, 0x00, 0x00, 0x00, 0x11, 0x80, 0x00, 0x00, 0x00, 0x00, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0x00, 0x00, 0x00, 0x00, 0x04, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t lcd_bitmap_inv[8][48];

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
void LCD_clear_pixel(uint8_t x, uint8_t y){
	/* Calc y position in RAM buf */
	if( (x < LCD_COLUMN_AMNT) && y < ((LCD_PAGES_AMNT * PAGE_LEN))){
		uint8_t page = y / PAGE_LEN;
		uint8_t Ypos_within_page = y % PAGE_LEN;
		uint8_t page_tmp = 0u;

		page_tmp = LCD_buf[page][x];

		UTIL_CLEAR_BIT(page_tmp, Ypos_within_page);
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
/*
 * According to Bresenham's line algorithm
 * */

uint8_t LCD_get_X_of_point_on_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t pointY){
	uint8_t delta_x = abs(x2 - x1);
	uint8_t delta_y = abs(y2 - y1);
	int16_t error = 0u;

	uint8_t ret_x = 0u;
	uint8_t y = 0u;
	uint8_t x = 0u;
	uint8_t line_end = 0u;
	int16_t coef = 0u;
	int8_t direction = 0u;
	uint8_t error_div = 1u;
	uint8_t main_dir = 0;

	/* Check if the X coordinate is already out of line */
	if(y2 > y1){
		if(x2 > x1){
			if(pointY <= y1){
				return x1;
			}else if(pointY >= y2){
				return x2;
			}
		}else{
			if(pointY >= y2){
				return x2;
			}else if(pointY <= y1){
				return x1;
			}
		}
	}else{
		if(x2 > x1){
			if(pointY >= y1){
				return x1;
			}else if(pointY <= y2){
				return x2;
			}
		}else{
			if(pointY <= y2){
				return x2;
			}else if(pointY >= y1){
				return x1;
			}
		}
	}

	/* The X coordinate is within the line, let's calculate it */
	if(delta_x > delta_y){
		coef = (delta_y + 1);
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
		coef = (delta_x + 1);
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
			if(y == pointY){
				return x;
			}
		}else{
			if(x == pointY){
				return y;
			}
		}

		error += coef;
		if(error > error_div){
			y += direction;
			error -= error_div;
		}
	}

	return ret_x;
}

/*----------------------------------------------------------------------------*/
/*  vertex -> vertices */

void fill_rectangle(point rectangle_vertices[4]){
	uint8_t upper_cornerX = 0u, upper_cornerY = 0u;
	uint8_t lines_to_draw = 0u;
	uint8_t X1 = 0u, X2 = 0u;
	point upper_pointY, lower_pointY, upper_pointX, lower_pointX;

	for(uint8_t i = 0; i < 4; i++){
		if(rectangle_vertices[i].y > upper_cornerY){
			upper_cornerY = rectangle_vertices[i].y;
			upper_pointY = rectangle_vertices[i];
			lower_pointY = rectangle_vertices[(i + 2) % 4]; /* The oposite point */
		}
	}

	for(uint8_t i = 0; i < 4; i++){
		if(rectangle_vertices[i].x > upper_cornerX){
			upper_cornerX = rectangle_vertices[i].x;
			upper_pointX = rectangle_vertices[i];
			lower_pointX = rectangle_vertices[(i + 2) % 4]; /* The oposite point */
		}
	}

	lines_to_draw = upper_pointY.y - lower_pointY.y;
	for(uint8_t i = 0; i < lines_to_draw; i++){
		uint8_t currY = lower_pointY.y + i;
		X1 = LCD_get_X_of_point_on_line(lower_pointY.x, lower_pointY.y, lower_pointX.x, lower_pointX.y, currY); /* Refactor that part */
		if(X1 <= lower_pointX.x && currY >= lower_pointX.y){
			X1 = LCD_get_X_of_point_on_line(upper_pointY.x, upper_pointY.y, lower_pointX.x, lower_pointX.y, currY);
		}
		X2 = LCD_get_X_of_point_on_line(lower_pointY.x, lower_pointY.y, upper_pointX.x, upper_pointX.y, currY);
		if(X2 >= upper_pointX.x && currY >= upper_pointX.y){
			X2 = LCD_get_X_of_point_on_line(upper_pointY.x, upper_pointY.y, upper_pointX.x, upper_pointX.y, currY);
		}
		for(uint8_t j = X1; j <= X2; j++){
			LCD_draw_pixel (j, currY);
		}
	}
}

/*----------------------------------------------------------------------------*/
void rotate_point(uint8_t x, uint8_t y,
				  uint8_t origin_x, uint8_t origin_y,
				  float rotate_sin, float rotate_cos,
				  uint8_t* res_x, uint8_t* res_y){

	float x_tmp, y_tmp;

	x_tmp = (x - origin_x) * rotate_cos - (y - origin_y) * rotate_sin;
	y_tmp = (x - origin_x) * rotate_sin + (y - origin_y) * rotate_cos;

	x_tmp = floor(x_tmp);
	y_tmp = floor(y_tmp);

	*res_x = x_tmp + origin_x;
	if(*res_x > LCD_COLUMN_AMNT){
		*res_x = LCD_COLUMN_AMNT;
	}

	*res_y = y_tmp + origin_y;
	if(*res_y > (LCD_PAGES_AMNT * PAGE_LEN)){
		*res_y = (LCD_PAGES_AMNT * PAGE_LEN);
	}
}
/*----------------------------------------------------------------------------*/
void LCD_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation, uint8_t isFilled){
	if(rotation){

		/* Calculate roatation matrix */
		uint8_t centerX, centerY;
		point rectangle_vertices[4u] = {0u};

		float _sin = sin(rotation);
		float _cos = cos(rotation);

		centerX = x1 + ((x2 - x1)/2);
		centerY = y1 + ((y2 - y1)/2);

		rotate_point(x1, y1, centerX, centerY, _sin, _cos, &rectangle_vertices[0u].x, &rectangle_vertices[0u].y);
		rotate_point(x2, y1, centerX, centerY, _sin, _cos, &rectangle_vertices[1u].x, &rectangle_vertices[1u].y);
		rotate_point(x2, y2, centerX, centerY, _sin, _cos, &rectangle_vertices[2u].x, &rectangle_vertices[2u].y);
		rotate_point(x1, y2, centerX, centerY, _sin, _cos, &rectangle_vertices[3u].x, &rectangle_vertices[3u].y);

		if(!isFilled){
			LCD_draw_line(rectangle_vertices[0u].x, rectangle_vertices[0u].y, rectangle_vertices[1u].x, rectangle_vertices[1u].y);
			LCD_draw_line(rectangle_vertices[1u].x, rectangle_vertices[1u].y, rectangle_vertices[2u].x, rectangle_vertices[2u].y);
			LCD_draw_line(rectangle_vertices[2u].x, rectangle_vertices[2u].y, rectangle_vertices[3u].x, rectangle_vertices[3u].y);
			LCD_draw_line(rectangle_vertices[3u].x, rectangle_vertices[3u].y, rectangle_vertices[0u].x, rectangle_vertices[0u].y);
		}else{
			fill_rectangle(rectangle_vertices);
		}

	}else{
		if(!isFilled){
			LCD_draw_line(x1, y1, x2, y1);
			LCD_draw_line(x2, y1, x2, y2);
			LCD_draw_line(x2, y2, x1, y2);
			LCD_draw_line(x1, y2, x1, y1);
		}else{
			uint8_t lines_to_draw = 0u;
			uint8_t len = 0u;
			uint8_t x_start, y_start;
			if(y2 > y1){
				lines_to_draw = y2 - y1;
				y_start = y1;
			}else{
				lines_to_draw = y1 - y2;
				y_start = y2;
			}

			if(x2 > x1){
				x_start = x1;
				len = x2 - x1;
			}else{
				x_start = x2;
				len = x1 - x2;
			}
			for(uint8_t i = y_start; i <= y_start + lines_to_draw; i++){
				LCD_draw_line(x_start, i, x_start + len, i);
			}
		}
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


/*----------------------------------------------------------------------------*/
void LCD_draw_circle(uint8_t X0, uint8_t Y0, uint8_t R, uint8_t isFilled){
	uint8_t x = 0;
	uint8_t y = R;
	int8_t delta = 3 - 2 * y;
	while (x <= y) {
		LCD_draw_pixel(X0 + x, Y0 + y);
		LCD_draw_pixel(X0 + x, Y0 - y);
		LCD_draw_pixel(X0 - x, Y0 + y);
		LCD_draw_pixel(X0 - x, Y0 - y);
		LCD_draw_pixel(X0 + y, Y0 + x);
		LCD_draw_pixel(X0 + y, Y0 - x);
		LCD_draw_pixel(X0 - y, Y0 + x);
		LCD_draw_pixel(X0 - y, Y0 - x);
		if(isFilled){
			uint8_t lineLen = 2 * x;
			for(uint8_t i = 0u; i <= lineLen; i++){
				LCD_draw_pixel(X0 - x + i, Y0 - y);
				LCD_draw_pixel(X0 - x + i, Y0 + y);
			}

			lineLen = 2 * y;
			for(uint8_t i = 0u; i <= lineLen; i++){
				LCD_draw_pixel(X0 - y + i, Y0 - x);
				LCD_draw_pixel(X0 - y + i, Y0 + x);
			}
		}
		if(delta < 0){
			delta += 4 * x + 6;
		}else{
			delta += 4 * (x - y--) + 10;
		}
		x++;
	}
}
/******************************************************************************/
