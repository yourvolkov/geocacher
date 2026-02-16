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
#define _135_GRAD_TO_RAD (2.35619f)
#define MAX_ENTITIES_ON_SINGLE_FRAME	10u
#define MAX_PROPERTY_STRUCT_SIZE  12u
#define MAX_ENTITIES	25u

//#define LCD_DEBUG
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	COMMAND = 0u,
	DATA
}dtCmdPinMode;

/*----------------------------------------------------------------------------*/
typedef enum{
	TEXT = 0u,
	LINE,
	RECTANGLE,
	CIRCLE,
	TRIANGLE,
	ARROW,
	POINT,
	BITMAP,
	NAVIGATION_ARROW
}dtEntityType;

/*----------------------------------------------------------------------------*/
typedef enum{
	FULL_RENDER_PENDING = 0,
	NOT_RENDERED,
	RENDER_PENDING,
	RENDERED
}dtFrameStatus;

/*----------------------------------------------------------------------------*/
typedef struct{
	uint8_t x;
	uint8_t y;
}dtPoint;
/*----------------------------------------------------------------------------*/
typedef struct{
	uint8_t x1;
	uint8_t y1;
	uint8_t x2;
	uint8_t y2;
}dtReturnRenderArea;
/*----------------------------------------------------------------------------*/
typedef struct{
	uint16_t type;
	uint16_t id;
}dtEntityBaseData;

typedef struct{
	dtPoint renderAreaLowerPoint;
	dtPoint renderAreaUpperPoint;
	uint8_t status;
}dtEntityStatus;

/*----------------------------------------------------------------------------*/
typedef struct{ // size 8
	char* text;
	uint8_t textLen;
	dtPoint textStartPoint;
	uint8_t isInversed;
}dtPropertyText;

typedef struct{ // size 8
	dtPoint lineBegin;
	dtPoint lineEnd;
	uint8_t isInversed;
}dtPropertyLine;

typedef struct{ // size 12
	dtPoint vertex1;
	dtPoint vertex2;
	uint8_t isFilled;
	uint8_t isInversed;
	float rotation;
}dtPropertyRectangle;

typedef struct{ // size 8
	dtPoint vertex;
	uint8_t width;
	uint8_t height;
	float rotation;
}dtPropertyNavigationArrow;

typedef struct{ // size 8
	uint8_t* bitmap;
	uint8_t width;
	uint8_t height;
	dtPoint bitmapStartPoint;
}dtPropertyBitmap;

typedef union{
	uint8_t bin[MAX_PROPERTY_STRUCT_SIZE]; // a memory placeholder to have the same size for each instance of different entities
	dtPropertyText Text;
	dtPropertyLine Line;
	dtPropertyRectangle Rectangle;
	dtPropertyBitmap Bitmap;
	dtPropertyNavigationArrow NavigationArrow;
}dtEntityProperties;

/*----------------------------------------------------------------------------*/
typedef struct{
	dtEntityBaseData main;
	dtEntityStatus status;
	dtEntityProperties properties;
}dtEntity;

/*----------------------------------------------------------------------------*/


typedef struct{
//	uint8_t LCD_buf[LCD_PAGES_AMNT][LCD_COLUMN_AMNT];
	dtEntity entitiesHeap[MAX_ENTITIES];
	uint16_t entitiesIDCnt;
	dtFrame* currentFrame;
	uint8_t updatedArea_PageStart;
	uint8_t updatedArea_PageEnd;
	uint8_t updatedArea_LineStart;
	uint8_t updatedArea_LineEnd;
	dtFrameStatus status;
}dtDisplay;
/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
uint8_t LCD_buf[LCD_PAGES_AMNT][LCD_COLUMN_AMNT];

dtDisplay Display;
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/
extern SPI_HandleTypeDef hspi1;
/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/
void LCD_draw_pixel(uint8_t x, uint8_t y);
void LCD_clear_pixel(uint8_t x, uint8_t y);
void LCD_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t isInversed);

dtReturnRenderArea LCD_print(uint8_t cursorX, uint8_t cursorY, char* line, size_t len, uint8_t isInversed);
dtReturnRenderArea LCD_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap);
dtReturnRenderArea LCD_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation, uint8_t isFilled);
/******************************************************************************/
/********************** Platform dependent functions **************************/
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

	return OK;
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
/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/

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

/*----------------------------------------------------------------------------*/
int _sort_points_by_x(const void* A, const void* B){
	dtPoint* tmpA = (dtPoint*)A;
	dtPoint* tmpB = (dtPoint*)B;
	if(tmpA->x > tmpB->x){
		return 1u;
	}else if(tmpA->x < tmpB->x){
		return -1;
	}else{
		return 0;
	}
}

/*----------------------------------------------------------------------------*/
int _sort_points_by_y(const void* A, const void* B){
	dtPoint* tmpA = (dtPoint*)A;
	dtPoint* tmpB = (dtPoint*)B;
	if(tmpA->y > tmpB->y){
		return 1u;
	}else if(tmpA->y < tmpB->y){
		return -1;
	}else{
		return 0;
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

	x_tmp = round(x_tmp);
	y_tmp = round(y_tmp);

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
void LCD_draw_pixel(uint8_t x, uint8_t y){
	/* Calc y position in RAM buf */
	if( (x < LCD_COLUMN_AMNT) && y < ((LCD_PAGES_AMNT * PAGE_LEN))){
		uint8_t page = y / PAGE_LEN;
		uint8_t Ypos_within_page = y % PAGE_LEN;
		uint8_t page_tmp = 0u;


		page_tmp = LCD_buf[page][x];

		UTIL_SET_BIT(page_tmp, Ypos_within_page);
		LCD_buf[page][x] = page_tmp;

#if 0
		/* Let's now draw each pixel to the LCD which is not very effitient but will be updated in the future */
		LCD_sendCmd(CMD_DISPLAY_SET_PAGE(page));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(x));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(x));
		LCD_sendData(&LCD_buf[page][x], 1u); // only one page containing pixel
#endif
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

#if 0
		/* Let's now draw each pixel to the LCD which is not very effitient but will be updated in the future */
		LCD_sendCmd(CMD_DISPLAY_SET_PAGE(page));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(x));
		LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(x));
		LCD_sendData(&LCD_buf[page][x], 1u); // only one page containing pixel
#endif
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
dtReturnRenderArea LCD_print(uint8_t cursorX, uint8_t cursorY, char* line, size_t len, uint8_t isInversed){
	dtReturnRenderArea ret_render_area = {0u, 0u, 0u, 0u};
	if(line != NULL && (cursorY + 8 < LCD_LINE_AMNT ) && cursorX < LCD_COLUMN_AMNT){
		uint8_t EOL = cursorX + len * (FONT_CHAR_WIDTH + 1);
		uint8_t curr_page = cursorY / PAGE_LEN;
		uint8_t page_tilt = cursorY % PAGE_LEN;
		uint8_t page_end = page_tilt ? curr_page + 1: curr_page;

		uint8_t next_page_mask = 0u;
		/* calculate next page mask explicitly as >> operation is implementation defined */
		for(uint8_t k = 0u; k <= 8; k++){
			if(k < page_tilt){
				next_page_mask |= (1 << k);
			}
		}

		for(uint8_t i = 0; i < len; i++){
			uint8_t pos = cursorX + (i * (FONT_CHAR_WIDTH + 1));
			if(pos + (FONT_CHAR_WIDTH + 1) < LCD_COLUMN_AMNT){
				if(!isInversed){
					if(!page_tilt){
						memcpy(&LCD_buf[curr_page][pos], &font_data[FONT_CHAR_WIDTH * (uint8_t)line[i]], FONT_CHAR_WIDTH);
						LCD_buf[curr_page][pos + FONT_CHAR_WIDTH] = 0u; /* Space between chars */
					}else{
						uint8_t char_tmp[FONT_CHAR_WIDTH + 1];
						uint8_t page_tmp[FONT_CHAR_WIDTH + 1];

						memcpy(char_tmp, &font_data[FONT_CHAR_WIDTH * (uint8_t)line[i]], FONT_CHAR_WIDTH);
						char_tmp[FONT_CHAR_WIDTH] = 0u;  /* Space between chars */
						memcpy(page_tmp, &LCD_buf[curr_page][pos], FONT_CHAR_WIDTH + 1);

						for(uint8_t j = 0; j < FONT_CHAR_WIDTH + 1; j++){
							/* Do we need to clear area before writing? */
							page_tmp[j] |= (uint8_t)(char_tmp[j] << page_tilt);
						}
						memcpy(&LCD_buf[curr_page][pos], page_tmp, FONT_CHAR_WIDTH + 1);

						/* Next page handling */
						if(page_end < LCD_PAGES_AMNT){
							memcpy(page_tmp, &LCD_buf[curr_page + 1][pos], FONT_CHAR_WIDTH + 1);

							for(uint8_t j = 0; j < FONT_CHAR_WIDTH + 1; j++){
								/* Do we need to clear area before writing? */
								page_tmp[j] |= (uint8_t)((char_tmp[j] >> (8 - page_tilt)) & next_page_mask);
							}
							memcpy(&LCD_buf[curr_page + 1][pos], page_tmp, FONT_CHAR_WIDTH + 1);
						}
					}
				}else{
					uint8_t char_tmp[FONT_CHAR_WIDTH + 1];
					uint8_t page_tmp[FONT_CHAR_WIDTH + 1];
					memcpy(char_tmp, &font_data[FONT_CHAR_WIDTH * (uint8_t)line[i]], FONT_CHAR_WIDTH);
					char_tmp[FONT_CHAR_WIDTH] = 0u;  /* Space between chars */
					if(!page_tilt){
						for(uint8_t j = 0; j < FONT_CHAR_WIDTH + 1; j++){
							uint8_t tmp = ~char_tmp[j];
							char_tmp[j] = tmp;
						}
						memcpy(&LCD_buf[curr_page][pos], char_tmp, FONT_CHAR_WIDTH + 1);
					}else{
						memcpy(page_tmp, &LCD_buf[curr_page][pos], FONT_CHAR_WIDTH + 1);

						for(uint8_t j = 0; j < FONT_CHAR_WIDTH + 1; j++){
							/* Do we need to clear area before writing? -- yes we do */
							page_tmp[j] &= ~(uint8_t)(0xFFu << page_tilt);
							page_tmp[j] |= (uint8_t)((~char_tmp[j]) << page_tilt);
						}
						memcpy(&LCD_buf[curr_page][pos], page_tmp, FONT_CHAR_WIDTH + 1);

						/* Next page handling */
						if(page_end < LCD_PAGES_AMNT){
							memcpy(page_tmp, &LCD_buf[curr_page + 1][pos], FONT_CHAR_WIDTH + 1);

							for(uint8_t j = 0; j < FONT_CHAR_WIDTH + 1; j++){
								/* Do we need to clear area before writing? */
								page_tmp[j] &= ~(uint8_t)(next_page_mask);
								page_tmp[j] |= (uint8_t)(((~char_tmp[j]) >> (8 - page_tilt)) & next_page_mask);
							}
							memcpy(&LCD_buf[curr_page + 1][pos], page_tmp, FONT_CHAR_WIDTH + 1);
						}
					}
				}
			}
		}

		/* Calculate render area */
		ret_render_area.x1 = cursorX;
		ret_render_area.y1 = cursorY;
		ret_render_area.x2 = EOL - 1;
		ret_render_area.y2 = cursorY + 7u; // Height of the font is 8 pixels

		LCD_setUpdateArea(curr_page, page_end < LCD_PAGES_AMNT ? page_end : curr_page, cursorX, EOL);
	}
	return ret_render_area;
}
/*----------------------------------------------------------------------------*/

dtReturnRenderArea LCD_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap){
	dtReturnRenderArea ret_render_area = {0u, 0u, 0u, 0u};
	uint8_t pageBegin = y / PAGE_LEN;
	uint8_t pageEnd = ((y + height) / PAGE_LEN) - 1; // ???
	uint8_t Ypos_within_begin_page = y % PAGE_LEN;
	uint8_t Ypos_within_end_page = (y + height) % PAGE_LEN;
	uint8_t page_tmp = 0u;
	uint8_t mask_end = 0u;
	uint8_t page_tilt = Ypos_within_begin_page;


	if(Ypos_within_end_page || Ypos_within_begin_page){
		/* calculate next page mask explicitly as >> operation is implementation defined */
		for(uint8_t k = 0u; k < 8; k++){
			if(k <= Ypos_within_end_page){
				mask_end |= (1 << k);
			}
		}
	}

	for(uint8_t currPage = pageBegin, bitmapPage = 0u; currPage <= pageEnd; currPage++, bitmapPage++){
		if(((currPage == pageBegin) && (Ypos_within_begin_page != 0)) ||
		   ((currPage == pageEnd) && (mask_end != 0))){
			for(uint8_t i = x; i <= x + width; i++){
				page_tmp = LCD_buf[currPage][i];
				uint8_t bitmap_byte = *(bitmap + (bitmapPage * width) + i);
				if(currPage == pageBegin){
					page_tmp |= (uint8_t)(bitmap_byte << page_tilt);
					LCD_buf[currPage][i] = page_tmp;
				}else if(currPage == pageEnd){
					page_tmp|= (uint8_t)((bitmap_byte >> (8 - page_tilt)) & mask_end);
					LCD_buf[currPage][i] = page_tmp;
				}else{
					/* Defensive programming */
				}
			}
		}else{
			memcpy(&LCD_buf[currPage][x], (bitmap + (bitmapPage * width)), width);
		}
	}

	ret_render_area.x1 = x;
	ret_render_area.x2 = x + width;
	ret_render_area.y1 = y;
	ret_render_area.y2 = y + height;
	LCD_setUpdateArea(pageBegin, pageEnd, x, x + width);

	return ret_render_area;
}
/*----------------------------------------------------------------------------*/

/*
 * According to Bresenham's line algorithm
 * */
void LCD_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t isInversed){
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
			if(!isInversed){
				LCD_draw_pixel(x, y);
			}else{
				LCD_clear_pixel(x, y);
			}
		}else{
			if(!isInversed){
				LCD_draw_pixel(y, x);
			}else{
				LCD_clear_pixel(y, x);
			}
		}

		error += coef;
		if(error > error_div){
			y += direction;
			error -= error_div;
		}
	}
}
/*----------------------------------------------------------------------------*/

void fill_rectangle(dtPoint rectangle_vertices[4]){
	uint8_t lines_to_draw = 0u;
	uint8_t X1 = 0u, X2 = 0u;
	dtPoint upper_pointY, lower_pointY, upper_pointX, lower_pointX;
	uint8_t is_lower_vertex_dublicated = FAIL;

	dtPoint vertices_sorted_by_x[4u];
	dtPoint vertices_sorted_by_y[4u];

	memcpy(vertices_sorted_by_x, rectangle_vertices, sizeof(dtPoint) * 4u);
	memcpy(vertices_sorted_by_y, rectangle_vertices, sizeof(dtPoint) * 4u);

	qsort((void*)vertices_sorted_by_x, 4u, sizeof(dtPoint), _sort_points_by_x);
	qsort((void*)vertices_sorted_by_y, 4u, sizeof(dtPoint), _sort_points_by_y);

	if(vertices_sorted_by_x[0].x == vertices_sorted_by_x[1].x || vertices_sorted_by_x[2].x == vertices_sorted_by_x[3].x ||
	   vertices_sorted_by_y[0].y == vertices_sorted_by_y[1].y || vertices_sorted_by_y[2].y == vertices_sorted_by_y[3].y){
		is_lower_vertex_dublicated = PASS;
	}

	lower_pointY = vertices_sorted_by_y[0];
	upper_pointY = vertices_sorted_by_y[3]; /* The oposite point */
	lower_pointX = vertices_sorted_by_x[0];
	upper_pointX = vertices_sorted_by_x[3]; /* The oposite point */

	lines_to_draw = upper_pointY.y - lower_pointY.y;
	for(uint8_t i = 0; i < lines_to_draw; i++){
		uint8_t currY = lower_pointY.y + i;
		if(is_lower_vertex_dublicated == FAIL){
			X1 = LCD_get_X_of_point_on_line(lower_pointY.x, lower_pointY.y, lower_pointX.x, lower_pointX.y, currY); /* Refactor that part */
			if(X1 <= lower_pointX.x && currY >= lower_pointX.y){
				X1 = LCD_get_X_of_point_on_line(upper_pointY.x, upper_pointY.y, lower_pointX.x, lower_pointX.y, currY);
			}
			X2 = LCD_get_X_of_point_on_line(lower_pointY.x, lower_pointY.y, upper_pointX.x, upper_pointX.y, currY);
			if(X2 >= upper_pointX.x && currY >= upper_pointX.y){
				X2 = LCD_get_X_of_point_on_line(upper_pointY.x, upper_pointY.y, upper_pointX.x, upper_pointX.y, currY);
			}
		}else{
			X1 = lower_pointX.x;
			X2 = upper_pointX.x;
		}
		LCD_draw_line(X1, currY, X2, currY, FAIL);
	}
}
/*----------------------------------------------------------------------------*/

void fill_triangle(dtPoint triangle_vertices[3], uint8_t isInversed){
	uint8_t lines_to_draw = 0u;
	uint8_t X1 = 0u, X2 = 0u;
	dtPoint vertices_sorted_by_y[3u];
	uint8_t is_lower_vertex_dublicated = FAIL;

	memcpy(vertices_sorted_by_y, triangle_vertices, sizeof(dtPoint) * 3u);
	qsort((void*)vertices_sorted_by_y, 3u, sizeof(dtPoint), _sort_points_by_y);

	if(vertices_sorted_by_y[0u].y == vertices_sorted_by_y[1u].y){
		is_lower_vertex_dublicated = PASS;
	}

	lines_to_draw = vertices_sorted_by_y[2].y - vertices_sorted_by_y[0].y;
	if(is_lower_vertex_dublicated == FAIL){
		for(uint8_t i = 0; i < lines_to_draw; i++){
			uint8_t currY = vertices_sorted_by_y[0].y + i;

			X1 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[0].x, vertices_sorted_by_y[0].y, vertices_sorted_by_y[1].x, vertices_sorted_by_y[1].y, currY);
			if(X1 <= vertices_sorted_by_y[1].x && currY >= vertices_sorted_by_y[1].y){
				X1 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[1].x, vertices_sorted_by_y[1].y, vertices_sorted_by_y[2].x, vertices_sorted_by_y[2].y, currY);
			}

			X2 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[0].x, vertices_sorted_by_y[0].y, vertices_sorted_by_y[2].x, vertices_sorted_by_y[2].y, currY);

			if(X2 >= vertices_sorted_by_y[2].x && currY >= vertices_sorted_by_y[2].y){
				X2 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[0].x, vertices_sorted_by_y[0].y, vertices_sorted_by_y[1].x, vertices_sorted_by_y[1].y, currY);
			}

			LCD_draw_line(X1, currY, X2, currY, isInversed);
		}
	}else{
		for(uint8_t i = 0; i < lines_to_draw; i++){
			uint8_t currY = vertices_sorted_by_y[0].y + i;
			X1 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[2].x, vertices_sorted_by_y[2].y, vertices_sorted_by_y[1].x, vertices_sorted_by_y[1].y, currY);
			X2 = LCD_get_X_of_point_on_line(vertices_sorted_by_y[2].x, vertices_sorted_by_y[2].y, vertices_sorted_by_y[0].x, vertices_sorted_by_y[0].y, currY);
			LCD_draw_line(X1, currY, X2, currY, isInversed);
		}
	}
}
/*----------------------------------------------------------------------------*/

dtReturnRenderArea LCD_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation, uint8_t isFilled){
	dtReturnRenderArea ret_render_area = {0u, 0u, 0u, 0u};
	uint8_t pageBegin = 0u;
	uint8_t pageEnd = 0u;
	if(rotation){

		/* Calculate roatation matrix */
		uint8_t centerX, centerY;
		dtPoint rectangle_vertices[4u] = {0u};

		float _sin = sin(rotation);
		float _cos = cos(rotation);

		centerX = x1 + ((x2 - x1)/2);
		centerY = y1 + ((y2 - y1)/2);

		rotate_point(x1, y1, centerX, centerY, _sin, _cos, &rectangle_vertices[0u].x, &rectangle_vertices[0u].y);
		rotate_point(x2, y1, centerX, centerY, _sin, _cos, &rectangle_vertices[1u].x, &rectangle_vertices[1u].y);
		rotate_point(x2, y2, centerX, centerY, _sin, _cos, &rectangle_vertices[2u].x, &rectangle_vertices[2u].y);
		rotate_point(x1, y2, centerX, centerY, _sin, _cos, &rectangle_vertices[3u].x, &rectangle_vertices[3u].y);

		if(!isFilled){
			LCD_draw_line(rectangle_vertices[0u].x, rectangle_vertices[0u].y, rectangle_vertices[1u].x, rectangle_vertices[1u].y, FAIL);
			LCD_draw_line(rectangle_vertices[1u].x, rectangle_vertices[1u].y, rectangle_vertices[2u].x, rectangle_vertices[2u].y, FAIL);
			LCD_draw_line(rectangle_vertices[2u].x, rectangle_vertices[2u].y, rectangle_vertices[3u].x, rectangle_vertices[3u].y, FAIL);
			LCD_draw_line(rectangle_vertices[3u].x, rectangle_vertices[3u].y, rectangle_vertices[0u].x, rectangle_vertices[0u].y, FAIL);
		}else{
			fill_rectangle(rectangle_vertices);
		}

		/* Calculate a render area by x */
		qsort((void*)rectangle_vertices, 4u, sizeof(dtPoint), _sort_points_by_x);
		ret_render_area.x1 = rectangle_vertices[0u].x;
		ret_render_area.x2 = rectangle_vertices[3u].x;
		/* Calculate a render area by y */
		qsort((void*)rectangle_vertices, 4u, sizeof(dtPoint), _sort_points_by_y);
		ret_render_area.y1 = rectangle_vertices[0u].y;
		ret_render_area.y2 = rectangle_vertices[3u].y;

	}else{
		if(!isFilled){
			LCD_draw_line(x1, y1, x2, y1, FAIL);
			LCD_draw_line(x2, y1, x2, y2, FAIL);
			LCD_draw_line(x2, y2, x1, y2, FAIL);
			LCD_draw_line(x1, y2, x1, y1, FAIL);
		}else{
			uint8_t lines_to_draw = 0u;
			uint8_t len = 0u;
			uint8_t x_start, y_start;
			if(y2 > y1){
				lines_to_draw = y2 - y1;
				y_start = y1;

				ret_render_area.y1 = y1;
				ret_render_area.y2 = y2;
			}else{
				lines_to_draw = y1 - y2;
				y_start = y2;

				ret_render_area.y1 = y2;
				ret_render_area.y2 = y1;
			}

			if(x2 > x1){
				x_start = x1;
				len = x2 - x1;

				ret_render_area.x1 = x1;
				ret_render_area.x2 = x2;
			}else{
				x_start = x2;
				len = x1 - x2;

				ret_render_area.x1 = x1;
				ret_render_area.x2 = x2;
			}
			for(uint8_t i = y_start; i <= y_start + lines_to_draw; i++){
				LCD_draw_line(x_start, i, x_start + len, i, FAIL);
			}
		}
	}
	pageBegin = ret_render_area.y1 / PAGE_LEN;
	pageEnd = (ret_render_area.y2 / PAGE_LEN); // ???
	LCD_setUpdateArea(pageBegin, pageEnd, ret_render_area.x1, ret_render_area.x2);

	return ret_render_area;
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
/*----------------------------------------------------------------------------*/

void LCD_draw_triangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation, uint8_t isFilled){
	dtPoint vertices[3u] = {{x, y}, {x, y + height}, {x + width, y}};
	if(rotation){
		dtPoint rotated_vertices[3u] = {0u};
		uint8_t centerX = 0u, centerY = 0u;
		/* Calculate coordinates of the center */
		float A1, B1, C1;
		float A2, B2, C2;
		float D;

		dtPoint median1[2u] = {{x, y + (height / 2u)}, {x + width, y}};
		dtPoint median2[2u] = {{x, y + height}, {x + (width / 2), y}};

		A1 = median1[1u].y - median1[0u].y; // A = y2 - y1
		B1 = median1[0u].x - median1[1u].x; // B = x2 - x1
		C1 = ((-1)* A1 * median1[0u].x ) - (B1 * median1[0u].y); // C = -A*x1 -B*y1

		A2 = median2[1u].y - median2[0u].y; // A = y2 - y1
		B2 = median2[0u].x - median2[1u].x; // B = x2 - x1
		C2 = ((-1)* A2 * median2[0u].x ) - (B2 * median2[0u].y); // C = -A*x1 -B*y1

		D = (A1 * B2) - (A2 * B1);

		if(D){
			float centX_f, centY_f;
			centX_f = ((B1 * C2) - (B2 * C1)) / D;
			centY_f = ((C1 * A2) - (C2 * A1)) / D;

			centerX = round(centX_f);
			centerY = round(centY_f);
		}

		/* Calculate roatation matrix */

		float _sin = sin(rotation);
		float _cos = cos(rotation);

		rotate_point(vertices[0u].x, vertices[0u].y, centerX, centerY, _sin, _cos, &rotated_vertices[0u].x, &rotated_vertices[0u].y);
		rotate_point(vertices[1u].x, vertices[1u].y, centerX, centerY, _sin, _cos, &rotated_vertices[1u].x, &rotated_vertices[1u].y);
		rotate_point(vertices[2u].x, vertices[2u].y, centerX, centerY, _sin, _cos, &rotated_vertices[2u].x, &rotated_vertices[2u].y);


		LCD_draw_line(rotated_vertices[0u].x, rotated_vertices[0u].y, rotated_vertices[1u].x, rotated_vertices[1u].y, FAIL);
		LCD_draw_line(rotated_vertices[0u].x, rotated_vertices[0u].y, rotated_vertices[2u].x, rotated_vertices[2u].y, FAIL);
		LCD_draw_line(rotated_vertices[1u].x, rotated_vertices[1u].y, rotated_vertices[2u].x, rotated_vertices[2u].y, FAIL);
		if(isFilled){
			/* Fill triangle! */
			fill_triangle(rotated_vertices, FAIL);
		}
	}else{

		LCD_draw_line(vertices[0u].x, vertices[0u].y, vertices[1u].x, vertices[1u].y, FAIL);
		LCD_draw_line(vertices[0u].x, vertices[0u].y, vertices[2u].x, vertices[2u].y, FAIL);
		LCD_draw_line(vertices[1u].x, vertices[1u].y, vertices[2u].x, vertices[2u].y, FAIL);

		if(isFilled){
			for(uint8_t i = y; i <= y + height; i++){
				uint8_t EOL = LCD_get_X_of_point_on_line(vertices[1u].x, vertices[1u].y, vertices[2u].x, vertices[2u].y, i);
				LCD_draw_line(x, i, EOL, i, FAIL);
			}
		}
	}
}
/*----------------------------------------------------------------------------*/

void LCD_draw_arrow(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation, uint8_t isFilled){

	/* !!!isFilled arg is not used at the moment, arrow will be always filled!!! */

	/* Here some const params which responsible for arrow proportions */
	uint8_t rectangle_width = (float)(height) * 0.5f;
	uint8_t rectangle_len = (float)(width) * 0.5f;
	uint8_t overlay = 2u; // in pixels to fix some glitches with rotation


	dtPoint vertices[3u] = {{x + rectangle_len - overlay, y}, {x + width, y + height/2}, {x + rectangle_len - overlay, y + height}};
	dtPoint rotated_vertices[3u] = {0u};
	uint8_t centerX = x + width/2, centerY = y + height/2;

	dtPoint rectangle_vertices[4u] = {{x, y + (height - rectangle_width)/2},{x + rectangle_len, y + (height - rectangle_width)/2},{x + rectangle_len, (y + height) - (height - rectangle_width)/2}, {x,  (y + height) - (height - rectangle_width)/2}};
	dtPoint rotated_rectangle_vertices[4u] = {0u};

	float _sin = sin(rotation);
	float _cos = cos(rotation);

	rotate_point(vertices[0u].x, vertices[0u].y, centerX, centerY, _sin, _cos, &rotated_vertices[0u].x, &rotated_vertices[0u].y);
	rotate_point(vertices[1u].x, vertices[1u].y, centerX, centerY, _sin, _cos, &rotated_vertices[1u].x, &rotated_vertices[1u].y);
	rotate_point(vertices[2u].x, vertices[2u].y, centerX, centerY, _sin, _cos, &rotated_vertices[2u].x, &rotated_vertices[2u].y);

	rotate_point(rectangle_vertices[0].x, rectangle_vertices[0].y, centerX, centerY, _sin, _cos, &rotated_rectangle_vertices[0u].x, &rotated_rectangle_vertices[0u].y);
	rotate_point(rectangle_vertices[1].x, rectangle_vertices[1].y, centerX, centerY, _sin, _cos, &rotated_rectangle_vertices[1u].x, &rotated_rectangle_vertices[1u].y);
	rotate_point(rectangle_vertices[2].x, rectangle_vertices[2].y, centerX, centerY, _sin, _cos, &rotated_rectangle_vertices[2u].x, &rotated_rectangle_vertices[2u].y);
	rotate_point(rectangle_vertices[3].x, rectangle_vertices[3].y, centerX, centerY, _sin, _cos, &rotated_rectangle_vertices[3u].x, &rotated_rectangle_vertices[3u].y);


	fill_triangle(rotated_vertices, FAIL);
	fill_rectangle(rotated_rectangle_vertices);

}
/*----------------------------------------------------------------------------*/

dtReturnRenderArea LCD_draw_navigation_arrow(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation){
	dtReturnRenderArea ret_render_area = {0u, 0u, 0u, 0u};
	dtPoint renderArea_vertices[4u];
	uint8_t pageBegin = 0u;
	uint8_t pageEnd = 0u;

	/* Here some const params which responsible for arrow proportions */
	uint8_t inner_triangle_len = (float)(width) * 0.4f;


	dtPoint vertices_outer_triangle[3u] = {{x, y}, {x + width, y + height/2}, {x, y + height}};
	dtPoint vertices_inner_triangle[3u] = {{x, y}, {x + inner_triangle_len, y + height/2}, {x, y + height}};
	dtPoint rotated_vertices_outer_triangle[3u] = {0u};
	dtPoint rotated_vertices_inner_triangle[3u] = {0u};
	uint8_t centerX = x + width/2, centerY = y + height/2;

	float _sin = sin(rotation);
	float _cos = cos(rotation);

	rotate_point(vertices_outer_triangle[0u].x, vertices_outer_triangle[0u].y, centerX, centerY, _sin, _cos, &rotated_vertices_outer_triangle[0u].x, &rotated_vertices_outer_triangle[0u].y);
	rotate_point(vertices_outer_triangle[1u].x, vertices_outer_triangle[1u].y, centerX, centerY, _sin, _cos, &rotated_vertices_outer_triangle[1u].x, &rotated_vertices_outer_triangle[1u].y);
	rotate_point(vertices_outer_triangle[2u].x, vertices_outer_triangle[2u].y, centerX, centerY, _sin, _cos, &rotated_vertices_outer_triangle[2u].x, &rotated_vertices_outer_triangle[2u].y);

	/* Just one additional point has to be rotated for inner triangle */
	rotate_point(vertices_inner_triangle[1u].x, vertices_inner_triangle[1u].y, centerX, centerY, _sin, _cos, &rotated_vertices_inner_triangle[1u].x, &rotated_vertices_inner_triangle[1u].y);
	rotated_vertices_inner_triangle[0u] = rotated_vertices_outer_triangle[0u];
	rotated_vertices_inner_triangle[2u] = rotated_vertices_outer_triangle[2u];

	fill_triangle(rotated_vertices_outer_triangle, FAIL);
	fill_triangle(rotated_vertices_inner_triangle, PASS);


	/* Calculate a render area by x */
	renderArea_vertices[0u] = rotated_vertices_outer_triangle[0u];
	renderArea_vertices[1u] = rotated_vertices_outer_triangle[2u];
	rotate_point(x + width, y, centerX, centerY, _sin, _cos, &renderArea_vertices[2u].x, &renderArea_vertices[2u].y);
	rotate_point(x + width, y + height, centerX, centerY, _sin, _cos, &renderArea_vertices[3u].x, &renderArea_vertices[3u].y);

	qsort((void*)renderArea_vertices, 4u, sizeof(dtPoint), _sort_points_by_x);
	ret_render_area.x1 = renderArea_vertices[0u].x;
	ret_render_area.x2 = renderArea_vertices[3u].x + 1; /* Clear 1 pixel more to remove some glitches */
	/* Calculate a render area by y */
	qsort((void*)renderArea_vertices, 4u, sizeof(dtPoint), _sort_points_by_y);
	ret_render_area.y1 = renderArea_vertices[0u].y;
	ret_render_area.y2 = renderArea_vertices[3u].y + 1; /* Clear 1 pixel more to remove some glitches */

	pageBegin = ret_render_area.y1 / PAGE_LEN;
	pageEnd = (ret_render_area.y2 / PAGE_LEN); // ???
	LCD_setUpdateArea(pageBegin, pageEnd, ret_render_area.x1, ret_render_area.x2);

	return ret_render_area;
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
void LCD_setUpdateArea(uint8_t pageBegin, uint8_t pageEnd, uint8_t lineBegin, uint8_t lineEnd){
	uint8_t areaUpdated = FAIL;

	if(Display.updatedArea_PageStart >= pageBegin){
		Display.updatedArea_PageStart = pageBegin;
		areaUpdated = PASS;
	}

	if(Display.updatedArea_PageEnd <= pageEnd){
		Display.updatedArea_PageEnd = pageEnd;
		areaUpdated = PASS;
	}

	if(Display.updatedArea_LineStart >= lineBegin){
		Display.updatedArea_LineStart = lineBegin;
		areaUpdated = PASS;
	}

	if(Display.updatedArea_LineEnd <= lineEnd){
		Display.updatedArea_LineEnd = lineEnd;
		areaUpdated = PASS;
	}

	if(areaUpdated){
		Display.status = RENDER_PENDING;
	}
}
/*----------------------------------------------------------------------------*/
void LCD_updateScreen(void){
	if(Display.status != RENDERED){
		uint8_t pageBegin = Display.updatedArea_PageStart;
		uint8_t pageEnd = Display.updatedArea_PageEnd;
		uint8_t lineBegin = Display.updatedArea_LineStart;
		uint8_t lineEnd = Display.updatedArea_LineEnd;
		/* Send into display RAM */
		if(lineEnd >= LCD_COLUMN_AMNT){
			lineEnd = LCD_COLUMN_AMNT - 1;
		}
		if(pageEnd >= LCD_PAGES_AMNT){
			pageEnd = LCD_PAGES_AMNT - 1;
		}
		uint8_t len = lineEnd - lineBegin;
		for(uint8_t currPage = pageBegin; currPage <= pageEnd; currPage++){
			LCD_sendCmd(CMD_DISPLAY_SET_PAGE(currPage));
			LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_L(lineBegin));
			LCD_sendCmd(CMD_DISPLAY_SET_COLUMN_H(lineBegin));
			LCD_sendData(&LCD_buf[currPage][lineBegin], len);
		}

		/* Clear rendering area status */
		Display.updatedArea_PageStart = LCD_PAGES_AMNT;
		Display.updatedArea_PageEnd = 0u;
		Display.updatedArea_LineStart = LCD_COLUMN_AMNT;
		Display.updatedArea_LineEnd = 0u;
		Display.status = RENDERED;
	}
}
/*----------------------------------------------------------------------------*/
void LCD_clearArea(dtPoint LowerVertex, dtPoint HigherVertex){
	if((HigherVertex.x != LowerVertex.x) && (HigherVertex.y != LowerVertex.y)){
		if(HigherVertex.x >= LCD_COLUMN_AMNT){
			HigherVertex.x = LCD_COLUMN_AMNT - 1;
		}
		if(HigherVertex.y >= LCD_LINE_AMNT){
			HigherVertex.y = LCD_LINE_AMNT - 1;
		}

		uint8_t pageBegin = LowerVertex.y / PAGE_LEN;
		uint8_t pageEnd = HigherVertex.y / PAGE_LEN;
		uint8_t Ypos_within_begin_page = LowerVertex.y % PAGE_LEN;
		uint8_t Ypos_within_end_page = HigherVertex.y % PAGE_LEN;
		uint8_t area_len = HigherVertex.x - LowerVertex.x;
		uint8_t page_tmp = 0u;
		uint8_t mask_begin = 0u;
		uint8_t mask_end = 0u;

		mask_begin = 0xFFu << Ypos_within_begin_page;

		if(Ypos_within_end_page || Ypos_within_begin_page){
			/* calculate next page mask explicitly as >> operation is implementation defined */
			for(uint8_t k = 0u; k < 8; k++){
				if(k <= Ypos_within_end_page){
					mask_end |= (1 << k);
				}
			}
		}
		for(uint8_t currPage = pageBegin; currPage <= pageEnd; currPage++){
			if(((currPage == pageBegin) && (Ypos_within_begin_page != 0)) ||
			   ((currPage == pageEnd) && (mask_end != 0))){
				for(uint8_t i = LowerVertex.x; i <= HigherVertex.x; i++){
					page_tmp = LCD_buf[currPage][i];
					if(currPage == pageBegin){
						page_tmp &= ~(mask_begin);
						LCD_buf[currPage][i] = page_tmp;
					}else if(currPage == pageEnd){
						page_tmp &= ~(mask_end);
						LCD_buf[currPage][i] = page_tmp;
					}else{
						/* Defensive programming */
					}
				}
			}else{
				/* Just clear the whole page */
				memset(&LCD_buf[currPage][LowerVertex.x], 0x00u, area_len);
			}
		}
		LCD_setUpdateArea(pageBegin, pageEnd, LowerVertex.x, HigherVertex.x);
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

/*----------------------------------------------------------------------------*/
uint8_t LCD_set_current_frame(dtFrame* newFrame){
	if(Display.currentFrame != newFrame){
		/* Clear buffer and display RAM each current frame change */
		LCD_clearScreen();
		/* Not the currently rendering frame */
		Display.currentFrame = newFrame;
		Display.updatedArea_PageStart = LCD_PAGES_AMNT;
		Display.updatedArea_PageEnd = 0u;
		Display.updatedArea_LineStart = LCD_COLUMN_AMNT;
		Display.updatedArea_LineEnd = 0u;
		Display.currentFrame->status = FULL_RENDER_PENDING;

		Display.status = FULL_RENDER_PENDING;

		for(uint8_t i = 0; i < Display.currentFrame->entities_cnt; i++){
			dtEntity* entityTmp = (dtEntity*)Display.currentFrame->entities[i];
			entityTmp->status.status = NOT_RENDERED;
		}

		return PASS;
	}else{
		return FAIL;
	}
}

/*----------------------------------------------------------------------------*/
dtFrame* LCD_get_current_frame(void){
	return Display.currentFrame;
}

/*----------------------------------------------------------------------------*/
void render_entity(void* entity, uint8_t isForceRender){

	dtEntity* currEntity = (dtEntity*)entity;
	dtReturnRenderArea RenderArea = {0u};
	if(currEntity->status.status != RENDERED || isForceRender){
		/* Clear area of previous render of entity */
		if((currEntity->status.renderAreaUpperPoint.x && currEntity->status.renderAreaUpperPoint.y) && currEntity->status.status != NOT_RENDERED){
			LCD_clearArea(currEntity->status.renderAreaLowerPoint, currEntity->status.renderAreaUpperPoint);
		}

		if(currEntity->main.type == TEXT){
			/* Render text */
			RenderArea = LCD_print(	currEntity->properties.Text.textStartPoint.x,
									currEntity->properties.Text.textStartPoint.y,
									currEntity->properties.Text.text,
									currEntity->properties.Text.textLen,
									currEntity->properties.Text.isInversed);

		}else if(currEntity->main.type == RECTANGLE){
			/* Render rectangle */
			RenderArea = LCD_draw_rectangle(currEntity->properties.Rectangle.vertex1.x,
											currEntity->properties.Rectangle.vertex1.y,
											currEntity->properties.Rectangle.vertex2.x,
											currEntity->properties.Rectangle.vertex2.y,
											currEntity->properties.Rectangle.rotation,
											currEntity->properties.Rectangle.isFilled);
		}else if(currEntity->main.type == BITMAP){
			/* Render bitmap */
			RenderArea = LCD_draw_bitmap(	currEntity->properties.Bitmap.bitmapStartPoint.x,
											currEntity->properties.Bitmap.bitmapStartPoint.y,
											currEntity->properties.Bitmap.width,
											currEntity->properties.Bitmap.height,
											currEntity->properties.Bitmap.bitmap);

		}else if(currEntity->main.type == NAVIGATION_ARROW){
			/* Render bitmap */
			RenderArea = LCD_draw_navigation_arrow(	currEntity->properties.NavigationArrow.vertex.x,
													currEntity->properties.NavigationArrow.vertex.y,
													currEntity->properties.NavigationArrow.width,
													currEntity->properties.NavigationArrow.height,
													currEntity->properties.NavigationArrow.rotation);
		}

		if(RenderArea.x2 != 0 && RenderArea.y2 != 0){
			currEntity->status.status = RENDERED;
		}
		currEntity->status.renderAreaLowerPoint.x = RenderArea.x1;
		currEntity->status.renderAreaLowerPoint.y = RenderArea.y1;
		currEntity->status.renderAreaUpperPoint.x = RenderArea.x2;
		currEntity->status.renderAreaUpperPoint.y = RenderArea.y2;
	}
}

/*----------------------------------------------------------------------------*/
uint8_t LCD_render_current_frame(void){
	if(Display.currentFrame != NULL){
		uint8_t entities_cnt = Display.currentFrame->entities_cnt;
		void** entities = Display.currentFrame->entities;
		uint8_t isForceRender = FAIL;
		if(Display.currentFrame->status == FULL_RENDER_PENDING){
			/* Force render when the current frame has been changed */
			isForceRender = PASS;
		}
		for(uint8_t i = 0u; i < entities_cnt; i++){
			render_entity(*(entities + i), isForceRender);
#ifdef LCD_DEBUG
			LCD_updateScreen();
#endif
		}
		Display.currentFrame->status = RENDERED;
		return PASS;
	}else{
		return FAIL;
	}
}
/*----------------------------------------------------------------------------*/
uint8_t allocate_entity(dtEntity** newEntityPtr){
	for(uint8_t i = 0u; i < MAX_ENTITIES; i++){
		if(Display.entitiesHeap[i].main.id == 0xFFFFu && Display.entitiesHeap[i].main.type == 0xFFFFu){
			*newEntityPtr = &Display.entitiesHeap[i];
			Display.entitiesHeap[i].main.id = Display.entitiesIDCnt;
			Display.entitiesIDCnt++;
			return PASS;
		}
	}
	return FAIL;
}

uint8_t free_entity(dtEntity* entityPtr){
	for(uint8_t i = 0u; i < MAX_ENTITIES; i++){
		if(&Display.entitiesHeap[i] == entityPtr){
			Display.entitiesHeap[i].main.id = 0xFFFFu;
			Display.entitiesHeap[i].main.type = 0xFFFFu;
			return PASS;
		}
	}
	return FAIL;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
uint16_t add_text_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, char* textPtr, uint8_t textLen, uint8_t isInversed){
	dtEntity* newEntity = NULL;
	uint16_t retID = 0xFFFFu;
	if(frame->entities_cnt < MAX_ENTITIES_ON_SINGLE_FRAME - 1){
		if(allocate_entity(&newEntity) == PASS){
			newEntity->main.type = TEXT;
			newEntity->properties.Text.textStartPoint.x = x;
			newEntity->properties.Text.textStartPoint.y = y;
			newEntity->properties.Text.text = textPtr;
			newEntity->properties.Text.textLen = textLen;
			newEntity->properties.Text.isInversed = isInversed;

			newEntity->status.status = NOT_RENDERED;

			frame->entities[frame->entities_cnt++] = (void*)newEntity;

			retID = newEntity->main.id;
		}
	}
	return retID;
}

/*----------------------------------------------------------------------------*/
uint8_t update_text_entity_position(dtFrame* frame, uint16_t id ,uint8_t x, uint8_t y){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == TEXT)){
			currEntity->properties.Text.textStartPoint.x = x;
			currEntity->properties.Text.textStartPoint.y = y;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_text_entity_text(dtFrame* frame, uint16_t id ,char* textPtr, uint8_t textLen){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == TEXT)){
			currEntity->properties.Text.text = textPtr;
			currEntity->properties.Text.textLen = textLen;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_text_entity_inversion(dtFrame* frame, uint16_t id, uint8_t isInversed){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == TEXT)){
			currEntity->properties.Text.isInversed = isInversed;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

uint16_t add_bitmap_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap){
	dtEntity* newEntity = NULL;
	uint16_t retID = 0xFFFFu;
	if(frame->entities_cnt < MAX_ENTITIES_ON_SINGLE_FRAME - 1){
		if(allocate_entity(&newEntity) == PASS){
			newEntity->main.type = BITMAP;
			newEntity->properties.Bitmap.bitmapStartPoint.x = x;
			newEntity->properties.Bitmap.bitmapStartPoint.y = y;
			newEntity->properties.Bitmap.height = height;
			newEntity->properties.Bitmap.width = width;
			newEntity->properties.Bitmap.bitmap = bitmap;

			newEntity->status.status = NOT_RENDERED;

			frame->entities[frame->entities_cnt++] = (void*)newEntity;

			retID = newEntity->main.id;
		}
	}
	return retID;
}

/*----------------------------------------------------------------------------*/
uint8_t update_bitmap_entity_position(dtFrame* frame, uint16_t id ,uint8_t x, uint8_t y){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == BITMAP)){
			currEntity->properties.Bitmap.bitmapStartPoint.x = x;
			currEntity->properties.Bitmap.bitmapStartPoint.y = y;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_bitmap_entity_bitmap(dtFrame* frame, uint16_t id ,uint8_t width, uint8_t height, uint8_t* bitmap){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == BITMAP)){
			currEntity->properties.Bitmap.height = height;
			currEntity->properties.Bitmap.width = width;
			currEntity->properties.Bitmap.bitmap = bitmap;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

uint16_t add_rectangle_entity_to_frame(dtFrame* frame, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation, uint8_t isFilled){
	dtEntity* newEntity = NULL;
	uint16_t retID = 0xFFFFu;
	if(frame->entities_cnt < MAX_ENTITIES_ON_SINGLE_FRAME - 1){
		if(allocate_entity(&newEntity) == PASS){
			newEntity->main.type = RECTANGLE;
			newEntity->properties.Rectangle.vertex1.x = x1;
			newEntity->properties.Rectangle.vertex1.y = y1;
			newEntity->properties.Rectangle.vertex2.x = x2;
			newEntity->properties.Rectangle.vertex2.y = y2;
			newEntity->properties.Rectangle.rotation = rotation;
			newEntity->properties.Rectangle.isFilled = isFilled;

			newEntity->status.status = NOT_RENDERED;

			frame->entities[frame->entities_cnt++] = (void*)newEntity;

			retID = newEntity->main.id;
		}
	}
	return retID;
}

/*----------------------------------------------------------------------------*/
uint8_t update_rectangle_entity_position(dtFrame* frame, uint16_t id, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == RECTANGLE)){
			currEntity->properties.Rectangle.vertex1.x = x1;
			currEntity->properties.Rectangle.vertex1.y = y1;
			currEntity->properties.Rectangle.vertex2.x = x2;
			currEntity->properties.Rectangle.vertex2.y = y2;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_rectangle_entity_rotation(dtFrame* frame, uint16_t id, float rotation){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == RECTANGLE)){
			currEntity->properties.Rectangle.rotation = rotation;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_rectangle_entity_filled(dtFrame* frame, uint16_t id, uint8_t isFilled){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == RECTANGLE)){
			currEntity->properties.Rectangle.isFilled = isFilled;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

uint16_t add_navigation_arrow_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation){
	dtEntity* newEntity = NULL;
	uint16_t retID = 0xFFFFu;
	if(frame->entities_cnt < MAX_ENTITIES_ON_SINGLE_FRAME - 1){
		if(allocate_entity(&newEntity) == PASS){
			newEntity->main.type = NAVIGATION_ARROW;
			newEntity->properties.NavigationArrow.vertex.x = x;
			newEntity->properties.NavigationArrow.vertex.y = y;
			newEntity->properties.NavigationArrow.width = width;
			newEntity->properties.NavigationArrow.height = height;
			newEntity->properties.NavigationArrow.rotation = rotation;

			newEntity->status.status = NOT_RENDERED;

			frame->entities[frame->entities_cnt++] = (void*)newEntity;

			retID = newEntity->main.id;
		}
	}
	return retID;
}

/*----------------------------------------------------------------------------*/
uint8_t update_navigation_arrow_entity_position(dtFrame* frame, uint16_t id, uint8_t x, uint8_t y){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == NAVIGATION_ARROW)){
			currEntity->properties.NavigationArrow.vertex.x = x;
			currEntity->properties.NavigationArrow.vertex.y = y;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
uint8_t update_navigation_arrow_entity_rotation(dtFrame* frame, uint16_t id, float rotation){
	for(uint8_t i = 0u; i < frame->entities_cnt; i++){
		dtEntity* currEntity = (dtEntity*)frame->entities[i];
		if((currEntity->main.id == id) && (currEntity->main.type == NAVIGATION_ARROW)){
			currEntity->properties.NavigationArrow.rotation = rotation;
			currEntity->status.status = RENDER_PENDING;
			frame->status = RENDER_PENDING;
			return PASS;
		}
	}
	return FAIL;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void LCD_Handler(void){
	if(Display.currentFrame != NULL){
		if(Display.currentFrame->status != RENDERED){
			LCD_render_current_frame();
		}
	}
}
/*----------------------------------------------------------------------------*/
void Display_Init(void){
	Display.currentFrame = NULL;
	memset(Display.entitiesHeap, 0xFFu, sizeof(Display.entitiesHeap));
	Display.updatedArea_PageStart = LCD_PAGES_AMNT;
	Display.updatedArea_PageEnd = 0u;
	Display.updatedArea_LineStart = LCD_COLUMN_AMNT;
	Display.updatedArea_LineEnd = 0u;

	Display.entitiesIDCnt = 0u;
	Display.status = FULL_RENDER_PENDING;
}

/******************************************************************************/
