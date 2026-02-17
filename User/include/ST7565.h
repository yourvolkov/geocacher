#ifndef __ST7565_H_
#define __ST7565_H_

/******************************************************************************/
/**************************** Library version  ********************************/
/******************************************************************************/
/*
 * Current version 
 * 		Release notes:
 * 			version :  
 */
/******************************************************************************/
/****************************** How To Use  ***********************************/
/******************************************************************************/

/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "main.h"
#include "stdint.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define LCD_PAGES_AMNT    8u
#define LCD_COLUMN_AMNT   132u

#define PAGE_LEN          8u
#define LCD_LINE_AMNT     LCD_PAGES_AMNT * PAGE_LEN

#define RES_PIN           LCD_RES_Pin
#define RES_PORT          LCD_RES_GPIO_Port

#define CMD_PIN           LCD_RS_Pin
#define CMD_PORT          LCD_RS_GPIO_Port

#define CS_PIN            LCD_CS_Pin
#define CS_PORT           LCD_CS_GPIO_Port

#define MAX_ENTITIES_ON_SINGLE_FRAME	20u
#define MAX_TEXT_LEN	21u
/******************************************************************************/
/****************************** Public types  *********************************/
/******************************************************************************/
typedef enum{
	OK = 0u,
	BUSY,
	FAILED
}dtReturnValue;


typedef struct{
	void* entities[MAX_ENTITIES_ON_SINGLE_FRAME];
	uint8_t entities_cnt;
	uint8_t status;
}dtFrame;
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/
extern const uint8_t lcd_bitmap[];
extern uint8_t lcd_arrow_conv[2][24];
extern const uint8_t lcd_arrow[];
extern uint8_t lcd_bitmap_inv[8][48];
/******************************************************************************/
/****************************** Public function prototypes ********************/
/******************************************************************************/
void LCD_init(void);
void LCD_clearScreen(void);
void LCD_updateScreen(void);

void LCD_test(void);


void LCD_draw_circle(uint8_t X0, uint8_t Y0, uint8_t R, uint8_t isFilled);
void LCD_draw_triangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation, uint8_t isFilled);
void LCD_draw_arrow(uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation, uint8_t isFilled);

void convert_horizontal_bitmap(uint8_t* in, uint8_t* out, uint8_t w, uint8_t h);


/* New API */
void LCD_Handler(void);
void Display_Init(void);

uint8_t LCD_set_current_frame(dtFrame* newFrame);
dtFrame* LCD_get_current_frame(void);
/* API for text entity */
uint16_t add_text_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, char* textPtr, uint8_t textLen, uint8_t isInversed, uint32_t inversionMask);
uint8_t update_text_entity_position(dtFrame* frame, uint16_t id ,uint8_t x, uint8_t y);
uint8_t update_text_entity_text(dtFrame* frame, uint16_t id ,char* textPtr, uint8_t textLen);
uint8_t update_text_entity_inversion(dtFrame* frame, uint16_t id, uint8_t isInversed, uint32_t inversionMask);

uint16_t add_bitmap_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap);
uint8_t update_bitmap_entity_position(dtFrame* frame, uint16_t id ,uint8_t x, uint8_t y);
uint8_t update_bitmap_entity_bitmap(dtFrame* frame, uint16_t id ,uint8_t width, uint8_t height, uint8_t* bitmap);

uint16_t add_rectangle_entity_to_frame(dtFrame* frame, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation, uint8_t isFilled);
uint8_t update_rectangle_entity_position(dtFrame* frame, uint16_t id, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
uint8_t update_rectangle_entity_rotation(dtFrame* frame, uint16_t id, float rotation);
uint8_t update_rectangle_entity_filled(dtFrame* frame, uint16_t id, uint8_t isFilled);

uint16_t add_navigation_arrow_entity_to_frame(dtFrame* frame, uint8_t x, uint8_t y, uint8_t width, uint8_t height, float rotation);
uint8_t update_navigation_arrow_entity_position(dtFrame* frame, uint16_t id, uint8_t x, uint8_t y);
uint8_t update_navigation_arrow_entity_rotation(dtFrame* frame, uint16_t id, float rotation);
/******************************************************************************/
#endif // __ST7565_H_
