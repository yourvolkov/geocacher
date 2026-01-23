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

#define RES_PIN           LCD_RES_Pin
#define RES_PORT          LCD_RES_GPIO_Port

#define CMD_PIN           LCD_RS_Pin
#define CMD_PORT          LCD_RS_GPIO_Port

#define CS_PIN            LCD_CS_Pin
#define CS_PORT           LCD_CS_GPIO_Port
/******************************************************************************/
/****************************** Public types  *********************************/
/******************************************************************************/
typedef enum{
	OK = 0u,
	BUSY,
	FAILED
}dtReturnValue;
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/
extern const uint8_t lcd_bitmap[];
extern uint8_t lcd_arrow_conv[2][24];
extern const uint8_t lcd_arrow[];
extern uint8_t lcd_bitmap_inv[8][64];
/******************************************************************************/
/****************************** Public function prototypes ********************/
/******************************************************************************/
void LCD_init(void);
void LCD_clearScreen(void);

void LCD_test(void);

void LCD_print(uint8_t cursorX, uint8_t cursorY, char* line, size_t len);

void LCD_draw_pixel(uint8_t x, uint8_t y);
void LCD_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void LCD_draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, float rotation);
void LCD_draw_bitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* bitmap);

void convert_horizontal_bitmap(uint8_t* in, uint8_t* out, uint8_t w, uint8_t h);
/******************************************************************************/
#endif // __ST7565_H_
