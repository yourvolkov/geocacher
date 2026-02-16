/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "ST7565.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "stdlib.h"
#include "Utils.h"
#include "NMEA.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define MAIN_STATE_TEXT    "Geocacher!"
#define MENU_TEXT_1    "START NEW GAME"
#define MENU_TEXT_2    "CURRENT GPS DATA"
#define MENU_TEXT_3    "SETTINGS"

#define LOW_SIG_MESSAGE " LOW SIGNAL!"

#define MENU_RECTANGLE_HEIGHT	9u
#define MAIN_MENU_TEXT_START_POS	20u
#define MAIN_MENU_TEXT_GAP	10u

#define GPS_DATA_START_POS	12u
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	START_FRAME = 0u,
	MAIN_MENU_BEGIN,
	NEW_GAME_FRAME,
	RESUME_GAME_FRAME,
	GPS_DATA_FRAME,
	SETTINGS_FRAME

}dtMainState;


/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
dtMainState mainState = START_FRAME;
dtFrame startFrame;
dtFrame menuFrame;
dtFrame newGameFrame;
dtFrame gpsDataFrame;
dtFrame settingsFrame;
dtFrame gamePlayFrame;

uint8_t main_menu_previous_pos = 0u;

uint16_t bitmap1ID;

uint16_t text1ID, text2ID, text3ID;
uint16_t rectangle1ID;

uint16_t newGameFrame_text1ID;
uint16_t newGameFrame_rectangle1ID;

uint16_t gpsDataFrame_titleTextID, gpsDataFrame_latitudeTextID, gpsDataFrame_LongitudetextID, gpsDataFrame_AltitudetextID, gpsDataFrame_SpeedTextID, gpsDataFrame_SattelitesTextID;
uint16_t gpsDataFrame_serviceMessageTextID;
uint16_t gpsDataFrame_rectangle1ID;

uint16_t settingsFrame_text1ID;
uint16_t settingsFrame_rectangle1ID;

uint16_t gamePlayFrame_navigation_arrowID;

char latitude[MAX_TEXT_LEN] = {0u};
char longitude[MAX_TEXT_LEN] = {0u};
char speed[MAX_TEXT_LEN] = {0u};
char sat_amnt[MAX_TEXT_LEN] = {0u};
char altitude[MAX_TEXT_LEN] = {0u};


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

/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/

/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/
uint8_t center_text(uint8_t len_str){
	return (127u - len_str * 6) / 2;
}

/*----------------------------------------------------------------------------*/
void Init_start_frame(void){
	bitmap1ID = add_bitmap_entity_to_frame(&startFrame, 45u, 0u, 48, 64, lcd_bitmap_inv);
}
/*----------------------------------------------------------------------------*/
void Init_main_menu_frame(void){
	rectangle1ID = add_rectangle_entity_to_frame(&menuFrame, 0, 19, 127, 28, 0.0, PASS);

	text1ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_1)),
										MAIN_MENU_TEXT_START_POS,
										MENU_TEXT_1,
										strlen(MENU_TEXT_1),
										PASS);
	text2ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_2)),
										MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP,
										MENU_TEXT_2,
										strlen(MENU_TEXT_2),
										FAIL);
	text3ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_3)),
										MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP * 2,
										MENU_TEXT_3,
										strlen(MENU_TEXT_3),
										FAIL);
}

/*----------------------------------------------------------------------------*/
void Init_new_game_frame(void){
	newGameFrame_rectangle1ID = add_rectangle_entity_to_frame(&newGameFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS);

	newGameFrame_text1ID = add_text_entity_to_frame(&newGameFrame,
													center_text(strlen(MENU_TEXT_1)),
													1u,
													MENU_TEXT_1,
													strlen(MENU_TEXT_1),
													PASS);

	gamePlayFrame_navigation_arrowID = add_navigation_arrow_entity_to_frame(&newGameFrame, 44, 22, 40, 26, 0.0);
}

/*----------------------------------------------------------------------------*/
void Init_gps_data_frame(void){
	gpsDataFrame_rectangle1ID = add_rectangle_entity_to_frame(&gpsDataFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS);

	gpsDataFrame_titleTextID = add_text_entity_to_frame(&gpsDataFrame,
													center_text(strlen(MENU_TEXT_2)),
													1u,
													MENU_TEXT_2,
													strlen(MENU_TEXT_2),
													PASS);

	gpsDataFrame_latitudeTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS,
													latitude,
													0u,
													FAIL);

	gpsDataFrame_LongitudetextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP,
													longitude,
													0u,
													FAIL);

	gpsDataFrame_AltitudetextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 2,
													altitude,
													0u,
													FAIL);

	gpsDataFrame_SpeedTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 3,
													speed,
													0u,
													FAIL);

	gpsDataFrame_SattelitesTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 4,
													sat_amnt,
													0u,
													FAIL);

	gpsDataFrame_serviceMessageTextID = add_text_entity_to_frame(&gpsDataFrame,
																center_text(strlen(LOW_SIG_MESSAGE)),
																28u,
																LOW_SIG_MESSAGE,
																strlen(LOW_SIG_MESSAGE),
																PASS);

}

/*----------------------------------------------------------------------------*/
void Init_settings_frame(void){
	settingsFrame_rectangle1ID = add_rectangle_entity_to_frame(&settingsFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS);

	settingsFrame_text1ID = add_text_entity_to_frame(&settingsFrame,
													center_text(strlen(MENU_TEXT_3)),
													1u,
													MENU_TEXT_3,
													strlen(MENU_TEXT_3),
													PASS);

}



void handle_gps_current_data_frame(void){
	dtNMEACoordinate lati, longi;
	float speed_f, alti;
	uint8_t sat_amnt_ui;
	char tmpStr[10] = {0u};

	GPS_get_current_latitude(&lati);
	GPS_get_current_longitude(&longi);
	GPS_get_current_altitude(&alti);
	GPS_get_current_speed(&speed_f);
	GPS_get_current_satellite_amount(&sat_amnt_ui);

	if(lati.qualifier == DATA_NOT_AVAILABLE || longi.qualifier == DATA_NOT_AVAILABLE){
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_serviceMessageTextID, LOW_SIG_MESSAGE, strlen(LOW_SIG_MESSAGE));

		/* Hide this daata */
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_latitudeTextID, latitude, 0u);
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_LongitudetextID, longitude, 0u);
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_AltitudetextID, altitude, 0u);
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_SpeedTextID, speed, 0u);

		sprintf(sat_amnt, "Satellites: %d", sat_amnt_ui);
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_SattelitesTextID, sat_amnt, strlen(sat_amnt));
	}else{
		print_float(tmpStr, lati.minute, 3u);
		sprintf(latitude, "%d %s' %c", lati.degree, tmpStr, lati.cardinalPoint);

		print_float(tmpStr, longi.minute, 3u);
		sprintf(longitude, "%d %s' %c", longi.degree, tmpStr, longi.cardinalPoint);

		print_float(tmpStr, alti, 1u);
		sprintf(altitude, "Altitude: %s m", tmpStr);

		print_float(tmpStr, speed_f, 1u);
		sprintf(speed, "Speed: %s km/h", tmpStr);
		sprintf(sat_amnt, "Satellites: %d", sat_amnt_ui);

		update_text_entity_text(&gpsDataFrame, gpsDataFrame_serviceMessageTextID, LOW_SIG_MESSAGE, 0u);
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_latitudeTextID, latitude, strlen(latitude));
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_LongitudetextID, longitude, strlen(longitude));
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_AltitudetextID, altitude, strlen(altitude));
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_SpeedTextID, speed, strlen(speed));
		update_text_entity_text(&gpsDataFrame, gpsDataFrame_SattelitesTextID, sat_amnt, strlen(sat_amnt));
	}
}
/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
void Geocacher_Init(void){
	/* init frames */
	convert_horizontal_bitmap(lcd_bitmap, lcd_bitmap_inv, 8u, 48u);
	convert_horizontal_bitmap(lcd_arrow, lcd_arrow_conv, 2, 24);

	LCD_set_current_frame(&startFrame);

	Init_start_frame();
	Init_main_menu_frame();
	Init_new_game_frame();
	Init_gps_data_frame();
	Init_settings_frame();

}


void Geocacher_Handler(void){
	static int cur_enc = 0u;
	static float ang = 0.0;
	static uint8_t textArray[20];
	static uint8_t isInv = 0u;
	uint8_t button_pressed_flag = FAIL;
	uint8_t encoder_changed_flag = FAIL;

	/* Pressed button means changed state */
	if(KY040_isButtonPressed() == PASS){
		button_pressed_flag = PASS;
		LED_Blink();
		KY040_buttonPressedClearFlag();
	}

	if(KY040_isEncoderValueChanged() == PASS){
		cur_enc = KY040_get_enc_current_value();
		encoder_changed_flag = PASS;
		//ang = 0.08/*0.785398*/ * cur_enc;
		//LCD_draw_rectangle(30,20,80,40, ang, 1);
		//LCD_draw_triangle(63, 31, 15, 15, ang, 1);
		//LCD_draw_arrow(48, 16, 30, 30, ang, 1);
		KY040_encoderValueChangedClearFlag();
	}

	switch(mainState){
		case START_FRAME:
			if(button_pressed_flag){
				/* Go to menu frame */
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
				KY040_set_enc_default_value(0u);
				KY040_set_enc_limits(0, 2u);
			}
			if(encoder_changed_flag){
				/* Do nothing */
			}
			break;
		case MAIN_MENU_BEGIN:
			if(button_pressed_flag){
				if(cur_enc == 0){
					/* New game */
					LCD_set_current_frame(&newGameFrame);
					mainState = NEW_GAME_FRAME;
					KY040_set_enc_default_value(0u);
					KY040_set_enc_limits(-50, 50u);
				}else if(cur_enc == 1){
					/* Gps data show */
					LCD_set_current_frame(&gpsDataFrame);
					mainState = GPS_DATA_FRAME;
				}else{
					/* Settings */
					LCD_set_current_frame(&settingsFrame);
					mainState = SETTINGS_FRAME;
				}
				main_menu_previous_pos = cur_enc;
			}
			if(encoder_changed_flag){
				update_rectangle_entity_position(&menuFrame, rectangle1ID, 0, (20 + cur_enc * 10) - 1, 127, (20 + (cur_enc + 1) * 10 ) - 2);
				if(cur_enc == 0){
					update_text_entity_inversion(&menuFrame, text1ID, PASS);
					update_text_entity_inversion(&menuFrame, text2ID, FAIL);
					update_text_entity_inversion(&menuFrame, text3ID, FAIL);
				}else if(cur_enc == 1){
					update_text_entity_inversion(&menuFrame, text1ID, FAIL);
					update_text_entity_inversion(&menuFrame, text2ID, PASS);
					update_text_entity_inversion(&menuFrame, text3ID, FAIL);
				}else{
					update_text_entity_inversion(&menuFrame, text1ID, FAIL);
					update_text_entity_inversion(&menuFrame, text2ID, FAIL);
					update_text_entity_inversion(&menuFrame, text3ID, PASS);
				}
			}
			break;
		case NEW_GAME_FRAME:
			if(button_pressed_flag){
				/* For no let's go back to main menu in any case */
				KY040_set_enc_default_value(main_menu_previous_pos);
				KY040_set_enc_limits(0, 2u);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
			}
			if(encoder_changed_flag){
				update_navigation_arrow_entity_rotation(&newGameFrame, gamePlayFrame_navigation_arrowID, cur_enc * 0.08);
			}
			break;
		case GPS_DATA_FRAME:
			handle_gps_current_data_frame();

			if(button_pressed_flag){
				/* For no let's go back to main menu in any case */
				KY040_set_enc_default_value(main_menu_previous_pos);
				KY040_set_enc_limits(0, 2u);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
			}
			break;
		case SETTINGS_FRAME:
			if(button_pressed_flag){
				/* For no let's go back to main menu in any case */
				KY040_set_enc_default_value(main_menu_previous_pos);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
			}
			break;
		default:
			/* Defensive programming */;
	}

}
/******************************************************************************/
