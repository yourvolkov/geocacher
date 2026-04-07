/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "ST7565.h"
#include "KY040.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "stdlib.h"
#include "Utils.h"
#include "NMEA.h"
#include "Geocacher.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define MAIN_STATE_TEXT    "<GEOCACHER>"
#define MENU_TEXT_1    "START NEW GAME"
#define MENU_TEXT_2    "CURRENT GPS DATA"
#define MENU_TEXT_3    "SETTINGS"

#define NEW_GAME_TEXT_1 "Enter coordinates:"
#define NEW_GAME_TEXT_2 "Latitude"
#define NEW_GAME_TEXT_3 "Longitude"

#define START_TEXT "START!"
#define OK_TEXT "OK"
#define BACK_TEXT "BACK"

#define ON_TEXT "On"
#define OFF_TEXT "OFF"

#define SETTINGS_TEXT_1 "Sound: "
#define SETTINGS_TEXT_2 "Show time: "

#define LOW_SIG_MESSAGE " LOW SIGNAL!"

#define MENU_RECTANGLE_HEIGHT	9u
#define MAIN_MENU_TEXT_START_POS	20u
#define MAIN_MENU_TEXT_GAP	10u

#define GPS_DATA_START_POS	12u

#define CURSOR_BLINK_TIME 400 // ms
#define COORDINATES_MAX_STR_LEN     (13u)
#define DELTA_DISTANCE_TO_RECALC_AZIMUTH    0.05f // 50m
#define COORDINATE_DELTA		(0.0000001) // ?
#define CURRENT_ARROW_ANGLE_OFFSET	-1.5708f//-90 deg
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	START_FRAME = 0u,
	MAIN_MENU_BEGIN,
	NEW_GAME_FRAME,
	RESUME_GAME_FRAME,
	GPS_DATA_FRAME,
	SETTINGS_FRAME,
	GAMEPLAY_FRAME

}dtMainState;

typedef enum{
	CURSOR_INACTIVE,
	CURSOR_AVTIVE_HIGHLIGHT_ON,
	CURSOR_AVTIVE_HIGHLIGHT_ON_PROCESS,
	CURSOR_AVTIVE_HIGHLIGHT_OFF,
	CURSOR_AVTIVE_HIGHLIGHT_OFF_PROCESS
}dtCursorState;


typedef struct{
	dtFrame* currFrame;
	uint16_t textObjID;
	uint16_t cursorObjID;
	uint32_t cursor_position;
	uint32_t timer;
	dtCursorState state;
}dtCursor;
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
uint8_t new_game_cursor_position = 0u;
uint8_t settings_cursor_position = 0u;

uint16_t bitmap1ID;
uint16_t start_text1ID;

uint16_t text1ID, text2ID, text3ID;
uint16_t rectangle1ID;


uint16_t newGameFrame_text1ID, newGameFrame_text2ID ,newGameFrame_text3ID,newGameFrame_text4ID, newGameFrame_text5ID,newGameFrame_text6ID, newGameFrame_text7ID, newGameFrame_text8ID;
uint16_t newGameFrame_rectangle1ID, newGameFrame_rectangle2ID, newGameFrame_rectangle3ID, newGameFrame_rectangle4ID;

uint16_t gpsDataFrame_titleTextID, gpsDataFrame_latitudeTextID, gpsDataFrame_LongitudetextID, gpsDataFrame_AltitudetextID, gpsDataFrame_SpeedTextID, gpsDataFrame_SattelitesTextID;
uint16_t gpsDataFrame_serviceMessageTextID;
uint16_t gpsDataFrame_rectangle1ID;

uint16_t settingsFrame_text1ID, settingsFrame_text2ID, settingsFrame_text3ID, settingsFrame_text4ID, settingsFrame_text5ID, settingsFrame_text6ID;
uint16_t settingsFrame_rectangle1ID, settingsFrame_rectangle2ID, settingsFrame_rectangle3ID;

uint16_t gamePlayFrame_navigation_arrowID;
uint16_t gamePlayFrame_text1ID;

uint16_t gamePlayFrame_DEBUG_text1ID, gamePlayFrame_DEBUG_text2ID;

uint8_t start_frame_bitmap_position = 45u;
uint8_t intro_frame_delay = FAIL;

char latitude[MAX_TEXT_LEN] = {0u};
char longitude[MAX_TEXT_LEN] = {0u};
char speed[MAX_TEXT_LEN] = {0u};
char sat_amnt[MAX_TEXT_LEN] = {0u};
char altitude[MAX_TEXT_LEN] = {0u};


const uint8_t lcd_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x68, 0x00, 0x00, 0x00, 0x40, 0x00, 0x6C, 0x00, 0x00, 0x01, 0xB0, 0x00, 0x82, 0x00, 0x00, 0x01, 0x88, 0x00, 0x92, 0x00, 0x00, 0x01, 0x88, 0x00, 0x92, 0x00, 0x00, 0x02, 0x48, 0x00, 0x91, 0x80, 0x00, 0x02, 0x46, 0x03, 0x11, 0x80, 0x00, 0x02, 0x46, 0x03, 0x11, 0x80, 0x00, 0x02, 0x77, 0xFF, 0x1C, 0x40, 0x00, 0x02, 0x7F, 0xFF, 0xFD, 0xC0, 0x00, 0x02, 0x7F, 0xFF, 0xFD, 0xC0, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x40, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x20, 0x00, 0x02, 0x7F, 0xFF, 0xFE, 0x30, 0x00, 0x03, 0xF8, 0xFF, 0xFF, 0xB0, 0x00, 0x03, 0xF0, 0x38, 0xFF, 0xF0, 0x00, 0x03, 0xF0, 0x38, 0xFF, 0xF0, 0x00, 0x03, 0xF0, 0x38, 0x7F, 0xF0, 0x00, 0x07, 0xE0, 0x38, 0x3F, 0xC8, 0x00, 0x0F, 0xC0, 0x38, 0x1F, 0xC8, 0x00, 0x0D, 0xCE, 0x38, 0xEF, 0x88, 0x00, 0x0C, 0xC7, 0x3B, 0x80, 0x04, 0x00, 0x0C, 0x47, 0x3B, 0x80, 0x06, 0x00, 0x10, 0x07, 0x3B, 0x80, 0x06, 0x00, 0x10, 0x00, 0x30, 0x00, 0x07, 0x00, 0x10, 0x00, 0x20, 0x00, 0x07, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0xF8, 0x00, 0x01, 0x00, 0x10, 0x00, 0xF8, 0x00, 0x01, 0x00, 0x1C, 0x01, 0xFC, 0x00, 0x07, 0x00, 0x0C, 0x08, 0xF8, 0x60, 0x09, 0x00, 0x0C, 0x08, 0x78, 0x60, 0x09, 0x00, 0x0E, 0x06, 0x20, 0xE0, 0x31, 0x00, 0x0F, 0x86, 0x23, 0x80, 0x41, 0x00, 0x07, 0x82, 0x33, 0x80, 0x41, 0x00, 0x03, 0xC1, 0xFC, 0x80, 0x06, 0x00, 0x02, 0x70, 0xDB, 0x01, 0x86, 0x00, 0x02, 0x70, 0xDB, 0x01, 0x06, 0x00, 0x0E, 0x70, 0xDB, 0x00, 0x06, 0x00, 0x0E, 0x30, 0xC4, 0x00, 0x01, 0x00, 0x0E, 0x30, 0x44, 0x00, 0x01, 0x00, 0x0E, 0x30, 0x38, 0x00, 0x01, 0x00, 0x13, 0xB8, 0x00, 0x00, 0x01, 0x00, 0x13, 0x9C, 0x00, 0x00, 0x01, 0x00, 0x13, 0x8E, 0x00, 0x00, 0x01, 0x00, 0x13, 0x87, 0x00, 0x00, 0x01, 0x00, 0x13, 0x83, 0x80, 0x00, 0x01, 0x00, 0x12, 0x41, 0xC0, 0x00, 0x01, 0x00, 0x12, 0x30, 0x20, 0x00, 0x01, 0x00, 0x12, 0x10, 0x20, 0x00, 0x01, 0x00, 0x12, 0x08, 0x18, 0x00, 0x00, 0x00, 0x11, 0x80, 0x00, 0x00, 0x00, 0x00, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x40, 0x00, 0x00, 0x00, 0x00, 0x04, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const double PI = 3.141592653;
const float earth_rad = 6371.f; /* according to WGS84 */

uint8_t lcd_bitmap_inv[8][48];

uint8_t geocaching_latitude[12] = "47 29.184 N";//"00 00.000 N";
uint8_t geocaching_longitude[13] = "019 02.845 E";//"000 00.000 E";
uint8_t distance_text[10u];
dtCursor cursor;
uint32_t animation_timer = 2000u; // 2sec

uint8_t sound_option_status = FAIL;
uint8_t time_option_status = FAIL;


double starting_latitude = 0u;
double starting_longitude = 0u;

double current_latitude = 0u;
double current_longitude = 0u;

double geochache_latitude = 0u;
double geochache_longitude = 0u;

double main_distance = 0u;
double main_azimuth_rad = 0u;

__attribute__((section(".fee_sec"))) const uint32_t my_data[100] = {0xAA};

uint8_t debug_text_1[10] = "0";
uint8_t debug_text_2[10] = "0";
/******************************************************************************/
/****************************** Externs ***************************************/
/******************************************************************************/

/******************************************************************************/
/********************** Private functions prototypes **************************/
/******************************************************************************/
double distance_calc_test(void);
void handle_new_game_start(void);
void handle_coordinates_while_playing(void);
/******************************************************************************/
/********************** Platform dependent functions **************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Private functions ********************************/
/******************************************************************************/
double convert_coordinates_to_rad(uint8_t coordinate_degree, float coordinate_minute){
	double coordinate = (double)coordinate_degree + (coordinate_minute / 60.f);
	double result = coordinate * (PI / 180.f);

	return result;
}
/*----------------------------------------------------------------------------*/

float calculate_azimuth_between_current_position_and_object(double current_latitude_rad, double current_longitude_rad, double object_latitude_rad, double object_longitude_rad){
	double cos_long = cos(object_longitude_rad - current_longitude_rad);
	double sin_long = sin(object_longitude_rad - current_longitude_rad);
	double x = sin_long * cos(object_latitude_rad);
	double y = (cos(current_latitude_rad) * sin(object_latitude_rad)) - (sin(current_latitude_rad) * cos(object_latitude_rad) * cos_long);

	double azimuth = atan2(x, y);
	float azimuth_deg = azimuth * (180.f / PI);

	return azimuth_deg;
}
/*----------------------------------------------------------------------------*/

double calculate_distance_between_current_position_and_object(double current_latitude_rad, double current_longitude_rad, double object_latitude_rad, double object_longitude_rad){
	const float earth_rad = 6371.f; /* according to WGS84 */
	double sin_lat = sin((object_latitude_rad - current_latitude_rad) / 2.0);
	double sin_long = sin((object_longitude_rad - current_longitude_rad) / 2.0);
	double asin_arg = sqrt((sin_lat * sin_lat) + (cos(current_latitude_rad) * cos(object_latitude_rad) * sin_long * sin_long));
	double distance_km = 2 * earth_rad * asin(asin_arg);

	return distance_km;
}

/*----------------------------------------------------------------------------*/
double convert_text_coordinates_to_double(char* text_coordinates){
	/* example string format: '00 00.000 N' */
	char tmp[COORDINATES_MAX_STR_LEN];
	uint8_t i, pos, field_cnt, coordinate_degree;
	float coordinate_minute;
	double result_coordinate_rad;

	field_cnt = 0u;
	for(i = 0, pos = 0; i < strlen(text_coordinates) && i < COORDINATES_MAX_STR_LEN; i++){
		if(text_coordinates[i] == ' '){
			tmp[pos] = '\0';
			if(field_cnt == 0u){
				/* degrees */
				coordinate_degree = (uint8_t)_atoi(tmp);
			}else if(field_cnt == 1u){
				/* minutes */
				coordinate_minute = _atof(tmp);
			}else{
				/* unexpected */
				break;
			}
			field_cnt++;
			pos = 0;
		}else{
			tmp[pos++] = text_coordinates[i];
		}
	}

	result_coordinate_rad = (double)coordinate_degree + (coordinate_minute / 60.f);
	result_coordinate_rad *= (PI / 180.f);

	return result_coordinate_rad;

}
/*----------------------------------------------------------------------------*/
uint8_t center_text(uint8_t len_str){
	return (127u - len_str * 6) / 2;
}

uint8_t get_end_pos_text(uint8_t x, uint8_t len_str){
	return x + (len_str * 6) + 2u;
}

/*----------------------------------------------------------------------------*/
void Init_start_frame(void){
	bitmap1ID = add_bitmap_entity_to_frame(&startFrame, 45u, 0u, 48, 64, lcd_bitmap_inv);
	start_text1ID = add_text_entity_to_frame(&startFrame, 60u, 31u, MAIN_STATE_TEXT, 0, FAIL, 0u);
}
/*----------------------------------------------------------------------------*/
void Init_main_menu_frame(void){
	rectangle1ID = add_rectangle_entity_to_frame(&menuFrame, 0, 19, 127, 28, 0.0, PASS, FAIL);

	text1ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_1)),
										MAIN_MENU_TEXT_START_POS,
										MENU_TEXT_1,
										strlen(MENU_TEXT_1),
										PASS,
										0u);
	text2ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_2)),
										MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP,
										MENU_TEXT_2,
										strlen(MENU_TEXT_2),
										FAIL,
										0u);
	text3ID = add_text_entity_to_frame(&menuFrame,
										center_text(strlen(MENU_TEXT_3)),
										MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP * 2,
										MENU_TEXT_3,
										strlen(MENU_TEXT_3),
										FAIL,
										0u);
}

/*----------------------------------------------------------------------------*/
void Init_new_game_frame(void){
	newGameFrame_rectangle1ID = add_rectangle_entity_to_frame(&newGameFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS, FAIL);
	newGameFrame_rectangle2ID = add_rectangle_entity_to_frame(&newGameFrame, 51, 26, 57, 36, 0.0, PASS, FAIL);
	newGameFrame_rectangle3ID = add_rectangle_entity_to_frame(&newGameFrame, 15, 51, 58 , 63, 0.0, FAIL, FAIL);
	newGameFrame_rectangle4ID = add_rectangle_entity_to_frame(&newGameFrame, 68, 51, 112, 63, 0.0, FAIL, FAIL);

	newGameFrame_text1ID = add_text_entity_to_frame(&newGameFrame,
													center_text(strlen(MENU_TEXT_1)),
													1u,
													MENU_TEXT_1,
													strlen(MENU_TEXT_1),
													PASS,
													0u);

	newGameFrame_text2ID = add_text_entity_to_frame(&newGameFrame,
													center_text(strlen(NEW_GAME_TEXT_1)),
													16u,
													NEW_GAME_TEXT_1,
													strlen(NEW_GAME_TEXT_1),
													FAIL,
													0u);

	newGameFrame_text3ID = add_text_entity_to_frame(&newGameFrame,
													0u,
													28u,
													NEW_GAME_TEXT_2,
													strlen(NEW_GAME_TEXT_2),
													FAIL,
													0u);

	newGameFrame_text4ID = add_text_entity_to_frame(&newGameFrame,
													0u,
													38u,
													NEW_GAME_TEXT_3,
													strlen(NEW_GAME_TEXT_3),
													FAIL,
													0u);

	newGameFrame_text5ID = add_text_entity_to_frame(&newGameFrame,
													52u,
													28u,
													geocaching_latitude,
													strlen(geocaching_latitude),
													PASS,
													1u);

	newGameFrame_text6ID = add_text_entity_to_frame(&newGameFrame,
													57u,
													38u,
													geocaching_longitude,
													strlen(geocaching_longitude),
													FAIL,
													0u);

	newGameFrame_text7ID = add_text_entity_to_frame(&newGameFrame,
													20u,
													54u,
													START_TEXT,
													strlen(START_TEXT),
													FAIL,
													0u);

	newGameFrame_text8ID = add_text_entity_to_frame(&newGameFrame,
													78u,
													54u,
													BACK_TEXT,
													strlen(BACK_TEXT),
													FAIL,
													0u);

}

/*----------------------------------------------------------------------------*/
void Init_gps_data_frame(void){
	gpsDataFrame_rectangle1ID = add_rectangle_entity_to_frame(&gpsDataFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS, FAIL);

	gpsDataFrame_titleTextID = add_text_entity_to_frame(&gpsDataFrame,
													center_text(strlen(MENU_TEXT_2)),
													1u,
													MENU_TEXT_2,
													strlen(MENU_TEXT_2),
													PASS,
													0u);

	gpsDataFrame_latitudeTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS,
													latitude,
													0u,
													FAIL,
													0u);

	gpsDataFrame_LongitudetextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP,
													longitude,
													0u,
													FAIL,
													0u);

	gpsDataFrame_AltitudetextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 2,
													altitude,
													0u,
													FAIL,
													0u);

	gpsDataFrame_SpeedTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 3,
													speed,
													0u,
													FAIL,
													0u);

	gpsDataFrame_SattelitesTextID = add_text_entity_to_frame(&gpsDataFrame,
													0u,
													GPS_DATA_START_POS + MAIN_MENU_TEXT_GAP * 4,
													sat_amnt,
													0u,
													FAIL,
													0u);

	gpsDataFrame_serviceMessageTextID = add_text_entity_to_frame(&gpsDataFrame,
																center_text(strlen(LOW_SIG_MESSAGE)),
																28u,
																LOW_SIG_MESSAGE,
																strlen(LOW_SIG_MESSAGE),
																PASS,
																0u);

}

/*----------------------------------------------------------------------------*/
void Init_settings_frame(void){
	settingsFrame_rectangle1ID = add_rectangle_entity_to_frame(&settingsFrame, 0, 0, 127, MENU_RECTANGLE_HEIGHT, 0.0, PASS, FAIL);
	settingsFrame_rectangle2ID = add_rectangle_entity_to_frame(&settingsFrame, 68u, MAIN_MENU_TEXT_START_POS - 2u, 88u, MAIN_MENU_TEXT_START_POS + 7u, 0.0, PASS, FAIL);
	settingsFrame_rectangle3ID = add_rectangle_entity_to_frame(&settingsFrame, 43u, 51u, 83u, 63u, 0.0, FAIL, FAIL);
	settingsFrame_text1ID = add_text_entity_to_frame(&settingsFrame,
													center_text(strlen(MENU_TEXT_3)),
													1u,
													MENU_TEXT_3,
													strlen(MENU_TEXT_3),
													PASS,
													0u);

	settingsFrame_text2ID = add_text_entity_to_frame(&settingsFrame,
													5u,
													MAIN_MENU_TEXT_START_POS,
													SETTINGS_TEXT_1,
													strlen(SETTINGS_TEXT_1),
													FAIL,
													0u);

	settingsFrame_text3ID = add_text_entity_to_frame(&settingsFrame,
													5u,
													MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP,
													SETTINGS_TEXT_2,
													strlen(SETTINGS_TEXT_2),
													FAIL,
													0u);

	settingsFrame_text4ID = add_text_entity_to_frame(&settingsFrame,
													70u,
													MAIN_MENU_TEXT_START_POS,
													OFF_TEXT,
													strlen(OFF_TEXT),
													PASS,
													0u);

	settingsFrame_text5ID = add_text_entity_to_frame(&settingsFrame,
													70u,
													MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP,
													OFF_TEXT,
													strlen(OFF_TEXT),
													FAIL,
													0u);

	settingsFrame_text6ID = add_text_entity_to_frame(&settingsFrame,
													center_text(strlen(BACK_TEXT)),
													54u,
													BACK_TEXT,
													strlen(BACK_TEXT),
													FAIL,
													0u);

}

/*----------------------------------------------------------------------------*/
void Init_gameplay_frame(void){

	gamePlayFrame_navigation_arrowID = add_navigation_arrow_entity_to_frame(&gamePlayFrame, 42, 15, 40, 26, 0.0);
	gamePlayFrame_text1ID = add_text_entity_to_frame(&gamePlayFrame, 0u, 54u, NULL, 0u, FAIL, 0u);

	/* Debugging azimuth */
	gamePlayFrame_DEBUG_text1ID = add_text_entity_to_frame(&gamePlayFrame, 0u, 30u, debug_text_1, strlen(debug_text_1), FAIL, 0u);
	gamePlayFrame_DEBUG_text2ID = add_text_entity_to_frame(&gamePlayFrame, 0u, 40u, debug_text_2, strlen(debug_text_2), FAIL, 0u);
}

/*----------------------------------------------------------------------------*/
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
/*----------------------------------------------------------------------------*/
void handle_encoder_in_start_game_frame(uint8_t current_enc){
	if(current_enc >= 0u && current_enc < 2u){
		/* Latitude degrees */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, PASS, (1u << current_enc));
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 51 + ((current_enc * 6)), 26, 57 + current_enc * 6, 36);
	}else if(current_enc >= 2u && current_enc < 4u){
		/* Latitude minutes */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, PASS, (1u << (current_enc + 1))); // skip space
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 51 + (((current_enc + 1) * 6)), 26, 57 + (current_enc + 1) * 6, 36);
	}else if(current_enc >= 4u && current_enc < 7u){
		/* Latitude fract */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, PASS, (1u << (current_enc + 2))); // skip space and a point
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 51 + (((current_enc + 2) * 6)), 26, 57 + (current_enc + 2) * 6, 36);
	}else if(current_enc == 7u){
		/* Latitude cardinal point */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, PASS, (1u << (current_enc + 3))); // skip space and a point and another space
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 51 + (((current_enc + 3) * 6)), 26, 57 + (current_enc + 3) * 6, 36);
	}else if(current_enc >= 8u && current_enc < 11u){
		/* Longitude degrees */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, PASS, (1u << (current_enc - 8u))); // skip latitude
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 57 + (((current_enc - 8u) * 6)), 36, 62 + (current_enc - 8u) * 6, 46);
	}else if(current_enc >= 11u && current_enc < 13u){
		/* Longitude minutes */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, PASS, (1u << (current_enc - 8u + 1))); // skip latitude + skip space
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 57 + (((current_enc - 8u + 1u) * 6)), 36, 62 + (current_enc - 8u + 1u) * 6, 46);
	}else if(current_enc >= 13u && current_enc < 16u){
		/* Longitude fract */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, PASS, (1u << (current_enc - 8u + 2))); // skip latitude + skip space + point
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 57 + (((current_enc - 8u + 2u) * 6)), 36, 62 + (current_enc - 8u + 2u) * 6, 46);
	}else if(current_enc == 16u){
		/* Longitude cardinal point */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, PASS, (1u << (current_enc - 8u + 3))); // skip latitude + skip space + point + another space
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 57 + (((current_enc - 8u + 3u) * 6)), 36, 62 + (current_enc - 8u + 3u) * 6, 46);

	}else if(current_enc == 17u){
		/* Start button */
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle3ID, PASS);
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle4ID, FAIL);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text7ID, PASS, 0u);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text8ID, FAIL, 0u);
	}else if(current_enc == 18u){
		/* Back button */
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle3ID, FAIL);
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle4ID, PASS);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text7ID, FAIL, 0u);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text8ID, PASS, 0u);
	}else{
		/* Defensive programming */
	}


	/* Clearing objects focus */
	if(current_enc < 8u){
		/* Clear focus from buttons */
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle3ID, FAIL);
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle4ID, FAIL);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text7ID, FAIL, 0u);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text8ID, FAIL, 0u);
		/* Turn off inversion on longitude */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, FAIL, 0u);
	}else if(current_enc >= 8u && current_enc < 17u){
		/* Clear focus from buttons */
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle3ID, FAIL);
		update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle4ID, FAIL);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text7ID, FAIL, 0u);
		update_text_entity_inversion(&newGameFrame, newGameFrame_text8ID, FAIL, 0u);
		/* Turn off inversion on latitude */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, FAIL, 0u);
	}else{
		/* Turn off inversion on latitude */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, FAIL, 0u);
		/* Turn off inversion on longitude */
		update_text_entity_inversion(&newGameFrame, newGameFrame_text6ID, FAIL, 0u);
		/* Turn off cursor */
		update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 0, 0, 0, 0);
	}
}

void handle_encoder_in_start_game_frame_when_cursor_is_active(uint8_t current_enc){

	if(new_game_cursor_position < 2u){
		/* Latitude degrees */
		geocaching_latitude[new_game_cursor_position] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text5ID, geocaching_latitude, strlen(geocaching_latitude));

	}else if(new_game_cursor_position >= 2u && new_game_cursor_position < 4u){
		/* Latitude minutes */
		geocaching_latitude[new_game_cursor_position + 1u] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text5ID, geocaching_latitude, strlen(geocaching_latitude));

	}else if(new_game_cursor_position >= 4u && new_game_cursor_position < 7u){
		/* Latitude fract */
		geocaching_latitude[new_game_cursor_position + 2u] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text5ID, geocaching_latitude, strlen(geocaching_latitude));
	}else if(new_game_cursor_position == 7u){
		/* Latitude cardinal point */
		if(current_enc == 0){
			geocaching_latitude[new_game_cursor_position + 3u] = 'N';
		}else{
			geocaching_latitude[new_game_cursor_position + 3u] = 'S';
		}
		update_text_entity_text(&newGameFrame, newGameFrame_text5ID, geocaching_latitude, strlen(geocaching_latitude));
	}else if(new_game_cursor_position >= 8u && new_game_cursor_position < 11u){
		/* Longitude degrees */
		geocaching_longitude[new_game_cursor_position - 8u] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text6ID, geocaching_longitude, strlen(geocaching_longitude));
	}else if(new_game_cursor_position >= 11u && new_game_cursor_position < 13u){
		/* Longitude minutes */
		geocaching_longitude[new_game_cursor_position - 8u + 1u] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text6ID, geocaching_longitude, strlen(geocaching_longitude));
	}else if(new_game_cursor_position >= 13u && new_game_cursor_position < 16u){
		/* Longitude fract */
		geocaching_longitude[new_game_cursor_position - 8 + 2u] = '0' + current_enc;
		update_text_entity_text(&newGameFrame, newGameFrame_text6ID, geocaching_longitude, strlen(geocaching_longitude));
	}else if(new_game_cursor_position == 16u){
		/* Longitude cardinal point */
		if(current_enc == 0){
			geocaching_longitude[new_game_cursor_position - 8 + 3u] = 'E';
		}else{
			geocaching_longitude[new_game_cursor_position - 8 + 3u] = 'W';
		}
		update_text_entity_text(&newGameFrame, newGameFrame_text6ID, geocaching_longitude, strlen(geocaching_longitude));
	}else{
		/* Defensive programming */
	}
}
/*----------------------------------------------------------------------------*/
void set_cursor_active(dtFrame* frame, uint16_t text_obj_id, uint16_t cursor_obj_id, uint32_t cursor_position){
	cursor.currFrame = frame;
	cursor.textObjID = text_obj_id;
	cursor.cursorObjID = cursor_obj_id;
	cursor.cursor_position = cursor_position;
	cursor.state = CURSOR_AVTIVE_HIGHLIGHT_ON;
	cursor.timer = CURSOR_BLINK_TIME;
}
/*----------------------------------------------------------------------------*/
void set_cursor_inactive(void){
	cursor.state = CURSOR_INACTIVE;
	update_rectangle_entity_inversion(cursor.currFrame, cursor.cursorObjID, FAIL);
	update_text_entity_inversion(cursor.currFrame, cursor.textObjID, PASS, (1u << cursor.cursor_position));
}

/*----------------------------------------------------------------------------*/
void Handle_cursor(void){

	switch(cursor.state){
		case CURSOR_INACTIVE:
			/* do nothing */
			break;
		case CURSOR_AVTIVE_HIGHLIGHT_ON:
			/* Turn on highlight and set a timer */
			update_rectangle_entity_inversion(cursor.currFrame, cursor.cursorObjID, FAIL);
			update_text_entity_inversion(cursor.currFrame, cursor.textObjID, PASS, cursor.cursor_position != UINT32_MAX ? (1u << cursor.cursor_position) : 0u);
			cursor.timer = CURSOR_BLINK_TIME;
			cursor.state = CURSOR_AVTIVE_HIGHLIGHT_ON_PROCESS;
			break;
		case CURSOR_AVTIVE_HIGHLIGHT_ON_PROCESS:
			if(!cursor.timer){
				cursor.state = CURSOR_AVTIVE_HIGHLIGHT_OFF;
			}
			break;
		case CURSOR_AVTIVE_HIGHLIGHT_OFF:
			/* Turn off highlight and set a timer */
			update_rectangle_entity_inversion(cursor.currFrame, cursor.cursorObjID, PASS);
			update_text_entity_inversion(cursor.currFrame, cursor.textObjID, FAIL, 0u);
			cursor.timer = CURSOR_BLINK_TIME;
			cursor.state = CURSOR_AVTIVE_HIGHLIGHT_OFF_PROCESS;
			break;
		case CURSOR_AVTIVE_HIGHLIGHT_OFF_PROCESS:
			if(!cursor.timer){
				cursor.state = CURSOR_AVTIVE_HIGHLIGHT_ON;
			}
			break;
	}

}
/*----------------------------------------------------------------------------*/
void handle_button_in_start_game_frame(uint8_t current_enc){

	if(cursor.state == CURSOR_INACTIVE){
		if(current_enc < 2u){
			/* Latitude degrees */

			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text5ID, newGameFrame_rectangle2ID, current_enc);
			/* Set new boundaries for encoder */
			KY040_set_enc_default_value(geocaching_latitude[current_enc] - '0');
			if(current_enc == 0u){
				KY040_set_enc_limits(0, 8u);
			}else if(current_enc == 1u){
				KY040_set_enc_limits(0, 9u);
			}

		}else if(current_enc >= 2u && current_enc < 4u){
			/* Latitude minutes */
			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text5ID, newGameFrame_rectangle2ID, current_enc + 1u);
			/* Set new boundaries for encoder */
			KY040_set_enc_default_value(geocaching_latitude[current_enc + 1u] - '0');
			if(current_enc == 2u){
				KY040_set_enc_limits(0, 6u);
			}else if(current_enc == 3u){
				KY040_set_enc_limits(0, 9u);
			}
		}else if(current_enc >= 4u && current_enc < 7u){
			/* Latitude fract */
			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text5ID, newGameFrame_rectangle2ID, current_enc + 2u);
			/* Set new boundaries for encoder */
			KY040_set_enc_default_value(geocaching_latitude[current_enc + 2u] - '0');
			KY040_set_enc_limits(0, 9u);

		}else if(current_enc == 7u){
			/* Latitude cardinal point */
			set_cursor_active(&newGameFrame, newGameFrame_text5ID, newGameFrame_rectangle2ID, current_enc + 3u);
			/* Set new boundaries for encoder */
			if(geocaching_latitude[current_enc + 3u] == 'N'){
				KY040_set_enc_default_value(0u);
			}else{
				KY040_set_enc_default_value(1u);
			}
			KY040_set_enc_limits(0u, 1u);
		}else if(current_enc >= 8u && current_enc < 11u){
			/* Longitude degrees */
			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text6ID, newGameFrame_rectangle2ID, current_enc - 8u);
			/* Set new boundaries for encoder */
			if(current_enc == 8u){
				KY040_set_enc_limits(0, 1u);
			}else if(current_enc == 9u){
				if(geocaching_longitude[0] == '1'){
					KY040_set_enc_limits(0, 8u);
				}else{
					KY040_set_enc_limits(0, 9u);
				}
			}else if(current_enc == 10u){
				if(geocaching_longitude[0] == '1' && geocaching_longitude[1] == '8'){
					KY040_set_enc_limits(0, 0u);
				}else{
					KY040_set_enc_limits(0, 9u);
				}
			}
			KY040_set_enc_default_value(geocaching_longitude[current_enc - 8u] - '0');
		}else if(current_enc >= 11u && current_enc < 13u){
			/* Longitude minutes */
			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text6ID, newGameFrame_rectangle2ID, current_enc - 8u + 1u);
			/* Set new boundaries for encoder */
			KY040_set_enc_default_value(geocaching_longitude[current_enc - 8u + 1u] - '0');
			if(current_enc == 11u){
				KY040_set_enc_limits(0, 6u);
			}else if(current_enc == 12u){
				KY040_set_enc_limits(0, 9u);
			}
		}else if(current_enc >= 13u && current_enc < 16u){
			/* Longitude fract */
			/* Activate cursor */
			set_cursor_active(&newGameFrame, newGameFrame_text6ID, newGameFrame_rectangle2ID, current_enc - 8u + 2u);
			/* Set new boundaries for encoder */
			KY040_set_enc_default_value(geocaching_longitude[current_enc - 8u + 2u] - '0');
			KY040_set_enc_limits(0, 9u);
		}else if(current_enc == 16u){
			/* Longitude cardinal point */
			set_cursor_active(&newGameFrame, newGameFrame_text6ID, newGameFrame_rectangle2ID, current_enc - 8u + 3u);
			/* Set new boundaries for encoder */
			if(geocaching_longitude[current_enc - 8u + 3u] == 'E'){
				KY040_set_enc_default_value(0u);
			}else{
				KY040_set_enc_default_value(1u);
			}
			KY040_set_enc_limits(0u, 1u);
		}else if(current_enc == 17u){
			/* Start button */
//			KY040_set_enc_default_value(-10);
//			KY040_set_enc_limits(-50, 50u);
			LCD_set_current_frame(&gamePlayFrame);
			mainState = GAMEPLAY_FRAME;
			handle_new_game_start();
		}else if(current_enc == 18u){
			/* Back button */
			/* let's go back to main menu in any case */
				KY040_set_enc_default_value(main_menu_previous_pos);
				KY040_set_enc_limits(0, 2u);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
		}else{
			/* Defensive programming */
		}

		/* Save cursor position */
		new_game_cursor_position = current_enc;
	}else{
		/* Cursor was active */
		set_cursor_inactive();
		KY040_set_enc_limits(0, 18u);
		KY040_set_enc_default_value(new_game_cursor_position);

	}
}

/*----------------------------------------------------------------------------*/
void handle_button_in_settings_frame(uint8_t current_enc){
	if(cursor.state == CURSOR_INACTIVE){
		if(current_enc == 0u){
			/* Sound settings */
			/* Activate cursor */
			set_cursor_active(&settingsFrame, settingsFrame_text4ID, settingsFrame_rectangle2ID, UINT32_MAX);
			KY040_set_enc_limits(0, 1u);
			if(sound_option_status){
				KY040_set_enc_default_value(1u);
			}else{
				KY040_set_enc_default_value(0u);
			}

		}else if(current_enc == 1u){
			/* Timer settings */
			set_cursor_active(&settingsFrame, settingsFrame_text5ID, settingsFrame_rectangle2ID, UINT32_MAX);
			KY040_set_enc_limits(0, 1u);
			if(time_option_status){
				KY040_set_enc_default_value(1u);
			}else{
				KY040_set_enc_default_value(0u);
			}
		}else if(current_enc == 2u){
			/* let's go back to main menu  */
			KY040_set_enc_default_value(main_menu_previous_pos);
			KY040_set_enc_limits(0, 2u);
			LCD_set_current_frame(&menuFrame);
			mainState = MAIN_MENU_BEGIN;
		}

		/* Save cursor position */
		settings_cursor_position = current_enc;
	}else{
		/* Cursor was active */
		set_cursor_inactive();
		KY040_set_enc_limits(0, 2u);
		KY040_set_enc_default_value(settings_cursor_position);

	}
}

/*----------------------------------------------------------------------------*/
void handle_encoder_in_settings_frame_when_cursor_is_active(uint8_t current_enc){
	if(settings_cursor_position == 0u){
		/* Sound settings */
		if(current_enc == 0u){
			/* OFF */
			sound_option_status = FAIL;
			update_text_entity_text(&settingsFrame, settingsFrame_text4ID, OFF_TEXT, strlen(OFF_TEXT));
		}else{
			/* ON */
			sound_option_status = PASS;
			update_text_entity_text(&settingsFrame, settingsFrame_text4ID, ON_TEXT, strlen(ON_TEXT));
		}

	}else if(settings_cursor_position == 1u){
		/* Timer settings */
		if(current_enc == 0u){
			/* OFF */
			time_option_status = FAIL;
			update_text_entity_text(&settingsFrame, settingsFrame_text5ID, OFF_TEXT, strlen(OFF_TEXT));
		}else{
			/* ON */
			time_option_status = PASS;
			update_text_entity_text(&settingsFrame, settingsFrame_text5ID, ON_TEXT, strlen(ON_TEXT));
		}
	}
}

/*----------------------------------------------------------------------------*/
void handle_encoder_in_settings_frame(uint8_t current_enc){
	if(current_enc == 0u){
		/* Sound settings */
		update_text_entity_inversion(&settingsFrame, settingsFrame_text4ID, PASS, UINT32_MAX);
		update_rectangle_entity_position(&settingsFrame, settingsFrame_rectangle2ID, 68u, MAIN_MENU_TEXT_START_POS - 2u, 88u, MAIN_MENU_TEXT_START_POS + 7u);

		update_text_entity_inversion(&settingsFrame, settingsFrame_text5ID, FAIL, 0u);
		update_text_entity_inversion(&settingsFrame, settingsFrame_text6ID, FAIL, 0u);
		update_rectangle_entity_filled(&settingsFrame, settingsFrame_rectangle3ID, FAIL);
	}else if(current_enc == 1u){
		/* Timer settings */
		update_text_entity_inversion(&settingsFrame, settingsFrame_text5ID, PASS, UINT32_MAX);
		update_rectangle_entity_position(&settingsFrame, settingsFrame_rectangle2ID, 68u, MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP - 2u, 88u, MAIN_MENU_TEXT_START_POS + MAIN_MENU_TEXT_GAP + 7u);

		update_text_entity_inversion(&settingsFrame, settingsFrame_text4ID, FAIL, 0u);
		update_text_entity_inversion(&settingsFrame, settingsFrame_text6ID, FAIL, 0u);
		update_rectangle_entity_filled(&settingsFrame, settingsFrame_rectangle3ID, FAIL);
	}else if(current_enc == 2u){
		/* Back */
		update_rectangle_entity_filled(&settingsFrame, settingsFrame_rectangle3ID, PASS);
		update_text_entity_inversion(&settingsFrame, settingsFrame_text4ID, FAIL, 0u);
		update_text_entity_inversion(&settingsFrame, settingsFrame_text5ID, FAIL, 0u);
		update_text_entity_inversion(&settingsFrame, settingsFrame_text6ID, PASS, 0u);
		/* Turn off cursor */
		update_rectangle_entity_position(&settingsFrame, settingsFrame_rectangle2ID, 0, 0, 0, 0);
	}
}
/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
void Geocacher_Init(void){
	/* init frames */
	cursor.currFrame = NULL;
	cursor.timer = 0u;
	cursor.state = CURSOR_INACTIVE;

	convert_horizontal_bitmap(lcd_bitmap, lcd_bitmap_inv, 8u, 48u);
	LCD_set_current_frame(&startFrame);

	Init_start_frame();
	Init_main_menu_frame();
	Init_new_game_frame();
	Init_gps_data_frame();
	Init_settings_frame();
	Init_gameplay_frame();

	//cursor.state = my_data[0];
}

/*----------------------------------------------------------------------------*/
void Geocacher_Handler(void){
	static int cur_enc = 0u;

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
		KY040_encoderValueChangedClearFlag();
	}

	switch(mainState){
		case START_FRAME:
			if(button_pressed_flag || (intro_frame_delay && !animation_timer)){
				/* Go to menu frame */
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
				KY040_set_enc_default_value(0u);
				KY040_set_enc_limits(0, 2u);
			}
			if(encoder_changed_flag){
				/* Do nothing */
			}
			if(!animation_timer && start_frame_bitmap_position){
				/* Go to menu frame after 3 sec */
				start_frame_bitmap_position -= 5;
				update_bitmap_entity_position(&startFrame, bitmap1ID, start_frame_bitmap_position, 0u);
				animation_timer = 100;
			}else if(!start_frame_bitmap_position && !intro_frame_delay){
				update_text_entity_text(&startFrame, start_text1ID, MAIN_STATE_TEXT, sizeof(MAIN_STATE_TEXT));
				animation_timer = 2000;
				intro_frame_delay = PASS;
			}
			break;
		case MAIN_MENU_BEGIN:
			if(button_pressed_flag){
				if(cur_enc == 0){
					/* New game */
					LCD_set_current_frame(&newGameFrame);
					mainState = NEW_GAME_FRAME;
					KY040_set_enc_default_value(0u);
					KY040_set_enc_limits(0, 18u);

					/* Put the cursor at the first symbol of latitude, turn off START and BACK buttons */
					update_rectangle_entity_position(&newGameFrame, newGameFrame_rectangle2ID, 51, 26, 57, 36);
					update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle3ID, FAIL);
					update_rectangle_entity_filled(&newGameFrame, newGameFrame_rectangle4ID, FAIL);
					update_text_entity_inversion(&newGameFrame, newGameFrame_text5ID, PASS, 1u);
					update_text_entity_inversion(&newGameFrame, newGameFrame_text7ID, FAIL, 0u);
					update_text_entity_inversion(&newGameFrame, newGameFrame_text8ID, FAIL, 0u);
				}else if(cur_enc == 1){
					/* Gps data show */
					LCD_set_current_frame(&gpsDataFrame);
					mainState = GPS_DATA_FRAME;
				}else{
					/* Settings */
					LCD_set_current_frame(&settingsFrame);
					mainState = SETTINGS_FRAME;
					KY040_set_enc_default_value(0u);
					KY040_set_enc_limits(0, 2u);

				}
				main_menu_previous_pos = cur_enc;
			}
			if(encoder_changed_flag){
				update_rectangle_entity_position(&menuFrame, rectangle1ID, 0, (20 + cur_enc * 10) - 1, 127, (20 + (cur_enc + 1) * 10 ) - 2);
				if(cur_enc == 0){
					update_text_entity_inversion(&menuFrame, text1ID, PASS, 0u);
					update_text_entity_inversion(&menuFrame, text2ID, FAIL, 0u);
					update_text_entity_inversion(&menuFrame, text3ID, FAIL, 0u);
				}else if(cur_enc == 1){
					update_text_entity_inversion(&menuFrame, text1ID, FAIL, 0u);
					update_text_entity_inversion(&menuFrame, text2ID, PASS, 0u);
					update_text_entity_inversion(&menuFrame, text3ID, FAIL, 0u);
				}else{
					update_text_entity_inversion(&menuFrame, text1ID, FAIL, 0u);
					update_text_entity_inversion(&menuFrame, text2ID, FAIL, 0u);
					update_text_entity_inversion(&menuFrame, text3ID, PASS, 0u);
				}
			}
			break;
		case NEW_GAME_FRAME:
			if(button_pressed_flag){
				handle_button_in_start_game_frame(cur_enc);
			}
			if(encoder_changed_flag){
				if(cursor.state == CURSOR_INACTIVE){
					handle_encoder_in_start_game_frame(cur_enc);
				}else{
					handle_encoder_in_start_game_frame_when_cursor_is_active(cur_enc);
				}
			}
			break;
		case GPS_DATA_FRAME:
			handle_gps_current_data_frame();

			if(button_pressed_flag){
				/* let's go back to main menu */
				KY040_set_enc_default_value(main_menu_previous_pos);
				KY040_set_enc_limits(0, 2u);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
			}
			break;
		case SETTINGS_FRAME:
			if(button_pressed_flag){
				handle_button_in_settings_frame(cur_enc);
			}
			if(encoder_changed_flag){
				if(cursor.state == CURSOR_INACTIVE){
					handle_encoder_in_settings_frame(cur_enc);
				}else{
					handle_encoder_in_settings_frame_when_cursor_is_active(cur_enc);
				}
			}
			break;
		case GAMEPLAY_FRAME:
			if(button_pressed_flag){
				/* For now let's go back to main menu on click */
				KY040_set_enc_limits(0, 2u);
				KY040_set_enc_default_value(main_menu_previous_pos);
				LCD_set_current_frame(&menuFrame);
				mainState = MAIN_MENU_BEGIN;
			}

			handle_coordinates_while_playing();

			if(current_latitude == 0 && current_longitude == 0){
				/* No GPS signal received yet, let's just show the rotating arrow */
				if(!animation_timer){
					static float curr_angle = -1.5708f;
					static int8_t dir = 1;
					if(curr_angle < -PI){
						dir = 1;
					}else if(curr_angle > 0){
						dir = -1;
					}
					curr_angle += (dir * 0.08);
					update_navigation_arrow_entity_rotation(&gamePlayFrame, gamePlayFrame_navigation_arrowID, curr_angle);

					animation_timer = 100u; //ms
				}
			}else{
				/* Check and show distance */
				uint8_t tmp[10u];
				if(main_distance < 1.f){
					/* less than km, let's convert to meters */
					uint16_t distance_m = (uint16_t)(main_distance * 1000);
					sprintf(distance_text, "%dm", distance_m);
				}else{
					print_float(tmp, main_distance, 2u);
					sprintf(distance_text, "%skm", tmp);
				}
				update_text_entity_text(&gamePlayFrame, gamePlayFrame_text1ID, distance_text, strlen(distance_text));
				update_text_entity_position(&gamePlayFrame, gamePlayFrame_text1ID, center_text(strlen(distance_text)), 54u);
				update_navigation_arrow_entity_rotation(&gamePlayFrame, gamePlayFrame_navigation_arrowID, main_azimuth_rad + CURRENT_ARROW_ANGLE_OFFSET);

			}

			break;
		default:
			/* Defensive programming */;
	}

	Handle_cursor();
//	distance_calc_test();
}
/*----------------------------------------------------------------------------*/
void Geocacher_TimerHandler(void){
	if(cursor.timer){
		cursor.timer--;
	}

	if(animation_timer){
		animation_timer--;
	}
}
/*----------------------------------------------------------------------------*/
double distance_calc_test(void){
	uint8_t curr_lat_deg = 47;
	uint8_t curr_long_deg = 19;
	float curr_lat_min = 29.15811;
	float curr_long_min = 3.71763;

	uint8_t goal_lat_deg = 47;
	uint8_t goal_long_deg = 18;
	float goal_lat_min = 29.15811;
	float goal_long_min = 31.931;

	double curr_lat = 0u;
	double curr_long = 0u;

	double goal_lat = 0u;
	double goal_long = 0u;

	double distance = 0u;
	float azimuth = 0;

	curr_lat = convert_coordinates_to_rad(curr_lat_deg, curr_lat_min);
	curr_long = convert_coordinates_to_rad(curr_long_deg, curr_long_min);
	goal_lat = convert_coordinates_to_rad(goal_lat_deg, goal_lat_min);
	goal_long = convert_coordinates_to_rad(goal_long_deg, goal_long_min);

	distance = calculate_distance_between_current_position_and_object(curr_lat, curr_long, goal_lat, goal_long);
	azimuth = calculate_azimuth_between_current_position_and_object(curr_lat, curr_long, goal_lat, goal_long);
	return distance;
}

/*----------------------------------------------------------------------------*/
void handle_new_game_start(void){
	dtNMEACoordinate lati, longi;
	/* convert coordinates */
	geochache_latitude = convert_text_coordinates_to_double(geocaching_latitude);
	geochache_longitude = convert_text_coordinates_to_double(geocaching_longitude);

	/* get current coordinates */
	GPS_get_current_latitude(&lati);
	GPS_get_current_longitude(&longi);
	if(lati.qualifier != DATA_NOT_AVAILABLE && longi.qualifier != DATA_NOT_AVAILABLE){
		starting_latitude = convert_coordinates_to_rad(lati.degree, lati.minute);
		starting_longitude = convert_coordinates_to_rad(longi.degree, longi.minute);
	}
}
/*----------------------------------------------------------------------------*/
void round_azimuth(float* azimuth){
	if(*azimuth > 0){
		if(*azimuth > 180){
			*azimuth -= 180;
			*azimuth = -180 + *azimuth;
		}
	}

	if(*azimuth < 0){
		if(*azimuth > -180){
			*azimuth += 180;
			*azimuth = 180 - *azimuth;
		}
	}
}
/*----------------------------------------------------------------------------*/
void handle_coordinates_while_playing(void){
	dtNMEACoordinate lati, longi;
	double distance = 0u, distance_to_geo = 0u;
	float current_azimuth = 0u, azimuth_to_geo = 0u;
	/* get current coordinates */
	GPS_get_current_latitude(&lati);
	GPS_get_current_longitude(&longi);

	if(lati.qualifier != DATA_NOT_AVAILABLE && longi.qualifier != DATA_NOT_AVAILABLE){
		if(!starting_latitude && !starting_longitude){
			/* starting coordinates were not initialized yet */
			starting_latitude = convert_coordinates_to_rad(lati.degree, lati.minute);
			starting_longitude = convert_coordinates_to_rad(longi.degree, longi.minute);
		}else{
			/* Let's check if the distance since the last position has changed significantly and
			 * recalculate current azimuth then */
			current_latitude = convert_coordinates_to_rad(lati.degree, lati.minute);
			current_longitude = convert_coordinates_to_rad(longi.degree, longi.minute);
			double lat_delta = fabs(current_latitude - starting_latitude);
			double long_delta = fabs(current_longitude - starting_longitude);
			if((lat_delta > COORDINATE_DELTA) || (long_delta > COORDINATE_DELTA)){
				distance = calculate_distance_between_current_position_and_object(starting_latitude, starting_longitude, current_latitude, current_longitude);
				azimuth_to_geo = calculate_azimuth_between_current_position_and_object(current_latitude, current_longitude, geochache_latitude, geochache_longitude);
				distance_to_geo = calculate_distance_between_current_position_and_object(current_latitude, current_longitude, geochache_latitude, geochache_longitude);
				main_distance = distance_to_geo;

				if(distance > DELTA_DISTANCE_TO_RECALC_AZIMUTH){
					/* in degerees */
					current_azimuth = calculate_azimuth_between_current_position_and_object(starting_latitude, starting_longitude, current_latitude, current_longitude);
					starting_latitude = current_latitude;
					starting_longitude = current_longitude;

					/* debug print */
					sprintf(debug_text_1, "%d", (int)current_azimuth);
					sprintf(debug_text_2, "%d", (int)azimuth_to_geo);
					update_text_entity_text(&gamePlayFrame, gamePlayFrame_DEBUG_text1ID, debug_text_1, strlen(debug_text_1));
					update_text_entity_text(&gamePlayFrame, gamePlayFrame_DEBUG_text2ID, debug_text_2, strlen(debug_text_2));
					/* Round azimuth*/
					round_azimuth(&current_azimuth);
					round_azimuth(&azimuth_to_geo);
					/* Setting azimuth */
					if((azimuth_to_geo > 0 && azimuth_to_geo  < 180) && (current_azimuth > 0 && current_azimuth < 180)){
						/* The geocache and we're both pointing east */
						if(current_azimuth > azimuth_to_geo){
							main_azimuth_rad = current_azimuth - azimuth_to_geo;
						}else{
							main_azimuth_rad = azimuth_to_geo - current_azimuth;
						}
					}else if((azimuth_to_geo < 0 && azimuth_to_geo > -180) && (current_azimuth < 0 && current_azimuth > -180)){
						/* The geocache and we're both pointing west */
						if(current_azimuth < azimuth_to_geo){
							main_azimuth_rad = current_azimuth - azimuth_to_geo;
						}else{
							main_azimuth_rad = azimuth_to_geo - current_azimuth;
						}
					}else if(azimuth_to_geo < 0 && current_azimuth > 0){
						/* The geocache pointing west while we're pointing east */
						main_azimuth_rad = azimuth_to_geo - current_azimuth;
					}else if(azimuth_to_geo > 0 && current_azimuth < 0){
						/* The geocache pointing east while we're pointing west */
						main_azimuth_rad = azimuth_to_geo + current_azimuth;
					}else{
						main_azimuth_rad = azimuth_to_geo;
					}

					main_azimuth_rad *= (PI / 180.f);
				}
			}
		}
	}
}

/******************************************************************************/
