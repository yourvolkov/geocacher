/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "KY040.h"
#include "main.h"
#include "Utils.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define BUTTON_PIN    ENC_BUTTON_Pin
#define BUTTON_PORT   ENC_BUTTON_GPIO_Port

#define CLK_PIN       ENC_CLK_Pin
#define CLK_PORT      ENC_CLK_GPIO_Port

#define DT_PIN        ENC_DT_Pin
#define DT_PORT       ENC_DT_GPIO_Port

#define JITTER_DEBOUNCE_BUTTON		30u
#define JITTER_DEBOUNCE_ENCODER		10u
#define JITTER_POST_DEBOUNCE_ENCODER 5u

#define ENCODER_DEBOUNCE_SAMPLES_AMNT 3u
/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/
typedef enum{
	BUTTON_IDLE = 0u,
	BUTTON_DEBOUNCE,

}dtButtonHandleState;


typedef enum{
	ENC_IDLE = 0u,
	ENC_DEBOUNCE,
	ENC_POST_DEBOUNCE,

}dtEncoderHandleState;

typedef struct{
	dtEncoderHandleState EncState;
	uint8_t Clk_flag;
	uint8_t enc_clk_samples[ENCODER_DEBOUNCE_SAMPLES_AMNT];
	uint8_t enc_dt_samples[ENCODER_DEBOUNCE_SAMPLES_AMNT];
	uint8_t enc_samples_cnt;
	int32_t enc_current_value;
	int32_t enc_prev_value;
	int32_t enc_min_value;
	int32_t enc_max_value;
	uint8_t enc_value_updated_flag;
}dtEncoderState;

/*----------------------------------------------------------------------------*/
typedef struct{
	dtEncoderHandleState button_state;
	uint8_t button_flag;
	uint32_t button_pressed_cnt;
	uint32_t button_pressed_prev_cnt;
	uint8_t button_pressed_flag;
}dtButtonState;
/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
dtEncoderState encoder;
dtButtonState button;
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


/******************************************************************************/
/**************************** Public functions ********************************/
/******************************************************************************/
void KY040_Init(int32_t initCounterValue, int32_t minValue, int32_t maxValue){
	encoder.Clk_flag = FAIL;
	encoder.EncState = ENC_IDLE;
	encoder.enc_samples_cnt = 0u;
	encoder.enc_value_updated_flag = FAIL;
	encoder.enc_current_value = initCounterValue;
	encoder.enc_prev_value = initCounterValue;
	encoder.enc_min_value = minValue;
	encoder.enc_max_value = maxValue;

	button.button_flag = FAIL;
	button.button_pressed_cnt = 0u;
	button.button_pressed_prev_cnt = 0u;
	button.button_pressed_flag = FAIL;
	button.button_state = BUTTON_IDLE;
}

/*----------------------------------------------------------------------------*/
void EXTI_ISR_Handler(uint16_t GPIO_Pin){
	if(GPIO_Pin == BUTTON_PIN){
		button.button_flag = PASS;
	}else if(GPIO_Pin == CLK_PIN){
		if(encoder.Clk_flag == FAIL){
			encoder.enc_clk_samples[encoder.enc_samples_cnt] = HAL_GPIO_ReadPin(CLK_PORT, CLK_PIN);
			encoder.enc_dt_samples[encoder.enc_samples_cnt] = HAL_GPIO_ReadPin(DT_PORT, DT_PIN);
			encoder.enc_samples_cnt++;
			encoder.Clk_flag = PASS;
		}
	}else{
		/* Defensive programming */
	}
}

/*----------------------------------------------------------------------------*/
void _Timer_ISR_Handler(void){
	/* Handle encoder */
	if(encoder.Clk_flag){
		if(encoder.enc_samples_cnt < ENCODER_DEBOUNCE_SAMPLES_AMNT){
			encoder.enc_clk_samples[encoder.enc_samples_cnt] = HAL_GPIO_ReadPin(CLK_PORT, CLK_PIN);
			encoder.enc_dt_samples[encoder.enc_samples_cnt] = HAL_GPIO_ReadPin(DT_PORT, DT_PIN);
			encoder.enc_samples_cnt++;
		}
	}
}

/*----------------------------------------------------------------------------*/
void KY040_Hanlder(void){
	/* Handle button */
	switch(button.button_state){
		case BUTTON_IDLE:
			if(button.button_flag == PASS){
				_setDelay(JITTER_DEBOUNCE_BUTTON);
				button.button_state = BUTTON_DEBOUNCE;
			}
			break;
		case BUTTON_DEBOUNCE:
			if(_isDelayExpired() == PASS){
				button.button_state = BUTTON_IDLE;
				button.button_flag = FAIL;
				if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)){
					/* If the button is still pressed */
					button.button_pressed_cnt++;
					button.button_pressed_flag = PASS;
				}
			}
			break;
		default:
			button.button_state = BUTTON_IDLE;
	}

	/* Handle encoder */
	switch(encoder.EncState){
		case ENC_IDLE:
			if(encoder.Clk_flag == PASS){
				encoder.EncState = ENC_DEBOUNCE;
				encoder.enc_samples_cnt = 0u;
			}
			break;
		case ENC_DEBOUNCE:
			if(encoder.enc_samples_cnt == ENCODER_DEBOUNCE_SAMPLES_AMNT){
				if(!majority_function(&encoder.enc_clk_samples[2u])){
					if(majority_function(&encoder.enc_dt_samples[2u])){
						if(encoder.enc_current_value > encoder.enc_min_value){
							encoder.enc_current_value--;
							encoder.enc_value_updated_flag = PASS;
						}
					}else{
						if(encoder.enc_current_value < encoder.enc_max_value){
							encoder.enc_current_value++;
							encoder.enc_value_updated_flag = PASS;
						}
					}
				}
				encoder.EncState = ENC_IDLE;
				encoder.Clk_flag = FAIL;
				encoder.enc_samples_cnt = 0u;
			}
			break;
		default:
			encoder.EncState = ENC_IDLE;
	}
}

/*----------------------------------------------------------------------------*/
void KY040_set_enc_default_value(int defaultEncValue){

	if(encoder.enc_current_value != defaultEncValue){
		/* Let's consider enc_current_value to be updated */
		encoder.enc_value_updated_flag = PASS;
	}

	if(defaultEncValue <= encoder.enc_min_value){
		encoder.enc_current_value = encoder.enc_min_value;
	}else if(defaultEncValue >= encoder.enc_max_value){
		encoder.enc_current_value = encoder.enc_max_value;
	}else{
		encoder.enc_current_value = defaultEncValue;
	}
}

/*----------------------------------------------------------------------------*/
void KY040_set_enc_limits(int32_t minValue, int32_t maxValue){
	encoder.enc_max_value = maxValue;
	encoder.enc_min_value = minValue;
}

/*----------------------------------------------------------------------------*/
int KY040_get_enc_current_value(void){
	return encoder.enc_current_value;
}

/*----------------------------------------------------------------------------*/
uint8_t KY040_isEncoderValueChanged(void){
	return encoder.enc_value_updated_flag;
}

/*----------------------------------------------------------------------------*/
uint8_t KY040_isButtonPressed(void){
	return button.button_pressed_flag;
}

/*----------------------------------------------------------------------------*/
void KY040_encoderValueChangedClearFlag(void){
	encoder.enc_value_updated_flag = FAIL;
}

/*----------------------------------------------------------------------------*/
void KY040_buttonPressedClearFlag(void){
	button.button_pressed_flag = FAIL;
}
/******************************************************************************/
