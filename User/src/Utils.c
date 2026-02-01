/******************************************************************************/
/****************************** Includes  *************************************/
/******************************************************************************/
#include "Utils.h"
#include "string.h"
/******************************************************************************/
/****************************** Deifnes ***************************************/
/******************************************************************************/
#define INT_MAX_LEN 10u
#define FLOAT_MAX_LEN_PART 10u

/******************************************************************************/
/****************************** Private types *********************************/
/******************************************************************************/


/******************************************************************************/
/****************************** Globals ***************************************/
/******************************************************************************/
const char numbers[10u] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
uint32_t delayCnt = 0u;
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
int8_t atoi_nibble_hex(uint8_t symb){
	uint8_t nibbleDec = 0;
	if((symb >= (uint8_t)'A') && (symb <= (uint8_t)'F')){
		nibbleDec = 10u + (symb - (uint8_t)'A');
	}else if((symb >= (uint8_t)'a') && (symb <= (uint8_t)'f')){
		nibbleDec = 10u + (symb - (uint8_t)'a');
	}else if((symb >= (uint8_t)'0') && (symb <= (uint8_t)'9')){
		nibbleDec = symb - (uint8_t)'0';
	}else{
		return -1;
	}
	return (int8_t)nibbleDec;
}

/*----------------------------------------------------------------------------*/
int _pow(int num, int pow){
	int res = 1u;
	if(pow > 0){
		for(uint8_t i = 0u; i < pow; i++){
			res *= num;
		}
	}else{
		/* Only positive pow supported at the moment */
	}
	return res;
}

/*----------------------------------------------------------------------------*/
int _atoi(uint8_t* str){
	int tmp = 0u;
	uint8_t pow = 0u, len = 0u;
	int8_t isNegative = 1;

	if(*str == '-'){
		isNegative = -1;
		str++;
	}
	len = strlen(str);
	if(len < INT_MAX_LEN){
		while(len > 0){
			if((*str >= (uint8_t)'0') && (*str <= (uint8_t)'9')){
				tmp += (*(str + (len--) -1) - (uint8_t)'0') * (_pow(10, pow++));
			}else{
				/* Forbidden symbol for atoi operation */
				break;
			}
		}
	}

	return tmp * isNegative;
}

/*----------------------------------------------------------------------------*/
float _atof(uint8_t* str){
	int number = 0u, fraction = 0u, fraction_tmp = 0u;
	uint8_t fract_pos = 0u;
	float tmp = 0.0f;
	uint8_t len = 0u;
	int8_t isNegative = 1;
	uint8_t tmpBuf[FLOAT_MAX_LEN_PART] = {0u};
	uint8_t* tmpPtr = NULL;
	uint8_t fract_len  = 0u;
	uint8_t isFormatCorrect = FAIL;

	if(*str == '-'){
		isNegative = -1;
		str++;
	}
	len = strlen((const char*)str);
	tmpPtr = str;
	while (*tmpPtr != '\0' && fract_pos < FLOAT_MAX_LEN_PART){
		if(*tmpPtr == '.'){
			isFormatCorrect = PASS;
			break;
		}
		fract_pos++;
		tmpPtr++;
	}
	if(isFormatCorrect){
		memcpy(tmpBuf, str, fract_pos);
		tmpBuf[fract_pos] = '\0';
		number = _atoi(tmpBuf);

		len -= fract_pos;
		len--; // minus one byte containing comma
		memcpy(tmpBuf, (str + fract_pos+ 1u), len); // plus one byte containing comma
		tmpBuf[len] = '\0';
		fraction = _atoi(tmpBuf);

		tmp = (float)number;

		fraction_tmp = fraction;
		do{
			fraction_tmp /= 10u;
			fract_len++;
		}while (fraction_tmp > 0u);

		tmp += (float)fraction / (float)(_pow(10, (fract_len)));
	}
	return tmp * isNegative;
}

/*----------------------------------------------------------------------------*/
void print_float(char* str, float num, uint8_t precision){
	int int_part = num, fract_part = 0u;
	num -= (float)int_part; /* only fract part left */
	fract_part = num * _pow(10u, precision);
	sprintf(str, "%d.%d", int_part, fract_part);
}

/*----------------------------------------------------------------------------*/
uint8_t majority_function(uint8_t* middle_sample){
	return 	( (*middle_sample)&(*(middle_sample - 1)) )|
			( (*(middle_sample - 1))&(*(middle_sample + 1)) )|
			( (*middle_sample)&(*(middle_sample + 1)) );
}

/*----------------------------------------------------------------------------*/
/* Custom delay functions
/*----------------------------------------------------------------------------*/

uint8_t _setDelay(uint32_t delayMs){
	if(delayCnt){
		/* Another delay is in progress */
		return FAIL;
	}else{
		delayCnt = delayMs;
		return PASS;
	}
}

void _clearDelay(void){
	delayCnt = 0u;
}

uint8_t _isDelayExpired(void){
	if(!delayCnt){
		return PASS;
	}else{
		return FAIL;
	}
}

void _Delay(uint32_t delayMs){
	while(delayCnt);
}

void _Delay1ms_tick_hanlder(void){
	if(delayCnt){
		delayCnt--;
	}
}
/******************************************************************************/
