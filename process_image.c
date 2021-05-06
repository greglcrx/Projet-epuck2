#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static uint8_t code_array[MAX_CODE_LENGTH];
static uint8_t mode = SEARCH_MODE;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

void init_array8(uint8_t *array, uint16_t length){
	for(uint8_t i = 0; i < length; i++){
		array[i] = 0;
	}
}

void init_array16(uint16_t *array, uint16_t length){
	for(uint8_t i = 0; i < length; i++){
		array[i] = 0;
	}
}

/*
 *  Returns the code extracted from the image buffer given
 *  Returns 0 in first index of the array if code not found, else returns 1 in first index
 */
void extract_code(uint8_t *buffer, uint8_t *code){

	uint16_t slopes[MAX_NB_SLOPES];					//array of detected slopes [position in pixels]
	uint16_t current_px = 0;						//current pixel which is being analyzed
	uint8_t current_slope = 0;						//current slope which is being analyzed
	uint8_t slope_index = 0, code_index = 0;		//index is used to keep track of the number of slopes/codes that were detected
	uint8_t start = 0, end = 0;						//position of starting and ending code
	uint32_t mean = 0;								//average value of all pixels
	uint8_t nb_codes = 0;							//number of codes coded between two slopes (one large black line can be composed of 3 small black lines)
	uint8_t codes[2*(MAX_CODE_LENGTH+1)];			//array of the black and white lines which have been read

	//initialize arrays to zero
	init_array8(codes, 2*(MAX_CODE_LENGTH+1));
	init_array16(slopes, MAX_NB_SLOPES);

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;


	//---- analyzing pixels ----

	//find first area that is white so we ignore an eventual black to white slope at the beginning
	bool jump_slope = false;
	while(buffer[current_px] < mean && buffer[current_px + WIDTH_SLOPE] < mean && current_px < IMAGE_BUFFER_SIZE - WIDTH_SLOPE){
		current_px++;
		jump_slope = true;
		if(current_px >= IMAGE_BUFFER_SIZE - WIDTH_SLOPE){
			code[0] = END;
			return;
		}
	}
	if(jump_slope){
		current_px += WIDTH_SLOPE;
	}

	//searching for all slopes (black to white or white to black) and adding them to the slopes array
	while(current_px < IMAGE_BUFFER_SIZE - WIDTH_SLOPE){
		if(slope_index >= MAX_NB_SLOPES){
			break;
		}
		if(buffer[current_px] > mean && buffer[current_px + WIDTH_SLOPE] < mean){
			slopes[slope_index] = current_px;
			slope_index++;
			current_px += WIDTH_SLOPE - 1;
		}else if(buffer[current_px] < mean && buffer[current_px + WIDTH_SLOPE] > mean){
			slopes[slope_index] = current_px;
			slope_index++;
			current_px += WIDTH_SLOPE - 1;
		}
		current_px++;
	}


	//---- noise handling ----

	//deleting random peaks that are smaller than PEAK_SIZE wide
	for(uint8_t i=0; i<end-1; i++){
		if(abs(slopes[i+1] - slopes[i]) <= PEAK_SIZE){
			slopes[i] = 0;
			slopes[i+1] = 0;
		}
	}
	//reorganizing the slopes array (remove all zeros created while removing the noise)
	uint8_t k = 0;
	for(uint8_t i=0; i<end; i++){
		if(slopes[i]){
			if(k!=i){
				slopes[k] = slopes[i];
				slopes[i] = 0;
			}
			k++;
		}
	}


	//---- analyzing the slopes ----

	//looking for starting code
	for(uint8_t i = 0; i < slope_index-1; i++){
		if((slopes[i+1] - slopes[i]) > (STARTING_CODE_WIDTH - 2*BINARY_WIDTH_MARGIN) && (slopes[i+1] - slopes[i]) < (STARTING_CODE_WIDTH + 2*BINARY_WIDTH_MARGIN)){
			if(i < (MAX_NB_SLOPES - 2) && (slopes[i+2] - slopes[i+1]) < (BINARY_WIDTH/2)){			//eliminating small white line after starting code
				start = i+2;
				current_slope = i+2;
			}else{
				start = i+1;
				current_slope = i+1;
			}
			break;
		}
	}
	if(!start){
		code[0] = END;
		return;
	}

	//looking for ending code
	for(uint8_t i = current_slope; i < slope_index; i++){
		if((slopes[i+1] - slopes[i]) > (ENDING_CODE_WIDTH - 2*BINARY_WIDTH_MARGIN) && (slopes[i+1] - slopes[i]) < (ENDING_CODE_WIDTH + 2*BINARY_WIDTH_MARGIN)){
			if((slopes[i] - slopes[i-1]) < (BINARY_WIDTH - 3*BINARY_WIDTH_MARGIN)){				//eliminating small white line before ending code
				end = i-1;
			}else{
				end = i;
			}
		}
	}
	if(!end){
		code[0] = END;
		return;
	}

	//reading code
	for(uint8_t i = start; i < end; i++){
		nb_codes = (uint8_t)(slopes[i+1]-slopes[i])/(BINARY_WIDTH-BINARY_WIDTH_MARGIN);
		for(uint8_t j = 0; j < nb_codes; j++){
			if(i%2){
				codes[code_index + j] = WHITE;
			}else{
				codes[code_index + j] = BLACK;
			}
		}
		code_index += nb_codes;
	}


	//---- evaluating codes ----

	for(uint8_t i = 0; codes[i+1] != 0; i+=2){
		if(codes[i] == WHITE && codes[i+1] == WHITE){
			code[i/2 + 1] = ADVANCE;
		}
		else if(codes[i] == WHITE && codes[i+1] == BLACK){
			code[i/2 + 1] = RIGHT;
		}
		else if(codes[i] == BLACK && codes[i+1] == WHITE){
			code[i/2 + 1] = LEFT;
		}
		else if(codes[i] == BLACK && codes[i+1] == BLACK){
			code[i/2 + 1] = RETREAT;
		}
		else{
			code[0] = END;
			return;
		}
	}

	code[0] = VALID_CODE;
	return;
}

uint8_t get_mode(void){
	return mode;
}

void set_mode(uint8_t new_mode){
	mode = new_mode;
}

void get_tab(uint8_t *tab){
	for(uint8_t i = 0; i < MAX_CODE_LENGTH; i++){
		tab[i] = code_array[i];
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t code[MAX_CODE_LENGTH + 1];

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//reset every element of the array at 0
		init_array8(code, MAX_CODE_LENGTH+1);

		//search for a code in the image
		extract_code(image, code);

//		for (int i = 0; i < MAX_CODE_LENGTH+1; i++ ) {
//			chprintf((BaseSequentialStream *)&SD3,"tab %d : %d - \n", i, code[i]);
//		}

		if(code[0] && mode == SEARCH_MODE){
			for(uint8_t i = 0; i < MAX_CODE_LENGTH; i++){
				code_array[i] = code[i + 1];
			}
			set_mode(EXE_MODE);
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
