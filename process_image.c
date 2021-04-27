#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

static uint8_t code_array[MAX_CODE_LENGTH];
static uint8_t mode = SEARCH_MODE;

static uint8_t binary_width = 40;
static uint8_t binary_width_margin;
static uint8_t starting_code_width;
static uint8_t ending_code_width;
static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the code extracted from the image buffer given
 *  Returns 0 in first index if code not found, else returns 1 in first index
 */
void extract_code(uint8_t *buffer, uint8_t *code){

	uint32_t mean = 0;
	//variables of position of beginning and ending code (code to read is in between)
	uint16_t starting_code = 0, ending_code = 0;
	//variables that change during the loops
	uint16_t begin = 0, current_px = 0, last_code = 0;
	uint8_t index = 1;
	//variables to check state
	bool found_start = false, found_end = false, found_code = false;

	binary_width_margin = binary_width/4;
	starting_code_width = binary_width*2.5;
	ending_code_width = binary_width*3/4;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	//searching for starting code
	while(!found_start && current_px < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE - starting_code_width - binary_width_margin)){
		//looking for beginning
		if(buffer[current_px] > mean && buffer[current_px + WIDTH_SLOPE] < mean){
			begin = current_px;
			current_px += WIDTH_SLOPE - 1;
		}
		//if beginning found, looking for end
		if(begin && buffer[current_px] < mean && buffer[current_px + WIDTH_SLOPE] > mean){
			if((current_px - begin) > (starting_code_width - binary_width_margin) && (current_px - begin) < (starting_code_width + binary_width_margin)){
				found_start = true;
				current_px += WIDTH_SLOPE - 1;
			}
		}
		current_px++;
	}

	//check if start was found. If not return with error code[0]=0
	if(!found_start){
		code[0] = END;
		return;
	}else{
		starting_code = current_px;
	}

	//searching for ending code
	while(!found_end && current_px < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE - ending_code_width - binary_width_margin)){
		//looking for beginning
		if(buffer[current_px] > mean && buffer[current_px + WIDTH_SLOPE] < mean){
			begin = current_px;
			current_px += WIDTH_SLOPE - 1;
		}
		//if beginning found, looking for end
		if(begin && buffer[current_px] < mean && buffer[current_px + WIDTH_SLOPE] > mean){
			if((current_px - begin) > (ending_code_width - binary_width_margin) && (current_px - begin) < (ending_code_width + binary_width_margin)){
				found_end = true;
			}
		}
		current_px++;
	}

	//check if end was found. If not return with error (code[0]=0)
	if(!found_start){
		code[0] = END;
		return;
	}else{
		ending_code = begin;
	}

	//reading code between begin and end
	current_px = starting_code + binary_width_margin;
	last_code = starting_code;
	while(current_px <= ending_code){
		//checking two bits at a time
		//if first bit is white
		if(buffer[current_px] > mean){
			while(current_px <= last_code + 2*(binary_width - binary_width_margin)){
				if(buffer[current_px + WIDTH_SLOPE] < mean){
					if(index < MAX_CODE_LENGTH){
						//code 01
						code[index] = ADVANCE;
						found_code = true;
						break;
					}else{
						code[0] = END;
						return;
					}
				}
				current_px++;
			}
			//if second bit is white
			if(!found_code){
				if(index < MAX_CODE_LENGTH){
					//code 00
					code[index] = RIGHT;
				}else{
					code[0] = END;
					return;
				}
			}
		}
		//if first bit is black
		else{
			while(current_px <= last_code + 2*(binary_width - binary_width_margin)){
				if(buffer[current_px + WIDTH_SLOPE] > mean){
					if(index < MAX_CODE_LENGTH){
						//code 10
						code[index] = LEFT;
						found_code = true;
						break;
					}else{
						code[0] = END;
						return;
					}
				}
				current_px++;
			}
			//if second bit is black
			if(!found_code){
				if(index < MAX_CODE_LENGTH){
					//code 11
					code[index] = RETREAT;
				}else{
					code[0] = END;
					return;
				}
			}
		}
		index++;
		found_code = false;
		current_px = last_code + 2*(binary_width + binary_width_margin);
		last_code += 2*binary_width;
	}
	code[0] = VALID_CODE;
//	uint16_t distance = VL53L0X_get_dist_mm();
//	for (int i = 0; i < MAX_CODE_LENGTH+1; i++ ) {
//		chprintf((BaseSequentialStream *)&SD3,"code %d : %d - ", i, code[i]);
//	}
//	chprintf((BaseSequentialStream *)&SD3,"mode : %d - distance : %d\n", mode, distance);
	return;
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
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

void init_code(uint8_t *code){
	for(uint8_t i = 0; i < MAX_CODE_LENGTH + 1; i++){
		code[i] = 0;
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

//		//search for a line in the image and gets its width in pixels
//		lineWidth = extract_line_width(image);
//
//		//converts the width into a distance between the robot and the camera
//		if(lineWidth){
//			distance_cm = PXTOCM/lineWidth;
//		}

		init_code(code);
		//search for a code in the image
		extract_code(image, code);

//		for (int i = 0; i < MAX_CODE_LENGTH+1; i++ ) {
//			chprintf((BaseSequentialStream *)&SD3,"tab %d : %d - ", i, code[i]);
//		}

		if(code[0] && mode == SEARCH_MODE){
//			chprintf((BaseSequentialStream *)&SD3,"------- coucou c'est moi ------\n");
			for(uint8_t i = 0; i < MAX_CODE_LENGTH; i++){
				code_array[i] = code[i + 1];
			}
			set_mode(EXE_MODE);
//			chprintf((BaseSequentialStream *)&SD3,"mode : %d\n", mode);
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
