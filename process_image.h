#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint8_t * extract_code(uint8_t *buffer);
float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
