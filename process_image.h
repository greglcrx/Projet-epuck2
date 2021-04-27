#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint8_t get_mode(void);
void set_mode(uint8_t new_mode);
void get_tab(uint8_t *tab);
float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
