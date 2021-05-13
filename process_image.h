#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void init_array8(uint8_t *array, uint16_t length);
void init_array16(uint16_t *array, uint16_t length);
void extract_code(uint8_t *buffer, uint8_t *code);
uint8_t get_mode(void);
void set_mode(uint8_t new_mode);
void get_tab(uint8_t *tab);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
