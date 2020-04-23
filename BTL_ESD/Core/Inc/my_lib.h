#include "nokia5110.h"
#define DATA_START_ADDRESS 		 	((uint32_t)0x0803FC00)	//Page 64
#define LENGTH_START_ADDRESS 		((uint32_t)0x08040000)	//Page 63
uint8_t bird_pos;
uint8_t bar_height;
uint8_t bar_pos;
uint8_t colision;
uint8_t dead;
uint8_t game;
int score;
volatile uint8_t debouncing;
char buffer [504];
void print_number(char myBitmap[10][120],uint8_t x, uint8_t number);
void shift_img();
void LCD_set_pixel(uint8_t x, uint8_t y, uint8_t pixel);
void LCD_set_pixel_im(uint8_t x, uint8_t y, uint8_t pixel);
void delete_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width,uint8_t gap);
void draw_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width, uint8_t gap);
void shift_im();
void deleteBuffer();
void printImage(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy);
void printImageReverse(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy);
void deleteImageReverse(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy);
void deleteImage(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy);
void printChar(uint8_t x, uint8_t y, char c);
void printStr(char *str, uint8_t x, uint8_t y);
void deleteStr(char *str, uint8_t x, uint8_t y);
int  printRandoms(int lower, int upper);
uint8_t Flappy_Bird(uint8_t game, char bird[],char over[]);


