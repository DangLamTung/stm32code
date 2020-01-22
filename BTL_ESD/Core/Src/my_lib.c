
#include "my_lib.h"

int lower = 5, upper = 7, count = 1;
void print_number(char myBitmap[10][120],uint8_t x, uint8_t number){
	 LCD_goXY(x, 1);
	 for(int i = 0; i<5;i++){
		 LCD_goXY(x, i + 1);
	 for(int j = 0; j<25;j++){
		  		  LCD_write(myBitmap[number][j + 25*i],  LCD_DATA);
		}
	 }
}

void shift_img(char bird[]){

	for(int i = 0; i<504; i++)
		buffer[i] = 0x00;
	for(int i = 0 ; i<3;i++ ){
		for(int j = 0; j<20; j++)
		buffer[i*84 +j] = bird[20*i + j  ];
	}
}
void LCD_set_pixel(uint8_t x, uint8_t y, uint8_t pixel){
  if(x >= LCD_WIDTH)
    x = LCD_WIDTH - 1;
  if(y >= LCD_HEIGHT)
    y = LCD_HEIGHT - 1;

  if(pixel != 0){
    buffer[x + (y / 8) * LCD_WIDTH] |= 1 << (y % 8);
  }
  else{
    buffer[x + (y / 8) * LCD_WIDTH] &= ~(1 << (y % 8));
  }
}

void LCD_set_pixel_im(uint8_t x, uint8_t y, uint8_t pixel){
  if(x >= LCD_WIDTH)
    x = LCD_WIDTH - 1;
  if(y >= LCD_HEIGHT)
    y = LCD_HEIGHT - 1;

  if(pixel != 0){
	  buffer[x + (y / 8) * LCD_WIDTH] |= 1 << (y % 8);
	  if(buffer[x + (y / 8) * LCD_WIDTH] == 0){
}
  }
else{

}
}

void delete_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width,uint8_t gap){

	for(int i = x; i<x+width; i++){
		for(int j = y;j<y+height-6;j++){
			LCD_set_pixel(i,j, 0);
		}
	}
	for(int i = x; i<x+width; i++){
		for(int j = 48;j>  (y+height)+ gap + 6;j--){
				LCD_set_pixel(i,j, 0);
			}
		}
	if(x<82 && x>2){
	for(int i = x-2; i<x+width +2 ; i++){
			for(int j = y+height-5;j<y+height;j++){
				LCD_set_pixel(i,j, 0);
			}
		}
	for(int i = x-2; i<x+width +2 ; i++){
				for(int j = (y+height)+ gap + 5;j>  (y+height)+ gap;j--){
					LCD_set_pixel(i,j,0);
				}
			}
	}
}
void draw_bar(uint8_t x, uint8_t y, uint8_t height, uint8_t width, uint8_t gap){

	for(int i = x; i<x+width; i++){
		for(int j = y;j<y+height-6;j++){
			LCD_set_pixel(i,j, 1);
		}
	}
	for(int i = x; i<x+width; i++){
		for(int j = 48;j>  (y+height)+ gap + 6;j--){
			LCD_set_pixel(i,j, 1);
		}
	}

	if(x<82 && x>2){
	for(int i = x-2; i<x+width +2 ; i++){
			for(int j = y+height-5;j<y+height;j++){
				LCD_set_pixel(i,j, 1);
			}
		}
	for(int i = x-2; i<x+width +2 ; i++){
				for(int j = (y+height)+ gap + 5;j>  (y+height)+ gap;j--){
					LCD_set_pixel(i,j, 1);
				}
			}

//	for(int i = x-2; i<x+width +2 ; i++){
//				for(int j = 48;j>(y+height)- gap ;j++){
//					LCD_set_pixel(i,j, 1);
//				}
//			}
	}
//	LCD_printBuffer(buffer);
}
void shift_im(char bird []){
	uint16_t temp[60];
	uint8_t im[80];
	for(int i = 0 ; i<3;i++ ){
			for(int j = 0; j<20; j++)
				temp[20*i + j] = bird[20*i + j  ];
		}
	for(int i = 0; i<60; i++){
		temp[i] <<5;
	}
////	for(int i = 0; i<20; i++){
////		im[i] =(uint8_t) temp[i] ;
////	}
//	for(int i = 20; i<40; i++){
//			im[i] = (uint8_t)temp[i - 20] & 0x00FF;
//		}
//	for(int i = 20; i<40; i++){
//				im[i] = (uint8_t)im[i] | ((uint8_t)temp[i] && 0xFF00);
//			}
//   for(int i = 40; i<60; i++){
//	   im[i] =(uint8_t) temp[i-20] & 0x00FF;
//   }
//   for(int i = 60; i<80; i++){
//  	   im[i] = (uint8_t)temp[i] & 0xFF00;
//     }
   for(int i = 0; i<504; i++)
   		buffer[i] = 0x00;
   	for(int i = 0 ; i<4;i++ ){
   		for(int j = 0; j<20; j++)
   		buffer[i*84 +j] = im[20*i + j  ];
   	}
}
void deleteBuffer_(){
	 for(int i = 0; i<504; i++)
	   		buffer[i] = 0x00;
}
void printImage(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0 ; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 0; k<8;k++){
			             LCD_set_pixel_im(j+x , y + k + 8*(sx -1 - i) , data[i*sy+j] & (1 << (7 - k)));
		     			}
		     		}
		 	  }
    }
void printImageReverse(uint8_t x, uint8_t y, char data [], uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 7; k>0;k--){
			             LCD_set_pixel_im((j +x) ,  y + k + 8*i ,data[i*sy+(sy-j)] & (1<< (7-k)));
		     			}
		     		}
		 	  }
}
void deleteImageReverse(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 7; k>0;k--){
			             LCD_set_pixel_im((j +x) ,  y + k + 8*i ,0);
		     			}
		     		}
		 	  }
}
void deleteImage(uint8_t x, uint8_t y, uint8_t sx,uint8_t sy){
//	 for(int i = 0; i<504; i++)
//	   		buffer[i] = 0x00;
		  for(int i = 0 ; i<sx;i++ ){
		     		for(int j = 0; j<sy; j++){
		     			for(int k = 0; k<8;k++){

			             LCD_set_pixel(j+x , y + k + 8*(sx -1 - i) , 0);
		     			}
		     		}
		 	  }

}
void printChar(uint8_t x, uint8_t y, char c){

	printImageReverse(x,y, ASCII[c - 0x20], 1,6);

}
void delChar(uint8_t x, uint8_t y){

	deleteImage(x,y, 1,6);
}
void printStr(char *str, uint8_t x, uint8_t y){

  uint8_t i = x;
  uint8_t str_len = strlen(str);
  for(int j = str_len-1; j >= 0;j--){
	  printChar(i,y,str[j]);
	  i+=6;
  }
}
void deleteStr(char *str, uint8_t x, uint8_t y){

  uint8_t i = x;
  uint8_t str_len = strlen(str);
  for(int j = str_len-1; j >= 0;j--){
	  delChar(i,y);
	  i+=6;
  }
}
int  printRandoms(int lower, int upper)
{
	HAL_GetTick();
        int num;
        num = (rand() % (upper - lower + 1)) + lower;
        return num;
}
void drawVline(int x, int y, int l){
	 if ((x>=0) && (x<84) && (y>=0) && (y<48)){
		    for (int cy=0; cy<= l; cy++){
		    	LCD_set_pixel(x, y+cy, 1);
		    }
		  }
}

void drawHLine(int x, int y, int l){
  int by, bi;

  if ((x>=0) && (x<LCD_WIDTH) && (y>=0) && (y<LCD_HEIGHT)){
    for (int cx=0; cx<l; cx++){
      by=((y/8)*84)+x;
      bi=y % 8;
      buffer[by+cx] |= (1<<bi);
    }
  }
}
void Tetris(){
	deleteBuffer();
	drawVline(0,47,47);
	  LCD_printBuffer(buffer);
}
uint8_t Flappy_Bird(uint8_t game, char bird[],char over[]){
	  dead = 0;


	  if(game == 0)
	  {

	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */



		  LCD_goXY(0, 0);
	      draw_bar(bar_pos, 0,bar_height,8, 15);

	      printImage(15, bird_pos,  bird, 2,16);
	      char c;
	      colision = 0;
	      if(bar_pos>15 && bar_pos<23){

	         if(bird_pos > bar_height -3 && bird_pos < bar_height + 3  ){
	        	 colision = 0;
	        	 printChar(0, 0, '0');
	             c = '0';
	         }
	         else{
	        	 colision = 1;
	        	 printChar(0, 0, '1');
	        	 c = '1';
	        	 dead = 1;
	        	 while(1){
	        		 char high[5];
	        		 LCD_printBuffer(over);
	        		 char buffer_[5];
	        		 //		  char high[5];


	        		 HAL_Delay(1000);
	        		 deleteBuffer_();

	        		 HAL_Delay(1000);
	        		 return dead;
	        	 }
	         }
	      }
	      uint8_t temp_score;
	      temp_score = 0;
	      if(bar_pos == 4 && dead ==0){
	    	  score++;
	      }

	      char  score_buffer [10];
	      itoa(score,score_buffer,10);
	      printStr("Score", 48,  40);
	      printStr(score_buffer, 30,  40);
	      LCD_printBuffer(buffer);

	      HAL_Delay(100);
	      delChar(0, 0);
	      delChar(30,  40);
		  deleteImage(15, bird_pos, 2,16);
		  delete_bar(bar_pos, 0,bar_height,8, 15);

		  if(bird_pos>0)
		  bird_pos--;

		  if(bar_pos>0){
		  bar_pos-=4;

		  }
		  else{
		  bar_pos = 84;
		  bar_height = printRandoms(10, 40);
		  }
	  }
 	 return dead;
}
