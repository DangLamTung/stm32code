/*
 * print_func.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: tung
 */

#ifndef INC_PRINT_FUNC_HPP_
#define INC_PRINT_FUNC_HPP_
#include "mpu_data_type.hpp"
void print_raw(IMU_data data_raw){
	        char buffer[10];
            ftoa(data_raw.Gyro_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Gyro_z, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_x, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_y, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data_raw.Acc_z, buffer, 2);
            strcat(buffer,"\n");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////
//            ftoa(data_raw.Mag_x, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_y, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_z, buffer, 2);
//         	strcat(buffer,"\n");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}

void print_euler(EULER_angle data){
	        char buffer[10];

            ftoa(data.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.pitch, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.yaw, buffer, 2);
            strcat(buffer,"\n");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////
//            ftoa(data_raw.Mag_x, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_y, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_z, buffer, 2);
//         	strcat(buffer,"\n");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}
void print_magnet(char mode){
	char buffer[10];
	if(mode == 1){
	                	    ftoa(magnet_calib[0], buffer, 2);
	                	    strcat(buffer," ");
	            //    	    CDC_Transmit_FS((uint8_t *)&BU, sizeof(rx_data));
	                	    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

	                	    ftoa(magnet_calib[1], buffer, 2);
	                	    strcat(buffer," ");
	                	    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

	                	    ftoa(magnet_calib[2], buffer, 2);
	                	    strcat(buffer,"\n");


	}
	else{

	}
}
void print_every_thing(IMU_data data_raw,EULER_angle data){
    char buffer[10];
    ftoa(data_raw.Gyro_x, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Gyro_y, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Gyro_z, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_x, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_y, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

    ftoa(data_raw.Acc_z, buffer, 2);
    strcat(buffer," ");
    HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

	print_euler(data);
}
void print_msg(char*msg){
	HAL_UART_Transmit(&huart3,(uint8_t*) msg, strlen(msg),1000);
}


void print_euler_compare(EULER_angle data, EULER_angle data1,EULER_angle data2){
	        char buffer[10];

            ftoa(data.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.pitch, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data.yaw, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data1.roll, buffer, 2);
            strcat(buffer," ");
            HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

            ftoa(data1.pitch, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data1.yaw, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.roll, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.pitch, buffer, 2);
             strcat(buffer," ");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);

             ftoa(data2.yaw, buffer, 2);
             strcat(buffer,"\n");
             HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
////
//            ftoa(data_raw.Mag_x, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_y, buffer, 2);
//         	strcat(buffer," ");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);
//
//         	ftoa(data_raw.Mag_z, buffer, 2);
//         	strcat(buffer,"\n");
//         	HAL_UART_Transmit(&huart3,buffer, strlen(buffer),1000);

}



#endif /* INC_PRINT_FUNC_HPP_ */
