/*
 * EKF.hpp
 *
 *  Created on: Feb 7, 2020
 *      Author: tung
 */

#ifndef SRC_EKF_HPP_
#define SRC_EKF_HPP_

#include "Matrix.hpp"
#define  PI 3.141592654
Matrix P;
Matrix Q;
Matrix R;
Matrix K;
Matrix A;
Matrix B;
Matrix x;
void init_EKF();
void update_EKF();



#endif /* SRC_EKF_HPP_ */
