#include "EKF.h"

void init_EKF(){
   float Q_ = {0,1,2,3,4,5,6,7,8};
   P = diag_mat(7,7);
   Q = init_mat(3,3);
   Q.data = &Q_;
   float a = Q.data[0];
   float ba = Q.data[6];

}
