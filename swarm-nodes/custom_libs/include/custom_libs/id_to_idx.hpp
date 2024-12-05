// This is the header file for the id conversion to idx

#ifndef ID_TO_IDX_H
#define ID_TO_IDX_H


#include <math.h>

class id_to_idx{

        public:

		int get_state_idx(int id, int *groups) ; // Given the id , this returns the idx on the state array before transforming
		int get_integral_idx(int id,int *groups) ; // Given the id , this returns the idx on the integral array before transforming
		int get_aug_state_idx(int id,int *groups) ; // Given the id , this returns the idx on the augmented state
		int get_group_nbr(int id,int *groups) ; // Given the id, this returns the group number the id belongs to, indexed from 1...N 
		int get_velocity_idx(int id,int *groups) ; //Given the id , this returns the idx on the velocity states
};



#endif
