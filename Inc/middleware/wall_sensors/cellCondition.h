/*
 *  cellCondition.h
 *
 *  Created on: 25 avr. 2015
 *      Author: PLF
 */

#ifndef CELL_CONDITION_H_
#define CELL_CONDITION_H_

#define CELL_CONDITION_E_SUCCESS  0
#define CELL_CONDITION_E_ERROR    MAKE_ERROR(CELL_CONDITION_MODULE_ID, 1)

/* Types definitions */
typedef struct
{
	char left_front_wall;
	char right_front_wall;
	char left_diag_wall;
	char right_diag_wall;
}cell_state_struct;

extern cell_state_struct cell_state;

#endif /* CELL_CONDITION_H_ */
