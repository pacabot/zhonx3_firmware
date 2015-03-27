/**************************************************************************/
/*!
    @file    transfertFunction.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __TRANSFERTFUNCTION_H__
#define __TRANSFERTFUNCTION_H__

/* Module Identifier */
#define TRANSFERT_FUNCTION_MODULE_ID  105

/* Error codes */
#define TRANSFERT_FUNCTION_E_SUCCESS  0
#define TRANSFERT_FUNCTION_E_ERROR    MAKE_ERROR(TRANSFERT_FUNCTION_MODULE_ID, 1)

/* Types definitions */
typedef struct
{
	int left_motor_pwm;
	int right_motor_pwm;
	float pwm_ratio;

}transfert_function_struct;

extern transfert_function_struct transfert_function;

int transfertFunctionInit(void);
int transfertFunctionLimiter(void);
int transfertFunctionLoop(void);

#endif //TRANSFERTFUNCTION
