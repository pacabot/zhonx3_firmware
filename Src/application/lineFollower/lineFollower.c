/**************************************************************************/
/*!
    @file    line_follower.c
    @author   BM Pacabot.com
    @date     05 May 2015
    @version  0.01
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/tone/tone.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/wallFollowControl.h"

/* Declarations for this module */
#include "application/lineFollower/lineFollower.h"
#include "application/statistiques/statistiques.h"

line_follower_struct line_follower;
ground_sensors_struct max_Floor;	//global data to memorize maximum value of sensors
ground_sensors_struct coef_Floor;	//global data to memorize coeff value (0..1000]
ground_sensors_struct min_Floor;	//global data to memorize minimum value of sensors

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

void averageSensor(ground_sensors_struct, ground_sensors_struct * , int);


void averageSensor(ground_sensors_struct current, ground_sensors_struct *tab, int cpt)
{
    #define size 10
	ground_sensors_struct average;

	memcpy(&average, &current, sizeof(ground_sensors_struct) );

    int j= cpt%size;
    tab[j].front=current.front;
    tab[j].left=current.left;
    tab[j].right=current.right;
    tab[j].rightExt=current.rightExt;
    tab[j].leftExt=current.leftExt;
	if (cpt>size)
	{
	   for (int k=1;k<size;k++)
	   {
		   average.front += tab[k].front;
		   average.right += tab[k].right;
		   average.left += tab[k].left;
		   average.leftExt += tab[k].leftExt;
		   average.rightExt += tab[k].rightExt;
	   }
	   average.front/=10;
	   average.left/=10;
	   average.right/=10;
	   average.leftExt/=10;
	   average.rightExt/=10;

/*	   ssd1306ClearScreen();
	   ssd1306PrintInt(10, 15, ",",average.leftExt, &Font_5x8); //,left,front,right,rightExt
	   ssd1306PrintInt(10, 25, ",",average.left, &Font_5x8); //,left,front,right,rightExt
	   ssd1306PrintInt(10, 35, ",",average.front, &Font_5x8); //,left,front,right,rightExt
	   ssd1306PrintInt(10, 45, ",",average.right, &Font_5x8); //,left,front,right,rightExt
	   ssd1306PrintInt(10, 55, ",",average.rightExt, &Font_5x8); //,left,front,right,rightExt
	   ssd1306Refresh();*/
	}
	memcpy(&current, &average, sizeof(ground_sensors_struct) );
}

void lineSensortest(void)
{
	ground_sensors_struct current;
	ground_sensors_struct average;
	ground_sensors_struct tab[10];
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	int i=0;
	for (i=0;i<1000;i++)
	{

		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		memcpy(&average, &current, sizeof(ground_sensors_struct) );

		averageSensor(current, tab, i);


/*
       j= i%10;
       tab[j].front=current.front;
	   if (i>9)
	   {
	      for (int k=1;k<10;k++)
	      {
			  average.front += tab[k].front;
	      }
	      average.front/=10;
	   }*/
	}
}

void lineSensorSendBluetooth(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	motorsSleepDriver(OFF);
	int leftExt=0;
	int left=0;
	int front=0;
	int right=0;
	int rightExt=0;


	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		left=lineSensors.left.adc_value;
		front=lineSensors.front.adc_value;
		right=lineSensors.right.adc_value;
		leftExt=lineSensors.left_ext.adc_value;
		rightExt=lineSensors.right_ext.adc_value;
// ===================================================================================
// Envoi BLUETOOTH
// ===================================================================================

		bluetoothPrintf("%d , %d , %d , %d , %d \n",leftExt,left,front,right,rightExt);
		ssd1306ClearScreen();
		ssd1306DrawString(10, 5, "send hello ZHONX III", &Font_5x8);
		ssd1306PrintInt(10, 15, "",leftExt, &Font_5x8);
		ssd1306PrintInt(10, 25, ",",leftExt, &Font_5x8);
		ssd1306PrintInt(10, 35, ",",leftExt, &Font_5x8);
		ssd1306PrintInt(10, 45, ",",leftExt, &Font_5x8);
		ssd1306PrintInt(10, 55, ",",leftExt, &Font_5x8); //,left,front,right,rightExt
		ssd1306Refresh();

	}
	antiBounceJoystick();
}


//----------------------------------------------------------------
// Initialize data sensor to memorize the max and min value for each 5 sensors
void lineSensorsCalibration(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	motorsSleepDriver(OFF);

	ground_sensors_struct current;
	ground_sensors_struct average;
	ground_sensors_struct tab[10];
	ground_sensors_struct grostab[4000];

	tone(a, 500);
//	HAL_Delay(1000);
	move(45, 0, 100, 0);
	while(isEndMove() != TRUE){}
	move(-90, 0, 100, 0);
// -------------------------------------------------------------
// Init line Sensor

	max_Floor.left=lineSensors.left.adc_value;
	max_Floor.front=lineSensors.front.adc_value;
	max_Floor.right=lineSensors.right.adc_value;
	max_Floor.leftExt=lineSensors.left_ext.adc_value;
	max_Floor.rightExt=lineSensors.right_ext.adc_value;
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	memcpy(&current, &min_Floor, sizeof(ground_sensors_struct) );

	int i=0; int j=0;


	while(isEndMove() != TRUE)
	{

		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		if (current.left < min_Floor.left) min_Floor.left = current.left;
		if (current.front < min_Floor.front) min_Floor.front = current.front;
		if (current.right < min_Floor.right) min_Floor.right = current.right;
		if (current.leftExt < min_Floor.leftExt) min_Floor.leftExt = current.leftExt;
		if (current.rightExt < min_Floor.rightExt) min_Floor.rightExt = current.rightExt;

		if (current.left > max_Floor.left) max_Floor.left = current.left;
		if (current.front > max_Floor.front) max_Floor.front = current.front;
		if (current.right > max_Floor.right) max_Floor.right = current.right;
		if (current.leftExt > max_Floor.leftExt) max_Floor.leftExt = current.leftExt;
		if (current.rightExt > max_Floor.rightExt) max_Floor.rightExt = current.rightExt;

		memcpy(&average, &current, sizeof(ground_sensors_struct) );
		averageSensor(current, tab, i);
		i++;
		if(i%10==0)
		{
			if (j<4000)
			{
			    memcpy(&grostab[j], &average, sizeof(ground_sensors_struct) );
			}
			else
			{
				tone(e, 50);
			}
			j=j+1;
		}
	}
	move(45, 0, 100, 0);
	tone(b, 500);
	tone(c, 500);



	ssd1306ClearScreen();
	ssd1306PrintInt(5, 5,  "compteur ", (uint16_t) i, &Font_5x8);
	ssd1306PrintInt(80, 5,  "-", (uint16_t) j, &Font_5x8);

   ssd1306PrintInt(10, 15, ",",min_Floor.leftExt, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(10, 25, ",",min_Floor.left, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(10, 35, ",",min_Floor.front, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(10, 45, ",",min_Floor.right, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(10, 55, ",",min_Floor.rightExt, &Font_5x8); //,left,front,right,rightExt

   ssd1306PrintInt(42, 15, ",",max_Floor.leftExt, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(42, 25, ",",max_Floor.left, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(42, 35, ",",max_Floor.front, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(42, 45, ",",max_Floor.right, &Font_5x8); //,left,front,right,rightExt
   ssd1306PrintInt(42, 55, ",",max_Floor.rightExt, &Font_5x8); //,left,front,right,rightExt

	ssd1306Refresh();
	// desactivate PID
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
	HAL_Delay(20000);
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	control_params.line_follow_state = TRUE;
	motorsSleepDriver(OFF);


	if (max_Floor.left-min_Floor.left< 100.0)
	{
		tone(a, 3000);
		max_Floor.left=2000.0;
		max_Floor.front=2000.0;
		max_Floor.right=2000.0;
		max_Floor.leftExt=2000.0;
		max_Floor.rightExt=2000.0;
		min_Floor.left=150.0;
		min_Floor.front=150.0;
		min_Floor.right=150.0;
		min_Floor.leftExt=150.0;
		min_Floor.rightExt=150.0;
		tone(b, 3000);

	//	return;
	}

	tone(c, 100);
	coef_Floor.left=100.0/(max_Floor.left-min_Floor.left);     //  100/(max_capteur-min_capteur) (0..100)
	coef_Floor.front=100.0/(max_Floor.front-min_Floor.front);
	coef_Floor.right=100.0/(max_Floor.right-min_Floor.right);
	coef_Floor.leftExt=100.0/(max_Floor.leftExt-min_Floor.leftExt);
	coef_Floor.rightExt=100.0/(max_Floor.rightExt-min_Floor.rightExt);



	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) min_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) min_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) min_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) min_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) min_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
//	HAL_Delay(900);
	tone(c, 100);

	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) max_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) max_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) max_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) max_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) max_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
//	HAL_Delay(900);


//	HAL_Delay(500);

	line_follower.active_state = TRUE;
	move(0, 10000, MAXSPEED, 0);
//	while(isEndMove() != TRUE);
	char foreward = TRUE;
	char cpt=0;
	int  error;
	int coef;
	while(expanderJoyFiltered()!=JOY_LEFT && foreward)
	{
		//error=follow_control.follow_error*10;
		int left=((double)lineSensors.left.adc_value - min_Floor.left) * coef_Floor.left ;
		int front=((double)lineSensors.front.adc_value- min_Floor.front) * coef_Floor.front ;
		int right=((double)lineSensors.right.adc_value- min_Floor.right) * coef_Floor.right ;
		int left_ext=((double)lineSensors.left_ext.adc_value - min_Floor.leftExt) * coef_Floor.leftExt ;
		int right_ext=((double)lineSensors.right_ext.adc_value - min_Floor.rightExt) * coef_Floor.rightExt ;

		if (left_ext>left && left_ext>front && left_ext>right && left_ext> right_ext)
		{
			if (left<2)
			{
				coef=(-100+left_ext)*2-800;
			} else
			{
				coef=(100-left_ext)*2-800;
			}
		} else if (left>left_ext && left>front && left>right && left> right_ext)
		{
			if (left_ext>front)
			{
				coef=(-100+left)*2-400;
			} else
			{
				coef=(100-left)*2-400;
			}
		}else if (front>left_ext && front>left && front>right && front> right_ext)
		{
			if (right<=left)
			{
				coef=(-100+front)*2;
			} else
			{
				coef=(100-front)*2;
			}
		} else if (right>left_ext && right>left && right>front && right> right_ext)
		{
			if (right_ext<=right)
			{
				coef=(-100+right)*2+400;
			} else
			{
				coef=(100-right)*2+400;
			}
		} else if (right_ext>left_ext && right_ext>left && right_ext>front && right_ext> right)
		{
			if (right>2)
			{
				coef=(-100+right_ext)*2+800;
			} else
			{
				coef=(100-right_ext)*2+800;
			}
		}



		error=line_follower.position*200;
		ssd1306ClearScreen();
		ssd1306PrintInt(5, 5,  "LEFT_EXT  =  ", left_ext, &Font_5x8);
		ssd1306PrintInt(5, 15, "LEFT      =  ", left, &Font_5x8);
		ssd1306PrintInt(5, 25, "FRONT --  =  ", front, &Font_5x8);
		ssd1306PrintInt(5, 35, "RIGHT     =  ", right, &Font_5x8);
		ssd1306PrintInt(5, 45, "RIGHT_EXT =  ", right_ext, &Font_5x8);

		ssd1306PrintInt(5, 55, "coef=", coef, &Font_5x8);
		ssd1306PrintInt(70, 55, "I ", error, &Font_5x8);
		ssd1306Refresh();


// -----------------------------------------------------------------------
// Condition to stop zhonx if no line
// -----------------------------------------------------------------------
		if ((double)lineSensors.front.adc_value < min_Floor.front *1.2 &&
			(double)lineSensors.left.adc_value < min_Floor.left *1.2 &&
			(double)lineSensors.right.adc_value < min_Floor.right *1.2 &&
			(double)lineSensors.left_ext.adc_value < min_Floor.leftExt *1.2 &&
			(double)lineSensors.right_ext.adc_value < min_Floor.rightExt *1.2)
		{
			cpt++;
			if (cpt>5)
			{
			    foreward = FALSE;
			    move(0, 150, 250, 0);
			    tone(c, 500);tone(d, 500);
			}
		}
// -----------------------------------------------------------------------
// Condition to stop if right priority
// -----------------------------------------------------------------------
//		if (((double)lineSensors.left_ext.adc_value*1.2) > max_Floor.leftExt )
//		{
//			move(0, 30, 30, 0);
//			tone(c, 500);
//			// capteur telemeter ON
//			move(0, 10000, MAXSPEED, 0);
//		}

	}
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

//----------------------------------------------------------------------
// fonction pour asservir zhonx sur la ligne
//
void controlLoop(void)
{
	/*
	static int maxfront=0;  // memorize the max level of front sensors line

	int left=(lineSensors.left.adc_value - min_Floor.left) * coef_Floor.left ;
	int front=(lineSensors.front.adc_value- min_Floor.front) * coef_Floor.front ;
	int right=(lineSensors.right.adc_value- min_Floor.right) * coef_Floor.right ;
    int rightExt=(lineSensors.right_ext.adc_value - min_Floor.rightExt) * coef_Floor.rightExt;
    int leftExt=(lineSensors.left_ext.adc_value  - min_Floor.leftExt)* coef_Floor.leftExt;

    int midle=0;		// take account if the center sensor line is out the line
    int inside=right-left; //take account the sensor just right and left of front
    int	outside=0;	// take account the external sensor line


	if (inside>20)
	{
		midle=(maxfront-front);
	}
	else if (inside<-20)
	{
		midle=-(maxfront-front);
	}else
	{
		maxfront=front;
	}
	// check if we are for the center out of the line to take account the gaucheExt and droiteExt
//    if (devant<100)
//    {
//    	exterieur = droiteExt - gaucheExt;
//    }
*/
	int coef=0;
	int left=((double)lineSensors.left.adc_value - min_Floor.left) * coef_Floor.left ;
	int front=((double)lineSensors.front.adc_value- min_Floor.front) * coef_Floor.front ;
	int right=((double)lineSensors.right.adc_value- min_Floor.right) * coef_Floor.right ;
	int left_ext=((double)lineSensors.left_ext.adc_value - min_Floor.leftExt) * coef_Floor.leftExt ;
	int right_ext=((double)lineSensors.right_ext.adc_value - min_Floor.rightExt) * coef_Floor.rightExt ;

	if (left_ext>left && left_ext>front && left_ext>right && left_ext> right_ext)
	{
		if (left<2)
		{
			coef=(-100+left_ext)*2-800;
		} else
		{
			coef=(100-left_ext)*2-800;
		}
	} else if (left>left_ext && left>front && left>right && left> right_ext)
	{
		if (left_ext>front)
		{
			coef=(-100+left)*2-400;
		} else
		{
			coef=(100-left)*2-400;
		}
	}else if (front>left_ext && front>left && front>right && front> right_ext)
	{
		if (right<=left)
		{
			coef=(-100+front)*2;
		} else
		{
			coef=(100-front)*2;
		}
	} else if (right>left_ext && right>left && right>front && right> right_ext)
	{
		if (right_ext<=right)
		{
			coef=(-100+right)*2+400;
		} else
		{
			coef=(100-right)*2+400;
		}
	} else if (right_ext>left_ext && right_ext>left && right_ext>front && right_ext> right)
	{
		if (right>2)
		{
			coef=(-100+right_ext)*2+800;
		} else
		{
			coef=(100-right_ext)*2+800;
		}
	}
	coef = right-left;

    line_follower.position = (double)(coef) * 0.1;

}
void lineFollower_IT(void)
{
	// Rapide
//	static int vitesse=0;

	controlLoop();

//	if (follow_control.follow_error > 3.0 && vitesse==0)
//	{
//		// deceleration
//		move(0, 30, MAXSPEED, MINSPEED);
//		vitesse=-1;
//	}
//	else if (follow_control.follow_error < 3.0 && vitesse==0)
//	{
//		// acceleration
//		move(0, 30, MINSPEED, MAXSPEED);
//		vitesse=1;
//	}
//
//	if (isEndMove() == TRUE)
//	{
//		if (vitesse<0)
//		{
//			move(0, 10000, MINSPEED, MINSPEED);
//		}
//		else if (vitesse>0)
//		{
//			move(0, 10000, MAXSPEED, MAXSPEED);
//		}
//		vitesse=0;
//	}
	// -----------------------------------------------------------------------
	// Condition to stop zhonx if no line
	// -----------------------------------------------------------------------
	if ((double)lineSensors.front.adc_value < min_Floor.front *1.2 &&
		(double)lineSensors.left.adc_value < min_Floor.left *1.2 &&
		(double)lineSensors.right.adc_value < min_Floor.right *1.2 &&
		(double)lineSensors.left_ext.adc_value < min_Floor.leftExt *1.2 &&
		(double)lineSensors.right_ext.adc_value < min_Floor.rightExt *1.2)
	{
	    move(0, 150, 250, 0);
	}
}

