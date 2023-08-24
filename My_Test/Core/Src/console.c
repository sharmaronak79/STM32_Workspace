/*
 * console.c
 *
 *  Created on: Aug 24, 2023
 *      Author: Ronakkumar_Sharma
 */
#include "console.h"
#include<string.h>
#include<stdio.h>
#include<stdlib.h>


/*Private variable Definition*/
static dynamic_menu_cmd_t addition_cmd =
{
   "add", NULL, "Make addition of Two values", true, true, NULL
};

static dynamic_menu_cmd_t subtraction_cmd =
{
   "add", NULL, "Make subtraction of Two values", true, true, NULL
};

static dynamic_menu_cmd_t multilication_cmd =
{
   "add", NULL, "Make addition of Two values", true, true, NULL
};

static dynamic_menu_cmd_t division_cmd =
{
   "add", NULL, "Make addition of Two values", true, true, NULL
};

/* Private Function Definition*/
void print(char msg[50]){
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
}

int add_op(char a[],char b[]){
	float val1,val2,result;
	val1=atof(a);
	val2=atof(b);
	char msg[50];

	result = val1 + val2;

	sprintf(msg,"\n\rAddition is  : %0.2f",result);
	print(msg);

	return 0;
}

int sub_op(char a[],char b[]){
	float val1,val2,result;
	val1=atof(a);
	val2=atof(b);
	char msg[50];

	result = val1 - val2;

	sprintf(msg,"\n\rSubtraction is  : %0.2f",result);
	print(msg);

	return 0;
}

int mul_op(char a[],char b[]){
	float val1,val2,result;
	val1=atof(a);
	val2=atof(b);
	char msg[50];

	result = val1 * val2;

	sprintf(msg,"\n\rMultiplication is  : %0.2f",result);
	print(msg);

	return 0;
}

int div_op(char a[],char b[]){
	float val1,val2,result;
	val1=atof(a);
	val2=atof(b);
	char msg[50];

	result = val1 / val2;

	sprintf(msg,"\n\rDivision is  : %0.2f",result);
	print(msg);

	return 0;
}
