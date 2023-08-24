/*
 * console.h
 *
 *  Created on: Aug 22, 2023
 *      Author: Ronakkumar_Sharma
 */

#ifndef INC_CONSOLE_H_
#define INC_CONSOLE_H_

#include "main.h"
#include<stdbool.h>

/*Private variable Declaration*/
extern char arg1[10];
extern char arg2[10];

extern UART_HandleTypeDef huart3;

typedef struct dynamic_menu_cmd
{
   char *opt;                       /* command name */
   void (*func)(char *);            /* function to execute the command */
   char *desc;                      /* command description */
   bool public;                     /* is the command displayed by the help? */
   bool execute;                    /* can the command be executed? */
   struct dynamic_menu_cmd *next;
} dynamic_menu_cmd_t;


/* Private Function Declaration*/
int add_op(char arg1[],char arg2[]);
int sub_op(char arg1[],char arg2[]);
int mul_op(char arg1[],char arg2[]);
int div_op(char arg1[],char arg2[]);

void print(char *);

#endif /* INC_CONSOLE_H_ */
