/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>
#include<stdint.h>

int main()
{
   uint16_t TACH = 800;// for 4000 RPM , TACH = 983, and for 9000 RPM , TACH = 436, so TACH will be between 436 to 983 
   uint8_t TTL = (TACH<<3) & 0xFF;
   uint8_t TTH = (TACH>>5) & 0xFF;
   printf("TTL : %d \n", TTL);
   printf("TTH : %d \n", TTH);
    

    return 0;
}
