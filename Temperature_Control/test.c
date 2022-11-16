#include<stdio.h>
#include<stdint.h>

 

int main(){   

uint16_t TACH_max = 983;
uint16_t TACH_min = 436;
uint16_t TACH = 600;
uint8_t TTH;
uint8_t TTL;
float set_temp=40;
float temp;
    

  printf("Enter value of temp : ");
  scanf("%d",&temp);

  if(temp > set_temp){
      TACH-- ;
      if(TACH <= TACH_min){
          TACH += 1;
      }
  }else if(temp<set_temp){
      TACH++;
      if(TACH >= TACH_max){
          TACH -= 1;
      }
  }else{
      TACH;
  }

  TTL = (TACH<<3) & 0xFF;
  TTH = (TACH>>5) & 0xFF;
  printf("TTL : %d \n", TTL);
  printf("TTH : %d \n", TTH);

    

    return 0;
}
