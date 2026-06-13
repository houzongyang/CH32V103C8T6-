#include "GPS.h"

#include "Laser.h"
#include "Oled.h"
extern u8 Rx1Buffer[RxSize1] ;

void  GPS_Process(){
        char  tempbuffer[20]={0}; 
        u8 jing[8];      
        u8 wei[11];
        u32  jingNum;
        u32   weiNum;
        float jingf;
        float weif;

        memcpy(jing,&Rx1Buffer[31],8);
        memcpy(wei,&Rx1Buffer[44],11);
      
        jingNum = asciiToDigit(jing[0])*1000+asciiToDigit(jing[1])*100+asciiToDigit(jing[2])*10+asciiToDigit(jing[3])*1;
          weiNum=asciiToDigit(wei[0])*10000+asciiToDigit(wei[1])*1000+asciiToDigit(wei[2])*100+asciiToDigit(wei[1])*10+asciiToDigit(wei[0])*1;
        jingf=jingNum/100.0;
        weif=weiNum/100.0;

        snprintf(tempbuffer, sizeof(tempbuffer), "j:%.f,w:%.1f", jingf,weif);   
        OLED_ShowString(3, 1, tempbuffer);  
       



}





