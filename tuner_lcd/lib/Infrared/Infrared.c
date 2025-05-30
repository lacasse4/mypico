/*****************************************************************************
* | File        :   Infrared.C
* | Function    :   test Demo
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-03-16
* | Info        :   
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "Infrared.h"

int Infrared_PIN;

/******************************************************************************
function:	Configure pins for pull-up input
parameter:
		PIN		： GPIO 
******************************************************************************/
void SET_Infrared_PIN(uint8_t PIN)
{
    Infrared_PIN = PIN;
    DEV_GPIO_Mode(Infrared_PIN, 0);
    gpio_pull_up(Infrared_PIN);//Need to pull up
}

/******************************************************************************
function:	Read the value of the pin
parameter:
******************************************************************************/
int Read_Infrared_Value(void)
{
    uint8_t i,idx,cnt;
    uint8_t count;
    uint8_t data[4];
    data[0]=0;
    data[1]=0;
    data[2]=0;
    data[3]=0;
    if(DEV_Digital_Read(Infrared_PIN) == 0){
        count = 0;
        while(DEV_Digital_Read(Infrared_PIN) == 0 && count++ < 200)   //9ms
            sleep_us(60);
        
        count = 0;
        while(DEV_Digital_Read(Infrared_PIN) == 1 && count++ < 80)	  //4.5ms
            sleep_us(60);
        
        idx = 0;
        cnt = 0;
        data[0]=0;
        data[1]=0;
        data[2]=0;
        data[3]=0;
        for(i =0;i<32;i++)
        {
            count = 0;
            while(DEV_Digital_Read(Infrared_PIN) == 0 && count++ < 15)  //0.56ms
                sleep_us(60);
            
            count = 0;
            while(DEV_Digital_Read(Infrared_PIN) == 1 && count++ < 40)  //0: 0.56ms; 1: 1.69ms
                sleep_us(60);

            if (count > 25)data[idx] |= (1<<cnt);
            if(cnt == 7)
            {
                cnt = 0;
                idx++;
            }
            else cnt++;
        }

        if(data[0]+data[1] == 0xFF && data[2]+data[3]==0xFF){	//check	
            // printf("Get the key: 0x%02x\n",data[2]);
            return data[2];
        }else {
            return -1;//Get data check error
        }
    }
    
    return -2;//no data
}

