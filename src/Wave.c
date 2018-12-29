#include "includes.h"

// ===== Variables ======
//---- GLOBAL ----

U32 wavetimef;
U32 wavetime;
U32 wavetimeus;
int distance;
int distance_tmp;
int distance_last;
int distance_buffer[5];
int distance_ave;
int distance_diff;
int distance_sum;

WaveState waveState;
uint8 wave_lost_cnt = 0;
uint8 wave_abslost_cnt = 0;

void StartUltrasound(u8 x){
  if(x)
    PTB->PSOR |= 1<<23;
  else
    PTB->PCOR |= 1<<23;
}

void Wave_Init()
{
  PORTB->PCR[21] |= PORT_PCR_MUX(1) |PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11);    // CCD2AO接口用作输入，超声波接收
  PORTB->PCR[23] |= PORT_PCR_MUX(1);    // CCD1AO接口用作输出，超声波发送
  PTB->PDDR |= (0x1<<23);
  NVIC_EnableIRQ(PORTB_IRQn);
  NVIC_SetPriority(PORTB_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 1)); //PORTC中断服务程序在cam.c中
  waveState = LOST;
}

void PORTB_IRQHandler(){
  if((PORTB->ISFR)&PORT_ISFR_ISF(1 << 21)){  //超声波接收中断
    PORTB->ISFR |= PORT_ISFR_ISF(1 << 21);
    if((PTB->PDIR>>21)&1)  
    {
      wavetimef=PIT2_VAL();
      wave_abslost_cnt = 0;
    }
    else 
    {
        wavetime=wavetimef-PIT2_VAL();
        wavetimeus = wavetime / (g_bus_clock/1000000); //1us
        distance_tmp=wavetimeus*34/200;    //距离单位//毫米
        
        switch(waveState)
        {
           case STABLE :
             if( ((distance_tmp - distance_last) <= 80)&&((distance_tmp - distance_last) >= -80) )
              {
                distance = distance_tmp;
                wave_lost_cnt = 0;
                if(waveState == ABSLOST) waveState = LOST;
               }
             break;
           
            case LOST :
              for(int i = 0;i<4;++i)
              {
                distance_buffer[i] = distance_buffer[i+1];
              }
              distance_buffer[4] = distance_tmp;
              
              distance_sum = 0;
              for(int i = 0;i<5;++i)
              {
                distance_sum += distance_buffer[i];
              }
              distance_ave = distance_sum/5;
              
              distance_diff = 0;
              for(int i = 0;i<5;++i)
              {
                distance_diff += abs(distance_buffer[i]-distance_ave);
              }
              
              if( distance_diff<20 && distance_ave<800) 
              {
                distance = distance_ave;
                distance_last = distance;
                waveState = STABLE;
              }
              
              break;
              
            default:
              break;
        }
        
        distance_last = distance;
    }
  }
}
