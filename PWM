#include <Arduino.h>
#include <PWM.h>


//PWM

extern unsigned int PWM1, PWM2;

void setDuty_2(int duty) //MotorDireito
{
    OCR1A = duty; 

}

void setDuty_2 (int duty) //MotorEsquerdo
{

    OCR18 = dut; //registrador de PWM 


}


void setFreq (char option)
{
    /*
    TABLE:
  	//no fast Mode
        option  frequency (as frequências no timer 1 são menores do que as frequências nos timers 0 e 2)
        
          9      16    kHz
          10       2    kHz
          11     250     Hz
          12     62,5    Hz
          13     15,6    Hz 
     */
    TCCR1B = 12;
}
	void pwm_set_duty_service(unsigned int duty, unsigned char channel)
{   
  if (channel == 1)
  {
      OCR1A = duty; //valores de 0 - 1023
      return;
  }
  OCR1B = duty; //registrador de PWM do OC1B
}



