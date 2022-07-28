#include<Arduino.h>
#include "uart.h"
#include <avr/interrupt.h>
#include "ADC.h"
#include <stdio.h>
#include <util/delay.h>
#include "PWM.h"

#define set_bit(reg, bit) (reg|= (1 << bit))
#define clear_bit(reg, bit) (reg &= ~(1 << bit))
#define test_bit(reg,bit)  (reg & (1<<bit)) 

#define In1 PB5 //13
#define In2 PB4 //14  
#define In3 PD7 //7  
#define In4 PD6 //6 
#define ENA PB2 //10 
#define ENB PB1 //9 

/*#define In1 PD4 //direita
#define In2 PD5 //direita
#define In3 PD6 //esquerda
#define In4 PD7 //esquerda*/

int PotPrim = 510; // Potência do motor principal da curva
int PotSec = 125;      // Potênica do motor secundário durante a curva
int PotFrente = 255;   // Potência quando o robô estiver seguindo em frente

// Sensores//
#define s2 A0
#define s3 A1
#define s4 A2
#define s5 A3
#define s6 A4
#define s7 A5
#define sp 3

/* Valor dos Sensores
int valors2 = 0;
int valors3 = 0;
int valors4 = 0;
int valors5 = 0;
int valors6 = 0;
int valors7 = 0; */
//valor do Sensor lateral
int valorsp = 0; 

// PID
float Kp = 0.6, Ki = 0.0, Kd = 0.0; // constante de Proporcionalidade, Integral e Diferencial
int P = 0, I = 0, D = 0, PID = 0;
double Setpoint = 0;
int erro = 0, erro_anterior = 0;

signed int media_p;
int valor_max = 244;
int valor_min = 203;

char valor_parada = 0;
char flag_stop = 0;


unsigned char sensores[6];
char buffer[10];
char tempo = 0;
int PWM_esquerdo; // Potprim-pid
int PWM_direito;      //Potprim+pid
// Declaração de funções

void parada(void);
void timers1(void);
void f_timer1(void);
void f_timer2(void);
void calculo_erro(void);
void calculoPID(int erro);
void controle_motor(void);
void MP(void);
void leitura_sensores(void);

ISR(ADC_vect)
{
  tratar_leitura_do_ADC();
}

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6;
  timers1();
}

int main()
{
  setup();
  
  adc_setup();
  tratar_leitura_do_ADC();
  TCCR0B = 0x03;   // defino o prescaler em 64
  TCNT0 = 6;       // começa em 6 e vai até 255 gerando um tempo de 1ms
  TIMSK0 = 0x01;   // habilito a interrupção do timer0
  uart_setup(103); // 9600bps
  PWM_init();
  setup_pwm_setFreq(12);

  sei(); // habilita a chave geral das interrupções

  while (1)
    loop();
}

void setup(void)
{
  // Motores
  DDRD = 0b11110110;
  DDRB = 0b11111111; //Saídas 4,5,6,7 para motores, 1 para TX ; Entrada 3 para sensor lateral e Entrada 0 para RX 
  // Sensores
  DDRC  = 0x00;

}

void loop(void)
{
 

  if(!flag_stop)
  {
    
    parada();
  }
}


void MP(void)
{
  leitura_sensores();

  int p2 = -3000, p3 = -2000, p4 = -1000, p5 = 1000, p6 = 2000, p7 = 3000;

  static int denominador = 0;
  for(int i = 0; i < 6; i++)
  {
    denominador += sensores[i];
  }

  if(!denominador)  media_p = 0;

  else
  {
    media_p = (((sensores[0] * p2) + (sensores[1] * p3) + (sensores[2] * p4) + (sensores[3] * p5) + (sensores[4] * p6) + (sensores[5] * p7)) / denominador);
  }

  for(int i = 0; i < 6; i++)
  {
    sprintf(buffer, "%d\t", sensores[i]);
    uart_string_sending_service(buffer);
  }
  sprintf(buffer, "%d\n", media_p);
  uart_string_sending_service(buffer);
  calculo_erro();

  controle_motor();
}


void timers1(void)
{

  static unsigned int c_timer1 = 0;
 

  if (c_timer1 < 10)
  {
    c_timer1++;
  }

  else
  {
    f_timer1();
    c_timer1 = 0;
  }

}

void f_timer1(void)
{
  MP();
}

void calculo_erro(void)
{

  erro = Setpoint - media_p;

  calculoPID(erro);

 
}
void calculoPID(int erro)
{
  P = erro;  // Proporcional
  I += erro; // Integral

  D = erro - erro_anterior;
  erro_anterior = erro;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
}

void controle_motor(void)
{

  PWM_esquerdo = PotPrim - PID;
  PWM_direito = PotPrim + PID;

  set_bit(PORTB, PB5);
  clear_bit(PORTB, PB4);
  set_bit(PORTD, PD6);
  clear_bit(PORTD, PD7);

if (PWM_esquerdo < 0)
  {
    set_bit(PORTB, PB5);
    clear_bit(PORTB, PB4);
    clear_bit(PORTD, PD6);
    set_bit(PORTD, PD7);
}
  else if (PWM_direito < 0)
  {
    clear_bit(PORTB, PB5);
    set_bit(PORTB, PB4);
    set_bit(PORTD, PD6);
    clear_bit(PORTD, PD7);
  }
  pwm_set_duty_service(PotPrim - PID, PWM_CHANNEL_1);
  pwm_set_duty_service(PotPrim + PID, PWM_CHANNEL_2);

}
void leitura_sensores(void)
{
  for(int i = 0; i < 6; i++)
  {
    if(AD_pins[i] > valor_max)
    {
      sensores[i] = valor_max;
    }

    else if (AD_pins[i] < valor_min)
    {
      sensores[i] = valor_min;
    }
    else
    {
      sensores[i] = AD_pins[i];
    }
  }
}

void parada (void)
{

  if (valorsp < valor_max)
  {
    valor_parada++;
  }
  else if (valorsp < valor_max && valor_parada >= 8)
  {
    valor_parada = 0;
    delay(200);
    flag_stop = 1;
  }
}




// https://replit.com/ //
