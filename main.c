/*///////////////////////////////////*
 * File:   main.c                    * 
 * Autor: Josef                      *
 *                                   *
 * Criado dia 7 de Julho de 2022     *
 * Último update 29 de Julho de 2022 *
 *///////////////////////////////////*



#include <Arduino.h>
#include "uart.h"
#include <avr/interrupt.h>
#include "ADC.h"
#include <stdio.h>
#include <util/delay.h>
#include "PWM.h"

#define set_bit(reg, bit) (reg |= (1 << bit))
#define clear_bit(reg, bit) (reg &= ~(1 << bit))
#define test_bit(reg, bit) (reg & (1 << bit))

#define In1 PB5 // 13
#define In2 PB4 // 12
#define In3 PD7 // 7
#define In4 PD6 // 6
#define ENA PB2 // 10
#define ENB PB1 // 9

int PotPrim = 510;   // Potência do motor principal da curva
int PotSec = 125;    // Potênica do motor secundário durante a curva
int PotFrente = 255; // Potência quando o robô estiver seguindo em frente

// Sensores//
#define s2 PC0
#define s3 PC1
#define s4 PC2
#define s5 PC3
#define s6 PC4
#define s7 PC5
#define sp PD3

// valor do Sensor lateral
int valorsp = 0;

// PID
float Kp = 2.5, Ki = 0., Kd = 2.5; // constante de Proporcionalidade, Integral e Diferencial
int P = 0, I = 0, D = 0, PID = 0;
double Setpoint = 0;
int erro = 0, erro_anterior = 0;

signed int media_p;
int valor_max = 235;
int valor_min = 212;

char valor_parada = 0;
char flag_stop = 0;

unsigned char sensores[6];
char buffer[10];
int pesos[6] = {-300, -200, -100, 100, 200, 300};
char tempo = 0;
int PWM_esquerdo; // Potprim-pid
int PWM_direito;  // Potprim+pid
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
  DDRB = 0b11111111; // Saídas 4,5,6,7 para motores, 1 para TX ; Entrada 3 para sensor lateral e Entrada 0 para RX
  // Sensores
  DDRC = 0x00;
}

void loop(void)
{

  if (!flag_stop)
  {

    parada();
  }
}

void MP(void)
{
  leitura_sensores();

  int denominador = 0;
  int numerador = 0;

  for (int i = 0; i < 6; i++)
  {
    denominador += sensores[i];
  }
  numerador = ((sensores[0] * pesos[0]) + (sensores[1] * pesos[1]) + (sensores[2] * pesos[2]) + (sensores[3] * pesos[3]) + (sensores[4] * pesos[4]) + (sensores[5] * pesos[5]));

  if (!denominador)
    media_p = 0;

  else
  {
    media_p = (numerador) / (denominador);
  }

  for (int i = 0; i < 6; i++)
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
  for (int i = 0; i < 6; i++)
  {
    if (AD_pins[i] > valor_max)
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

void parada(void)
{
  digitalRead(PD3);

  if (valorsp < valor_max)
  {
    valor_parada++;
  }
  else if (sensores[PD3] < valor_max && valor_parada >= 8)
  {
    valor_parada = 0;

    flag_stop = 1;
  }
}
