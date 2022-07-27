#include <Arduino.h>
//#include <SoftwareSerial.h>
//#include "time.h"
#include "uart.h"
#include <avr/interrupt.h>
#include "ADC.h"
#include <stdio.h>
#include <util/delay.h>

//String inputString = ""; //string necessária para ler as informações no app pelo celular

/*int DirVel = ;  // Velocidade do motor direito
int EsqVel = 7;  // Velocidade do motoe esquerdo
int DirCont = 6; // Controle direito
int EsqCont = ; // Controle esquerdo*/

#define In1 PD4 //direita
#define In2 PD5 //direita
#define In3 PD6 //esquerda
#define In4 PD7 //esquerda

int PotPrim = 255;   // Potência do motor principal da curva
int PotSec = 125;     // Potênica do motor secundário durante a curva
int PotFrente = 255; // Potência quando o robô estiver seguindo em frente

// Sensores//
#define s2  A0
#define s3  A1
#define s4  A2
#define s5  A3
#define s6  A4
#define s7  A5
#define sp  3

// Valor dos Sensores//
int valors2 = 0;
int valors3 = 0;
int valors4 = 0;
int valors5 = 0;
int valors6 = 0;
int valors7 = 0;
int valorsp = 0;
int acionador = 0, crono = 0;

// Variáveis Bluetooth
int junk = 0;

// PID
float Kp =0.495, Ki =0.425, Kd =0.05; // constante de Proporcionalidade, Integral e Diferencial
int P = 0, I = 0, D = 0, PID = 0;
double Setpoint = 500;
int erro = 0, erro_anterior = 0;
char p2,p3,p4,p5,p6,p7;
unsigned int media_p;

unsigned char flag_timer = 0; //flag pra ativar o contador 
unsigned int MaxTimer1;
int flag_curva = 0;
int flag_


int linha = 650; // valor de referência que indica se o sensoer está lendo a linha branca ou o tapete preto

unsigned int sensores [6];
char buffer [10];

// Declaração de funções

void parada(void);
void timers1(void);
void f_timer1(void);
void f_timer2(void);
void calculo_erro(void);
void calculoPID(void);
void controle_motor(void);
void bluethooth(void);
void Walk(void);
void MP (void);

ISR(ADC_vect)
{
  tratar_leitura_do_ADC();

}

ISR(TIMER0_OVF_vect)
{
  TCNT0 = 6;
  timers1();
}
// SoftwareSerial(0 , 1) //entradas RX e TX

int main()
{
  //setup();
  DDRC = 0x00;
  adc_setup();
  tratar_leitura_do_ADC();
  TCCR0B = 0x03; // defino o prescaler em 64
  TCNT0 = 6;     // começa em 6 e vai até 255 gerando um tempo de 1ms
  TIMSK0 = 0x01; // habilito a interrupção do timer0
  uart_setup(16);//57600bps
  sei();         // habilita a chave geral das interrupções
  //MaxTimer1 = 500;
  while(1)  loop();
}

void setup(void)
{

  // Motores
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT); // DDRD = 0b11111110
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9 , OUTPUT);
  pinMode(10, OUTPUT);

  // Sensores
  pinMode(A0, INPUT); //
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT); // input dos 6 sensores frontais + sensor lateral
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(3, INPUT); //

  //Serial.begin(9600); // Serial para Monitor Arduino para debug
  pinMode(0, INPUT);  // RX
  pinMode(1, INPUT);  // TX
  
}

void loop(void)
{

  for (int i = 0; i < 6; i++)
  {
    sensores[i] = AD_pins[i];
  }
  
  
  //calculo_erro();
  //calculoPID();
  //controle_motor();
  //Walk(); 

 //bluethooth();

  //parada();

  for (int i = 0; i < 6; i++)
  {
    sprintf(buffer, "%d\t", sensores[i]);
    uart_string_sending_service(buffer);
  }

  uart_caractere_sending_service('\n');

  _delay_ms(200);
}

void MP (void)
{
  for (int i ; i < 255; i++);
{
  media_p = ((valors2*p2 + valors3*p3 + valors4*p4 + valors5*p5 + valors6*p6 +valors7*p7));
}

if (media_p > 0)
{
  calculo_erro();
  calculoPID();
  controle_motor();

}


}
/*
 frente =
 analogWrite(DirVel, PotFrente); //velocidade mínima do motor equivale a 0
 analogWrite(EsqVel, PotFrente); //velociade máxima do motor equivale a 255
 digitalWrite(DirCont, LOW); //motor girando no sentido antihorário
 digitalWrite(EsqCont, LOW); //motor girando no sentido antihorário

 trás =
 analogWrite(DirVel, PotFrente); //velocidade mínima do motor equivale a 0
 analogWrite(EsqVel, PotFrente); //velociade máxima do motor equivale a 255
 digitalWrite(DirCont, HIGH); //motor girando no sentido horário
 digitalWrite(EsqCont, HIGH); //motor girando no sentido horário

 curva_direita =
 analogWrite(DirVel, 100);
 analogWrite(EsqVel, 100);
 digitalWrite(DirCont, HIGH);
 digitalWrite(EsqCont, LOW);

 curva_esquerda =
 analogWrite(DireVel, 100);
 analogWrite(EsqVel, 100);
 digitalWrite(DirCont, LOW);
 digitalWrite(EsqCont, HIGH);

*/

void timers1(void)
{
  
  static unsigned char c_timer1 = 0;
  static unsigned char c_timer2 = 0;

  if (c_timer1 < 500)
  {
    c_timer1++;
  }

    else
    {
      f_timer1();
      c_timer1 = 0;
    }

  if (c_timer2 < 300)
  {
    c_timer2++;
  }
    else
    {
      f_timer2();
      c_timer2 = 0;
    }
    
}



void calculo_erro(void)
{

  valors2 = analogRead(A0);
  valors3 = analogRead(A1);
  valors4 = analogRead(A2);
  valors5 = analogRead(A3);
  valors6 = analogRead(A4);
  valors7 = analogRead(A5);

  if ((valors2 >= linha) && (valors3 >= linha) && (valors4 <= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = 0;
  }
  else if ((valors2 >= linha) && (valors3 <= linha) && (valors4 <= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = 1;
  }
  else if ((valors2 >= linha) && (valors3 >= linha) && (valors4 <= linha) && (valors5 <= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = -1;
  }
  else if ((valors2 >= linha) && (valors3 >= linha) && (valors4 >= linha) && (valors5 <= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = 1.5;
  }
  else if ((valors2 >= linha) && (valors3 <= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = -1.5;
  }
  else if ((valors2 >= linha) && (valors3 >= linha) && (valors4 >= linha) && (valors5 <= linha) && (valors6 <= linha) && (valors7 >= linha))
  {
    erro = 1.75;
  }
  else if ((valors2 <= linha) && (valors3 <= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = -1.75;
  }
  else if ((valors2 <= linha) && (valors3 <= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = 1.9;
  }
  else if ((valors2 <= linha) && (valors3 <= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 <= linha))
  {
    erro = -1.9;
  }
  else if ((valors2 >= linha) && (valors3 >= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 <= linha) && (valors7 <= linha))
  {
    erro = 2;
  }
  else if ((valors2 <= linha) && (valors3 >= linha) && (valors4 >= linha) && (valors5 >= linha) && (valors6 >= linha) && (valors7 >= linha))
  {
    erro = -2;
  }
}
void calculoPID(void)
{
  if (erro == 0)
  {
    I = 0;
  }

  P = erro;  // Proporcional
  I += erro; // Integral

  if (I > 255)
  {
    I = 255;
  }
  else if (I < -255)
  {
    I = -255;
  }

  D = erro - erro_anterior;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
}

void controle_motor(void)
{
  if (PID >= 0)
  {
    PD4 = PotPrim;
    PD6 = PotPrim - PID;
  }
  else
  {
    PD4 = PotPrim + PID;
    PD6 = PotPrim;
  }
}

/*Protótipo do código que será utilizado para
mandar informações do módulo bluetooth HC-05
até um aplicativo de celular*/
void bluethooth(void)
{
  //if (Serial.available())
  //{
   // while (Serial.available())
   // {
      //char inChar = (char)Serial.read(); // lê a serial
      //inputString += inChar;             // monta a string
   // }
    //Serial.println(inputString);
   // while (Serial.available() > 0)
   // {
     // junk = Serial.read(); // limpa buffer da serial
    //}
    /* if (inputString = "a")
   { digitalWrite(11, HIGH);}
       else if (inputString =="b")
       {digitalWrite (8, OUTPUY)
   {
     inputString =" " //Limpa strig da Serial


   }

   }

  */
  //}
}

//////////////////////////////////////////////////

void Walk(void)
{
  valors2 = analogRead(A0);
  valors3 = analogRead(A1);
  valors4 = analogRead(A2);
  valors5 = analogRead(A3);
  valors6 = analogRead(A4);
  valors7 = analogRead(A5);

  if (valors2 > linha && valors7 > linha)
  {
    analogWrite(PD4, PotFrente); // velocidade mínima do motor equivale a 0
    analogWrite(PD6, PotFrente); // velociade máxima do motor equivale a 255
    digitalWrite(PD5, LOW);     // motor girando no sentido antihorário
    digitalWrite(PD7, LOW);     // motor girando no sentido antihorário
  }

  else if (valors2 < linha && valors3 < linha)
  {
    analogWrite(PD4, PotSec);
    analogWrite(PD6, PotFrente);
    digitalWrite(PD5, LOW);
    digitalWrite(PD6, LOW);
  }

  else if (valors2 > linha && valors3 > linha && valors4 < linha)
  {
    analogWrite(PD4, PotSec);
    analogWrite(PD6, PotFrente);
    digitalWrite(PD5, LOW);
    digitalWrite(PD7, LOW);
  }

  else if (valors2 > linha && valors3 > linha && valors4 > linha && valors5 < linha)
  {
    analogWrite(PD4, PotSec);
    analogWrite(PD6, PotFrente);
    digitalWrite(PD5, LOW);
    digitalWrite(PD7, LOW);
  }

  else if (valors2 > linha && valors3 > linha && valors4 > linha && valors5 > linha && valors6 < linha)
  {
    analogWrite(PD4, PotSec);
    analogWrite(PD6, PotFrente);
    digitalWrite(PD5, LOW);
    digitalWrite(PD6, LOW);
  }
}

void parada(void)
{
  if ((valorsp < linha) && (acionador == 0))
  {
    crono++;
    acionador = 1;

  }
  if ((valorsp > linha) && (acionador = 1))
  {
    acionador = 0;
  }
  while (crono >= linha)
  {
    digitalWrite (PD4, 0);
    digitalWrite (PD6, 0);
    analogWrite (PD5, LOW);
    analogWrite (PD7, LOW);
  }
  
}

void rotina (void)
{
  

}
