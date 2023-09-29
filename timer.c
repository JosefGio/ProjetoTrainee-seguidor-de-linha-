#include <Arduino.h>
#include <avr/interrupt.h>


/*Variáveis globais*/
unsigned int contagem_maxima_1;

/*protótipo das funções*/
void sub_temporizador(void);
void rotina1();
ISR(TIMER0_OVF_vect) //toda vez que ocorrer overflow do timer0 o código será desviado para esta função
{
  TCNT0 = 6;
  sub_temporizador();
}

int main()
{
  DDRC = 0x30;
  PORTC = 0x00;
  TCCR0B = 0x03;  //defino o prescaler em 64
  //cs01 = 1 e cs00 = 1
  TCNT0 = 6;  //começa em 6 e vai até 255 gerando um tempo de 1ms
  TIMSK0 = 0x01;  //habilito a interrupção do timer0
  sei();  //habilita a chave geral das interrupções
  pinMode(A5, OUTPUT);
  MaxTimer1 = 500
  while(1); //loop
  return 0;
}

void sub_temporizador(void)
{
  static unsigned int contador1 = 1;

  if(contador1 < contagem_maxima_1) contador1++;  //incremento se for menor que contagem_maxima_1

  else
  {
    rotina1();  //rotina chamada a cada 500ms
    contador1 = 1;  //reseto para fazer a contagem de 500ms
  }
}

void rotina1(void)
