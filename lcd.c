/* ===========================================================================

    Projeto da Aula 3 do Módulo 2 Curso AVR Projetos Profissionais
	
	LCD com microcontrolador AVR

    ARQUIVO lcd.c
    
    Projeto Principal

    MCU:   Atmega328p
    Clock: 16MHz Cristal Externo

    Compilador: Microchip Studio 7.0.2542
    Autor: Dr. Eng. Wagner Rambo
    Data de criação: Março de 2022
    Última atualização: 19/03/2022

============================================================================ */


/* ========================================================================= */
/* --- Funções do LCD --- */
#include "lcd.h"


/* ========================================================================= */
/* --- Desenvolvimento das Funções do LCD --- */


/* ========================================================================= */
/* --- disp_number --- */
/* Converte um inteiro para exibir no display, remove zeros à esquerda */
void disp_number(unsigned num, char row, char col)
{
  char  dem, mil, cen, dez, uni;               /* variáveis para cálculo de cada dígito */
  short  no_zero = 0;                          /* variável local para limpeza de zeros à esquerda */

  dem = (char)(num/10000);                     /* calcula dezenas de milhares */
  mil = (char)(num%10000/1000);                /* calcula milhares */
  cen = (char)(num%1000/100);                  /* calcula centenas */
  dez = (char)(num%100/10);                    /* calcula dezenas */
  uni = (char)(num%10);                        /* calcula unidades */

  if(!dem && !no_zero)                         /* dígito das dezenas de milhares é zero e no_zero está limpa? */
    disp_wr_po(' ',row,col);                   /* sim, imprime um espaço em branco */
  else                                         /* não... */
  {
    disp_wr_po(dem+0x30,row,col);              /* imprime o dígito calculado */
    no_zero = 1;                               /* seta no_zero */
  } /* end else */

  if(!mil && !no_zero)                         /* dígito dos milhares é zero e no_zero está limpa? */
    disp_write(' ');                           /* sim, imprime um espaço em branco */
  else                                         /* não... */
  {
    disp_write(mil+0x30);                      /* imprime o dígito calculado */
    no_zero = 1;                               /* seta no_zero */
  } /* end else */

  if(!cen && !no_zero)                         /* dígito das centenas é zero e no_zero está limpa? */
    disp_write(' ');                           /* sim, imprime um espaço em branco */
  else                                         /* não... */
  {
    disp_write(cen+0x30);                      /* imprime o dígito calculado */
    no_zero = 1;                               /* seta no_zero */
  } /* end else */

  if(!dez && !no_zero)                         /* dígito das dezenas é zero e no_zero está limpa? */
    disp_write(' ');                           /* sim, imprime um espaço em branco */
  else                                         /* não... */
  {
    disp_write(dez+0x30);                      /* imprime o dígito calculado */
    no_zero = 1;                               /* seta no_zero */
  } /* end else */

  disp_write(uni+0x30);                        /* imprime dígito calculado */


} /* end disp_number */


/* ========================================================================= */
/* --- disp_wr_po --- */
/* função para escrever caracteres no LCD na posição indicada */
void disp_wr_po(unsigned char chr, char row, char col)
{
  if(!row)                                     /* linha 0? */
  {                                            /* sim */
    disp_cmd(0x80|col);                        /* envia comando para posicionar na coluna correta */
    disp_write(chr);                           /* escreve o caractere */
  } /* end if */
  else                                         /* senão... */
  {                                            /* linha 1 */
    disp_cmd(0xC0|col);                        /* envia comando para posicionar linha e coluna corretas */
    disp_write(chr);                           /* escreve o caractere */
  } /* end else */

} /* end disp_write */


/* ========================================================================= */
/* --- disp_text --- */
/* função para escrever uma string no LCD */
void disp_text(char *str, char row, char col)
{
  register int i;                              /* variável local para iterações */

  for(i=0; str[i]!='\0';i++,col++)             /* executa até encontrar o caractere nulo */
    disp_wr_po(str[i],row,col);                /* imprime caractere atual da string passada como parâmetro */

} /* end disp_text */


/* ========================================================================= */
/* --- disp_write --- */
/* função para escrever caracteres no LCD */
void disp_write(unsigned char chr)
{
  send_nibble(chr, 1);                         /* envia o nibble mais significativo do caractere, RS em high */
  chr <<= 4;                                   /* atualiza chr para enviar nibble menos significativo */
  send_nibble(chr, 1);                         /* envia o nibble menos significativo do caractere, RS em high */

} /* end disp_write */


/* ========================================================================= */
/* --- disp_cmd --- */
/* função para enviar comandos para o LCD */
void disp_cmd(unsigned char cmd)
{
  send_nibble(cmd, 0);                         /* envia o nibble mais significativo do comando, RS em low */
  cmd <<= 4;                                   /* atualiza chr para enviar nibble menos significativo */
  send_nibble(cmd, 0);                         /* envia o nibble menos significativo do comando, RS em low */
   
} /* end disp_cmd */


/* ========================================================================= */
/* --- disp_init --- */
/* função para inicializar o LCD */
void disp_init(void)
{
  _delay_ms(48);                               /* tempo para estabilização (datasheet recomenda no mínimo 40ms) */
  send_nibble(0x30,0);                         /* protocolo de inicialização */
  _delay_ms(5);                                /* tempo acima do sugerido pelo datasheet pag.46) */
  send_nibble(0x30,0);                         /* protocolo de inicialização */
  _delay_us(150);                              /* tempo acima do sugerido pelo datasheet pag.46) */
  send_nibble(0x30,0);                         /* protocolo de inicialização */
  send_nibble(0x20,0);                         /* lcd no modo de 4 bits */
  disp_cmd(0x28);                              /* 5x8 pontos por caractere, duas linhas */
  disp_cmd(0x0F);                              /* liga display, cursor e blink */
  disp_cmd(0x01);                              /* clear LCD */
  disp_cmd(0x06);                              /* modo de incremento de endereço para direita */
  disp_clear();                                /* limpa LCD */

} /* end disp_init*/


/* ========================================================================= */
/* --- Limpa LCD --- */
/* função para limpar o LCD */
void disp_clear(void)
{
  disp_cmd(0x02);                              /* return home */
  disp_cmd(0x01);                              /* limpa o display */

} /* end disp_clear */


/* ========================================================================= */
/* --- send_nibble --- */
/* função para envio de cada nibble separadamente e gerar pulso em enable */
void send_nibble(unsigned char nib, char rsel)
{
  if((nib>>4)&0x01)                            /* envia bit 4 de comando no barramento */
    set_bit(reg1,D4);                          /* . */
  else                                         /* . */
    clr_bit(reg1,D4);                          /* . */

  if((nib>>5)&0x01)                            /* envia bit 5 de comando no barramento */
    set_bit(reg2,D5);                          /* . */
  else                                         /* . */
    clr_bit(reg2,D5);                          /* . */  

  if((nib>>6)&0x01)                            /* envia bit 6 de comando no barramento */
    set_bit(reg2,D6);                          /* . */
  else                                         /* . */
    clr_bit(reg2,D6);                          /* . */
	
  if((nib>>7)&0x01)                            /* envia bit 7 de comando no barramento */
    set_bit(reg2,D7);                          /* . */
  else                                         /* . */
    clr_bit(reg2,D7);                          /* . */
 
  
  if(rsel)                                     /* se rsel for verdadeiro */
    set_bit(reg1,RS);                          /* envio de dados */
  else                                         /* se rsel for falso */
    clr_bit(reg1,RS);                          /* envio de comandos */

  /* -- Pulse Enable -- */
  set_bit(reg1,EN);                            /* seta EN */
  _delay_us(1200);                             /* aguarda 1200µs */
  clr_bit(reg1,EN);                            /* limpa EN */
  _delay_us(1200);                             /* aguarda 1200µs */

} /* end send nibble */


/* ============================================================================

                                       _
                                      / \
                                     |oo >      <-- (Hello, liquid crystal display!)
                                     _\=/_
                    ___         #   /  _  \   #
                   /<> \         \\//|/.\|\\//
                 _|_____|_        \/  \_/  \/
                | | === | |          |\ /|
                |_|  0  |_|          \_ _/
                 ||  0  ||           | | |
                 ||__*__||           | | |
                |* \___/ *|          []|[]
                /=\ /=\ /=\          | | |
________________[_]_[_]_[_]_________/_]_[_\_______________________________


============================================================================ */
/* --- Final do Arquivo lcd.c --- */