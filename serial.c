/*
 * File:   serial.c
 * Author: henrik
 *
 * Created on 1. maj 2021, 17:35
 */


#include <avr/io.h>
#include "seri_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"




char UART_Init(const long int baudrate)
{
  unsigned int x;
  x = (_XTAL_FREQ - baudrate*64)/(baudrate*64); //SPBRG for Low Baud Rate
  if(x>255) //If High Baud Rage Required
  {
    x = (_XTAL_FREQ - baudrate*16)/(baudrate*16); //SPBRG for High Baud Rate
    BRGH = 1; //Setting High Baud Rate
  }
  if(x<256)
  {
    SPBRG = x; //Writing SPBRG Register
    SYNC = 0; //Setting Asynchronous Mode, ie UART
    SPEN = 1; //Enables Serial Port
    TRISC7 = 1; //As Prescribed in Datasheet
    TRISC6 = 1; //As Prescribed in Datasheet
    CREN = 1; //Enables Continuous Reception
    TXEN = 1; //Enables Transmission
    return 1; //Returns 1 to indicate Successful Completion
  }
  return 0; //Returns 0 to indicate UART initialization failed
}

void UART_Write(char data)
{
  while(!TRMT);
  TXREG = data;
}

char UART_TX_Empty()
{
  return TRMT;
}

void UART_Write_Text(char *text)
{
  int i;
  for(i=0;text[i]!='\0';i++)
    UART_Write(text[i]);
}

char UART_Data_Ready()
{
  return RCIF;
}

char UART_Read()
{
  while(!RCIF);
  return RCREG;
}

void UART_Read_Text(char *Output, unsigned int length)
{
  unsigned int i;
  for(int i=0;i<length;i++)
  Output[i] = UART_Read();
}
