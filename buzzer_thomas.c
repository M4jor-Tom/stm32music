/******************************************************************************/
/*                                                                            */
/*                      MAIN BASIQUE STM32L152RE                              */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
/*                                                                            */
/*                       Olivier MAINARD                                      */
/*                        Octobre 2020                                        */
/*                                                                            */
/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/


#include "stm32l1xx_nucleo.h"

void init_TIM3(void)
{
  RCC->APB1ENR |= (1<<1); /* force bit1 at 1 activating the TIM2 on APB1 data bus */
  
  TIM3->PSC =  7999; /*PSC+1=1599+1=1600 soit F=10kHz donc T=100us*/
  TIM3->ARR = 3; /*40*100us = 4ms donc ARR = 40-1 =39*/
  
  TIM3->CR1 |= (1<<7); /*ARPE Autoreload register buffered*/
  TIM3->CR1 &= ~(1<<6); /*CMS edge alligned*/
  TIM3->CR1 &= ~(1<<5); /*CMS edge alligned*/
  TIM3->CR1 &= ~(1<<4); /*DIR counter*/
  
  TIM3->CR1 |= (1<<0); /*CEN counter enable*/
}

void init_PWM(void)
{
  TIM3->CCMR1 &= ~((1<<8) | (1<<9)); /*define CH1 as an output*/
  TIM3->CCMR1 &= ~(1<<12); /*PMW mode 1*/
  TIM3->CCMR1 |= ((1<<13) | (1<<14)); /*PMW mode 1*/
  
  TIM3->CCR2 = 1; /*duty cycle*/
  
  TIM3->CCER |= (1<<4); /*signal enable*/
}

void init_MOTEUR_EXT(void)
{
  /*force b1 at 1 activating the GPIOB on AHB data bus*/
  RCC->AHBENR |= (1<<2);
  
  /*set the port GPIOB as an alternative function*/
  
  GPIOC->MODER |= (1<<15); /*set MODER4 to 10 (OX1)*/
  GPIOC->MODER &= ~(1<<14);
  
  /*set the port GPIOB in a push-pull mode (deliver current)*/
  GPIOC->OTYPER &= ~(1<<7);
  
  GPIOC->AFR[0] &= ~(1<<28);
  GPIOC->AFR[0] |= (1<<29); /*registre de s�lection de la fonction multiplex�e*/
                            /*TIM3 AFRL(PB4) 0010 bit 17 � 1*/
  GPIOC->AFR[0] &= ~(1<<30);
  GPIOC->AFR[0] &= ~(1<<31);
}

void Delay(uint32_t Tempo)
{
  while(Tempo--);
}

void main()
{
  
  init_TIM3();
  init_PWM();
  init_MOTEUR_EXT();
  
  while(1)
  {
    TIM3->PSC = 11999;
    Delay(2000000);
    TIM3->PSC = 0;
    Delay(2000000);
  }
}
