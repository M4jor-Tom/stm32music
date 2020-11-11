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

/*
Spin frequencies // CCR1 values
low treshold[0x0009):   ???Hz
low treshold[0x0011]:   50Hz
do[11]:                 53Hz
dod[12]:                  55
re[13]:                   59
mib[15]:                  63
mi[16]:                   66
fa[17]:                   70
fad[19]:                  74
sol[1a]:                  79
sold[1c]:                 82.5
la[1f]:                   88
sib[22]:                  94.5
si[25]:                   99
high treshold[0x0026]:  108Hz
*/

#define EOA -1
#define _array int8_t []
#define PROC_SPEED 1600000
#include "stm32l1xx_nucleo.h"
#include <stdbool.h>

#define do 0
#define dod 1
#define re 2
#define mib 3
#define mi 4
#define fa 5
#define fad 6
#define sol 7
#define sold 8
#define la 9
#define sib 10
#define si 11

#define commandFactor 1

typedef uint64_t volatile reg;
typedef struct note
{
  uint8_t _note, octave;
  float wait, hold;
  bool blank, endFlag;
}note;

typedef struct partition
{
  note *melody;
  
  float readSpeed;
}partition;

//Sound functions
void setBuzzer(note __note);
void setMotor(note __note);
void resetBuzzer();
void resetMotor();

//Music functions
note newNote(uint8_t __note, uint8_t _octave);
note timeNote(uint8_t __note, uint8_t _octave, float _hold, float _wait);
note blankNote(float _hold);
note lastNote();
//void orchestra(partition buzzer, partition motor);
void play(partition _partition, uint8_t select);
void motor_kick();


void wait(uint64_t time);

//INIT function
void adc_port_init();
void adc_init();
void TIM3_init();
void PWM_init();
void motor_init();
void buzzerInit();

//Register functions
void registerSet(reg *registerPtr, uint8_t index, uint8_t value);
void registerSets(reg *registerPtr, int8_t *indexes, uint8_t value);

void main()
{
  //Partitionning
  partition monAmiPierrot =
  {
    .readSpeed = 5,
    .melody = (note [])
    {
      newNote(do, 4),
      newNote(do, 4),
      newNote(do, 4),
      newNote(re, 4),
      timeNote(mi, 4, 2, 0.1),
      timeNote(re, 4, 2, 0.1),
      newNote(do, 4),
      newNote(mi, 4),
      newNote(re, 4),
      newNote(re, 4),
      newNote(do, 4),
      blankNote(1),
      lastNote()
    }
  },
  pokemon =
  {
    .readSpeed = 3,
    .melody = (note [])
    {
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 2, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(do, 5, 1, 0.1),
      timeNote(la, 4, 0.5, 0.1),
      timeNote(fa, 4, 2, 0.1),
      timeNote(fa, 4, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(do, 5, 0.5, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(do, 5, 4, 0.1),
      blankNote(1),
      timeNote(mib, 5, 1, 0.1),
      timeNote(mib, 5, 1.5, 0.1),
      timeNote(mib, 5, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(do, 5, 0.5, 0.1),
      timeNote(sib, 4, 2, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(do, 5, 1, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(re, 5, 4, 0.1),
      blankNote(1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(do, 5, 1, 0.1),
      timeNote(la, 4, 0.5, 0.1),
      timeNote(fa, 4, 3, 0.1),
      timeNote(re, 5, 0.5, 0.1),
      timeNote(re, 5, 2, 0.1),
      timeNote(do, 5, 1, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(do, 5, 4, 0.1),
      blankNote(1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(mib, 5, 0.5, 0.1),
      timeNote(mib, 5, 0.5, 0.1),
      timeNote(mib, 5, 2, 0.1),
      timeNote(mib, 5, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(do, 5, 0.5, 0.1),
      timeNote(sib, 4, 2, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(re, 5, 1, 0.1),
      timeNote(do, 5, 1, 0.1),
      timeNote(sib, 4, 0.5, 0.1),
      timeNote(re, 5, 3, 0.1),
      lastNote()
    }
  };
  
  //Init
  TIM3_init();
  PWM_init();
  motor_init();
  buzzerInit();
  
  //Démarrage en force du moteur
  /*motor_kick();
  
  play(monAmiPierrot, 0);
  TIM3 -> CCR1 = 0;
  
  play(monAmiPierrot, 1);*/
  
  motor_kick();
  play(pokemon, 2);
  
  
  while(1);
}

note newNote(uint8_t __note, uint8_t _octave)
{
  return (note){._note = __note, .octave = _octave, .hold = 1, .wait = 0.1};
}

note timeNote(uint8_t __note, uint8_t _octave, float _hold, float _wait)
{
  return (note){._note = __note, .octave = _octave, .hold = _hold, .wait = _wait};
}

note blankNote(float _hold)
{
  return (note){.blank = 1, .hold = _hold};
}

note lastNote()
{
  return (note){.endFlag = 1, .blank = 1};
}

float getFrequency(uint8_t _note, uint8_t octave)
{
  float _frequency = 0.0;
  
  switch(_note)
  {
    case do:
      _frequency = 264;
      break;
      
    case dod:
      _frequency = 275;
      break;
      
    case re:
      _frequency = 297;
      break;
      
    case mib:
      _frequency = 316.8;
      break;
      
    case mi:
      _frequency = 330;
      break;
      
    case fa:
      _frequency = 352;
      break;
      
    case fad:
      _frequency = 371.25;
      break;
      
    case sol:
      _frequency = 396;
      break;
      
    case sold:
      _frequency = 412.5;
      break;
      
    case la:
      _frequency = 440;
      break;
      
    case sib:
      _frequency = 475.2;
      break;
      
    case si:
      _frequency = 495;
      break;
  }
  
  return _frequency * (octave + 1);
}

/*uint64_t getCurrentNote(partition _partition, uint64_t timeElapsed, float *remainingTime)
{
  uint64_t index = 0, measuredTime = 0;
  float remainingTime = 0;
  while(timeElapsed > measuredTime)
  {
    uint
    if(_partition.melody[index++].wait + _partition.melody[index++].hold > remainingTime)
    {
      measuredTime += _partition.melody[index++].hold;
    }
      
  }
}

void orchestra(partition buzzer, partition motor)
{
  uint64_t time = 0;
  bool buzzerEnded = 0, motorEnded = 0;
  while(buzzer.melody[time].endFlag != 1 || motor.melody[time].endFlag != 1)
  {
    uint64_t buzzerIndex = 0, motorIndex = 0;
    
    if(buzzer.melody[buzzerIndex].endFlag != 1 && buzzerEnded == 0)
      setBuzzer(buzzer.melody[buzzerIndex].frequency);
    else
      buzzerEnded = 1;
    
    time++;
  }
}*/

void play(partition _partition, uint8_t select)
{
  uint64_t index = 0;
  
  //Relacher la note
  resetBuzzer();
  resetMotor();
  while(_partition.melody[index].endFlag != 1)
  {
    //Attendre la note
    wait((uint64_t)(_partition.melody[index].wait * (float)PROC_SPEED / _partition.readSpeed));
    if(_partition.melody[index].blank != 1)
    {
      if(select == 0 || select == 2)
        setMotor(_partition.melody[index]);
      if(select == 1 || select == 2)
        setBuzzer(_partition.melody[index]);
    }
    
    //Maintenir la note
    wait((uint64_t)(_partition.melody[index].hold * (float)PROC_SPEED / _partition.readSpeed));
    
    //Relacher la note
    resetBuzzer();
    resetMotor();
    
    index++;
  }
}

void buzzerInit()
{
  /*force b1 at 1 activating the GPIOB on AHB data bus*/
  RCC->AHBENR |= (1<<2);
  
  /*set the port GPIOB as an alternative function*/
  
  GPIOC->MODER |= (1<<15); /*set MODER4 to 10 (OX1)*/
  GPIOC->MODER &= ~(1<<14);
  
  /*set the port GPIOB in a push-pull mode (deliver current)*/
  GPIOC->OTYPER &= ~(1<<7);
  
  GPIOC->AFR[0] &= ~(1<<28);
  GPIOC->AFR[0] |= (1<<29); /*registre de s?lection de la fonction multiplex?e*/
                            /*TIM3 AFRL(PB4) 0010 bit 17 ? 1*/
  GPIOC->AFR[0] &= ~(1<<30);
  GPIOC->AFR[0] &= ~(1<<31);
}

void adc_port_init()
{
  //Horloge
  registerSet((reg *)&RCC->AHBENR, 0, 1);
  
  //Entr?e analogique RV2 (PA0)
  registerSets((reg *)&GPIOA->MODER, (_array){0, 1, EOA}, 1);
}

void adc_init()
{ 
  //HSI ON
  registerSet((reg *)&RCC->CR, 0, 1);
  
  //Prescaler ? 1
  registerSet((reg *)&ADC->CCR, 16, 1);
  registerSet((reg *)&ADC->CCR, 17, 0);
  
  registerSet((reg *)&RCC->APB2ENR, 9, 1);
  
  //ADC sur PA0
  registerSet((reg *)&RI->ASCR1, 0, 1);
  
  //Resolution
  registerSets((reg *)&ADC1->CR1, (_array){24, 25, EOA}, 0);
  
  //Conversion sur un seul channel (RV2)
  registerSets((reg *)&ADC1->SQR1, (_array){20, 21, 22, 23, 24, EOA}, 0);
  
  //Conversion continue
  registerSet((reg *)&ADC1->CR2, 1, 1);
}

void TIM3_init()
{
  RCC->APB1ENR |= (1<<1); /* force bit1 at 1 activating the TIM2 on APB1 data bus */
  
  TIM3->PSC =  7999; /*PSC+1=1599+1=1600 soit F=10kHz donc T=100us*/
  TIM3->ARR = 39; /*40*100us = 4ms donc ARR = 40-1 =39*/
  
  TIM3->CR1 |= (1<<7); /*ARPE Autoreload register buffered*/
  TIM3->CR1 &= ~(1<<6); /*CMS edge alligned*/
  TIM3->CR1 &= ~(1<<5); /*CMS edge alligned*/
  TIM3->CR1 &= ~(1<<4); /*DIR counter*/
  
  TIM3->CR1 |= (1<<0); /*CEN counter enable*/
}

void PWM_init()
{ 
  TIM3->CCMR1 &= ~((1<<8) | (1<<9)); /*define CH1 as an output*/
  TIM3->CCMR1 &= ~(1<<12); /*PMW mode 1*/
  TIM3->CCMR1 |= ((1<<13) | (1<<14)); /*PMW mode 1*/
  
  TIM3->CCR2 = 1; /*duty cycle*/
  
  TIM3->CCER |= (1<<4); /*signal enable*/
  registerSets((reg *)&TIM3 -> CCMR1, (_array){0, 1, 4, 8, 9, 12, EOA}, 0);
  registerSets((reg *)&TIM3 -> CCMR1, (_array){5, 6, 13, 14, EOA}, 1);
  
  TIM3->CCR2 = 1; /*duty cycle*/
  TIM3 -> CCR1 = 0;
  
  registerSets((reg *)&TIM3 -> CCER, (_array){0, 4, EOA}, 1);
}

void motor_init()
{
  registerSets((reg *)&RCC->AHBENR, (_array){1, 2, EOA}, 1);
  
  registerSet((reg *)&GPIOB->MODER, 9, 1);
  registerSet((reg *)&GPIOB->MODER, 8, 0);
  
  registerSet((reg *)&GPIOB->OTYPER, 4, 0);
  
  registerSets((reg *)&GPIOB->AFR[0], (_array){16, 18, 19, EOA}, 0);
  registerSet((reg *)&GPIOB->AFR[0], 17, 1);
}

void motor_kick()
{
  TIM3 -> CCR1 = 0x0fff;
  wait(100000);
}

void registerSet(reg *registerPtr, uint8_t index, uint8_t value)
{
  if(value == 1)
   *registerPtr |= (1<<index);
  else if(value == 0)
   *registerPtr &= ~(1<<index);
  else
    //Valeur pas 1 ou 0
    while(1);
}

void registerSets(reg *registerPtr, int8_t *indexes, uint8_t value)
{
  uint64_t i = 0;
  while(indexes[i] != EOA)
    registerSet(registerPtr, (uint8_t)indexes[i++], value);
}

void wait(uint64_t time)
{
  while(time--);
}

void setBuzzer(note __note)
{
    TIM3->PSC = PROC_SPEED * 10 / (TIM3->ARR * getFrequency(__note._note, __note.octave));
}

uint16_t noteToMotorComamnd(uint8_t _note)
{
  uint16_t command = 0;
  switch(_note)
  {
    case do:
      command = 0x0011;
      break;
      
    case dod:
      command = 0x0012;
      break;
      
    case re:
      command = 0x0013;
      break;
      
    case mib:
      command = 0x0015;
      break;
      
    case mi:
      command = 0x0016;
      break;
      
    case fa:
      command = 0x0017;
      break;
      
    case fad:
      command = 0x0019;
      break;
      
    case sol:
      command = 0x001a;
      break;
      
    case sold:
      command = 0x001c;
      break;
      
    case la:
      command = 0x001f;
      break;
      
    case sib:
      command = 0x0022;
      break;
      
    case si:
      command = 0x0025;
      break;
  }
  return command;
}

void setMotorFreq(uint16_t command)
{
  TIM3 -> CCR1 = command * commandFactor;//(command * TIM3->ARR) / 4096;
}

void setMotor(note __note)
{
  setMotorFreq(noteToMotorComamnd(__note._note));
}

void resetBuzzer()
{
  TIM3 -> PSC = 0;
}

void resetMotor()
{
  TIM3 -> CCR1 = 0;
}