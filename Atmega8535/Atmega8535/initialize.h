/*
 * initialization.h
 *
 * Created: 06.04.2020 0:13:11
 *  Author: Admn
 */ 



#ifndef INITIALAZE_H_
#define INITIALAZE_H_

extern unsigned int counterStateFAN1;
extern unsigned int counterStateFAN2;
void port_initialize(void);                  /* Инициализация портов ввода-вывода                       */
void ADC_initialize(void);                   /* Инициализация АЦП                                       */
void externalInterruption_initialize(void);  /* Инициализация внешних прерываний для вентияляторов AF00 */
void counter0_initialize(void);              /* Инициализация таймера-счётчика 0                        */
void WDT_enable(void);                       /* Включение сторожевого таймера на 4.2 с                  */


#endif /* INITIALIZATION_H_ *//* INITIALIZE_H_ */