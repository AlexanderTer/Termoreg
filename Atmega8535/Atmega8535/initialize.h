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
void port_initialize(void);                  /* ������������� ������ �����-������                       */
void ADC_initialize(void);                   /* ������������� ���                                       */
void externalInterruption_initialize(void);  /* ������������� ������� ���������� ��� ������������� AF00 */
void counter0_initialize(void);              /* ������������� �������-�������� 0                        */
void WDT_enable(void);                       /* ��������� ����������� ������� �� 4.2 �                  */


#endif /* INITIALIZATION_H_ *//* INITIALIZE_H_ */