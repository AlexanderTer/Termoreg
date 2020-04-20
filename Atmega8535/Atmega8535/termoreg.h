/*
* termoreg.h
*
* Created: 05.04.2020 22:56:48
*  Author: Admn
*/


#ifndef TERMOREG_H_
#define TERMOREG_H_

typedef enum                   /* ��������� ������ �� ������������ �������� */
{
	STATE_INITIALIZE,          // �������������
	STATE_WAIT_SOLUTION_TURN,  // �������� ���������� ��������� ������������
	STATE_DIAGNOSTICS,         // ���������������
	STATE_WAIT_HEAT,           // �������� �������� ������ �����������
	STATE_WAIT_HOLD,           // �������� ������� ������ �����������
} StateWork;

typedef enum                    /* ���� ����� */
{
	STAND51,                    // ������ 51
	STAND52_54,                 // ������ 52/54
} typeStand;










int gettemperature();     /* �������� �����������                                                */


#endif /* TERMOREG_H_ *//* TERMOREG_H_ */