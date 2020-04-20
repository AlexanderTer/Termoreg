/*
* termoreg.h
*
* Created: 05.04.2020 22:56:48
*  Author: Admn
*/


#ifndef TERMOREG_H_
#define TERMOREG_H_

typedef enum                   /* Состояния работы по техническому описанию */
{
	STATE_INITIALIZE,          // Инициализация
	STATE_WAIT_SOLUTION_TURN,  // Ожидание разрешения включения вентиляторов
	STATE_DIAGNOSTICS,         // Самодиагностика
	STATE_WAIT_HEAT,           // Ожидание верхнего порога температуры
	STATE_WAIT_HOLD,           // Ожидание нижнего порога температуры
} StateWork;

typedef enum                    /* Типы стоек */
{
	STAND51,                    // Стойка 51
	STAND52_54,                 // Стойка 52/54
} typeStand;










int gettemperature();     /* Получить температуру                                                */


#endif /* TERMOREG_H_ *//* TERMOREG_H_ */