#include <avr/io.h>            // Содержит необходимые макроподстановки
#include <avr/interrupt.h>     // Содерержит вектры прерываний, функции разрешения/запрещения прерываний
//#include <util/delay.h>
#include "termoreg.h"
#include "initialize.h"        // Инициализация узлов контроллера

#define F_CPU 4000000UL
#define wdt_reset() asm("wdr") // Сброс сторожевого таймера
#define delay_ms(time_ms)       for(unsigned long i = 0; i < ( ( (F_CPU / 1000) / 18) * time_ms); ++i){asm("  nop;");  }

#define highTemperature 509    // 17 градусов в переводе по уровеню квантования
#define lowTemperature  465    // 13 градусов в переводе по уровеню квантования
#define TimeToFail      4000    // 4 с ожидания до статуса "авария" при неполадке вентилятора
#define MAXTIME         59900  // Время, до которого будет вестись глобальный отсчёт времени - минута

StateWork stateWork = STATE_INITIALIZE;                    // Начальный статус - инициализация
typeStand standType ;                                     // Переменная для хранения типа стойки

/*
ПРИМЕЧАНИЕ К РАБОТЕ КЛАССОВ "Fan" и "Led":

Т.к С++ не поддерживает переопределние методов для
конкретного объекта, то в методах классов для работы с
аппаратной чатью необходимо явно указывать,
для какого объекта вызываемый метод предназначен.
*/

class Time
{
	public:
	void setGlobalTime(unsigned int globalTime) {this->globalTime_ = globalTime;} // Установка глобальных часов в прерывании таймера-счётчика0
	unsigned int getGlobalTime(void)            {return globalTime_;}             // Метод получения текущего времени для его обновления в прерывании таймера-счётчика0
	

	void reset(void){flagTimer_ = true;}
	void start(unsigned int endTime)                             // Метод начала отсчёта
	{
		
		if(flagTimer_)                                       // Если флаг таймера поднят, запоминаем время начала отсчёта:
		{
			startTime_ = globalTime_;                            // Запоминаем текущее время
			flagTimer_ = false;                              // Опускаем флаг таймера, начинаем считать:
		}
		else                                                 // Если флаг таймера опущен
		{
			if (startTime_ + endTime > MAXTIME )                   // Суммарное время превышает максимальное число таймера
			{
				timeTimer_ = globalTime_ <= MAXTIME ? globalTime_ - startTime_: MAXTIME - startTime_ + globalTime_;
				if (globalTime_ == startTime_ - (MAXTIME -  endTime) ) // Делаем поправку, переводя время окончания на начало часов
				{
					flagTimer_ = true;                       // Поднимаем флажок для запоминания следующего времени
				}
			}
			else
			{
				timeTimer_ = globalTime_ - startTime_;
				if (globalTime_ == startTime_ + endTime) flagTimer_ = true;      // Если время окончания отсчёта не превышает максимум часов и отсчитали установленное время
			}
		}
		
		
	}
	unsigned int getTimerTime(void) {return timeTimer_;}
	bool endTimer(void) {return flagTimer_;}                 //  вернуть статус окончания отсчёта
	
	
	private:
	unsigned int globalTime_;
	unsigned int startTime_;
	bool flagTimer_= true;
	unsigned int timeTimer_;
};

class Fan
{
	public:
	typedef enum {enable_fan,disable_fan,}stateWorkFan;             /* Возможные состояния вентиляторов */
	typedef enum {fail,normal,}stateFeedbackFan;                    /* Статус вентиляторов */
	typedef enum{FAN_AR00,FAN_AF00,}typeFan;                        /* Типы вентиляторов */
	typedef enum {FAN1,FAN2} numberFan;

	void setType(typeFan type) {this->type_ = type;} // Устанавливает тип вентилятора
	
	typeFan getType() {return type_;}
	
	void setStatFeedback(stateFeedbackFan statFeedback) {this->statFeedback_ = statFeedback;} // Устанавливает статус обратной связи

	stateFeedbackFan getStatFeedback() {return statFeedback_;} // Возвращает статус обратной связи
	
	void setTimeToFail(unsigned int timeToFail){this->timeToFail_ = timeToFail;}
	
	unsigned int getTimeToFail(){return timeToFail_;}
	
	void enable(numberFan FAN) // Включает вентилятор
	{
		this->stateWork_ = enable_fan;
		if (FAN == FAN1) PORTC |=  (1 << PC3);
		else PORTC |=  (1 << PC4);
	}
	bool checkEnable(){return (stateWork_ == enable_fan ? true : false);}
	void disable(numberFan FAN) // Выключает вентилятор
	{
		this->stateWork_ = disable_fan;
		if (FAN == FAN1) PORTC &= ~(1 << PC3);
		else PORTC &= ~(1 << PC4);
	}
	
	void checkFeedback (numberFan FAN)
	{
		
		if (type_ == FAN_AR00)
		{
			if (FAN == FAN1)
			{
				if (PINB & (1 << PB0)) timeToFail_ = TimeToFail;
			}
			else if (PINB & (1 << PB1)) timeToFail_ = TimeToFail;
		}
		statFeedback_ = (timeToFail_ == 0) ? fail : normal;
	}

	private:
	typeFan type_;                  // Тип вентилятора       (AR00 / AF00)
	stateFeedbackFan statFeedback_; // Статус обратной связи (normal / fail)
	stateWorkFan stateWork_;        // Статус работы         (enable_fan / disable_fan)
	unsigned int timeToFail_;       //  Оставшееся время до принятия решения о неработоспособности вентилятра
};

class Led
{
	public:
	typedef enum{LED_N1,LED_N2,LED_A1,LED_A2 } numberLed;
	
	typedef enum {enable_led, disable_led, blink_led} stateLed;
	
	void setStateLed(stateLed state) {this->state_ = state;}
	
	stateLed getStateLed(){return state_;}
	
	void enable(numberLed led)
	{
		this->state_ = enable_led;
		switch (led)
		{
			case LED_N1: PORTD |=  (1 << PD6);
			break;
			case LED_N2: PORTD |=  (1 << PD7);
			break;
			case LED_A1: PORTD |=  (1 << PD4);
			break;
			case LED_A2: PORTD |=  (1 << PD5);
			break;
		}
	}
	
	void disable(numberLed led)
	{
		this->state_ = disable_led;
		switch (led)
		{
			case LED_N1: PORTD &=  ~(1 << PD6);
			break;
			case LED_N2: PORTD &=  ~(1 << PD7);
			break;
			case LED_A1: PORTD &=  ~(1 << PD4);
			break;
			case LED_A2: PORTD &=  ~(1 << PD5);
			break;
		}
	}
	
	void blink(numberLed led)
	{
		this->state_ = blink_led;
	}
	
	void invertLightLed(numberLed led)
	{
		switch (led)
		{
			case LED_N1: PORTD ^=  (1 << PD6);
			break;
			case LED_N2: PORTD ^=  (1 << PD7);
			break;
			case LED_A1: PORTD ^=  (1 << PD4);
			break;
			case LED_A2: PORTD ^=  (1 << PD5);
			break;
		}
	}
	private:
	stateLed state_;
};

Fan fan1;
Fan fan2;
Led led_N1;
Led led_N2;
Led led_A1;
Led led_A2;
Time timer;


int main(void)
{
	while (1)
	{
		wdt_reset();
		switch(stateWork)                                               // Выбор текущего статуса работы
		{
			case STATE_INITIALIZE:/*--------------------------- Статус "ИНИЦИАЛИЗАЦИЯ" --------------------------------------------------*/
			cli();
			port_initialize();                                      // Инициализируем порты ввода-вывода
			ADC_initialize();                                       // Инициализируем АЦП на еденичное преобразование
			externalInterruption_initialize();                      // Инициализируем внешние прерывания для чтения статуса вентилятора AF00
			counter0_initialize();                                  // Инициализируем и запускаем таймер-счётчик 0
			WDT_enable();                                           // Запускаем сторожевой таймер по алгоритму "уровня безопасности 2"
			sei();
			led_N1.enable(Led::LED_N1);                                          // Включаем все всетодиоды
			led_N2.enable(Led::LED_N2);
			led_A1.enable(Led::LED_A1);
			led_A2.enable(Led::LED_A2);
			standType = PINA & (1 << PA6) ? STAND52_54 : STAND51;                // Если в PA6(JMP1) еденица - стойка 52/54, иначе стойка 51
			fan1.setType(PINA & (1 << PA7) ?  Fan::FAN_AR00  : Fan::FAN_AF00);   // Если в PA7(JMP2) еденица - вентилятор AR00, иначе AF00
			fan2.setType(fan2.getType());
			if (fan1.getType() == Fan::FAN_AF00) GICR = (1 << INT1)|(1 << INT0); // Разрешение прерываний INT1 и INT0 для получения сигналов SFAN1 и SFAN2
			delay_ms(1000);
			stateWork = STATE_WAIT_SOLUTION_TURN;                                // Переход в состояние ожидания разрешения включения вентиляторов
			break; // конец case STATE_INITIALIZE:
			
			case STATE_WAIT_SOLUTION_TURN:/*------------------- Статус "ОЖИДАНИЕ РАЗРЕШЕНИЯ ВКЛЮЧЕНИЯ ВЕНТИЛЯТОРОВ" ------------------------*/
			led_N1.disable(Led::LED_N1);                                          // Включаем все всетодиоды
			led_N2.disable(Led::LED_N2);
			led_A1.disable(Led::LED_A1);
			led_A2.disable(Led::LED_A2);
			if(PINC & (1 << PC2)) stateWork = STATE_DIAGNOSTICS;   // Если на PC2 (ON) еденица - переход в режим самодиагностики
			break;  // конец case STATE_WAIT_SOLUTION_TURN:
			
			case STATE_DIAGNOSTICS:/*--------------------------- Статус "САМОДИАГНОСТИКА" ---------------------------------------------------*/
			if (standType == STAND51) //-------------------        Для стойки 51
			{
				timer.start(5000);                             // Начать отсчёт на 5 секунд
				fan1.enable(Fan::FAN1);                        // Включили все вентиляторы
				fan2.enable(Fan::FAN2);
				led_N1.blink(Led::LED_N1);                     // LED_N1 моргает
				led_N2.blink(Led::LED_N2);                     // LED_N2 моргает
				led_A1.enable(Led::LED_A1);                    // LED_А1 горит
				led_A2.enable(Led::LED_A2);                    // LED_А2 горит
				if (timer.endTimer())
				{
					fan1.disable(Fan::FAN1);                   // Отключили все вентиляторы
					fan2.disable(Fan::FAN2);
					if (fan1.getStatFeedback() == Fan::normal) // Смотрим состояние
					{
						led_N1.enable(Led::LED_N1);
						led_A1.disable(Led::LED_A1);
					}
					else
					{
						led_N1.disable(Led::LED_N1);
						led_A1.enable(Led::LED_A1);
					}
					if (fan2.getStatFeedback() == Fan::normal)
					{
						led_N2.enable(Led::LED_N2);
						led_A2.disable(Led::LED_A2);
					}
					else
					{
						led_N2.disable(Led::LED_N2);
						led_A2.enable(Led::LED_A2);
					}
					timer.reset();
					stateWork = STATE_WAIT_HEAT;        // 5 секунд теста окончены - переходим в состояние ожидания верхнего порога температуры
				}
			} // конец STAND51
			else//---------------------------------------------Стойка 52/54
			{
				timer.start(10000);
				if (timer.getTimerTime() < 5000)
				{
					fan1.enable(Fan::FAN1);
					fan2.disable(Fan::FAN2);
					led_N1.blink(Led::LED_N1);
					led_N2.disable(Led::LED_N2);
					led_A1.disable(Led::LED_A1);
					led_A2.disable(Led::LED_A2);
					
				}
				else if (timer.getTimerTime() == 5000)
				{
					
					if (fan1.getStatFeedback() == Fan::normal)
					{           // Смотрим состояние
						led_N1.enable(Led::LED_N1);
						led_A1.disable(Led::LED_A1);
					}
					else
					{
						led_N1.disable(Led::LED_N1);
						led_A1.enable(Led::LED_A1);
					}
				}
				else
				{                                    // 5 секунд теста вентилятора 1 истекло, тест вентилятора 2
					fan1.disable(Fan::FAN1);
					fan2.enable(Fan::FAN2);
					led_N2.blink(Led::LED_N2);
					led_A2.enable(Led::LED_A2);
					if (timer.endTimer())
					{             // Вторые 5 секунд теста для вентилятора 2 истекло : смотрим результат
						fan2.disable(Fan::FAN2);
						if (fan2.getStatFeedback() == Fan::normal)
						{           // Смотрим состояние
							led_N2.enable(Led::LED_N2);
							led_A2.disable(Led::LED_A2);
						}
						else
						{
							led_N2.disable(Led::LED_N2);
							led_A2.enable(Led::LED_A2);
						}
						timer.reset();
						stateWork = STATE_WAIT_HEAT;  // Суммарные 10 секунд теста окончены, преходим в состояня ожидания верхнего порога температуры
					}
				}
			}
			break; // конец case STATE_DIAGNOSTICS:
			
			case STATE_WAIT_HEAT:/*----------------------------- Статус "ОЖИДАНИЕ ВЕРХНЕГО ПОРОГА ТЕМПЕРАТУРЫ" -----------------------------*/
			if (gettemperature() >= highTemperature)       // Превысили верхний порог температуры
			{
				if (standType == STAND51) // Для стойки 51
				{
					timer.start(5000);
					fan1.enable(Fan::FAN1);                        // Включили все вентиляторы
					fan2.enable(Fan::FAN2);
					if (timer.endTimer())
					{
						if (fan1.getStatFeedback() == Fan::normal) // Смотрим состояние
						{
							led_N1.blink(Led::LED_N1);
							led_A1.disable(Led::LED_A1);
						}
						else
						{
							led_N1.disable(Led::LED_N1);
							led_A1.enable(Led::LED_A1);
						}
						if (fan2.getStatFeedback() == Fan::normal)
						{
							led_N2.blink(Led::LED_N2);
							led_A2.disable(Led::LED_A2);
						}
						else
						{
							led_N2.disable(Led::LED_N2);
							led_A2.enable(Led::LED_A2);
						}
						if(fan1.getStatFeedback() == Fan::normal || fan2.getStatFeedback() == Fan::normal)stateWork = STATE_WAIT_HOLD;
						
					}
				}
				else //------------------------------------------/Для стойки 52/54
				{
					timer.start(10000);
					if (timer.getTimerTime() < 5000)
					{
						fan1.enable(Fan::FAN1);
						fan2.disable(Fan::FAN2);
						
					}
					else if (timer.getTimerTime() == 5000)
					{
						
						if (fan1.getStatFeedback() == Fan::normal)
						{           // Смотрим состояние
							led_N1.blink(Led::LED_N1);
							led_A1.disable(Led::LED_A1);
							stateWork = STATE_WAIT_HOLD;
							
						}
						else
						{
							fan1.disable(Fan::FAN1);
							led_N1.disable(Led::LED_N1);
							led_A1.enable(Led::LED_A1);
						}
					}
					else
					{                                    // 5 секунд теста вентилятора 1 истекло, тест вентилятора 2
						fan2.enable(Fan::FAN2);
						if (timer.endTimer())  // Вторые 5 секунд теста для вентилятора 2 истекло : смотрим результат
						{
							if (fan2.getStatFeedback() == Fan::normal)
							{           // Смотрим состояние
								led_N2.blink(Led::LED_N2);
								led_A2.disable(Led::LED_A2);
								stateWork = STATE_WAIT_HOLD;
							}
							else
							{
								fan2.disable(Fan::FAN2);
								led_N2.disable(Led::LED_N2);
								led_A2.enable(Led::LED_A2);
							}
							
						}
					}
				}
			}
			else // Если верхний порог не превышен
			{
				fan1.disable(Fan::FAN1);
				fan2.disable(Fan::FAN2);
			}
			break;  // конец case STATE_WAIT_HEAT:
			
			case STATE_WAIT_HOLD:/*----------------------------- Статус "ОЖИДАНИЕ НИЖНЕГО ПОРОГА ТЕМПЕРАТУРЫ --------------------------------*/
			if (gettemperature() >= lowTemperature)     // Нижний порог не достигнут - процесс охлаждения
			{
				if (standType == STAND51) // Для стойки 51
				{
					if (fan1.getStatFeedback() == Fan::normal)
					{           // Смотрим состояние
						led_N1.blink(Led::LED_N1);
						led_A1.disable(Led::LED_A1);
					}
					else
					{
						led_N1.disable(Led::LED_N1);
						led_A1.enable(Led::LED_A1);
					}
					
					if (fan2.getStatFeedback() == Fan::normal)
					{           // Смотрим состояние
						led_N2.blink(Led::LED_N2);
						led_A2.disable(Led::LED_A2);
					}
					else
					{
						led_N2.disable(Led::LED_N2);
						led_A2.enable(Led::LED_A2);
					}
					
				}
				else//----------------------- Для стойки 52/54
				{
					timer.start(5000);
					if(timer.endTimer())
					{
						if (fan1.getStatFeedback() == Fan::normal && fan2.getStatFeedback() == Fan::normal)
						{
							fan2.disable(Fan::FAN2);
							fan1.enable(Fan::FAN1);
							led_N1.blink(Led::LED_N1);
							led_A1.disable(Led::LED_A1);
							led_N2.enable(Led::LED_N2);
							led_A2.disable(Led::LED_A2);
						}
						else if(fan1.getStatFeedback() == Fan::normal && fan2.getStatFeedback() == Fan::fail)
						{
							fan2.disable(Fan::FAN2);
							fan1.enable(Fan::FAN1);
							led_N1.blink(Led::LED_N1);
							led_A1.disable(Led::LED_A1);
							led_N2.disable(Led::LED_N2);
							led_A2.enable(Led::LED_A2);
						}
						else if (fan1.getStatFeedback() == Fan::fail && fan2.getStatFeedback() == Fan::normal)
						{
							fan1.disable(Fan::FAN1);
							fan2.enable(Fan::FAN2);
							led_N1.disable(Led::LED_N1);
							led_A1.enable(Led::LED_A1);
							led_N2.blink(Led::LED_N2);
							led_A2.disable(Led::LED_A2);
						}
						else // Оба вентилятора неисправны
						{
							led_N1.disable(Led::LED_N1);
							led_A1.enable(Led::LED_A1);
							led_N2.disable(Led::LED_N2);
							led_A2.enable(Led::LED_A2);
							if (fan1.checkEnable())
							{
								fan1.disable(Fan::FAN1);
								fan2.enable(Fan::FAN2);
							}
							else
							{
								fan1.enable(Fan::FAN1);
								fan2.disable(Fan::FAN2);
							}
							
						}
					}
				}
			}
			else// Достигли нижнего порога температуры
			{
				if (fan1.getStatFeedback() == Fan::normal)
				{           // Смотрим состояние
					led_N1.enable(Led::LED_N1);
					led_A1.disable(Led::LED_A1);
				}
				else
				{
					led_N1.disable(Led::LED_N1);
					led_A1.enable(Led::LED_A1);
				}
				
				if (fan2.getStatFeedback() == Fan::normal)
				{           // Смотрим состояние
					led_N2.enable(Led::LED_N2);
					led_A2.disable(Led::LED_A2);
				}
				else
				{
					led_N2.disable(Led::LED_N2);
					led_A2.enable(Led::LED_A2);
				}
				timer.reset();
				stateWork = STATE_WAIT_HEAT;
			}
			break; // конец case STATE_WAIT_HOLD:
		}// конец switch(stateWork)
	} // конец while(1)
} // конец main(void)


int gettemperature()      /* Получить температуру */
{
	/*
	АЦП в режиме одиночного преобразования, частота синхронизыции = 31250 кГц
	Результат записывается в 8ми битные регистры ADCH ADCL (c.217 русского даташита),
	однако эти регистры доступны как один 16ти битный ADC
	*/
	ADCSRA |= (1 << ADSC);             // Запуск преобразования. ADSC сбросится аппаратно в 0 после преобразования
	while (ADCSRA & (1 << ADSC)){};    // Ждём окончания преобразования : 13 тактов = 3.25 мкс
	return (int)ADC;                   // Возвращаем результат из регистров ADCH и ADCL
}// конец int gettemperature()

ISR (TIMER0_COMP_vect)          /* Прерывание по совпадению таймера-счётчика 0 */
{
	unsigned int lastTime = timer.getGlobalTime();
	lastTime < MAXTIME ? timer.setGlobalTime(lastTime + 100) : timer.setGlobalTime(0);
	
	if (lastTime % 1000 == 0)
	{
		
		if (led_N1.getStateLed() == Led::blink_led) led_N1.invertLightLed(Led::LED_N1);
		if (led_N2.getStateLed() == Led::blink_led) led_N2.invertLightLed(Led::LED_N2);
		if (led_N1.getStateLed() == Led::blink_led && led_N2.getStateLed() == Led::blink_led)
		{
			if (!(PIND & (1 << PD6)) && PIND & (1 << PD7)) led_N1.invertLightLed(Led::LED_N1); // Если оба светодиода мигают, синхронизируем их мигание
		}
	}
	
	unsigned int timeToFail = 0;
	if (fan1.checkEnable())
	{
		timeToFail = fan1.getTimeToFail();
		if (timeToFail >= 100) fan1.setTimeToFail(timeToFail - 100);
		fan1.checkFeedback(Fan::FAN1);
	}
	if (fan2.checkEnable())
	{
		timeToFail = fan2.getTimeToFail();
		if (timeToFail >= 100) fan2.setTimeToFail(timeToFail - 100);
		fan2.checkFeedback(Fan::FAN2);
	}
} // конец ISR (TIMER0_COMP_vect)

ISR (INT0_vect)                            /* Прерывание INT0 */
{
	fan1.setTimeToFail(TimeToFail);      // Восстанавливем счётчик вентилятора 1
} // конец ISR (INT0_vect)

ISR (INT1_vect)                            /* Прерывание INT1 */
{
	fan2.setTimeToFail(TimeToFail);                // Восстанавливем счётчик вентилятора 2
} // конец ISR (INT1_vect)
