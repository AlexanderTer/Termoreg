#include <avr/io.h>             // Содержит необходимые макроподстановки
#include <avr/interrupt.h>      // Содерержит вектры прерываний, функции разрешения/запрещения прерываний
//#include <util/delay.h>
#include "termoreg.h"
#include "initialize.h"         // Инициализация узлов контроллера

#define F_CPU 4000000UL
#define wdt_reset() asm("wdr")  // Сброс сторожевого таймера
#define delay_ms(time_ms)       for(unsigned long i = 0; i < ( ( (F_CPU / 1000) / 18) * time_ms); ++i){asm("  nop;");  }

#define HIGH_TEMPERATURE 509    // 17 градусов в переводе по уровеню квантования
#define LOW_TEMPERATURE  465    // 13 градусов в переводе по уровеню квантования
#define TIME_TO_FAIL      4000  // 4 с ожидания до статуса "авария" при неполадке вентилятора
#define MAX_TIME         59900  // Время, до которого будет вестись глобальный отсчёт времени - минута

StateWork stateWork = STATE_INITIALIZE;                   // Начальный статус работы - инициализация
typeStand standType ;                                     // Переменная для хранения типа стойки

/*
ПРИМЕЧАНИЕ К РАБОТЕ КЛАССОВ "Fan" и "Led":

Т.к С++ не поддерживает переопределние методов для
конкретного объекта, то в методах классов для работы с
аппаратной частью необходимо явно указывать,
для какого объекта вызываемый метод предназначен.
*/

class Time /*----------------------------------------------------------------------- Класс работы со временем*/
{
	public:
	void setGlobalTime(unsigned int globalTime) {this->globalTime_ = globalTime;} // Установка глобальных часов (в прерывании таймера-счётчика0)
	
	unsigned int getGlobalTime(void)            {return globalTime_;}             // Метод получения текущего времени для его обновления в прерывании таймера-счётчика0
	
	void reset(void){flagTimer_ = true;}                                          // Сброс таймера в 0
	
	void start(unsigned int endTime)                                              // Метод начала отсчёта таймера
	{                                                                             //
		if(flagTimer_)                                                            // Если флаг таймера поднят, запоминаем время начала отсчёта:
		{                                                                         //
			startTime_ = globalTime_;                                             // Запоминаем текущее время глобвльных часов
			flagTimer_ = false;                                                   // Опускаем флаг таймера, начинаем считать:
		}                                                                         //
		else                                                                      // Если флаг таймера опущен
		{                                                                         //
			if (startTime_ + endTime > MAX_TIME )                                 // Суммарное время превышает максимальное число таймера
			{                                                                     //
				timeTimer_ = globalTime_ <= MAX_TIME ? globalTime_ - startTime_: MAX_TIME - startTime_ + globalTime_; // Запоминаем текущее время таймере
				if (globalTime_ == startTime_ - (MAX_TIME -  endTime) )  flagTimer_ = true;                           // Делаем поправку, переводя время окончания на начало часов,
			}                                                                     // Когда отсчёт окончен, поднимаем флажок для запоминания следующего времени
			else                                                                  // Если время окончания отсчёта не преышает максимум часов
			{                                                                     //
				timeTimer_ = globalTime_ - startTime_;                            // Запоминаем текущее время таймера
				if (globalTime_ == startTime_ + endTime) flagTimer_ = true;       // Если время окончания отсчёта не превышает максимум часов и отсчитали установленное время,поднимаем флажок
			}                                                                     //
		}                                                                         //
	}// конец void start(unsigned int endTime)    
	
	unsigned int getTimerTime(void) {return timeTimer_;}                          // Получить текущее время таймера
	
	bool endTimer(void) {return flagTimer_;}                                      //  Вернуть статус окончания отсчёта
	
	
	private:
	unsigned int globalTime_;                                                     // Хранение глобального времени часов
	unsigned int startTime_;                                                      // Время начала отсчёта тймера
	bool flagTimer_= true;                                                        // Флаг окончания отсчёта таймера
	unsigned int timeTimer_;                                                      // Текуще время таймера после начала отсчёта
};

class Fan /*------------------------------------------------------------------------ Класс работы с вентиляторами*/
{
	public:
	typedef enum {enable_fan,disable_fan,}stateWorkFan;            // Возможные состояния вентиляторов 
	typedef enum {fail,normal,}stateFeedbackFan;                   // Статус вентиляторов 
	typedef enum{FAN_AR00,FAN_AF00,}typeFan;                       // Типы вентиляторов 
	typedef enum {FAN1,FAN2} numberFan;                            // Названия вентиляторов

	void setType(typeFan type) {this->type_ = type;}                                         // Устанавливает тип вентилятора

	typeFan getType() {return type_;}                                                        // Получает тип вентилятора
	
	void setStatFeedback(stateFeedbackFan statFeedback) {this->statFeedback_ = statFeedback;} // Устанавливает статус обратной связи (норма / ошибка)

	stateFeedbackFan getStatFeedback() {return statFeedback_;}                                // Возвращает статус обратной связи
	
	void setTimeToFail(unsigned int timeToFail){this->timeToFail_ = timeToFail;}              // Устанавливает время ло принятия решения о неработоспособности вентиляторов
	
	unsigned int getTimeToFail(){return timeToFail_;}                                         // Получает время ло принятия решения о неработоспособности вентиляторов
	
	void enable(numberFan FAN)                                                                // Включает вентилятор
	{                                                              //
		this->stateWork_ = enable_fan;                             // Обновить статус на "включено"
		if (FAN == FAN1) PORTC |=  (1 << PC3);                     // Если хотим включить вентилятор1, записываем 1 в PC3
		else PORTC |=  (1 << PC4);                                 // Если хотим включить вентилятор2, записываем 1 в PC4
	}                                                              //
	 
	bool checkEnable(){return (stateWork_ == enable_fan ? true : false);}                     // Проверяет, есть ли сигнал для включении вентилятора
		
	void disable(numberFan FAN)                                                               // Выключает вентилятор
	{                                                               //
		this->stateWork_ = disable_fan;                             // Обновить статус на "выключено"
		if (FAN == FAN1) PORTC &= ~(1 << PC3);                      // Если хотим выключить вентилятор1, записываем 0 в PC3
		else PORTC &= ~(1 << PC4);                                  // Если хотим выключить вентилятор2, записываем 0 в PC4
	}                                                               //
	
	void checkFeedback (numberFan FAN)                                                         // Метод проверки наличия обратной связи при включённом вентиляторе
	{                                                               //
		if (type_ == FAN_AR00)                                      // Для вентилятора типа AR00
		{                                                           //
			if (FAN == FAN1)                                        // Если проверяем первый вентилятор
			{                                                       //
				if (PINB & (1 << PB0)) timeToFail_ = TIME_TO_FAIL;  // Если есть 1 в PB0, то обратная связь есть, восстанавливаем счётчик времени до приятия решения о неработоспособности вентилтора
			}                                                       //
			else if (PINB & (1 << PB1)) timeToFail_ = TIME_TO_FAIL; // Оначе идёт проверка вентилятора2, если есть 1 в PB1, то обратная связь есть, восстанавливаем счётчик времени до приятия решения о неработоспособности вентилятора
		}                                                           //
		statFeedback_ = (timeToFail_ == 0) ? fail : normal;         // Если счётчик времени до принятия решения о неработоспособности вентилятора обнулился, то вентлятор авариен
	}                                                               //

	private:
	typeFan type_;                  // Тип вентилятора       (AR00 / AF00)
	stateFeedbackFan statFeedback_; // Статус обратной связи (normal / fail)
	stateWorkFan stateWork_;        // Статус работы         (enable_fan / disable_fan)
	unsigned int timeToFail_;       //  Оставшееся время до принятия решения о неработоспособности вентилятра
};

class Led /*------------------------------------------------------------------------ Класс работы со светодиодами*/
{
	public:
	typedef enum{LED_N1,LED_N2,LED_A1,LED_A2 } numberLed;           // Названия светодиодов
	typedef enum {enable_led, disable_led, blink_led} stateLed;     // Статус светодиодов
	
	void setStateLed(stateLed state) {this->state_ = state;}                                   // Установить статус
	
	stateLed getStateLed(){return state_;}                                                     // Получить статус
	
	void enable(numberLed led)                                                                 // Включить светодиод
	{                                                                //
		this->state_ = enable_led;                                   // Обновить статус на "включено"
		switch (led)                                                 // 
		{                                                            //
			case LED_N1: PORTD |=  (1 << PD6);                       // LED_N1 : записать 1 в PD6
			break;                                                   //
			case LED_N2: PORTD |=  (1 << PD7);                       // LED_N2 : записать 1 в PD7
			break;                                                   //
			case LED_A1: PORTD |=  (1 << PD4);                       // LED_A1 : записать 1 в PD4
			break;                                                   //
			case LED_A2: PORTD |=  (1 << PD5);                       // LED_A2 : записать 1 в PD5
			break;                                                   //
		}                                                            //
	}                                                          
	
	void disable(numberLed led)                                                               // Отключить светодиод
	{                                                                //
		this->state_ = disable_led;                                  // Обновить статус на "выключено"
		switch (led)                                                 //
		{                                                            //
			case LED_N1: PORTD &=  ~(1 << PD6);                      // LED_N1 : записать 0 в PD6
			break;                                                   //
			case LED_N2: PORTD &=  ~(1 << PD7);                      // LED_N2 : записать 0 в PD7
			break;                                                   //
			case LED_A1: PORTD &=  ~(1 << PD4);                      // LED_A1 : записать 0 в PD4
			break;                                                   //
			case LED_A2: PORTD &=  ~(1 << PD5);                      // LED_A2 : записать 0 в PD5
			break;                                                   //
		}                                                            //
	}
	
	void blink(numberLed led)                                                                  // Установть мерцание ветодиода
	{                                                                //
		/* Статус "мерцание" аппаратно  реализуется в прерывании     //
		таймера-счётчика 0                                           //
		*/                                                           //
		this->state_ = blink_led;                                    // Обновить статус на "мигание"
	}
	
	void invertLightLed(numberLed led)                                                          // Инвентировать состояние светодиода (вкл/выкл)                                             
	{                                                                //
		switch (led)                                                 //
		{                                                            //
			case LED_N1: PORTD ^=  (1 << PD6);                       // LED_N1 : инвентровать PD6
			break;                                                   //
			case LED_N2: PORTD ^=  (1 << PD7);                       // LED_N2 : инвентровать PD7
			break;                                                   //
			case LED_A1: PORTD ^=  (1 << PD4);                       // LED_A1 : инвентровать PD4
			break;                                                   //
			case LED_A2: PORTD ^=  (1 << PD5);                       // LED_A2 : инвентровать PD5
			break;                                                   //
		}                                                            //
	}
	
	private:
	stateLed state_;          // Переменная, хранящая статус светодиода
};

Fan fan1;      // Создаём объект "вентилятор       1"
Fan fan2;      // Создаём объект "вентилятор       2"
Led led_N1;    // Создаём объект "светодиод норма  1"
Led led_N2;    // Создаём объект "светодиод норма  2"
Led led_A1;    // Создаём объект "светодиод авария 1"
Led led_A2;    // Создаём объект "светодиод авария 2"
Time timer;    // Создаём объект "таймер            "

int main(void)
{
	while (1)
	{
		wdt_reset();
		switch(stateWork)                                                       // Выбор текущего статуса работы
		{
			case STATE_INITIALIZE:/*--------------------------- Статус "ИНИЦИАЛИЗАЦИЯ" --------------------------------------------------*/
			cli();
			port_initialize();                                                   // Инициализируем порты ввода-вывода
			ADC_initialize();                                                    // Инициализируем АЦП на еденичное преобразование
			externalInterruption_initialize();                                   // Инициализируем внешние прерывания для чтения статуса вентилятора AF00
			counter0_initialize();                                               // Инициализируем и запускаем таймер-счётчик 0
			WDT_enable();                                                        // Запускаем сторожевой таймер по алгоритму "уровня безопасности 2"
			sei();
			led_N1.enable(Led::LED_N1);                                          // Включаем все всетодиоды
			led_N2.enable(Led::LED_N2);
			led_A1.enable(Led::LED_A1);
			led_A2.enable(Led::LED_A2);
			standType = PINA & (1 << PA6) ? STAND52_54 : STAND51;                // Если в PA6(JMP1) еденица - стойка 52/54, иначе стойка 51
			fan1.setType(PINA & (1 << PA7) ?  Fan::FAN_AR00  : Fan::FAN_AF00);   // Если в PA7(JMP2) еденица - вентилятор AR00, иначе AF00
			fan2.setType(fan2.getType());                                        // Типы вентиляторов одинаковы - копируем тип в вентилятор 2
			if (fan1.getType() == Fan::FAN_AF00) GICR = (1 << INT1)|(1 << INT0); // Разрешение прерываний INT1 и INT0 для получения сигналов SFAN1 и SFAN2 в случае вентилятора типа "частота"
			delay_ms(1000);
			stateWork = STATE_WAIT_SOLUTION_TURN;                                // Переход в состояние ожидания разрешения включения вентиляторов
			break; // конец case STATE_INITIALIZE:
			
			case STATE_WAIT_SOLUTION_TURN:/*------------------- Статус "ОЖИДАНИЕ РАЗРЕШЕНИЯ ВКЛЮЧЕНИЯ ВЕНТИЛЯТОРОВ" ------------------------*/
			led_N1.disable(Led::LED_N1);                                          // Выключаем все всетодиоды
			led_N2.disable(Led::LED_N2);
			led_A1.disable(Led::LED_A1);
			led_A2.disable(Led::LED_A2);
			if(PINC & (1 << PC2)) stateWork = STATE_DIAGNOSTICS;                  // Если на PC2 (ON) еденица - переход в режим самодиагностики
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
				if (timer.endTimer())                          // Отсчёт 5ти секунд окончен - проверяем состояния
				{
					fan1.disable(Fan::FAN1);                   // Отключили все вентиляторы
					fan2.disable(Fan::FAN2);
					if (fan1.getStatFeedback() == Fan::normal) // Смотрим состояние : вентилятор1 в норме
					{
						led_N1.enable(Led::LED_N1);            // LED_N1 мигает
						led_A1.disable(Led::LED_A1);           // LED_А1 отключён
					}
					else                                       // Вентилятор1 : авария
					{
						led_N1.disable(Led::LED_N1);           // LED_N1 откоючён
						led_A1.enable(Led::LED_A1);            // LED_А1 горит
					}
					if (fan2.getStatFeedback() == Fan::normal) // Вентилятор2 : норма
					{
						led_N2.enable(Led::LED_N2);            // LED_N2 мигает
						led_A2.disable(Led::LED_A2);           // LED_А2 отключён
					}
					else                                       // Вентилятор2 : авария
					{
						led_N2.disable(Led::LED_N2);           // LED_N2 отколючён
						led_A2.enable(Led::LED_A2);            // LED_А2 горит
					}
					timer.reset();                             // 5 секунд теста окончены - сбросить таймер
					stateWork = STATE_WAIT_HEAT;               // Переходим в состояние ожидания верхнего порога температуры
				}
			} // конец STAND51
			else//---------------------------------------------Стойка 52/54
			{
				timer.start(10000);                                // Включить таймер на 10 секунд (5 на 1 вентилятор, 5 на другой)
				if (timer.getTimerTime() < 5000)                   // Пока идет тест вентилятора 1
				{
					fan1.enable(Fan::FAN1);                        // Включить вентилятор1
					fan2.disable(Fan::FAN2);                       // Отключить вентилятор2
					led_N1.blink(Led::LED_N1);                     // LED_N1 мигает
					led_N2.disable(Led::LED_N2);                   // LED_N2 отколючён
					led_A1.disable(Led::LED_A1);                   // LED_А1 отключён
					led_A2.disable(Led::LED_A2);                   // LED_А2 отключён
				}
				else if (timer.getTimerTime() == 5000)             // По истечению 5ти секунд смотрим состояние вентилятора 1
				{
					if (fan1.getStatFeedback() == Fan::normal)     // Вентилятор1 : норма
					{
						led_N1.enable(Led::LED_N1);                // LED_N1 горит
						led_A1.disable(Led::LED_A1);               // LED_А1 отключён
					}
					else                                           // Вентилятор1 : авария
					{
						led_N1.disable(Led::LED_N1);               // LED_N1 отключён
						led_A1.enable(Led::LED_A1);                // LED_А1 горит
					}
				}
				else                                               // 5 секунд теста вентилятора 1 истекло, тест вентилятора 2
				{
					fan1.disable(Fan::FAN1);                       // Отключить вентилятор1
					fan2.enable(Fan::FAN2);                        // Включить вентилятор2
					led_N2.blink(Led::LED_N2);                     // LED_N2 мигает
					led_A2.enable(Led::LED_A2);                    // LED_А2 горит
					if (timer.endTimer())                          // Суммарные 10 секунд теста истекло : смотрим результат для вентилятора 2
					{
						fan2.disable(Fan::FAN2);                   // Отключить вентилятор2
						if (fan2.getStatFeedback() == Fan::normal) // Вентилятор2 : норма
						{
							led_N2.enable(Led::LED_N2);            // LED_N2 горит
							led_A2.disable(Led::LED_A2);           // LED_А2 отключён
						}
						else                                       // Вентилятор2 : авария
						{
							led_N2.disable(Led::LED_N2);           // LED_N2 отключён
							led_A2.enable(Led::LED_A2);            // LED_А2 горит
						}
						timer.reset();                             // Сбрасываем таймер
						stateWork = STATE_WAIT_HEAT;               // Тесты окончены, преходим в состояня ожидания верхнего порога температуры
					}
				}
			} // Конец else STAND 52/54
			break; // конец case STATE_DIAGNOSTICS:
			
			case STATE_WAIT_HEAT:/*----------------------------- Статус "ОЖИДАНИЕ ВЕРХНЕГО ПОРОГА ТЕМПЕРАТУРЫ" -----------------------------*/
			if (gettemperature() >= HIGH_TEMPERATURE)              // Превысили верхний порог температуры
			{
				if (standType == STAND51) //------------------------- Для стойки 51
				{
					timer.start(5000);                              // Включить таймер на 5 секунд
					fan1.enable(Fan::FAN1);                         // Включили все вентиляторы
					fan2.enable(Fan::FAN2);
					if (timer.endTimer())                           // Таймер закончил отсчёт
					{
						if (fan1.getStatFeedback() == Fan::normal)  // Смотрим состояние : вентилятор1 в норме
						{
							led_N1.blink(Led::LED_N1);              // LED_N1 мигает
							led_A1.disable(Led::LED_A1);            // LED_A1 отключен
							stateWork = STATE_WAIT_HOLD;            // Перешли в ожидание нижнего порога температуры (после просмотра статуса венилятора2)
						}
						else                                        // Вентилятор 1 неисправен
						{
							led_N1.disable(Led::LED_N1);            // LED_N1 отключён
							led_A1.enable(Led::LED_A1);             // LED_A1 горит
						}
						if (fan2.getStatFeedback() == Fan::normal)  // Смотрим состояние : вентилятор2 в норме
						{
							led_N2.blink(Led::LED_N2);              // LED_N2 мигает
							led_A2.disable(Led::LED_A2);            // LED_A2 отключен
							stateWork = STATE_WAIT_HOLD;            // Перешли в ожидание нижнего порога температуры
						}
						else                                        // Вентилятор 2 неисправен
						{
							led_N2.disable(Led::LED_N2);            // LED_N2 отключён
							led_A2.enable(Led::LED_A2);             // LED_A2 горит
							
							//   В случае неиспраности обоих вентиляторов будем пытаться их включить в этом блоке
							// и каждые 5 секунд проверять состояние в надежде на улучшение
							// или до тех пор, пока температура сама не снизится.
						}
					}// конец if (timer.endTimer())
				}// конец if (standType == STAND51)
				else //--------------------------------------------Для стойки 52/54
				{
					timer.start(10000);                                 // Начать отсчёт 10ти секунд для поочерёдных тестов вентиляторов
					if (timer.getTimerTime() < 5000)                    // Пока идёт тест вентилятора1
					{
						fan1.enable(Fan::FAN1);                         // Вентилятор1 включён
						fan2.disable(Fan::FAN2);                        // Вентилятор2 отключён
					}
					else if (timer.getTimerTime() == 5000)              // 5 секунд теста вентилятора 1 прошли
					{
						
						if (fan1.getStatFeedback() == Fan::normal)      // Смотрим состояние вентилятора 1 : норма
						{
							led_N1.blink(Led::LED_N1);                  // LED_N1 мигает
							led_A1.disable(Led::LED_A1);                // LED_A1 отключён
							stateWork = STATE_WAIT_HOLD;                // Переход в состояние "Ожидание нижнего порога температур"
						}
						else                                            // Вентилятор1 неисправен
						{
							fan1.disable(Fan::FAN1);                    // Отключить вентилятор1
							led_N1.disable(Led::LED_N1);                // LED_N1 отключён
							led_A1.enable(Led::LED_A1);                 // LED_A1 горит
						}
					}
					else                                                // 5 секунд теста вентилятора1 истекло и он неисправен, то начинаем тест вентилятора 2
					{
						fan2.enable(Fan::FAN2);                         // Включить вентилятор2
						if (timer.endTimer())                           // Суммарные 10 секунд теста истекло : смотрим результат
						{
							if (fan2.getStatFeedback() == Fan::normal)  // Венилятор2 : норма
							{
								led_N2.blink(Led::LED_N2);              // LED_N2 мигает
								led_A2.disable(Led::LED_A2);            // LED_A2 отключён
								stateWork = STATE_WAIT_HOLD;            // Переход в состояние "Ожидание нижнего порога температур"
							}
							else                                        // Вентилятор2 : неисправен
							{
								fan2.disable(Fan::FAN2);                // Отключить вентилятор2
								led_N2.disable(Led::LED_N2);            // LED_N2 отключён
								led_A2.enable(Led::LED_A2);             // LED_A2 горит
							}
						}
					}
				} // конец else STAND 52/54
			} // конец 	if (gettemperature() >= HIGH_TEMPERATURE)
			else                                                        // Если верхний порог не превышен
			{
				fan1.disable(Fan::FAN1);                                // Оба вентилятора отключены
				fan2.disable(Fan::FAN2);
			}
			break;  // конец case STATE_WAIT_HEAT:
			
			case STATE_WAIT_HOLD:/*----------------------------- Статус "ОЖИДАНИЕ НИЖНЕГО ПОРОГА ТЕМПЕРАТУРЫ --------------------------------*/
			if (gettemperature() >= LOW_TEMPERATURE)                    // Нижний порог не достигнут - процесс охлаждения
			{
				if (standType == STAND51) //----------------------------- Для стойки 51
				{
					fan1.enable(Fan::FAN1);                             // Оба вентилятора всегда включены
					fan2.enable(Fan::FAN1);
					if (fan1.getStatFeedback() == Fan::normal)          // Вентилятор1 : норма
					{
						led_N1.blink(Led::LED_N1);                      // LED_N1 мигает
						led_A1.disable(Led::LED_A1);                    // LED_A1 отключён
					}
					else                                                // Вентилтор1 : авария
					{
						led_N1.disable(Led::LED_N1);                    // LED_N1 отключён
						led_A1.enable(Led::LED_A1);                     // LED_A1 горит
					}
					
					if (fan2.getStatFeedback() == Fan::normal)          // Вентилятор2 : норма
					{
						led_N2.blink(Led::LED_N2);                      // LED_N2 мигает
						led_A2.disable(Led::LED_A2);                    // LED_A2 отключён
					}
					else                                                // Вентилтор2 : авария
					{
						led_N2.disable(Led::LED_N2);                    // LED_N2 отключён
						led_A2.enable(Led::LED_A2);                     // LED_A2 горит
					}
				}
				else//----------------------- Для стойки 52/54
				{
					timer.start(5000);                                                            // Включить таймер на 5 секунд
					if(timer.endTimer())                                                          // После окончания отсчёта 5ти секунд
					{
						Fan::stateFeedbackFan stateFeedbackFan1 = fan1.getStatFeedback();         // Записываем статсы обратных связей в локальные переменные с целью оптимизации
						Fan::stateFeedbackFan stateFeedbackFan2 = fan2.getStatFeedback();
						
						if (stateFeedbackFan1 == Fan::normal && stateFeedbackFan2 == Fan::normal) // Оба вентилятора в норме
						{
							fan2.disable(Fan::FAN2);               // Вентилятор2 отключён
							fan1.enable(Fan::FAN1);                // Вентилятор1 включён
							led_N1.blink(Led::LED_N1);             // LED_N1 мигает
							led_A1.disable(Led::LED_A1);           // LED_A1 отключён
							led_N2.enable(Led::LED_N2);            // LED_N2 горит
							led_A2.disable(Led::LED_A2);           // LED_A2 отключён
						}
						else if(stateFeedbackFan1 == Fan::normal && stateFeedbackFan2 == Fan::fail) // В норме только вентилятор1
						{
							fan2.disable(Fan::FAN2);               // Вентилятор2 отключён
							fan1.enable(Fan::FAN1);                // Вентилятор1 включён
							led_N1.blink(Led::LED_N1);             // LED_N1 мигает
							led_A1.disable(Led::LED_A1);           // LED_A1 отключён
							led_N2.disable(Led::LED_N2);           // LED_N2 отключён
							led_A2.enable(Led::LED_A2);            // LED_A2 горит
						}
						else if (stateFeedbackFan1 == Fan::fail && stateFeedbackFan2 == Fan::normal) // В норме только вентилятор2
						{
							fan1.disable(Fan::FAN1);               // Вентилятор1 отключён
							fan2.enable(Fan::FAN2);                // Вентилятор2 включён
							led_N1.disable(Led::LED_N1);           // LED_N1 отключён
							led_A1.enable(Led::LED_A1);            // LED_A1 горит
							led_N2.blink(Led::LED_N2);             // LED_N2 мигает
							led_A2.disable(Led::LED_A2);           // LED_A2 отключён
						}
						else                                                                         // Оба вентилятора неисправны
						{
							led_N1.disable(Led::LED_N1);           // LED_N1 отключён
							led_A1.enable(Led::LED_A1);            // LED_A1 горит
							led_N2.disable(Led::LED_N2);           // LED_N2 отключён
							led_A2.enable(Led::LED_A2);            // LED_A2 горит
							if (fan1.checkEnable())                // Проверяем, есть ли сигнал включения для вентилятора1
							{
								fan1.disable(Fan::FAN1);           // Отключаем вентилятор1
								fan2.enable(Fan::FAN2);            // Включаем вентилятор2
							}
							else                                   // Если нет сигнала дя велючения вентилятора1,
							{                                      // значит есть сигнал для включении вентилятора2
								fan1.enable(Fan::FAN1);            // Включаем вентилятор1
								fan2.disable(Fan::FAN2);           // Отключаем вентилятор2
							}
						}
					} // конец if(timer.endTimer())     
				} // конец else для стойки 51_54
			} // конец if (gettemperature() >= LOW_TEMPERATURE)   
			else                                                   // Достигли нижнего порога температуры
			{
				if (fan1.getStatFeedback() == Fan::normal)         // Вентилятор1 : норма
				{         
					led_N1.enable(Led::LED_N1);                    // LED_N1 горит
					led_A1.disable(Led::LED_A1);                   // LED_А1 отключён
				}
				else                                               // Вентилятор1 : авария 
				{
					led_N1.disable(Led::LED_N1);                   // LED_N1 отключён
					led_A1.enable(Led::LED_A1);                    // LED_А1 горит
				}
				if (fan2.getStatFeedback() == Fan::normal)         // Вентилятор2 : норма
				{          
					led_N2.enable(Led::LED_N2);                    // LED_N2 горит
					led_A2.disable(Led::LED_A2);                   // LED_А2 отключён
				}
				else                                               // Вентилятор2 : авария 
				{
					led_N2.disable(Led::LED_N2);                   // LED_N2 отключён
					led_A2.enable(Led::LED_A2);                    // LED_А2 горит
				}
				timer.reset();                                     // Сбрасываем таймер
				stateWork = STATE_WAIT_HEAT;                       // Переходим в режим "Ожидание верхнего порога температуры"
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
	unsigned int lastTime = timer.getGlobalTime();                                              // Получаем время для его обновления
	lastTime < MAX_TIME ? timer.setGlobalTime(lastTime + 100) : timer.setGlobalTime(0);         // Увеличиваем время на 100 мс, если отсчитали минуту - обнуляем
	
	if (lastTime % 1000 == 0)                                                                   // Каждую секунду:
	{                                                                                           //
		if (led_N1.getStateLed() == Led::blink_led) led_N1.invertLightLed(Led::LED_N1);         // Если статус LED_N1 мигание - инвертируем его состояние
		if (led_N2.getStateLed() == Led::blink_led) led_N2.invertLightLed(Led::LED_N2);         // Если статус LED_N2 мигание - инвертируем его состояние
		if (led_N1.getStateLed() == Led::blink_led && led_N2.getStateLed() == Led::blink_led)   // Если оба светодиода мигают
		{                                                                                       //
			if (!(PIND & (1 << PD6)) && PIND & (1 << PD7)) led_N1.invertLightLed(Led::LED_N1);  // Синхронизируем их мигание, если в логический ровениь в PD6 и PD7 различен
		}
	}
	
	unsigned int timeToFail = 0;                                                                // Создаём локальную переменню времени до принятия решения о неработоспособности вентиляторов
	if (fan1.checkEnable())                                                                     // Ели есть сигнал включения вентилятора1
	{                                                                                           //
		timeToFail = fan1.getTimeToFail();                                                      // Получаем прошлое время до принятия решения о неработоспособности вентилятора1
		if (timeToFail >= 100) fan1.setTimeToFail(timeToFail - 100);                            // Уменьшаем его на 100 мс
		fan1.checkFeedback(Fan::FAN1);                                                          // Проверяем сигнал SFAN1, если его долго нет, обновляем статус на "fail" (внутри метода)
	}                                                                                           //
	if (fan2.checkEnable())                                                                     // Ели есть сигнал включения вентилятора2
	{                                                                                           //
		timeToFail = fan2.getTimeToFail();                                                      // Получаем прошлое время до принятия решения о неработоспособности вентилятора1
		if (timeToFail >= 100) fan2.setTimeToFail(timeToFail - 100);                            // Уменьшаем его на 100 мс
		fan2.checkFeedback(Fan::FAN2);                                                          // Проверяем сигнал SFAN2, если его долго нет, обновляем статус на "fail" (внутри метода)
	}                                                                                           //
	
} // конец ISR (TIMER0_COMP_vect)

ISR (INT0_vect)                            /* Прерывание INT0 */
{
	fan1.setTimeToFail(TIME_TO_FAIL);      // Восстанавливем счётчик вентилятора 1
} // конец ISR (INT0_vect)

ISR (INT1_vect)                            /* Прерывание INT1 */
{
	fan2.setTimeToFail(TIME_TO_FAIL);      // Восстанавливем счётчик вентилятора 2
	
} // конец ISR (INT1_vect)
