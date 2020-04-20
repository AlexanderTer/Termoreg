#include <avr/io.h>            // �������� ����������� ����������������
#include <avr/interrupt.h>     // ���������� ������ ����������, ������� ����������/���������� ����������
//#include <util/delay.h>
#include "termoreg.h"
#include "initialize.h"        // ������������� ����� �����������

#define F_CPU 4000000UL
#define wdt_reset() asm("wdr") // ����� ����������� �������
#define delay_ms(time_ms)       for(unsigned long i = 0; i < ( ( (F_CPU / 1000) / 18) * time_ms); ++i){asm("  nop;");  }

#define highTemperature 509    // 17 �������� � �������� �� ������� �����������
#define lowTemperature  465    // 13 �������� � �������� �� ������� �����������
#define TimeToFail      4000    // 4 � �������� �� ������� "������" ��� ��������� �����������
#define MAXTIME         59900  // �����, �� �������� ����� ������� ���������� ������ ������� - ������

StateWork stateWork = STATE_INITIALIZE;                    // ��������� ������ - �������������
typeStand standType ;                                     // ���������� ��� �������� ���� ������

/*
���������� � ������ ������� "Fan" � "Led":

�.� �++ �� ������������ �������������� ������� ���
����������� �������, �� � ������� ������� ��� ������ �
���������� ����� ���������� ���� ���������,
��� ������ ������� ���������� ����� ������������.
*/

class Time
{
	public:
	void setGlobalTime(unsigned int globalTime) {this->globalTime_ = globalTime;} // ��������� ���������� ����� � ���������� �������-��������0
	unsigned int getGlobalTime(void)            {return globalTime_;}             // ����� ��������� �������� ������� ��� ��� ���������� � ���������� �������-��������0
	

	void reset(void){flagTimer_ = true;}
	void start(unsigned int endTime)                             // ����� ������ �������
	{
		
		if(flagTimer_)                                       // ���� ���� ������� ������, ���������� ����� ������ �������:
		{
			startTime_ = globalTime_;                            // ���������� ������� �����
			flagTimer_ = false;                              // �������� ���� �������, �������� �������:
		}
		else                                                 // ���� ���� ������� ������
		{
			if (startTime_ + endTime > MAXTIME )                   // ��������� ����� ��������� ������������ ����� �������
			{
				timeTimer_ = globalTime_ <= MAXTIME ? globalTime_ - startTime_: MAXTIME - startTime_ + globalTime_;
				if (globalTime_ == startTime_ - (MAXTIME -  endTime) ) // ������ ��������, �������� ����� ��������� �� ������ �����
				{
					flagTimer_ = true;                       // ��������� ������ ��� ����������� ���������� �������
				}
			}
			else
			{
				timeTimer_ = globalTime_ - startTime_;
				if (globalTime_ == startTime_ + endTime) flagTimer_ = true;      // ���� ����� ��������� ������� �� ��������� �������� ����� � ��������� ������������� �����
			}
		}
		
		
	}
	unsigned int getTimerTime(void) {return timeTimer_;}
	bool endTimer(void) {return flagTimer_;}                 //  ������� ������ ��������� �������
	
	
	private:
	unsigned int globalTime_;
	unsigned int startTime_;
	bool flagTimer_= true;
	unsigned int timeTimer_;
};

class Fan
{
	public:
	typedef enum {enable_fan,disable_fan,}stateWorkFan;             /* ��������� ��������� ������������ */
	typedef enum {fail,normal,}stateFeedbackFan;                    /* ������ ������������ */
	typedef enum{FAN_AR00,FAN_AF00,}typeFan;                        /* ���� ������������ */
	typedef enum {FAN1,FAN2} numberFan;

	void setType(typeFan type) {this->type_ = type;} // ������������� ��� �����������
	
	typeFan getType() {return type_;}
	
	void setStatFeedback(stateFeedbackFan statFeedback) {this->statFeedback_ = statFeedback;} // ������������� ������ �������� �����

	stateFeedbackFan getStatFeedback() {return statFeedback_;} // ���������� ������ �������� �����
	
	void setTimeToFail(unsigned int timeToFail){this->timeToFail_ = timeToFail;}
	
	unsigned int getTimeToFail(){return timeToFail_;}
	
	void enable(numberFan FAN) // �������� ����������
	{
		this->stateWork_ = enable_fan;
		if (FAN == FAN1) PORTC |=  (1 << PC3);
		else PORTC |=  (1 << PC4);
	}
	bool checkEnable(){return (stateWork_ == enable_fan ? true : false);}
	void disable(numberFan FAN) // ��������� ����������
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
	typeFan type_;                  // ��� �����������       (AR00 / AF00)
	stateFeedbackFan statFeedback_; // ������ �������� ����� (normal / fail)
	stateWorkFan stateWork_;        // ������ ������         (enable_fan / disable_fan)
	unsigned int timeToFail_;       //  ���������� ����� �� �������� ������� � ������������������� ����������
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
		switch(stateWork)                                               // ����� �������� ������� ������
		{
			case STATE_INITIALIZE:/*--------------------------- ������ "�������������" --------------------------------------------------*/
			cli();
			port_initialize();                                      // �������������� ����� �����-������
			ADC_initialize();                                       // �������������� ��� �� ��������� ��������������
			externalInterruption_initialize();                      // �������������� ������� ���������� ��� ������ ������� ����������� AF00
			counter0_initialize();                                  // �������������� � ��������� ������-������� 0
			WDT_enable();                                           // ��������� ���������� ������ �� ��������� "������ ������������ 2"
			sei();
			led_N1.enable(Led::LED_N1);                                          // �������� ��� ����������
			led_N2.enable(Led::LED_N2);
			led_A1.enable(Led::LED_A1);
			led_A2.enable(Led::LED_A2);
			standType = PINA & (1 << PA6) ? STAND52_54 : STAND51;                // ���� � PA6(JMP1) ������� - ������ 52/54, ����� ������ 51
			fan1.setType(PINA & (1 << PA7) ?  Fan::FAN_AR00  : Fan::FAN_AF00);   // ���� � PA7(JMP2) ������� - ���������� AR00, ����� AF00
			fan2.setType(fan2.getType());
			if (fan1.getType() == Fan::FAN_AF00) GICR = (1 << INT1)|(1 << INT0); // ���������� ���������� INT1 � INT0 ��� ��������� �������� SFAN1 � SFAN2
			delay_ms(1000);
			stateWork = STATE_WAIT_SOLUTION_TURN;                                // ������� � ��������� �������� ���������� ��������� ������������
			break; // ����� case STATE_INITIALIZE:
			
			case STATE_WAIT_SOLUTION_TURN:/*------------------- ������ "�������� ���������� ��������� ������������" ------------------------*/
			led_N1.disable(Led::LED_N1);                                          // �������� ��� ����������
			led_N2.disable(Led::LED_N2);
			led_A1.disable(Led::LED_A1);
			led_A2.disable(Led::LED_A2);
			if(PINC & (1 << PC2)) stateWork = STATE_DIAGNOSTICS;   // ���� �� PC2 (ON) ������� - ������� � ����� ���������������
			break;  // ����� case STATE_WAIT_SOLUTION_TURN:
			
			case STATE_DIAGNOSTICS:/*--------------------------- ������ "���������������" ---------------------------------------------------*/
			if (standType == STAND51) //-------------------        ��� ������ 51
			{
				timer.start(5000);                             // ������ ������ �� 5 ������
				fan1.enable(Fan::FAN1);                        // �������� ��� �����������
				fan2.enable(Fan::FAN2);
				led_N1.blink(Led::LED_N1);                     // LED_N1 �������
				led_N2.blink(Led::LED_N2);                     // LED_N2 �������
				led_A1.enable(Led::LED_A1);                    // LED_�1 �����
				led_A2.enable(Led::LED_A2);                    // LED_�2 �����
				if (timer.endTimer())
				{
					fan1.disable(Fan::FAN1);                   // ��������� ��� �����������
					fan2.disable(Fan::FAN2);
					if (fan1.getStatFeedback() == Fan::normal) // ������� ���������
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
					stateWork = STATE_WAIT_HEAT;        // 5 ������ ����� �������� - ��������� � ��������� �������� �������� ������ �����������
				}
			} // ����� STAND51
			else//---------------------------------------------������ 52/54
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
					{           // ������� ���������
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
				{                                    // 5 ������ ����� ����������� 1 �������, ���� ����������� 2
					fan1.disable(Fan::FAN1);
					fan2.enable(Fan::FAN2);
					led_N2.blink(Led::LED_N2);
					led_A2.enable(Led::LED_A2);
					if (timer.endTimer())
					{             // ������ 5 ������ ����� ��� ����������� 2 ������� : ������� ���������
						fan2.disable(Fan::FAN2);
						if (fan2.getStatFeedback() == Fan::normal)
						{           // ������� ���������
							led_N2.enable(Led::LED_N2);
							led_A2.disable(Led::LED_A2);
						}
						else
						{
							led_N2.disable(Led::LED_N2);
							led_A2.enable(Led::LED_A2);
						}
						timer.reset();
						stateWork = STATE_WAIT_HEAT;  // ��������� 10 ������ ����� ��������, �������� � �������� �������� �������� ������ �����������
					}
				}
			}
			break; // ����� case STATE_DIAGNOSTICS:
			
			case STATE_WAIT_HEAT:/*----------------------------- ������ "�������� �������� ������ �����������" -----------------------------*/
			if (gettemperature() >= highTemperature)       // ��������� ������� ����� �����������
			{
				if (standType == STAND51) // ��� ������ 51
				{
					timer.start(5000);
					fan1.enable(Fan::FAN1);                        // �������� ��� �����������
					fan2.enable(Fan::FAN2);
					if (timer.endTimer())
					{
						if (fan1.getStatFeedback() == Fan::normal) // ������� ���������
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
				else //------------------------------------------/��� ������ 52/54
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
						{           // ������� ���������
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
					{                                    // 5 ������ ����� ����������� 1 �������, ���� ����������� 2
						fan2.enable(Fan::FAN2);
						if (timer.endTimer())  // ������ 5 ������ ����� ��� ����������� 2 ������� : ������� ���������
						{
							if (fan2.getStatFeedback() == Fan::normal)
							{           // ������� ���������
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
			else // ���� ������� ����� �� ��������
			{
				fan1.disable(Fan::FAN1);
				fan2.disable(Fan::FAN2);
			}
			break;  // ����� case STATE_WAIT_HEAT:
			
			case STATE_WAIT_HOLD:/*----------------------------- ������ "�������� ������� ������ ����������� --------------------------------*/
			if (gettemperature() >= lowTemperature)     // ������ ����� �� ��������� - ������� ����������
			{
				if (standType == STAND51) // ��� ������ 51
				{
					if (fan1.getStatFeedback() == Fan::normal)
					{           // ������� ���������
						led_N1.blink(Led::LED_N1);
						led_A1.disable(Led::LED_A1);
					}
					else
					{
						led_N1.disable(Led::LED_N1);
						led_A1.enable(Led::LED_A1);
					}
					
					if (fan2.getStatFeedback() == Fan::normal)
					{           // ������� ���������
						led_N2.blink(Led::LED_N2);
						led_A2.disable(Led::LED_A2);
					}
					else
					{
						led_N2.disable(Led::LED_N2);
						led_A2.enable(Led::LED_A2);
					}
					
				}
				else//----------------------- ��� ������ 52/54
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
						else // ��� ����������� ����������
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
			else// �������� ������� ������ �����������
			{
				if (fan1.getStatFeedback() == Fan::normal)
				{           // ������� ���������
					led_N1.enable(Led::LED_N1);
					led_A1.disable(Led::LED_A1);
				}
				else
				{
					led_N1.disable(Led::LED_N1);
					led_A1.enable(Led::LED_A1);
				}
				
				if (fan2.getStatFeedback() == Fan::normal)
				{           // ������� ���������
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
			break; // ����� case STATE_WAIT_HOLD:
		}// ����� switch(stateWork)
	} // ����� while(1)
} // ����� main(void)


int gettemperature()      /* �������� ����������� */
{
	/*
	��� � ������ ���������� ��������������, ������� ������������� = 31250 ���
	��������� ������������ � 8�� ������ �������� ADCH ADCL (c.217 �������� ��������),
	������ ��� �������� �������� ��� ���� 16�� ������ ADC
	*/
	ADCSRA |= (1 << ADSC);             // ������ ��������������. ADSC ��������� ��������� � 0 ����� ��������������
	while (ADCSRA & (1 << ADSC)){};    // ��� ��������� �������������� : 13 ������ = 3.25 ���
	return (int)ADC;                   // ���������� ��������� �� ��������� ADCH � ADCL
}// ����� int gettemperature()

ISR (TIMER0_COMP_vect)          /* ���������� �� ���������� �������-�������� 0 */
{
	unsigned int lastTime = timer.getGlobalTime();
	lastTime < MAXTIME ? timer.setGlobalTime(lastTime + 100) : timer.setGlobalTime(0);
	
	if (lastTime % 1000 == 0)
	{
		
		if (led_N1.getStateLed() == Led::blink_led) led_N1.invertLightLed(Led::LED_N1);
		if (led_N2.getStateLed() == Led::blink_led) led_N2.invertLightLed(Led::LED_N2);
		if (led_N1.getStateLed() == Led::blink_led && led_N2.getStateLed() == Led::blink_led)
		{
			if (!(PIND & (1 << PD6)) && PIND & (1 << PD7)) led_N1.invertLightLed(Led::LED_N1); // ���� ��� ���������� ������, �������������� �� �������
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
} // ����� ISR (TIMER0_COMP_vect)

ISR (INT0_vect)                            /* ���������� INT0 */
{
	fan1.setTimeToFail(TimeToFail);      // �������������� ������� ����������� 1
} // ����� ISR (INT0_vect)

ISR (INT1_vect)                            /* ���������� INT1 */
{
	fan2.setTimeToFail(TimeToFail);                // �������������� ������� ����������� 2
} // ����� ISR (INT1_vect)
