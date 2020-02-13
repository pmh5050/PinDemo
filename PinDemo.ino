/*
    Name:       PinDemo.ino
    Created:	2020-02-11 오후 7:34:29
    Author:     DESKTOP-RB2A5N8\pmh5050
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Encoder Pin A
/** 
* 외부 인터럽트 : 4개
INT2 : E0
INT3 : E1
INT4 : E2
INT5 : E3

* Pin Change 인터럽트 : 6개
** PCINT0_vect
PCINT0 : E4
PCINT1 : E5
** PCINT1_vect
(PCINT8의 경우 RX Pin과 동시에 연결되어 있으므로 지양합니다.)
PCINT9 : E6
PCINT10 : E7
** PCINT2_vect
PCINT16 : E8
PCINT17 : E9

[2개의 모터 추가 대비]
(Arduino Mega2560 상에서 Pin이 존재하지 않음)
INT6 : E10
INT7 : E11
*/

// Encoder Pin B
/**
PA0 : E0
PA1 : E1
PA2 : E2
PA3 : E3

PC0 : E4
PC1 : E5
PC2 : E6
PC3 : E7

PF0 : E8
PF1 : E9

[2개의 모터 추가 대비]
PF2 : E10
PF3 : E11
*/

// Motor V+, V-
/**
(analogWrite 함수를 사용하므로, Arduino의 Pin 번호 기준에 해당합니다.)
Motor V+, V-

* Motor[0:7] V+, V- : 
16개 Adafruit motor driver module로 결선됩니다.
* Motor8
D4 : M8_V+
D5 : M8_V- 
* Motor9
D7 : M9_V+
D8 : M9_V-

[2개의 모터 추가 대비]
* Motor10
D9 : M10_V+
D10 : M10_V-
* Motor11
D11 : M11_V+
D12 : M11_V-
*/

// Pin chnage interrupt에서 ISR의 원인이 되는 핀을 검출하기 위한 변수에 해당합니다.
int PreviousPINB; // PCINT0_vect
int PreviousPINJ; // PCINT1_vect
int PreviousPINK; // PCINT2_vect

const int EncAPins[] = { 19, 18, 2, 3 };

#define MASK_0_BIT & 0x01
#define MASK_1_BIT & 0x02
#define MASK_2_BIT & 0x04
#define MASK_3_BIT & 0x08
#define MASK_4_BIT & 0x10
#define MASK_5_BIT & 0x20
#define MASK_6_BIT & 0x40
#define MASK_7_BIT & 0x80

#define MOTOR_LENGTH 10

#define MOTOR_P_GAIN 2
#define MOTOR_D_GAIN 3 / 10 // 0.3

#define MIN_PWM -100
#define MAX_PWM 100

#define POSITIVE_INDEX 0
#define NEGATIVE_INDEX 1

int32_t DesiredEncCount[MOTOR_LENGTH] = { 0 };
int32_t CurrentEncCount[MOTOR_LENGTH] = { 0 };

int MotorIndex = 0;

uint8_t MotorPins[MOTOR_LENGTH][2];

Adafruit_PWMServoDriver AdafruitDriver = Adafruit_PWMServoDriver();

// PORT A : [E0:E3]
// PORT C : [E4:E7]
// PORT F : [E8:E9]

void RisedEncA0()
{
	if (PINA MASK_0_BIT)
	{
		CurrentEncCount[0]++;
	}
	else
	{
		CurrentEncCount[0]--;
	}

}

void RisedEncA1()
{
	if (PINA MASK_1_BIT)
	{
		CurrentEncCount[1]++;
	}
	else
	{
		CurrentEncCount[1]--;
	}

}

void RisedEncA2()
{
	if (PINA MASK_2_BIT)
	{
		CurrentEncCount[2]++;
	}
	else
	{
		CurrentEncCount[2]--;
	}

}

void RisedEncA3()
{
	if (PINA MASK_3_BIT)
	{
		CurrentEncCount[3]++;
	}
	else
	{
		CurrentEncCount[3]--;
	}

}

// B x (A xor B) = A x (not B)
ISR(PCINT0_vect)
{
	if ((PINB MASK_0_BIT) & !(PreviousPINB MASK_0_BIT)) // PINB0(E4_A)의 Rising Edge 검출 시
	{
		
		if (PINC MASK_0_BIT)
		{
			CurrentEncCount[4]++;
		}
		else
		{
			CurrentEncCount[4]--;
		}
		
	}
	else if ((PINB MASK_1_BIT) & !(PreviousPINB MASK_1_BIT)) // PINB1(E5_A)의 Rising Edge 검출 시
	{
		
		if (PINC MASK_1_BIT)
		{
			CurrentEncCount[5]++;
		}
		else
		{
			CurrentEncCount[5]--;
		}
	}

	PreviousPINB = PINB;

}

ISR(PCINT1_vect)
{
	if ((PINJ MASK_0_BIT) & !(PreviousPINJ MASK_0_BIT)) // PINJ0(E6_A)의 Rising Edge 검출 시
	{
		if (PINC MASK_2_BIT)
		{
			CurrentEncCount[6]++;
		}
		else
		{
			CurrentEncCount[6]--;
		}

	}
	else if ((PINJ MASK_1_BIT) & !(PreviousPINJ MASK_1_BIT)) // PINJ1(E7_A)의 Rising Edge 검출 시
	{		
		if (PINC MASK_3_BIT)
		{
			CurrentEncCount[7]++;
		}
		else
		{
			CurrentEncCount[7]--;
		}
	}

	PreviousPINJ = PINJ;

}

ISR(PCINT2_vect)
{
	if ((PINK MASK_0_BIT) & !(PreviousPINK MASK_0_BIT)) // PINJ0(E6_A)의 Rising Edge 검출 시
	{
		if (PINF MASK_4_BIT)
		{
			CurrentEncCount[8]++;
		}
		else
		{
			CurrentEncCount[8]--;
		}

	}
	else if ((PINK MASK_1_BIT) & !(PreviousPINK MASK_1_BIT))
	{
		// PINJ0(E7_A)의 Rising Edge 검출 시
		if (PINF MASK_5_BIT)
		{
			CurrentEncCount[9]++;
		}
		else
		{
			CurrentEncCount[9]--;
		}
	}

	PreviousPINK = PINK;

}

/** Motor의 Index를 입력받아 제어를 수행합니다. */
void MotorPositionController(int SelectedMotorIndex)
{
	int EncError = DesiredEncCount[SelectedMotorIndex] - CurrentEncCount[SelectedMotorIndex];
	int ControlTerm = EncError * MOTOR_P_GAIN + EncError * MOTOR_D_GAIN;

	ControlTerm = constrain(ControlTerm, MIN_PWM, MAX_PWM);
	SetMotorDriverWithPWM(SelectedMotorIndex, ControlTerm);

}

/** Motor의 Index와 -4095 ~ 4095 사이의 PWM 값을 입력받아 Motor를 설정합니다. */
void SetMotorDriverWithPWM(int MotorIndex, int PWM)
{
	bool bPosDir = (PWM > 0);

	if (MotorIndex < 8)
	{
		// Use Adafruit motor driver
		// V+ : MotorIndex * 2
		// V- : MotorIndex * 2 + 1
		if (bPosDir)
		{
			AdafruitDriver.setPWM(MotorIndex * 2, 1, PWM);
			AdafruitDriver.setPWM(MotorIndex * 2 + 1, 0, 4096); // Specific params (Off)
		}
		else
		{
			AdafruitDriver.setPWM(MotorIndex * 2, 0, 4096); // Specific params (Off)
			AdafruitDriver.setPWM(MotorIndex * 2 + 1, 1, PWM); 
		}
	}
	else
	{
		// Use L9110 motor driver
		if (bPosDir)
		{
			analogWrite(MotorPins[MotorIndex][POSITIVE_INDEX], PWM / 16);
			analogWrite(MotorPins[MotorIndex][NEGATIVE_INDEX], 0);
		}
		else
		{
			analogWrite(MotorPins[MotorIndex][POSITIVE_INDEX], 0);
			analogWrite(MotorPins[MotorIndex][NEGATIVE_INDEX], PWM / 16);
		}
	}

}

void setup()
{
	PCICR |= 0x07; // Enable PCINT[0:2]_vect
	PCMSK0 = 0x03; // Enable PCINT[0:1]
	PCMSK1 = 0x06; // Enable PCINT[9:10]
	PCMSK2 = 0x03; // Enable PCINT[16:17]

	// A, C, F Port의 하위 4비트는 Encoder Pin B
	DDRA = 0x00;
	DDRC = 0x00;
	DDRF = 0x00;
	 
	sei();

	// Encoder로 결정되는 초기 Pin 전압 상태를 저장합니다.
	PreviousPINB = 0x03 & PINB;
	PreviousPINJ = 0x06 & PINJ;
	PreviousPINK = 0x03 & PINK;

	// [Motor Index][V+ / V-]
	MotorPins[8][POSITIVE_INDEX] = 4;
	MotorPins[8][NEGATIVE_INDEX] = 5;
	MotorPins[9][POSITIVE_INDEX] = 7;
	MotorPins[9][NEGATIVE_INDEX] = 8;

	for (MotorIndex = 8; MotorIndex < MOTOR_LENGTH; MotorIndex++)
	{
		pinMode(MotorPins[MotorIndex][POSITIVE_INDEX], OUTPUT);
		pinMode(MotorPins[MotorIndex][NEGATIVE_INDEX], OUTPUT);
	}

	attachInterrupt(digitalPinToInterrupt(EncAPins[0]), RisedEncA0, RISING); // INT2
	attachInterrupt(digitalPinToInterrupt(EncAPins[1]), RisedEncA1, RISING); // INT3
	attachInterrupt(digitalPinToInterrupt(EncAPins[2]), RisedEncA2, RISING); // INT4
	attachInterrupt(digitalPinToInterrupt(EncAPins[3]), RisedEncA3, RISING); // INT5

	// Adafruit I2C Motor driver module settings
	AdafruitDriver.begin();
	Wire.setClock(400000); // 'Fast 400kHz I2C' mode를 사용합니다.

}

void loop()
{
	for (MotorIndex = 0; MotorIndex < MOTOR_LENGTH; MotorIndex++)
	{
		MotorPositionController(MotorIndex);
	}

}