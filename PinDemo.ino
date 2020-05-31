/*
	Name:       PinDemo.ino
	Created:	2020-02-11 오후 7:34:29
	Author:     DESKTOP-RB2A5N8\pmh5050
*/

/** AVR header file을 등록합니다. */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <MsTimer2.h>

/** Arduino Library header file을 등록합니다. */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUG_MOTOR_INDEX false // 디버그를 위해 특정 모터만 동작시킬지 유무를 결정하는 매크로에 해당합니다.
#define DEBUG_BUTTON_TEST false // 디버그를 위해 버튼 입력에 대한 동작을 수행할 지 유무를 결정하는 매크로에 해당합니다.
#define DEBUG_SERIAL false // Serial 통신이 가능할 경우 값을 출력할 지 유무를 결정하는 매크로에 해당합니다.

#if DEBUG_MOTOR_INDEX
int DebugMotorIndex = 0; // 디버그의 대상이 되는 모터의 Index에 해당합니다.
#endif

// Encoder Pin A
/**
  외부 인터럽트 : 4개
  INT2 : E0
  INT3 : E1
  INT4 : E2
  INT5 : E3

  Pin Change 인터럽트 : 6개
** PCINT0_vect
  PCINT0 : E4 -> PCINT4로 조정합니다(SPI 통신과의 중복 방지)
  PCINT1 : E5 -> PCINT5로 조정합니다(SPI 통신과의 중복 방지)
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

  Motor[0:7] V+, V- :
  16개 Adafruit motor driver module로 결선됩니다.
  Motor8
  D4 : M8_V+
  D5 : M8_V-
  Motor9
  D7 : M9_V+
  D8 : M9_V-

  [2개의 모터 추가 대비]
  Motor10
  D9 : M10_V+
  D10 : M10_V-
  Motor11
  D11 : M11_V+
  D12 : M11_V-
*/

// Pin chnage interrupt에서 ISR의 원인이 되는 핀을 검출하기 위한 변수에 해당합니다.
int PreviousPINB; // PCINT0_vect
int PreviousPINJ; // PCINT1_vect
int PreviousPINK; // PCINT2_vect

const int EncAPins[] = { 19, 18, 2, 3 }; // External interrupt pin에 할당된 Encoder A Pin 번호입니다.

// 하단의 매크로를 이용할 경우 각 위치에 해당하는 Bitwise 마스크를 적용합니다.
#define MASK_0_BIT & 0x01
#define MASK_1_BIT & 0x02
#define MASK_2_BIT & 0x04
#define MASK_3_BIT & 0x08
#define MASK_4_BIT & 0x10
#define MASK_5_BIT & 0x20
#define MASK_6_BIT & 0x40
#define MASK_7_BIT & 0x80

#define MOTOR_LENGTH 10 // 제어를 수행하는 전체 모터의 개수에 해당합니다.

// 각 모터의 Position 제어를 위한 PID Controller의 Gain들에 해당하고, 속도를 위해 정수 연산을 수행합니다.
#define MOTOR_P_GAIN 100
#define MOTOR_I_GAIN 1 / 20 // 0.05
#define MOTOR_D_GAIN 1 / 1000 // 0.001

#define MIN_PWM -4095 // 특정 모터에 인가되는 PWM의 하한치 기준(CW 회전)에 해당합니다.
#define MAX_PWM 4095 // 특정 모터에 인가되는 PWM의 상한치 기준(CCW 회전)에 해당합니다.

// MotorPins[POSITIVE_INDEX]에 V+, MotorPins[NEGATIVE_INDEX]에 V- 전압을 인가할 경우 CCW 방향으로 회전합니다.
#define POSITIVE_INDEX 0
#define NEGATIVE_INDEX 1
uint8_t MotorPins[MOTOR_LENGTH][2];

// 각 모터의 Desired Position을 Encoder Pulse 단위로 기록하기 위한 배열에 해당합니다.
int32_t DesiredEncCount[MOTOR_LENGTH] = { 0 };
// 각 모터의 현재 Position을 Encoder Pulse 단위로 기록하기 위한 배열에 해당합니다.
int32_t CurrentEncCount[MOTOR_LENGTH] = { 0 };

// Position Controller의 D Controller를 구현하기 위한 변수에 해당합니다.
int32_t PreErrCount[MOTOR_LENGTH] = { 0 };
int32_t IntErrCount[MOTOR_LENGTH] = { 0 };

// 제어하고자 하는 대상 모터의 Index에 해당합니다.
int MotorIndex = 0;

// Micro Controller Unit에 전원이 인가된 시간으로 부터 경과된 시간을 ms단위로 저장하는 변수에 해당합니다.
uint32_t CurrentTime = 0;

// 8개의 DC 모터에 제어 신호를 송신하기 위한 16ch motor driver에 해당합니다. 해당 모듈에 각 모터는 (MotorIndex, MotorIndex + 1)으로 결선되어 있습니다.
Adafruit_PWMServoDriver AdafruitDriver = Adafruit_PWMServoDriver();

// PORT A : [E0:E3]
// PORT C : [E4:E7]
// PORT F : [E8:E9]

// 첫 번째 버튼이 클릭되었는 지 유무를 저장하기 위한 변수에 해당합니다.
volatile bool bFirstButtonClicked = false;
// 두 번째 버튼이 클릭되었는 지 유무를 저장하기 위한 변수에 해당합니다.
volatile bool bSecondButtonClicked = false;

/** 0번째 모터의 Encoder A Pin에 Rising Edge가 검출되었을 경우 수행되는 ISR에 해당합니다. */
void RisedEncA0()
{
  // A상과 B상이 바뀌어있음
  if (PINA MASK_0_BIT)
  {
    CurrentEncCount[0]--;
  }
  else
  {
    CurrentEncCount[0]++;
  }

}

/** 1번째 모터의 Encoder A Pin에 Rising Edge가 검출되었을 경우 수행되는 ISR에 해당합니다. */
void RisedEncA1()
{
  // A상과 B상이 바뀌어있음
  if (PINA MASK_1_BIT)
  {
    CurrentEncCount[1]--;
  }
  else
  {
    CurrentEncCount[1]++;
  }

}

/** 2번째 모터의 Encoder A Pin에 Rising Edge가 검출되었을 경우 수행되는 ISR에 해당합니다. */
void RisedEncA2()
{
  // A상과 B상이 바뀌어있음
  if (PINA MASK_2_BIT)
  {
    CurrentEncCount[2]--;
  }
  else
  {
    CurrentEncCount[2]++;
  }

}

/** 3번째 모터의 Encoder A Pin에 Rising Edge가 검출되었을 경우 수행되는 ISR에 해당합니다. */
void RisedEncA3()
{
  if (PINA MASK_3_BIT)
  {
    CurrentEncCount[3]--;
  }
  else
  {
    CurrentEncCount[3]++;
  }

}

/** B x (A xor B) = A x (not B)의 연산을 통해 ISR을 발생시킨 Pin을 찾습니다. (Use Karnaugh map)*/
ISR(PCINT0_vect)
{
  if ((PINB MASK_4_BIT) & ~(PreviousPINB MASK_4_BIT)) // PINB4(E4_A)의 Rising Edge 검출 시
  {

    if (PINC MASK_0_BIT)
    {
      CurrentEncCount[4]--;
    }
    else
    {
      CurrentEncCount[4]++;
    }

  }
  else if ((PINB MASK_5_BIT) & ~(PreviousPINB MASK_5_BIT)) // PINB5(E5_A)의 Rising Edge 검출 시
  {

    if (PINC MASK_1_BIT)
    {
      CurrentEncCount[5]--;
    }
    else
    {
      CurrentEncCount[5]++;
    }
  }

  PreviousPINB = PINB;

}

/** B x (A xor B) = A x (not B)의 연산을 통해 ISR을 발생시킨 Pin을 찾습니다. (Use Karnaugh map)*/
ISR(PCINT1_vect)
{
  if ((PINJ MASK_0_BIT) & ~(PreviousPINJ MASK_0_BIT)) // PINJ0(E6_A)의 Rising Edge 검출 시
  {
    if (PINC MASK_2_BIT)
    {
      CurrentEncCount[6]--;
    }
    else
    {
      CurrentEncCount[6]++;
    }

  }
  else if ((PINJ MASK_1_BIT) & ~(PreviousPINJ MASK_1_BIT)) // PINJ1(E7_A)의 Rising Edge 검출 시
  {
    if (PINC MASK_3_BIT)
    {
      CurrentEncCount[7]--;
    }
    else
    {
      CurrentEncCount[7]++;
    }
  }

  PreviousPINJ = PINJ;

}

/** B x (A xor B) = A x (not B)의 연산을 통해 ISR을 발생시킨 Pin을 찾습니다. (Use Karnaugh map)*/
ISR(PCINT2_vect)
{
  if ((PINK MASK_0_BIT) & ~(PreviousPINK MASK_0_BIT)) // PINJ0(E6_A)의 Rising Edge 검출 시
  {
    if (PINF MASK_0_BIT)
    {
      CurrentEncCount[8]--;
    }
    else
    {
      CurrentEncCount[8]++;
    }

  }
  else if ((PINK MASK_1_BIT) & ~(PreviousPINK MASK_1_BIT))
  {
    // PINJ0(E7_A)의 Rising Edge 검출 시
    if (PINF MASK_1_BIT)
    {
      CurrentEncCount[9]--;
    }
    else
    {
      CurrentEncCount[9]++;
    }


  }

  PreviousPINK = PINK;

}

/** Motor의 Index를 입력받아 제어를 수행합니다. */
void MotorPositionController(int SelectedMotorIndex, uint32_t DeltaTime)
{
  int32_t EncError = DesiredEncCount[SelectedMotorIndex] - CurrentEncCount[SelectedMotorIndex];
  IntErrCount[SelectedMotorIndex] += EncError;

  int32_t ControlTerm = EncError * MOTOR_P_GAIN + IntErrCount[SelectedMotorIndex] * MOTOR_I_GAIN * DeltaTime;

  // P : EncError * MOTOR_P_GAIN;
  // PI : EncError * MOTOR_P_GAIN + IntErrCount[SelectedMotorIndex] * MOTOR_I_GAIN * DeltaTime;
  // PID :  EncError * MOTOR_P_GAIN + IntErrCount[SelectedMotorIndex] * MOTOR_I_GAIN * DeltaTime + (EncError - PreErrCount[MOTOR_LENGTH]) * MOTOR_D_GAIN / DeltaTime;

  IntErrCount[SelectedMotorIndex] = constrain(IntErrCount[SelectedMotorIndex], -300, 300); // 적분 제어의 항이 발산하지 못하도록 범위 내로 제한합니다.
  PreErrCount[MOTOR_LENGTH] = EncError;

  ControlTerm = constrain(ControlTerm, MIN_PWM, MAX_PWM); // 제어 항이 상, 하한선을 벗어나지 못하도록 제한합니다.
  SetMotorDriverWithPWM(SelectedMotorIndex, ControlTerm);

}

/** Motor의 Index와 -4095 ~ 4095 사이의 PWM 값을 입력받아 Motor를 설정합니다. */
void SetMotorDriverWithPWM(int MotorIndex, int32_t PWM)
{
  bool bPosDir = (PWM > 0);
  PWM = abs(PWM);

  if (!PWM == 0)
  {
	  static const int Bias = 1000;
	  PWM = constrain(PWM, Bias, MAX_PWM);
  }

  // PWM = constrain(PWM, 2000, MAX_PWM);
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

/** 첫 번째 스위치가 클릭되었을 때 수행되는 ISR에 해당합니다. */
void FirstButtonClicked()
{
  bFirstButtonClicked = true;
}

/** 두 번째 스위치가 클릭되었을 때 수행되는 ISR에 해당합니다. */
void SecondButtonClicked()
{
  bSecondButtonClicked = true;
}

void setup()
{
  PCICR |= 0x07; // Enable PCINT[0:2]_vect
  PCMSK0 = 0x30; // Enable PCINT[0:1] -> PCINT[4:5]
  PCMSK1 = 0x06; // Enable PCINT[9:10]
  PCMSK2 = 0x03; // Enable PCINT[16:17]

  // A, C, F Port의 하위 4비트는 Encoder Pin B
  DDRA = 0x00;
  DDRC = 0x00;
  DDRF = 0x00; // XXXX XX00

  // Pull up resistor setting
  // ENC A
  PORTD = 0x0C;

  DDRE = 0x00;
  PORTE = 0x30; // ENC Pull Up
  PORTE |= 0xC0; // Switch : External Interrupt Pin Pull-up(PE6, PE7)

  PORTB = 0x30;
  PORTJ = 0x03;
  PORTK = 0x03;

  // ENC B
  PORTA = 0x0F;
  PORTC = 0x0F;
  PORTF = 0x03;

  // Switch
  /** attachInterrupt에서 해당 내용을 수행합니다.
    EIMSK |= 0xC0;
    EICRB |= (1 << ISC71) | (1 << ISC61); // Falling Edge
    EICRB &= 0xAF; // Falling Edge
  */

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

  for (MotorIndex = 0; MotorIndex < 4; MotorIndex++)
  {
    pinMode(EncAPins[MotorIndex], INPUT);
    digitalWrite(EncAPins[MotorIndex], HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(EncAPins[0]), RisedEncA0, RISING); // INT2
  attachInterrupt(digitalPinToInterrupt(EncAPins[1]), RisedEncA1, RISING); // INT3
  attachInterrupt(digitalPinToInterrupt(EncAPins[2]), RisedEncA2, RISING); // INT4
  attachInterrupt(digitalPinToInterrupt(EncAPins[3]), RisedEncA3, RISING); // INT5
  attachInterrupt(6, FirstButtonClicked, FALLING); // INT5
  attachInterrupt(7, SecondButtonClicked, FALLING); // INT5

  // Adafruit I2C Motor driver module settings
  AdafruitDriver.begin();
  Wire.setClock(400000); // 'Fast 400kHz I2C' mode를 사용합니다.

#if DEBUG_MOTOR_INDEX
  // 3 * 235 Pulse / 1 Rev
  // 3 * 235 / 360 Pulse = 1'
  // 약 2 Pulse당 1'
  DesiredEncCount[DebugMotorIndex] = 2 * 360;
#else
  // 각 모터별로 Desired Encoder Count 값을 지정하는 단계입니다.
  /*
  for (MotorIndex = 7; MotorIndex < MOTOR_LENGTH; MotorIndex++)
  {
  DesiredEncCount[MotorIndex] = 1000; // -1000; // -1000 * (MotorIndex + 1);
  }
  */
  // DesiredEncCount[7] = 2 * 2 * 360; // 2바퀴
  // DesiredEncCount[9] = 2 * 3 * 360; // 3바퀴
  // DesiredEncCount[8] = 2 * 30;
#endif

#if DEBUG_SERIAL
  Serial.begin(9600);
#endif

  /*
    MsTimer2::set(2000, ToggleDesiredEncCount);
    MsTimer2::start();
  */

}

void ToggleDesiredEncCount()
{
  static bool bActive = true;
  if (bActive)
  {
    // 각 모터별로 Desired Encoder Count 값을 지정하는 단계입니다.
    for (MotorIndex = 0; MotorIndex < 5; MotorIndex++)
    {
      DesiredEncCount[MotorIndex] = -200;
    }
    for (MotorIndex = 5; MotorIndex < MOTOR_LENGTH; MotorIndex++)
    {
      DesiredEncCount[MotorIndex] = 200;
    }
    bActive = false;
  }
  else
  {
    // 각 모터별로 Desired Encoder Count 값을 지정하는 단계입니다.
    for (MotorIndex = 0; MotorIndex < MOTOR_LENGTH; MotorIndex++)
    {
      DesiredEncCount[MotorIndex] = 0;
    }
    bActive = true;
  }
}

/** 현재 Encoder의 누적된 Count 값을 출력합니다. */
void PrintEncCount()
{
#if DEBUG_SERIAL
  for (MotorIndex = 0; MotorIndex < MOTOR_LENGTH - 1; MotorIndex++)
  {
    Serial.print(CurrentEncCount[MotorIndex]);
    Serial.print(" ");
  }
  Serial.println(CurrentEncCount[MOTOR_LENGTH - 1]);
#endif
}

void loop()
{
  uint32_t DeltaTime = millis() - CurrentTime;
  CurrentTime = millis();

#if DEBUG_MOTOR_INDEX
  MotorPositionController(DebugMotorIndex, DeltaTime + 1);
#else
  for (MotorIndex = 0; MotorIndex < 10; MotorIndex++) // 5번
  {
    MotorPositionController(MotorIndex, DeltaTime + 1);
  }
#endif

#if DEBUG_BUTTON_TEST
  if (bFirstButtonClicked)
  {
    for (MotorIndex = 0; MotorIndex < 5; MotorIndex++) // 5번
    {
      MotorPositionController(MotorIndex, DeltaTime + 1);
    }
  }
  if (bSecondButtonClicked)
  {
    for (MotorIndex = 5; MotorIndex < 10; MotorIndex++) // 5번
    {
      MotorPositionController(MotorIndex, DeltaTime + 1);
    }
  }
#endif

}
