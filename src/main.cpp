#include <Arduino.h>
#include <myQueue.hpp>
#include <AccelStepper.h>
#include <TaskScheduler.h>
#include <Servo.h>

#define STEPPERS_NUM 3

// Arduino 핀 할당 (예: STEP핀 = 2,4,6, DIR핀 = 3,5,7)
const int STEP_PIN[STEPPERS_NUM] = {2, 4, 6};
const int DIR_PIN[STEPPERS_NUM] = {3, 5, 7};
const int VACUUM_PIN = 15; // 진공 펌프 핀
const int ENDSTOP_PIN1 = 20;
const int ENDSTOP_PIN2 = 21;
const int ENDSTOP_PIN3 = 22;

const int SERVO_PIN = 16; // 서보 핀
Servo myServo;			  // 서보 객체 생성

// AccelStepper 객체 배열 생성: (인터페이스 유형, 스텝핀, 방향핀)
AccelStepper stepper[STEPPERS_NUM] = {
	AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
	AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
	AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])};

typedef struct COMMAND_DATA
{
	char command;
	long data[3];

	COMMAND_DATA() {}

	COMMAND_DATA(char c)
	{
		command = c;
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
	}

	COMMAND_DATA(char c, long on_off)
	{
		command = c;
		data[0] = on_off;
		data[1] = 0;
		data[2] = 0;
	}

	COMMAND_DATA(char c, long p0, long p1, long p2)
	{
		command = c;
		data[0] = p0;
		data[1] = p1;
		data[2] = p2;
	}

} COMMAND_DATA;
my_Queue<COMMAND_DATA> command_queue;

void GetCommand();			   // 명령어 수신 함수 선언
String command;				   // 명령어 저장 배열
volatile int command_flag = 0; // 명령어 수신 플래그

void HandleCommand(); // 명령어 처리 함수 선언

void run();

///////////////////////////// 속도 동기화
void SpeedSynchronization(long[]);
#define MAX_SPPED 20000.0
#define MAX_ACCELEARTION 10000.0
////////////////////////////////

////////////////////////인터럽트
const int VACCUM_INTERRUPT_PIN = 18;
void VaccumInterrupt();
///////////////

Scheduler scheduler;												  // 스케줄러 객체 생성
Task tGetCommand(10, TASK_FOREVER, GetCommand, &scheduler, false);	  // 태스크 객체 생성
Task tHandleCommand(10, TASK_ONCE, HandleCommand, &scheduler, false); // 태스크 객체 생성
Task tRun(10, TASK_FOREVER, run, &scheduler, false);				  // 태스크 객체 생성

void setup()
{
	Serial.begin(115200);

	pinMode(VACUUM_PIN, OUTPUT);   // 진공 펌프 핀 설정
	digitalWrite(VACUUM_PIN, LOW); // 진공 펌프 OFF

	pinMode(ENDSTOP_PIN1, INPUT_PULLUP);
	pinMode(ENDSTOP_PIN2, INPUT_PULLUP);
	pinMode(ENDSTOP_PIN3, INPUT_PULLUP);

	myServo.attach(SERVO_PIN); // 서보 핀 설정
	myServo.write(110);		   // 서보 초기화

	for (int i = 0; i < STEPPERS_NUM; i++)
	{
		stepper[i].setMaxSpeed(20000.0);	 // 1초에 1000스텝(약 1/200도 × 1000 = 5도/초)
		stepper[i].setAcceleration(10000.0); // 가속도
	}

	tGetCommand.restartDelayed(0); // 태스크 활성화
	tRun.restartDelayed(0);		   // 태스크 활성화

	command_queue.clear();				   // 큐 초기화
	command_queue.push(COMMAND_DATA('R')); // 초기 위치 보정 명령어 추가

	// pinMode(VACCUM_INTERRUPT_PIN, INPUT_PULLUP); // 인터럽트
	pinMode(VACCUM_INTERRUPT_PIN, INPUT); // 인터럽트 핀 설정
	// attachInterrupt(digitalPinToInterrupt(VACCUM_INTERRUPT_PIN), VaccumInterrupt, FALLING);
	// detachInterrupt(digitalPinToInterrupt(VACCUM_INTERRUPT_PIN));
}

void loop()
{
	scheduler.execute(); // 스케줄러 실행

	stepper[0].run(); // 스텝퍼 실행
	stepper[1].run();
	stepper[2].run();
}

void GetCommand()
{
	if (Serial.available() && command_flag == 0)
	{
		command = Serial.readStringUntil('\n'); // 명령어 수신
		command.trim();							// 공백 제거
		command.toUpperCase();					// 대문자로 변환
		// Serial.println(command);				 // 수신된 명령어 출력
		if (command[0] == 'M' || command[0] == 'R' || command[0] == 'G')
		{									  // 명령어가 M, R, G일 때
			tHandleCommand.restartDelayed(0); // 태스크 재시작
			command_flag = 1;				  // 명령어 수신 플래그 설정
		}
		else
		{
			Serial.println("Invalid Command");
		}
	}
}

void HandleCommand()
{
	if (command[0] == 'M')
	{
		long position[3];

		int idx_st = 2;
		int idx_ed = idx_st;

		for (int i = 0; i < 3; i++)
		{
			while ((command[idx_ed] >= '0' && command[idx_ed] <= '9') || command[idx_ed] == '-' || command[idx_ed] == '+')
				idx_ed++;
			position[i] = command.substring(idx_st, idx_ed).toInt(); // 명령어에서 위치값 추출
			idx_st = ++idx_ed;
		}

		command_queue.push(COMMAND_DATA(command[0], position[0], position[1], position[2]));
		Serial.print("Received Command: ");
		Serial.print(command[0]);
		Serial.print(" ");
		Serial.print(position[0]);
		Serial.print(" ");
		Serial.print(position[1]);
		Serial.print(" ");
		Serial.println(position[2]);
	}
	else if (command[0] == 'G')
	{
		if (command[1] == '1')
		{
			command_queue.push(COMMAND_DATA(command[0], 1));
		}
		else if (command[1] == '0')
		{
			command_queue.push(COMMAND_DATA(command[0], 0));
		}
	}
	else if (command[0] == 'R')
	{
		command_queue.push(COMMAND_DATA(command[0]));
	}

	command_flag = 0; // 명령어 수신 플래그 초기화
}

void run()
{
	if (command_queue.is_empty())
		return; // 큐가 비어있으면 리턴

	if (stepper[0].distanceToGo() || stepper[1].distanceToGo() || stepper[2].distanceToGo())
		return; // 스텝퍼가 이동 중이면 리턴

	COMMAND_DATA command_data = command_queue.front();
	command_queue.pop();

	if (command_data.command == 'M')
	{
		SpeedSynchronization(command_data.data); // 속도 동기화
		for (int i = 0; i < STEPPERS_NUM; i++)
		{
			stepper[i].moveTo(command_data.data[i]);
		}
	}
	else if (command_data.command == 'R')
	{
		// 초기 위치 보정, 이 작업 할 땐 다른 건 다 멈춰도 되나? 일단 멈춰도 되는 걸로 간주
		stepper[0].setSpeed(800);
		stepper[1].setSpeed(800);
		stepper[2].setSpeed(800);
		while (1)
		{
			Serial.print("ADC: ");
			Serial.print(analogRead(26));
			Serial.print("\tEndstop1: ");
			Serial.print(digitalRead(ENDSTOP_PIN1));
			Serial.print(" Endstop2: ");
			Serial.print(digitalRead(ENDSTOP_PIN2));
			Serial.print(" Endstop3: ");
			Serial.println(digitalRead(ENDSTOP_PIN3));

			if (digitalRead(ENDSTOP_PIN1))
				stepper[0].runSpeed();
			if (digitalRead(ENDSTOP_PIN2))
				stepper[1].runSpeed();
			if (digitalRead(ENDSTOP_PIN3))
				stepper[2].runSpeed();
			if (!digitalRead(ENDSTOP_PIN1) && !digitalRead(ENDSTOP_PIN2) && !digitalRead(ENDSTOP_PIN3))
				break;
		}
		stepper[0].setCurrentPosition(-600); // 초기 위치 보정
		stepper[0].moveTo(0);
		stepper[1].setCurrentPosition(-600);
		stepper[1].moveTo(0);
		stepper[2].setCurrentPosition(-600);
		stepper[2].moveTo(0);
		// stepper[0].setCurrentPosition(0);
		// stepper[1].setCurrentPosition(0);
		// stepper[2].setCurrentPosition(0);
	}
	else if (command_data.command == 'G')
	{
		if (command_data.data[0] == 1)
		{
			digitalWrite(VACUUM_PIN, HIGH); // 진공 펌프 ON
			myServo.write(90);				// 서보 90도 회전. 밸브 닫힘
			Serial.println("Vacuum Pump ON");
			delay(10);
			attachInterrupt(digitalPinToInterrupt(VACCUM_INTERRUPT_PIN), VaccumInterrupt, FALLING);
		}
		else if (command_data.data[0] == 0)
		{
			detachInterrupt(digitalPinToInterrupt(VACCUM_INTERRUPT_PIN));
			delayMicroseconds(100);
			digitalWrite(VACUUM_PIN, LOW); // 진공 펌프 OFF
			myServo.write(110);			   // 서보 110도 회전, 밸브 열림
			Serial.println("Vacuum Pump OFF");
		}
	}
}

void SpeedSynchronization(long position[])
{

	long max = 0;
	long distance[3] = {0};
	for (int i = 0; i < 3; i++)
	{
		distance[i] = stepper[i].currentPosition() - position[i]; // 이동거리 계산
		if (distance[i] < 0)
			distance[i] *= -1; // 절대값
		if (max < distance[i])
			max = distance[i];
	}

	// if(max == 0) return; //모두 이동이 필요없는 경우....

	for (int i = 0; i < 3; i++)
	{
		if (distance[i] > 0)
		{ //
			stepper[i].setMaxSpeed(MAX_SPPED * (float)distance[i] / (float)max);
			stepper[i].setAcceleration(MAX_ACCELEARTION * (float)distance[i] / (float)max);
		}
	}
}

void VaccumInterrupt()
{
	for (int i = 0; i < STEPPERS_NUM; i++)
	{
		stepper[i].setSpeed(0);
		stepper[i].moveTo(stepper[i].currentPosition()); // 현재 위치를 목표 포지션으로 설정
	}

	command_queue.push_front(COMMAND_DATA('M', stepper[0].currentPosition() + 200,
										  stepper[1].currentPosition() + 200, stepper[2].currentPosition() + 200)); // 목표 포지션 설정
}