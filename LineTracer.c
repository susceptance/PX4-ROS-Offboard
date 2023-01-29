#include <mega2560.h>
#include <delay.h>


#define LEFT_MD_A PORTA.0
#define LEFT_MD_B PORTA.1
#define RIGHT_MD_A PORTA.2
#define RIGHT_MD_B PORTA.3

#define L_MOTOR_EN PORTG.5
#define R_MOTOR_EN PORTE.3

void DAC_CH_Write(unsigned int, unsigned int);
void DAC_setting(unsigned int);



void DAC_CH_Write(unsigned int ch1, unsigned int da)
{
	unsigned int data = ((ch1 << 12) & 0x7000) | ((da << 4) & 0x0FF0);
	DAC_setting(data);
}


void DAC_setting(unsigned int data)
{
	unsigned char S_DIN = 0;
	int i = 0;

	// SCLK 클럭을 1개 발생시키고 SYNC'를 0인 상태로 만들어 DIN에 있는 데이터가 입력될 수 있도록 설정
	PORTL = PORTL | 0x40; // S_SCLK = 1;
	delay_us(1);
	PORTL = PORTL & 0xBF; // S_SCLK = 0;
	delay_us(1);
	PORTL = PORTL & 0xDF; // S_SYNCN = 0 (SYNC' = 0)
	delay_us(1);

	// DIN에 입력된 데이터를 16비트 시프트 레지스터에 데이터 입력
	for (i = 16; i > 0; i--)
	{
		S_DIN = (data >> (i - 1) & 0x01);
		// 16비트 명령을 MSB부터 LSB까지 차례대로 이동시켜 1인지 0인지 판별하기 위해 S_DIN에 저장

		// 데이터를 DIN에 준비
		if (S_DIN == 1)
		{
			PORTL = PORTL | 0x80; // DIN = 1 (PL7 = 1)
		}
		else if (S_DIN == 0)
		{
			PORTL = PORTL & 0x7F; // DIN = 0 (PL7 = 0)
		}

		// SCLK를 1에서 0으로 (하강에지)로 인가하여 DIN에 있는 데이터를 DAC에 넣을 수 있도록 한다. (16번 반복)
		PORTL = PORTL | 0x40; // S_SCLK = 1;
		delay_us(1);
		PORTL = PORTL & 0xBF; // S_SCLK = 0;
		delay_us(1);
	}

	PORTL = PORTL | 0x20; // S_SYNCN = 1, 데이터 전송이 끝났으므로 다음 데이터 전송 준비
}

void main(void)
{
	int i = 0;
	unsigned char IR = 0;

	DDRA = 0x1f; //  PA4 (포토 다이오드(발광부) 연결 핀을 출력으로 설정하기 위해 SEN_EN = 1), LEFT_MD_A, RIGHT_MD_A 출력 설정 (PA0~4)
	DDRC = 0x00; // Digital Input (OP AMP로부터 입력 받는다.)
	DDRG = 0x20; // PG5 (L_MOTOR_EN) 출력 설정
	DDRE = 0x08; // PE3 (R_MOTOR_EN) 출력 설정

	// 초기 동기 신호를 설정
	DDRL = 0xE0; // DAC에서 S_DIN : PL7 (Output), S_SCLK : PL6 (Output), S_SYNCN PL5 (Output)
	PORTL = PORTL & 0xBF; // S_SCLK = 0 (PL6만 0으로 설정)
	PORTL = PORTL | 0x20; // S_SYNCN = 1 (PL5만 1로 설정)

	DAC_setting(0x9000); // WTM모드 : 채널의 레지스터에 데이터를 저장하여 DAC 출력을 변경

	// 채널 000 ~ 111, 아날로그로 변환활 디지털 값 166을 DAC에 입력
	for(i = 0; i < 8; i++)
	{
		DAC_CH_Write(i, 59);
	}

	PORTA = 0x10; // 적외선 발광 다이오드를 ON 해준다.

	TCCR1A = 0x81; // Phase Correct PWM(8비트), 업 카운트 중의 비교매치에서 OCnX=0로 클리어, 다운 카운트 중의 비교매치에서 OCnx=1 셋
	TCCR1B = 0x05; // 시스템 클럭 분주 1024

	TCNT1L = 0;
	TCNT1H = 0;

	OCR1AH = 0;
	OCR1AL = 0;

	TCCR3A = 0x81; // Phase Correct PWM(8비트), 업 카운트 중의 비교매치에서 OCnX=0로 클리어, 다운 카운트 중의 비교매치에서 OCnx=1 셋
	TCCR3B = 0x05; // 시스템 클럭 분주 1024

	TCNT3L = 0;
	TCNT3H = 0;

	OCR3AH = 0;
	OCR3AL = 0;


	while(1)
	{
		// 적외선 센서의 값을 디지털로 읽어 온다.
		IR = PINC; // 흰색이 '1', 검정색이 '0'으로 읽힌다.
		
		
		if(IR == 0xEF || IR == 0xE7 || IR == 0xF7)
		{
			OCR1AL = 100; // 왼쪽 모터 PWM 값
			OCR3AL = 100; // 오른쪽 모터 PWM 값
			
			// 왼쪽 모터 전진방향 회전
			LEFT_MD_A = 1;
			LEFT_MD_B = 0;

			// 오른쪽 모터 전진방향 회전
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 1;
		}
		else if(IR == 0x9F || IR == 0x3f || IR == 0x7f || IR == 0xcf || IR == 0xdf || IR == 0xbf)
		{
			OCR1AL = 220; // 왼쪽 모터 PWM 값
			
			// 오른쪽 모터 정지
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 0;
			
			// 우회전
			// 왼쪽모터 전진, 오른쪽 모터 정지
			LEFT_MD_A = 1;
			LEFT_MD_B = 0;
			L_MOTOR_EN = 1; // 왼쪽 모터 Enable
		}
		else if(IR == 0xf9 || IR == 0xfc || IR == 0xfe || IR == 0xf3 || IR == 0xfB || IR == 0xfd)
		{
			OCR3AL = 220; // 오른쪽 모터 PWM 값
			
			// 왼쪽 모터 정지
			LEFT_MD_A = 0;
			LEFT_MD_B = 0;
			
			// 좌회전 (1초)
			// 왼쪽모터 정지, 오른쪽 모터 전진방향으로 회전
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 1;
			R_MOTOR_EN = 1; // 오른쪽 모터 Enable
		}
	}
}
