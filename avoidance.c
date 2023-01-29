#include <mega2560.h>
#include <delay.h>
#include <string.h>

void Init_USART(void);
void Serial_Send0(unsigned char);
void Serial_Send1(unsigned char);

unsigned char buf[17];

// 전체 초음파 측정 데이터를 Tx_buf1[5]에 배열로 저장
unsigned char Tx_buf1[5] = {0x76, 0x00, 0xF0, 0x00, 0xF0}; // 전후방 : 0x76, 0x00, 0x10, 0x00, 0x10 // 전체 : 0x76, 0x00, 0xF0, 0x00, 0xF0
unsigned char ch[7];


#define LEFT_MD_A PORTA.0
#define LEFT_MD_B PORTA.1
#define RIGHT_MD_A PORTA.2
#define RIGHT_MD_B PORTA.3

#define L_MOTOR_EN PORTG.5
#define R_MOTOR_EN PORTE.3

void Init_USART(void)
{
	// 시리얼 포트 1은 초음파 센서 모듈과의 통신 포트이다.
	DDRD = 0x08;
	UCSR1A = 0x00;
	UCSR1B = 0x18; // TXE, RXE, Enable
	UCSR1C = 0x06; // 비동기, non parity, 1 stop bit
	UBRR1H = 0x00;
	UBRR1L = 0x08; // 115200bps
	DDRB = 0xff;
}



// 시리얼 포트 1에 1 Byte씩 전송하는 함수 (초음파 모듈로 송신하는 함수)
void Serial_Send1(unsigned char t)
{
	while(1) // 전송 준비가 될 때까지 대기
	{
		if((UCSR1A & 0x20) != 0) break;
	}
	
	UDR1 = t;
	UCSR1A = UCSR1A | 0x20;
}


// 시리얼 포트 1에서 데이터를 수신하는 함수 (초음파 모듈 데이터 수신 함수)
unsigned char Serial_Rece1(void)
{
	unsigned char data;
	
	while(1) // 데이터를 받을 때까지 대기
	{
		if((UCSR1A & 0x80) != 0) break;
	}
	
	data = UDR1;
	UCSR1A |= 0x80;
	return data;
}



void main(void)
{
	int i = 0;
	
	Init_USART(); // 시리얼 포트 0, 1 초기화

	DDRH = 0x40;

	DDRA = 0x0f;
	DDRG = 0x20;
	DDRE = 0x08;
	
	PORTA = 0;
	PORTG = 0;
	PORTE = 0;
	
	TCCR0A = 0x21; // 업 카운트 중의 비교매치에서 OCnB=0로 클리어, 다운 카운트 중의 비교매치에서 OCnB=1 셋, Phase Correct PWM 모드
	TCCR0B = 0x05; // 시스템 클럭 분주 1024

	TCNT0 = 0;
	OCR0B = 0;

	TCCR3A = 0x81; // 업 카운트 중의 비교매치에서 OCnX=0로 클리어, 다운 카운트 중의 비교매치에서 OCnx=1 셋, Phase Correct PWM(8비트)모드
	TCCR3B = 0x05; // 시스템 클럭 분주 1024

	TCNT3L = 0;
	TCNT3H = 0;

	OCR3AH = 0;
	OCR3AL = 0;
	
	OCR0B = 0; // 왼쪽 모터 PWM 초기값
	OCR3AL = 0; // 오른쪽 모터 PWM 초기값
	
	
	for(i = 0; i < 5; i++)
	{
		Serial_Send1(Tx_buf1[i]); // 전체 초음파 측정 데이터를 요청
	}
	
	while(1)
	{
		// 초음파 모듈에서 17바이트의 데이터를 받는다.
		for(i = 0; i < 17; i++)
		{
			buf[i] = Serial_Rece1();
		}
		if(buf[12] <= 0x14 || buf[14] <= 0x14)
		{
			PORTB = 0x10;
			PORTH = 0x40;
			
			OCR0B = 50; // 왼쪽 모터 PWM 초기값
			OCR3AL = 50; // 오른쪽 모터 PWM 초기값
			
			/// 왼쪽 모터 전진방향 회전
			LEFT_MD_A = 1;
			LEFT_MD_B = 0;
			L_MOTOR_EN = 1; // 왼쪽 모터 Enable
			
			// 오른쪽 모터 전진방향 회전
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 1;
			R_MOTOR_EN = 1; // 오른쪽 모터 Enable
		}
		else if(buf[12] > 0x14 || buf[14] > 0x14)
		{
			PORTB = 0x00;
			PORTH = 0x00;
			
			// 왼쪽 모터 정지
			LEFT_MD_A = 0;
			LEFT_MD_B = 0;

			// 오른쪽 모터 정지
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 0;
			
		}
		
		if(buf[5] <= 0x14 && buf[7] <= 0x14)
		{
			OCR0B = 0; // 왼쪽 모터 PWM 초기값
			OCR3AL = 0; // 오른쪽 모터 PWM 초기값
			
			// 왼쪽 모터 정지
			LEFT_MD_A = 0;
			LEFT_MD_B = 0;

			// 오른쪽 모터 정지
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 0;
			
			
			while(1)
			{
				for(i = 0; i < 17; i++)
				{
					buf[i] = Serial_Rece1();
				}
				
				OCR0B = 200; // 왼쪽 모터 PWM 초기값
				OCR3AL = 200; // 오른쪽 모터 PWM 초기값
				
				// 오른쪽 유턴
				// 왼쪽 모터 전진방향 회전
				LEFT_MD_A = 1;
				LEFT_MD_B = 0;

				// 오른쪽 모터 후진방향 회전
				RIGHT_MD_A = 1;
				RIGHT_MD_B = 0;
				
				if(buf[5] > 0x14)
				{
					// 왼쪽 모터 정지
					LEFT_MD_A = 0;
					LEFT_MD_B = 0;

					// 오른쪽 모터 정지
					RIGHT_MD_A = 0;
					RIGHT_MD_B = 0;
					

					
					break;
				}
			}
		}
		
		if(buf[9] <= 0x14 && buf[7] <= 0x14)
		{
			OCR0B = 0; // 왼쪽 모터 PWM 초기값
			OCR3AL = 0; // 오른쪽 모터 PWM 초기값
			
			// 왼쪽 모터 정지
			LEFT_MD_A = 0;
			LEFT_MD_B = 0;

			// 오른쪽 모터 정지
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 0;
			
			
			while(1)
			{
				for(i = 0; i < 17; i++)
				{
					buf[i] = Serial_Rece1();
				}
				
				OCR0B = 200; // 왼쪽 모터 PWM 초기값
				OCR3AL = 200; // 오른쪽 모터 PWM 초기값
				
				// 왼쪽 유턴
				// 왼쪽 모터 후진 방향으로 회전
				LEFT_MD_A = 0;
				LEFT_MD_B = 1;

				// 오른쪽 모터 전진방향 회전
				RIGHT_MD_A = 0;
				RIGHT_MD_B = 1;
				
				if(buf[9] > 0x14)
				{
					// 왼쪽 모터 정지
					LEFT_MD_A = 0;
					LEFT_MD_B = 0;

					// 오른쪽 모터 정지
					RIGHT_MD_A = 0;
					RIGHT_MD_B = 0;
					
					
					break;
				}
			}
		}
		
		if(buf[5] <= 0x19 || buf[6] <= 0x19)
		{
			OCR0B = 255; // 왼쪽 모터 PWM 초기값
			OCR3AL = 255; // 오른쪽 모터 PWM 초기값
			
			// 오른쪽 모터 정지
			RIGHT_MD_A = 0;
			RIGHT_MD_B = 0;
			
			// 우회전
			// 왼쪽모터 전진, 오른쪽 모터 정지
			LEFT_MD_A = 1;
			LEFT_MD_B = 0;
			L_MOTOR_EN = 1; // 왼쪽 모터 Enable
		}

		if(buf[8] <= 0x19 || buf[9] <= 0x19)
		{
			OCR0B = 255; // 왼쪽 모터 PWM 초기값
			OCR3AL = 255; // 오른쪽 모터 PWM 초기값
			
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