/*
 * Manipulator2.c
 *
 * Created: 2020-11-13 오후 5:01:00
 * Author : 이재원
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>

void UART0_INIT()
{
	DDRE = (1<<DDE1); //PE1 TX 설정

	UCSR0A = 0x00;
	UCSR0B = (1<<TXEN0) | (0<<UCSZ02); //Transmit Enable
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); //011 -> 8bit

	UBRR0H = 0;
	UBRR0L = 8;  // Baud Rate 11520
}

void UART0_Transmit(unsigned char Pdata)
{
	while(!(UCSR0A & (1<<UDRE0)));

	UDR0 = Pdata;
}

void Dynamixel_ID_Set(int ID, int Target_ID){

	char Check_Sum = ~ ( ID + 0x04 + 0x03 + 0x03 + Target_ID );

	UART0_Transmit(0xff);//통신시작
	UART0_Transmit(0xff);
	UART0_Transmit(ID);//ID

	UART0_Transmit(0x04);//페킷길이
	UART0_Transmit(0x03);
	UART0_Transmit(0x03);//데이터 쓰기
	UART0_Transmit(Target_ID);//ID변경
	UART0_Transmit(Check_Sum);
}

void Dynamixel_Rate_Set(int ID, int Target_Rate){

	char Check_Sum = ~ ( ID + 0x04 + 0x03 + 0x04 + Target_Rate );

	UART0_Transmit(0xff); UART0_Transmit(0xff);//통신시작
	UART0_Transmit(ID);//ID

	UART0_Transmit(0x04);//페킷길이
	UART0_Transmit(0x03);
	UART0_Transmit(0x04);//데이터 쓰기
	UART0_Transmit(Target_Rate);//통신속도 변경
	UART0_Transmit(Check_Sum);
}

void Dynamixel_Moving(int ID, int speed, int Position)
{
	UART0_Transmit(0xFF); UART0_Transmit(0xFF); //HEADER
	UART0_Transmit(ID); //ID
	UART0_Transmit(0x07); //LENGTH = Parameters(n) + 2
	UART0_Transmit(0x03); //WRITE DATA

	UART0_Transmit(0x1E);//SET_goal position
	UART0_Transmit(Position & 0xFF); UART0_Transmit((Position>>8) & 0xFF);
	UART0_Transmit(speed & 0xFF); UART0_Transmit((speed>>8) & 0xFF);
	

	char checkSum = ~(ID + 0x07 + 0x03 + 0x1E
	+ (speed & 0xFF) + ((speed>>8) & 0xFF)
	+ (Position & 0xFF) + ((Position>>8) & 0xFF));

	UART0_Transmit(checkSum);
}

void Dynamixel_LED(int ID, int ONOFF)
{
	char Check_Sum = ~ ( ID + 0x04 + 0x03 + 0x19 + ONOFF );

	UART0_Transmit(0xff);//통신시작
	UART0_Transmit(0xff);
	UART0_Transmit(ID);//ID

	UART0_Transmit(0x04);//페킷길이
	UART0_Transmit(0x03);
	UART0_Transmit(0x19);//데이터 쓰기
	UART0_Transmit(ONOFF);//LED
	UART0_Transmit(Check_Sum);

}

void UART1_INIT()
{
	DDRD = (1<<DDD3);
	
	UCSR1A = 0x00;
	UCSR1B = (1 << RXCIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12);
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10); //011 -> 8bit
	
	UBRR1H = 0;
	UBRR1L = 8;  // Baud Rate 11520
}

void UART1_Transmit(unsigned char cData)
{
	while (!(UCSR1A & (1<<UDRE1)));
	
	UDR1 = cData;
}

unsigned char UART1_Receive()
{
	while (!(UCSR1A & (1<<RXC1)));
	
	return UDR1;
}

void degree_manipulator(int x, int y, double* pd1, double* pd2)
{
	double l1, l2;
	l1 = 19.6; //고정 팔길이
	l2 = 11.65; //고정 팔길이

	double d1, d2; //각도1 각도2
	double cd2, sd2, cd1, sd1;
	//x, y 좌표 범위 13.6< <19.714 내 입력

	cd2 = (pow(x,2) + pow(y,2) - l1 * l1 - l2 * l2) / (2 * l1 * l2);
	sd2 = sqrt(1 - (cd2 * cd2));
	d2 = atan2(sd2, cd2);

	cd1 = ((l1 + l2 * cd2) * x + (l2 * sd2) * y) / ((l1 + l2 * cd2) * (l1 + l2 * cd2) + (l2 * sd2) * (l2 * sd2));
	sd1 = ((-1 * l2 * sd2) * x + (l1 + l2 * cd2) * y) / (((l1 + l2 * cd2) * (l1 + l2 * cd2)) + ((l2 * sd2) * (l2 * sd2)));

	d1 = atan2(sd1, cd1);
	
	d1 = (57.3) * d1; //라디안
	d2 = (57.3) * d2;

	*pd1 = d1;
	*pd2 = d2;
}

void minus_degree_manipulator(int x, int y, double* pd1, double* pd2)
{
	double l1, l2;
	l1 = 19.6; //고정 팔길이
	l2 = 11.65; //고정 팔길이

	double d1, d2; //각도1 각도2
	double cd2, sd2, cd1, sd1;
	//x, y 좌표 범위 13.6< <19.714 내 입력

	cd2 = (pow(x, 2) + pow(y, 2) - l1 * l1 - l2 * l2) / (2 * l1 * l2);
	sd2 = (-1) * sqrt(1 - (cd2 * cd2));
	d2 = atan2(sd2, cd2);

	cd1 = ((l1 + l2 * cd2) * x + (l2 * sd2) * y) / ((l1 + l2 * cd2) * (l1 + l2 * cd2) + (l2 * sd2) * (l2 * sd2));
	sd1 = ((-1 * l2 * sd2) * x + (l1 + l2 * cd2) * y) / (((l1 + l2 * cd2) * (l1 + l2 * cd2)) + ((l2 * sd2) * (l2 * sd2)));

	//d1 = atan2(y, x) - atan2((l1 + l2 * cd2), l2 * sd2);
	d1 = atan2(sd1, cd1);

	d1 = (57.3) * d1; //라디안
	d2 = (57.3) * d2;

	*pd1 = d1;
	*pd2 = d2;
}

void UART1_TransNum(int num)
{
	if(num < 0){
		UART1_Transmit('-');
		UART1_Transmit('-');
		num = -num;
	}
	UART1_Transmit(((num % 1000) / 100) + 48);		// 백의 자리 전송
	UART1_Transmit(((num % 100) / 10) + 48);		// 십의 자리 전송
	UART1_Transmit((num % 10) + 48);				// 일의 자리 전송
	UART1_Transmit(13);								//ASCII enter
}

void DC_MotorSet()
{
	DDRB = 0xFF; //B포트 출력설정(모터드라이브) -> 0b11111111
	DDRE = 0x0F; //E포트 출력(모터드라이브) -> 0b00001111

	TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10);
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(0<<CS02)|(0<<CS01)|(1<<CS00);
}
//Timer/Counter 0,1번
void TC0Set()
{

	TCCR0 = (0<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(0<<CS00); //(WGM01,WGM00)->0,0(Normal Mode) /(COM01,COM00)->0,0(Normal Mode)(WGM01,WGM00)->0,0(Normal Mode) ,CS는 256분주비 설정
	TIMSK = (1<<TOIE0)|(1<<TOIE1); //EIMSK처럼 Interrupt Enable	
									//(WGM01,WGM00)->0,0(Normal Mode) ,CS는 256분주비 설정
	TCNT0 = 131; //131로 TCNT설정 //T(desired) = 2ms
}

int i = 0, j = 0;
int start_i = 0;
int End_i = 0;
int cut_i = 0;
int cut2_i = 0;
int cut3_i = 0;

#define RX_BUFFER_SIZE 64

char RXbuffer[RX_BUFFER_SIZE];
char num1[20];
char num2[20];
char num3[20];
char num4[20];

int c1 = 0, c2 = 0, c3 = 0, c4 = 0; //coordinate 1,2 //전역변수

ISR(USART1_RX_vect) //UART1 interrupt
{

	RXbuffer[i] = UART1_Receive();

	if(RXbuffer[i] == 'S') //시작 알림
	start_i = i;

	else if(RXbuffer[i] == ',') //x, y 좌표 구분
	cut_i = i;

	else if(RXbuffer[i] == '/')
	cut2_i = i;

	else if(RXbuffer[i] == '-')
	cut3_i = i;

	else if(RXbuffer[i] == 'E') //끝 알림
	{
		End_i = i;
		RXbuffer[i+1] = 0;

		for(i = start_i+1; i < cut_i; i++)
		num1[j++] = RXbuffer[i];
		
		num1[j] = 0;
		j = 0;

		for(i = cut_i+1; i < cut2_i; i++)
		num2[j++] = RXbuffer[i];

		num2[j] = 0;
		j = 0;

		for(i = cut2_i+1; i < cut3_i; i++)
		num3[j++] = RXbuffer[i];

		num3[j] = 0;
		j = 0;

		for(i = cut3_i+1; i<End_i; i++)
		num4[j++] = RXbuffer[i];

		num4[j] = 0;
		j = 0;

		int real_num1 = 0;
		int real_num2 = 0;
		int real_num3 = 0;
		int real_num4 = 0;

		real_num1 = atoi(num1);
		real_num2 = atoi(num2);
		real_num3 = atoi(num3);
		real_num4 = atoi(num4);

		c1 = real_num1; //전역변수로 전달
		c2 = real_num2;
		c3 = real_num3;
		c4 = real_num4;
	}
	i++;
}

int flag = 0; //전역변수

ISR(INT0_vect) //0번 스위치
{
	flag = 1;
	//PORTA = 0xf0;
}
ISR(INT1_vect) //1번 스위치
{
	flag = 1;
	//PORTA = 0x0f;
}

void IntSet() //Switch Resister 세팅
{
	EICRA =	(1<<ISC01)|(0<<ISC00)|(1<<ISC11)|(0<<ISC10); //falling Edge
	//EICRB = (1<<ISC41)|(0<<ISC40)|(1<<ISC61)|(0<<ISC60); //falling Edge
	EIMSK = (1<<INT1)|(1<<INT0); //인터럽트 0번~7번을 어디에 사용할것인지, INT0,INT1 허용  (1<<INT4)(1<<INT6)
}

#define AX_MID 512
#define AX_Right_angle 200

double d1 = 0, d2 = 0;
int cnt;
ISR(TIMER0_OVF_vect) //Dynamixel
{
		cnt++;
		TCNT0 = 131;


		if(cnt == 200){ //600ms
				
			if(c2 == 0){
				
				Dynamixel_Moving(0x02, 0x96, 512);
				Dynamixel_Moving(0x03, 0x96, 512);
				

				UART1_Transmit('d');
				UART1_Transmit('e');
				UART1_Transmit('f');
				UART1_Transmit('a');
				UART1_Transmit('u');
				UART1_Transmit('l');
				UART1_Transmit('t');

				cnt = 0;
			}
			else if(c2 > 0){ //1사분면

				degree_manipulator(c1, c2, &d1, &d2); //Plus 연산
				UART1_Transmit('d');
				UART1_Transmit('1');
				UART1_Transmit('=');
				UART1_TransNum(d1);
				UART1_Transmit('d');
				UART1_Transmit('2');
				UART1_Transmit('=');
				UART1_TransNum(d2);

				Dynamixel_Moving(0x02, 0x96, (AX_MID-(d1 * 3.46))); //ID, speed, Position
				UART1_Transmit('m');
				UART1_Transmit('1');
				UART1_Transmit('=');
				UART1_TransNum(AX_MID-(d1 * 3.46));

				Dynamixel_Moving(0x03, 0x96, AX_MID + (d2 * 3.46)); //speed 24%, 중간 //0x3FF -> 1023, 0x1FF -> 511
				UART1_Transmit('m');
				UART1_Transmit('2');
				UART1_Transmit('=');
				UART1_TransNum(AX_MID + (d2 * 3.46));

					if(c3 == 1){
						Dynamixel_Moving(0x04, 0x96, 767);
						UART1_Transmit('m');
						UART1_Transmit('3');
						UART1_Transmit('=');
						UART1_TransNum(767);
					}
					else if(c3 == 0){
						Dynamixel_Moving(0x04, 0x96, 450);
						UART1_Transmit('m');
						UART1_Transmit('3');
						UART1_Transmit('=');
						UART1_TransNum(450);
					}
					else;

				cnt = 0;

			}
			else if(c2 < 0){ //4사분면

			minus_degree_manipulator(c1, c2, &d1, &d2); //minus 연산
			UART1_Transmit('d');
			UART1_Transmit('1');
			UART1_Transmit('=');
			UART1_TransNum(d1);
			UART1_Transmit('d');
			UART1_Transmit('2');
			UART1_Transmit('=');
			UART1_TransNum(d2);
				
			d1 = -d1;
			Dynamixel_Moving(0x02, 0x96, (AX_MID+(d1 * 3.46)));
			UART1_Transmit('m');
			UART1_Transmit('1');
			UART1_Transmit('=');
			UART1_TransNum((AX_MID+(d1 * 3.46)));

			d2 = -d2;
			Dynamixel_Moving(0x03, 0x96, AX_MID - (d2 * 3.46)); //speed 24%, 중간 //0x3FF -> 1023, 0x1FF -> 511
			UART1_Transmit('m');
			UART1_Transmit('2');
			UART1_Transmit('=');
			UART1_TransNum(AX_MID - (d2 * 3.46));

				if(c3 == 1){
					Dynamixel_Moving(0x04, 0x96, 767);
					UART1_Transmit('m');
					UART1_Transmit('3');
					UART1_Transmit('=');
					UART1_TransNum(767);
				}
				else if(c3 == 0){
					Dynamixel_Moving(0x04, 0x96, 450);
					UART1_Transmit('m');
					UART1_Transmit('3');
					UART1_Transmit('=');
					UART1_TransNum(450);
				}else;

				cnt = 0;
			}else;
						
		
			UART1_Transmit(13);

			d1 = 0;
			d2 = 0;

		}
}

int cnt1;
ISR(TIMER1_OVF_vect) //DC Motor
{
	cnt1++;
	
	if(cnt1 == 20){

		if(c4 == 1){
			
			PORTE = 0b00001000; // 앞 뒤
			
			OCR1A = 124; //OCR 설정 100% duty radio
			OCR1B = 124;

			cnt1 = 0;
		}
		else if(c4 == 2){
			
			PORTE = 0b00000100; // 앞 뒤
			
			OCR1A = 124; //OCR 설정 100% duty radio
			OCR1B = 124;

			cnt1 = 0;
		}
		else if(c4 == 0){

			PORTE = 0b00000100; // 앞 뒤
			
			OCR1A = 0; //OCR 설정 100% duty radio
			OCR1B = 0;

			cnt1 = 0;
		}else;
		
		if(flag == 1){
			
			PORTE = 0b00001000; // 앞 뒤
			PORTA = 0x00;

			OCR1A = 0; //OCR 설정 50% duty radio
			OCR1B = 0;

			c4 = 0;
			flag = 0;

			cnt1 = 0;

		}else flag = 0;
	}
}

int main(void)
{
	ICR1 = 124;
	DDRA = 0xff;
	PORTA = 0xff;

	UART0_INIT(); //Dynamixel
	UART1_INIT(); //serial 통신
	DC_MotorSet();
	IntSet();
	TC0Set();

	sei(); //전역 인터럽트 활성화

	while (1);
}


