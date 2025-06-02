### main.c

```
#include "board.h"
#include "timer0.h"
#include "uart.h"
#include "tone.h"
#include "ultrasonic.h"
#include <stdio.h>
extern volatile uint8_t time_update_flag;
extern volatile uint16_t time;

int main(void)
{
	sei(); // 전역인트럽트 활성화
	board_init();
	uart_init(9600);
	uart_print("start!\n");
	timer0_init();
	us_init();

	uint32_t prev_trigger = millis();
	while(1)
	{
		if (millis() - prev_trigger >=25)
		{
			prev_trigger = millis();
			us_trigger();
		}

		if (time_update_flag) // 요청확인
		{
			time_update_flag = 0;			
			uint16_t d = time * (0.0343 /2 * 278.3 / 262);
			char s[8]; // buffer for string
			sprintf( s, "%d\n" ,d  );
			uart_print(s);
		}
		_delay_ms(1);
		
	}
}

```


---


### board.c
```
#include "board.h"
// SWITCH 
#define TRIGGER_HIGH()	SET_BIT(PORTD,3)
#define TRIGGER_LOW()	CLEAR_BIT(PORTD,3)
#define ECHO			BIT_TEST(PIND,2)


void board_init()
{
	
	DDRD = (1<<5)| (1<<6);	// LED	
	DDRB = (1<<2);			// BUZZER
	
	DDRC = (1<<7);			// 485 
	PORTC |= 0b11111;		// DIP SWITCH : INTERNAL PULLUP INPUT
}


void led_test()
{
	while (1)
	{
		RED_LED_ON();
		GREEN_LED_ON();
		_delay_ms(500);
	
		RED_LED_OFF();
		GREEN_LED_OFF();
		_delay_ms(500);
	}
}

```

---

### board.h

```
#ifndef BOARD_H_
#define BOARD_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // 인터럽트 서비스 루틴

#define H(b)		(1<<(b))

#define BIT_TEST(r,b) (r & H(b))
#define SET_BIT(r,b)  r|= H(b)
#define CLEAR_BIT(r,b)  r&=~H(b)
#define TOGGLE_BIT(r,b)  r^=H(b)

#define SW1				BIT_TEST(PIND,7)

#define RED_LED_ON()	SET_BIT(PORTD,5)
#define RED_LED_OFF()	CLEAR_BIT(PORTD,5)
#define RED_LED_TOGGLE() TOGGLE_BIT(PORTD,5)

#define GREEN_LED_ON()	SET_BIT(PORTD,6)
#define GREEN_LED_OFF()	CLEAR_BIT(PORTD,6)
#define GREEN_LED_TOGGLE() TOGGLE_BIT(PORTD,5)


void board_init();
void led_test();


#endif /* BOARD_H_ */
```

### timer0.c

```
#define F_CPU 16000000UL
#include <avr/io.h> //reg
#include <util/delay.h>
#include <avr/interrupt.h> // 인터럽트 서비스 루틴

#include <util/atomic.h>
// 타이머 관련 전역 변수
volatile uint32_t timer0_overflow_count = 0;     // 타이머 오버플로우 횟수
volatile uint32_t timer0_millis = 0;             // 밀리초 카운터
static uint8_t timer0_fract = 0;                 // 분수 밀리초 (정밀도 향상용)

// 상수 정의 (16MHz, 64 분주비 기준)
// 타이머0 오버플로우 시간 = 256 틱 * (64 / 16000000) = 1.024ms
#define MICROS_PER_TIMER0_OVERFLOW (64 * 256 / (F_CPU / 1000000UL))
#define MILLIS_INC (MICROS_PER_TIMER0_OVERFLOW / 1000)             // 오버플로우당 밀리초 증가량 (1)
#define FRACT_INC ((MICROS_PER_TIMER0_OVERFLOW % 1000) >> 3)       // 오버플로우당 분수 밀리초 증가량 (3)
#define FRACT_MAX (1000 >> 3)                                      // 분수 밀리초 최대값 (125)

// Timer0 오버플로우 인터럽트 서비스 루틴
ISR(TIMER0_OVF_vect) {
	// 현재 밀리초와 분수 밀리초 값 가져오기
	uint32_t m = timer0_millis;
	uint8_t f = timer0_fract;
	// 오버플로우당 증가량 더하기
	m += MILLIS_INC;
	f += FRACT_INC;
	
	// 분수 밀리초가 최대값을 초과하면 밀리초 1 추가
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}
	// 값 업데이트
	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

// Timer0 초기화 함수
void timer0_init(void) {
	// 인터럽트 비활성화
	cli();
	
	// 타이머/카운터 0 레지스터 초기화
	TCCR0A = 0;  // 일반 모드
	
	// 분주비 64 설정 (CS01=1, CS00=1)
	// 16MHz에서 분주비 64일 때 타이머 클럭은 250kHz (4μs 간격)
	TCCR0B = (1 << CS01) | (1 << CS00);
	
	// 타이머 오버플로우 인터럽트 활성화
	TIMSK0 |= (1 << TOIE0);
	
	// 카운터와 플래그 초기화
	timer0_overflow_count = 0;
	timer0_millis = 0;
	timer0_fract = 0;
	
	// 인터럽트 활성화
	sei();
}


// 밀리초 반환 함수
uint32_t millis(void) {
	uint32_t m;
	
	// 원자적 블록 내에서 값 읽기 (인터럽트 영향 없이)
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = timer0_millis;
	}
	
	return m;
}

// 마이크로초 반환 함수
uint32_t micros(void) {
	uint32_t m;
	uint8_t t;
	
	// 원자적 블록 내에서 오버플로우 카운트와 현재 타이머 값 읽기
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		m = timer0_overflow_count;
		t = TCNT0;
		
		// 오버플로우 플래그가 설정되었지만 아직 인터럽트가 처리되지 않았는지 확인
		// 이 경우 수동으로 오버플로우 카운트 증가 필요
		if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
			m++;
		}
	}
	
	// 총 마이크로초 계산
	// (오버플로우 횟수 * 256 + 현재 타이머값) * 타이머 틱당 μs
	return ((m << 8) + t) * (64 / (F_CPU / 1000000UL));
}

// 딜레이 함수 (밀리초)
void delay(uint32_t ms) {
	uint32_t start = millis();
	
	while ((millis() - start) < ms) {
		// 대기
	}
}
```

### timer0.h

```
#ifndef TIMER0_H_
#define TIMER0_H_


// Timer0 초기화 함수
void timer0_init(void);

// 밀리초 반환 함수
uint32_t millis(void);

// 마이크로초 반환 함수
uint32_t micros(void);

// 딜레이 함수 (밀리초)
void delay(uint32_t ms);

#endif /* TIMER_H_ */

```

### tone.C
```
#include "board.h"
#include "tone.h"

// tone()을 위한 변수
static volatile uint8_t isToneActive = 0;

void tone(unsigned long frequency) {
	// 주파수 유효성 검사
	if (frequency == 0 || frequency > 10000) { // 상한값 추가
		tone_stop();
		return;
	}
	
	// PB1(OC1A) 핀을 출력으로 설정
	DDRB |= (1 << PB1);
	
	// 타이머 1 설정 - 정확한 CTC 모드
	TCCR1A = 0;                    // CTC 모드를 위해 COM1A1:0을 나중에 설정
	TCCR1B = (1 << WGM12);         // CTC 모드, OCR1A가 TOP 값
	
	// 클럭 선택 (분주율 설정)
	// 16MHz 시스템 클럭 가정, 분주율 8 => 2MHz 타이머 클럭
	TCCR1B |= (1 << CS11);         // 분주율 8
	
	// 주파수 계산: 2MHz / (2 * 주파수) - 1
	uint16_t ocr_val = (2000000 / (2 * frequency)) - 1;
	OCR1A = ocr_val;
	
	// 출력 핀 설정 - 비교 일치 시 토글
	TCCR1A |= (1 << COM1A0);
	
	// tone 활성화 상태 업데이트
	isToneActive = 1;
}

void tone_stop() {
	// 타이머 1의 출력 설정을 리셋
	TCCR1A = 0; // OC1A 핀에서 PWM 출력 중지
	TCCR1B = 0; // 타이머 1 비활성화
	isToneActive = 0; // 음을 멈춤
}

void tone_test() {
	while (1)
	{
		tone(440);
		_delay_ms(1000);
		tone_stop();
		_delay_ms(1000);

	}
}
```

---

### tone.h
```
#ifndef TONE_H_
#define TONE_H_

void tone(unsigned long frequency);
void tone_stop();

#endif /* TONE_H_ */

```
---

### uart.c
```
#include "board.h"
//#include <avr/io.h>
//#include <util/delay.h>
//#include <avr/interrupt.h> // 인터럽트 서비스 루틴

#define USART_TX_BUFFER_SIZE 64
#define USART_RX_BUFFER_SIZE 64

// volatile : 최적화 금지 (무조건 읽어라)
// interrupt <--> main
// ring buffer
volatile uint8_t tx_buffer[ USART_TX_BUFFER_SIZE];
volatile uint8_t tx_head =0;
volatile uint8_t tx_tail =0;
volatile uint8_t rx_buffer[ USART_RX_BUFFER_SIZE];
volatile uint8_t rx_head =0;
volatile uint8_t rx_tail =0;

ISR(  USART_RX_vect ) // 수신 : 생산 : rx_head
{
	uint8_t data = UDR0; // 버퍼로 부터 데이터를 읽음
	uint8_t next = rx_head + 1; // 다음인덱스
	
	if (next >= USART_RX_BUFFER_SIZE) next = 0; //ring
	
	if (next  == rx_tail) // buffer full??
		return ; // 처리 안함
	
	rx_buffer[rx_head] = data;
	rx_head = next;
}

ISR(  USART_UDRE_vect )// 송신버퍼 빌때 :  소비 : tx_tail
{
	if(tx_head == tx_tail) // blank buffer
	{
		//해당 인터럽트 꺼버리기(비활성화)
		UCSR0B &= ~(1<<UDRIE0); // 송신 버퍼 빔 인터럽트 비활성화
		return;
	}
	
	UDR0 = tx_buffer[tx_tail];
	uint8_t next = tx_tail + 1; // 다음인덱스
	
	if (next >= USART_TX_BUFFER_SIZE) next = 0; //ring
	tx_tail = next;
}

void uart_init(uint32_t baud) // ~115200 :32
{
	//baud rate
	UBRR0 = (F_CPU / 16 / baud) - 1;
	//UBRR0 = (F_CPU / (16 * baud)) - 1;
	// uart mode 설정 :비동기 ,no parity  1 stop
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); //8bit data
	//송수신 활성화
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0B |= (1<<UDRIE0)|(1<<RXCIE0); // 송수신 버퍼 인터럽트 활성화
	// 인터럽트 활성화
	sei();	
}

void uart_tx(uint8_t data) // non-block 함수 :생산 : tx_head
{
	uint8_t next = tx_head + 1; // 다음인덱스
	
	if (next >= USART_TX_BUFFER_SIZE) next = 0; //ring
	
	while (next  == tx_tail){} // buffer full??: 대기 ==>좀기다리면 자리가 나옴
	
	tx_buffer[tx_head] = data;
	tx_head = next;
	UCSR0B |= (1<<UDRIE0); // 송신 버퍼 빔 인터럽트 활성화
}

uint8_t uart_rx() // non-block 함수 :소비 : rx_tail
{
	while(rx_head == rx_tail){} // blank buffer ==> 대기 : blocking
	
	uint8_t data = rx_buffer[rx_tail];
	uint8_t next = rx_tail + 1; // 다음인덱스
	
	if (next >= USART_RX_BUFFER_SIZE) next = 0; //ring
	rx_tail = next;
	return data;
}

uint8_t uart_is_available()
{
	return (rx_head != rx_tail);
}

void uart_print(const char *s )
{
	char c;
	// *s : s가 가리키는 대상(문자)
	for(;(c =*s) ;s++ ) // s++ 다음 문자로 가리키는 대상 이동
	{
		uart_tx(c); //송신
	}
}

void uart_test()
{
	uart_init(9600);
	
	while (1)
	{
		RED_LED_TOGGLE();
		uart_print("hello\n");
		_delay_ms(1000);

	}
	
}


```
---

### uart.h
```
#ifndef UART_H_
#define UART_H_


void uart_init(uint32_t baud);
void uart_tx(uint8_t data);
uint8_t uart_rx();
uint8_t uart_is_available();
void uart_print(const char *s );
void uart_test();

#endif /* UART_H_ */
```
---

### ultrasonic.c
```
#include "board.h"
#include "timer0.h"

#define TRIGGER_HIGH()	SET_BIT(PORTD,3)
#define TRIGGER_LOW()	CLEAR_BIT(PORTD,3)
#define ECHO			BIT_TEST(PIND,2)


void us_trigger()
{
	TRIGGER_HIGH();
	_delay_us(10);
	TRIGGER_LOW();
}

volatile uint16_t time =0;
volatile uint8_t time_update_flag =0;
uint32_t start_time = 0;

ISR(INT0_vect)
{
	uint32_t us = micros();
	if (ECHO)
	{  // rising edge
		start_time = us;
	}
	else
	{ // falling edge
		time= us - start_time  ;
		time_update_flag = 1;
	}
}
void us_init()
{
	DDRD |= (1<<3); // trigger => out
	DDRD &= ~(1<<2); // echo => in / int0
	
	// echo pin : ext int0
	EICRA |=  (1<<ISC00); // mode:logical change
	EICRA &=  ~(1<<ISC01);
	EIMSK |=  (1<<INT0); // enable ext int0
	sei();
}
/*
void us_test()
{
	uart_init(9600);
	us_init();
	timer0_init();
	
	sei(); // 전역인터럽트 활성화
	
	us_trigger(); //첫 측정
	while(1)
	{
		if (time_update_flag) // 요청확인
		{
			time_update_flag = 0;
			us_trigger(); // 측정시작
			uint16_t d = time * (0.0343 /2 * 278.3 / 262);
			char s[8]; // buffer for string
			sprintf( s, "%d\n" ,d  );
			uart_print(s);
		}
		_delay_ms(1);
		
	}
}
*/
```

---

### ultrasonic.h
```
#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_


void us_init();	
void us_trigger(); 


#endif /* ULTRASONIC_H_ */
```
---
