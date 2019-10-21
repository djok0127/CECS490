// CECS447Lab3.c
// Runs on LM4F120/TM4C123
// Test the switch initialization functions by setting the LED
// color according to the status of the switches.
// Daniel and Jonathan Valvano
// May 23, 2014

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Section 4.2    Program 4.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// negative logic switches connected to PF0 and PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
// the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
// and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
// is written to the Port F GPIO Lock Register.  After Port F is
// unlocked, bit 0 of the Port F GPIO Commit Register must be set to
// allow access to PF0's control registers.  On the LM4F120, the other
// bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
// that the rest of Port F can always be freely re-configured at any
// time.  Requiring this procedure makes it unlikely to accidentally
// re-configure the JTAG pins as GPIO, which can lock the debugger out
// of the processor and make it permanently unable to be debugged or
// re-programmed.

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x01                      // on the left side of the Launchpad board
#define SW2       0x10                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define YELLOW    0x0A
#define SKYBLUE   0x0C
#define WHITE     0x0E
// initialize global variables

void PortF_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCTIMER_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void PortF_Inter_Init(void){
	GPIO_PORTF_IS_R &= ~0x01;					// setting for edge trigger
	GPIO_PORTF_IBE_R &= ~0x01;				// setting for one edge trigger
	GPIO_PORTF_IEV_R &= ~0x01;				// setting for falling edge trigger
	GPIO_PORTF_ICR_R |= 0x01;					// set the flags for interrupt
	GPIO_PORTF_IM_R |= 0x01;					// arm interrupt of PF4
	NVIC_PRI7_R = 0 << 21;						// setting the priority of 0
	NVIC_EN0_R = 0x40000000;					// enable the interrupt
}

void UART0_Init(void){
	SYSCTL_RCGCUART_R |= 0x0001; 			// activate UART0
	SYSCTL_RCGCGPIO_R |= 0x0001;			// activate GPIOA
	UART0_CTL_R &= ~0x0001;						// disable UART0
	UART0_IBRD_R = 81;								// IBRD, 16MHz, 9600 baud rate
	UART0_FBRD_R = 24;								// FBRD, (0.52083 * 64 + .05)
	UART0_LCRH_R = 0x0060;						// 8 bit (no parity, one stop, no FIFOs)
	UART0_CTL_R |= 0x0001; 						// enable UART
	GPIO_PORTA_AFSEL_R |= 0x03;				// enable alt funct on PA0,1
	GPIO_PORTA_PCTL_R &= ~0x000000FF; // configure PA0 and PA1 as U0Rx and U0Tx
	GPIO_PORTA_PCTL_R |= 0x00000011;
	GPIO_PORTA_AMSEL_R &= ~0x03;			// disable analog funct on PA0, PA1
	GPIO_PORTA_DEN_R |= 0x03;					// enable digital I/O on PA0, PA1
	}

void UART1_Init(void){
	SYSCTL_RCGCUART_R |= 0x0002; 			// activate UART1
	SYSCTL_RCGCGPIO_R |= 0x0002;			// activate GPIOB
	UART1_CTL_R &= ~0x0001;						// disable UART1
	UART1_IBRD_R = 81;								// IBRD, 50MHz, 38400 baud rate (50MHz / (16 * 38400))
	UART1_FBRD_R = 24;							  // FBRD, (0.3802 * 64 + .05)
	UART1_LCRH_R = 0x0060;						// 8 bit (no parity, one stop, no FIFOs)
	UART1_CTL_R |= 0x0001; 						// enable UART
	GPIO_PORTB_AFSEL_R |= 0x03;				// enable alt funct on PA0,1
	GPIO_PORTB_PCTL_R &= ~0x000000FF; // configure PB0 and PB1 as U1Rx and U1Tx
	GPIO_PORTB_PCTL_R |= 0x00000011;
	GPIO_PORTB_AMSEL_R &= ~0x03;			// disable analog funct on PB0, PB1
	GPIO_PORTB_DEN_R |= 0x03;					// enable digital I/O on PB0, PB1
}

void UART1_Configure(void){
	SYSCTL_RCGCUART_R |= 0x0002; 			// activate UART1
	UART1_CTL_R &= ~0x0001;						// disable UART1
	UART1_IBRD_R = 54;								// IBRD, 50MHz, 57600 baud rate (50MHz / (16 * 57600))
	UART1_FBRD_R = 16;								  // FBRD, (0.253472 * 64 + .05)
	UART1_LCRH_R = 0x000A;						// 8 bit (parity enable, one stop, no FIFOs)
	UART1_CTL_R |= 0x0001; 						// enable UART1
}

void Timer0_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCTIMER_R |= 0x01;				// activate timer 0
	delay = SYSCTL_RCGCTIMER_R;
	TIMER0_CTL_R = 0;									// disable the timer for setup
	TIMER0_CFG_R = 0x04;							// configure for a 16-bit timer mode
	TIMER0_TAMR_R |= 0x01;						// configure for a one shot, count down
	TIMER0_CTL_R |= 0x01;							// enable timer0
}

void Timer0_DelayMs(uint16_t time){
	uint16_t x;
	unsigned long volatile delay;
	SYSCTL_RCGCTIMER_R |= 1;	// enable Timer Block 0
	delay =SYSCTL_RCGCTIMER_R;
	for(x=0; x < time; x++){
		TIMER0_TAILR_R = 50000 - 1;	// interval load value of every 1ms
		TIMER0_ICR_R = 0x01;				// clear TimerA timeout flag
		TIMER0_CTL_R = 0x01;			// enable timer A
		while((TIMER0_RIS_R & 0x01) == 0) {
			if((TIMER0_RIS_R & 0x01) == 0)
				break;	// wait for timeout
		}
	}
	TIMER0_ICR_R = 0x1;				// clear TimerA timeout flag	
}

// wait for new input, then return ASCII code
char UART_Rx(void){
	if ((UART1_FR_R &0x0010) == 0)	// non-blocking receive 
		return ((unsigned char) (UART1_DR_R & 0xFF));
	return ((unsigned char) 0x00);
}

char UART0_Rx(void){
	if ((UART0_FR_R &0x0010) == 0)	// non-blocking receive 
		return ((unsigned char) (UART0_DR_R & 0xFF));
	return ((unsigned char) 0x00);
}

void UART_Tx(unsigned char data){
	while((UART1_FR_R & 0x0020) != 0);	// wait until TXFF is 0
	UART1_DR_R = data;
}

void UART0_Tx(unsigned char data){
	while((UART0_FR_R & 0x0020) != 0);	// wait until TXFF is 0
	UART0_DR_R = data;
}

void UART_transmit_String( const uint8_t *MessageString){
	while( *MessageString ){
		UART_Tx(*MessageString);
		MessageString++;
	}
}

void HC05_Init(){
	
		uint8_t data;
		GPIO_PORTF_DATA_R = RED;
		UART_transmit_String("AT\r\n");
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		/*
		data = ~(data&0x00);
		for(int i = 0; i <10; i++)
			Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = BLUE;
		UART_transmit_String("AT+NAME=490B\r\n");
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		data = (data&0x00);
		for(int i = 0; i <10; i++)
			Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = YELLOW;
		UART_transmit_String("AT+PSWD=490\r\n");
		
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		*/
		data = (data&0x00);
		for(int i = 0; i <10; i++)
			Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = SKYBLUE;
		UART_transmit_String("AT+ROLE=0\r\n");
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		data = (data&0x00);
		for(int i = 0; i <10; i++)
		Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = RED;
		UART_transmit_String("AT+PSWD=490\r\n");
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		data = (data&0x00);
		for(int i = 0; i <10; i++)
			Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = GREEN;
		UART_transmit_String("AT+UART=38400,1,0\r\n");
		//UART1_Configure();
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		data = (data&0x00);
		for(int i = 0; i <10; i++)	
			Timer0_DelayMs(10000);
		GPIO_PORTF_DATA_R = WHITE;
		UART_transmit_String("AT+RESET\r\n");
		while(data != 0x4F){
			data = UART_Rx();
			UART0_Tx(data);
		}
		GPIO_PORTF_DATA_R = WHITE;
		
		while(1);
}

void GPIOPortF_Handler(void){
	
	if(GPIO_PORTF_RIS_R|0x11){
		//UART0_Tx('!');
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R ^ 0x02;
		UART_transmit_String("hello\r\n");
	}
	GPIO_PORTF_ICR_R |= 0x11;
}


int main(void){
	uint8_t data_t;
	uint8_t pos_t = 0;
	uint8_t message[30];
	PLL_Init();
	PortF_Init();
	PortF_Inter_Init();
	UART0_Init();
	UART1_Init();
	Timer0_Init();
	//HC05_Init();
	char a;
	int done = 0;
	while(1){
		a = UART_Rx();
		if(a)
			UART0_Tx(a);

		//GPIO_PORTF_DATA_R = WHITE;
		
	}
}


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

