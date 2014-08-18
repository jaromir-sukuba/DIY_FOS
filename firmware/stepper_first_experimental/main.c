
// PIC18F2420 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1H
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 1         // Brown Out Reset Voltage bits ()

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)



#define _XTAL_FREQ 8000000UL

const unsigned char step_table4[4] = {	0b00000101,	0b00001001,	0b00001010,	0b00000110};
//											  1212		  1212		  1212		  1212	
//											  A	B		  A B		  A	B		  A B

const unsigned char step_table8[8] = {	0b00000101,	0b00000001,	0b00001001,	0b00001000,
//											  1212		  1212		  1212		  1212	
//										  A	B		  A B		  A	B		  A B
										0b00001010,	0b00000010,	0b00000110,	0b00000100};
//											  1212		  1212		  1212		  1212	
//											  A	B		  A B		  A	B		  A B


unsigned char step,test,test2;
unsigned char control,dir,step_u,step_h,step_l,state,ss_data,motor;
unsigned long steps;

#define		LED2	LATBbits.LATB5
#define		LED1	LATBbits.LATB4
#define		BUSY	LATAbits.LATA5

#define		M_A1	LATAbits.LATA3
#define		M_A2	LATAbits.LATA2
#define		M_B1	LATAbits.LATA1
#define		M_B2	LATAbits.LATA0

#define		M_C1	LATBbits.LATB3
#define		M_C2	LATBbits.LATB2
#define		M_D1	LATBbits.LATB1
#define		M_D2	LATBbits.LATB0

void dly_num (unsigned char num);

void main (void);
void do_steps1 (unsigned long n_steps, unsigned char control);
void do_steps2 (unsigned long n_steps, unsigned char control);
void system_init (void);

void main (void)
{
    unsigned char i;
    for (i=0;i<100;i++) __delay_ms (4);
system_init();
state = 0;

//while (1) do_steps1 (1300,0x06);


do_steps1 (1200,0x8F);		//1000 steps normal, slow

while (1);

/*
control = 0;
while (1)
	{
	while (state<20) {}

	if (state==20)
		{
		steps = (((unsigned long)(step_u))<<16) + (((unsigned long)(step_h))<<8) + step_l;
		if (motor==1)	do_steps1 (steps,control);
		if (motor==2)	do_steps2 (steps,control);
		control = 0x00;
		state = 0;
		}
	if (state==40)
		{
		step = 0;
		LATA=0x00;
		LATB=0x00;
		control = 0;
		state = 0;
		}

	}
*/




}



void do_steps1 (unsigned long n_steps, unsigned char control)
{
unsigned long counter;
for (counter=0;counter<n_steps;counter++)
	{
	if (control&0x80) 
			{
			test = step_table8[step];		//1/0 (bit 7 from control byte) - normal/reverse
			LED1=0;
			LED2=1;
			}
		else 
			{
			test = step_table8[7-step];		//1/0 (bit 7 from control byte) - normal/reverse
			LED1=1;
			LED2=0;
			}
	if (test&0x01) M_A1 = 1;
		else M_A1 = 0;
	if (test&0x02) M_A2 = 1;
		else M_A2 = 0;
	if (test&0x04) M_B1 = 1;
		else M_B1 = 0;
	if (test&0x08) M_B2 = 1;
		else M_B2 = 0;
	dly_num(6 + (control&0x3F));		// ((6 + (0->64))*100) us delay
	//dly_num(6 + (control&0x3F));		// ((6 + (0->64))*100) us delay
	step++;
	if (step==8) step=0;
	}
}

void dly_num (unsigned char num)
{
    unsigned char i;
    for (i=0;i<num;i++)
    {
    __delay_us (100);
    }

}


/*
void do_steps2 (unsigned long n_steps, unsigned char control)
{
unsigned long counter;
for (counter=0;counter<n_steps;counter++)
	{
	if (control&0x80) 
			{
			test = step_table8[step];		//1/0 (bit 7 from control byte) - normal/reverse
			LED1=0;
			LED2=1;
			}
		else 
			{
			test = step_table8[7-step];		//1/0 (bit 7 from control byte) - normal/reverse
			LED1=1;
			LED2=0;
			}
	if (test&0x01) M_C1 = 1;
		else M_A1 = 0;
	if (test&0x02) M_C2 = 1;
		else M_A2 = 0;
	if (test&0x04) M_D1 = 1;
		else M_B1 = 0;
	if (test&0x08) M_D2 = 1;
		else M_B2 = 0;
	Delay100TCYx(6 + (control&0x7F));		// ((6 + (0->127))*100) us delay
	Delay100TCYx(6 + (control&0x7F));		// ((6 + (0->127))*100) us delay
	step++;
	if (step==8) step=0;
	}
}
*/

void system_init (void)
{
OSCCON = 0x70;
//OSCTUNE = 0x40;
TRISB = 0;
TRISA = 0;
LATA=0x00;
LATB=0x00;

TRISCbits.TRISC6=1;
TRISCbits.TRISC7=1;

TRISCbits.TRISC3=1;
TRISCbits.TRISC5=0;

TRISAbits.TRISA5=1;
ADCON1 = 0x0F;

LED1=1;
LED2=1;
step = 0;
}
