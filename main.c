
/* Documentation notes:
 * Medicion de tiempo de giro de un encoder mediante temporizador.
 * author: Freddy Rodrigo Mendoza Ticona correo: freddy12120@gmail.com
 *
 * Afiliación:
 *  - Universidad Nacional de Ingeniria - Lima, Perú
 *  - Grupo de investigación en sistemas embebidos, procesamiento de señales e inteligencia computacional - SEPSIC
 *
 *
 * */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h" // QEI module
#include "driverlib/timer.h"

volatile int qeiPosition;


char d1,d2,d3,d4,d5,d6,d7,d8,d9;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif
//funciones
void sendvarToUART(uint32_t var,uint8_t aux);
void Init_measure(void);
void messureTimeforGiro(void);
void TxData(void);

char rx_char;
char comando[5];
uint8_t index=0;
uint8_t aux = 0;
uint8_t flaguart=0;
uint8_t flag_escape = 1;
uint8_t grados = 0;
uint32_t timer1_valuemSeg;

uint32_t acumul_pos;
uint32_t POSCNT;

//tiempo almacenado
uint32_t times_ms[4000];
uint16_t number_times;
uint16_t index_times=0;

//periodo del timer.
uint32_t ui32Period;  //32 bit unsigned integer



// Interrupcion por Rx
void UARTIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status

    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts

    //if(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    //{
       rx_char = UARTCharGet(UART0_BASE);
       switch(rx_char){
       	   case 'A':
       		   flag_escape = 1;
       		   break;
       	   case '\r':
       		   index=0;
       		   aux = aux+0x02;
       		   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, aux);//led rojo toogle
       		   flaguart = 1;
       		   break;
       	   default :
       		   comando[index]=rx_char;
       		   index++;
       		   break;
       }

    //}
}

 // timer interrupt
void Timer0IntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Read the current state of the GPIO pin and
	// write back the opposite state


	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
	}
}


int main(void) {
	
	// system control
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz


 //******************************************CONFIG QEI*****************************************************
    //habilita perfierico QEI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);


	//Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;


	//Set Pins to be PHA0 and PHB0
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);

	//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7.
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); // change push pull gpio

	//Disable peripheral and int before configuration
	QEIDisable(QEI0_BASE);
	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX); //desabilita todas las interrupciones

	// Configure quadrature encoder, use an arbitrary top limit of 1000
	// 4 pulsos por linea
	// encoder de 2048 PPR max 7500 RPM
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 8191);  //maxima posicion


	// Enable the quadrature encoder.
	QEIEnable(QEI0_BASE);

	//Set position to a middle value so we can see if things are working
	QEIPositionSet(QEI0_BASE, 0);


//*********************************************CONFIG UART *****************************************************************

	// habilita Periferico UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));



    //***************************************CONFIG LEDS*************************************************************

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); // PF1=RED PF2=BLUE PF3=GREEN
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);//led rojo off




  //********************************************CONFIG TIMER0 *********************************************************
    //Config TIMER

    //primero habilitar el clock en el periferico
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //FULL Width periodic timer 32 bit


	//ui32Period = (SysCtlClockGet() / 10) / 2;
    ui32Period = SysCtlClockGet()*10;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period-1);


	//***************************************Config Interrupt *********************************************************
	//IntPrioritySet(INT_UART0);
	//IntPrioritySet(INT_UART0, 1); // prioridad 1
	//IntPrioritySet(INT_TIMER0A, 0); // prioridad 0, maxima prioridad

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART0); //enable the UART interrupt
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and RT interrupts

	//IntEnable(INT_TIMER0A);
	//TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//IntMasterEnable();
	 TimerEnable(TIMER0_BASE, TIMER_A);


	 flag_escape = 1;
	 flaguart = 0;

    while (1)
    {


    	if(flaguart){

    	 Init_measure();

    	  while(flag_escape == 0){

    		  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    		  messureTimeforGiro();
    		  if(flag_escape == 1){break;}
    		  times_ms[index_times] = (ui32Period-1) - timer1_valuemSeg;
    		  ++index_times;
    		  acumul_pos = acumul_pos + POSCNT;
    		  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);

    		  if(index_times == 4000){break;}; // sale del bucle.

    	  }

    	  TxData();// envia los datos.
    	  sendvarToUART(acumul_pos,1);
    	  index_times = 0;
    	  acumul_pos  = 0;
    	  flag_escape = 1;
    	  flaguart = 0;
    	}

    }

}

void sendvarToUART(uint32_t var,uint8_t aux ){


	if(aux==0){
	UARTCharPut(UART0_BASE,'A');
	}else{
	UARTCharPut(UART0_BASE,'N');
	}

	if(var<=9){
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else if(var>=10 && var<=99){
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
        d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else if (var>=100 && var<=999){
		 d7=(char)(((int)'0')+((var/100)%10));
		 UARTCharPut(UART0_BASE,d7);
         d8=(char)(((int)'0')+((var/10)%10));
		 UARTCharPut(UART0_BASE,d8);
         d9=(char)(((int)'0')+(var%10));
		 UARTCharPut(UART0_BASE,d9);
	}else if(var>=1000  && var<=9999){

		d6 =(char)(((int)'0')+((var/1000)%10));
		UARTCharPut(UART0_BASE,d6);
		d7=(char)(((int)'0')+((var/100)%10));
	    UARTCharPut(UART0_BASE,d7);
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else if(var>=10000  && var<=99999){


		d5 =(char)(((int)'0')+((var/10000)%10));
		UARTCharPut(UART0_BASE,d5);
		d6 =(char)(((int)'0')+((var/1000)%10));
		UARTCharPut(UART0_BASE,d6);
		d7=(char)(((int)'0')+((var/100)%10));
		UARTCharPut(UART0_BASE,d7);
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else if(var>=100000  && var<=999999){

			d4 =(char)(((int)'0')+((var/100000)%10));
			UARTCharPut(UART0_BASE,d4);
			d5 =(char)(((int)'0')+((var/10000)%10));
			UARTCharPut(UART0_BASE,d5);
			d6 =(char)(((int)'0')+((var/1000)%10));
			UARTCharPut(UART0_BASE,d6);
			d7=(char)(((int)'0')+((var/100)%10));
			UARTCharPut(UART0_BASE,d7);
			d8=(char)(((int)'0')+((var/10)%10));
			UARTCharPut(UART0_BASE,d8);
			d9=(char)(((int)'0')+(var%10));
			UARTCharPut(UART0_BASE,d9);

	}else if(var>=1000000  && var<=9999999){


		d3 =(char)(((int)'0')+((var/1000000)%10));
		UARTCharPut(UART0_BASE,d3);
		d4 =(char)(((int)'0')+((var/100000)%10));
		UARTCharPut(UART0_BASE,d4);
		d5 =(char)(((int)'0')+((var/10000)%10));
		UARTCharPut(UART0_BASE,d5);
		d6 =(char)(((int)'0')+((var/1000)%10));
		UARTCharPut(UART0_BASE,d6);
		d7=(char)(((int)'0')+((var/100)%10));
		UARTCharPut(UART0_BASE,d7);
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else if(var>=10000000  && var<=99999999){

		d2 =(char)(((int)'0')+((var/10000000)%10));
		UARTCharPut(UART0_BASE,d2);
		d3 =(char)(((int)'0')+((var/1000000)%10));
		UARTCharPut(UART0_BASE,d3);
		d4 =(char)(((int)'0')+((var/100000)%10));
		UARTCharPut(UART0_BASE,d4);
		d5 =(char)(((int)'0')+((var/10000)%10));
		UARTCharPut(UART0_BASE,d5);
		d6 =(char)(((int)'0')+((var/1000)%10));
		UARTCharPut(UART0_BASE,d6);
		d7=(char)(((int)'0')+((var/100)%10));
		UARTCharPut(UART0_BASE,d7);
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}else{

		d1 =(char)(((int)'0')+((var/100000000)%10));
		UARTCharPut(UART0_BASE,d1);
		d2 =(char)(((int)'0')+((var/10000000)%10));
		UARTCharPut(UART0_BASE,d2);
		d3 =(char)(((int)'0')+((var/1000000)%10));
		UARTCharPut(UART0_BASE,d3);
		d4 =(char)(((int)'0')+((var/100000)%10));
		UARTCharPut(UART0_BASE,d4);
		d5 =(char)(((int)'0')+((var/10000)%10));
		UARTCharPut(UART0_BASE,d5);
		d6 =(char)(((int)'0')+((var/1000)%10));
		UARTCharPut(UART0_BASE,d6);
		d7=(char)(((int)'0')+((var/100)%10));
		UARTCharPut(UART0_BASE,d7);
		d8=(char)(((int)'0')+((var/10)%10));
		UARTCharPut(UART0_BASE,d8);
		d9=(char)(((int)'0')+(var%10));
		UARTCharPut(UART0_BASE,d9);

	}

	UARTCharPut(UART0_BASE,'\r');
	UARTCharPut(UART0_BASE,'\n');

}


void Init_measure(void){

    switch (comando[0]){
        case 'Q':
            grados = 1;
            flag_escape = 0;
            break;
        case 'W':
            grados = 2;
            flag_escape = 0;
            break;
        case 'E':
            grados = 3;
            flag_escape = 0;
            break;
        case 'R':
            grados = 4;
            flag_escape = 0;
            break;
        case 'T':
            grados = 5;
            flag_escape = 0;
            break;
        case 'Y':
            grados = 6;
            flag_escape = 0;
            break;
        case 'B':
        	UARTCharPut(UART0_BASE,'F');
        	UARTCharPut(UART0_BASE,'\n');
            UARTCharPut(UART0_BASE,'\r');
            break;
    }
}


void messureTimeforGiro(void){

	uint32_t pos = 0;

	TimerLoadSet(TIMER0_BASE, TIMER_A,  ui32Period-1);
	QEIPositionSet(QEI0_BASE, 4000);

    // High precision instrumental!
    switch (grados){
        case 1:
        	pos = QEIPositionGet(QEI0_BASE);

            while((pos < 4023) && (pos> 3977)) {
            	pos = QEIPositionGet(QEI0_BASE);
            	if(flag_escape==1){break;}
            }

            POSCNT =  23;
            timer1_valuemSeg = TimerValueGet(TIMER0_BASE,TIMER_A);

            break;
        case 2:
        	pos = QEIPositionGet(QEI0_BASE);
            while((pos < 4046) && (pos> 3954)){
            	pos = QEIPositionGet(QEI0_BASE);
            	if(flag_escape==1){break;}
            }

            POSCNT =  46;
            timer1_valuemSeg = TimerValueGet(TIMER0_BASE,TIMER_A);

            break;
        case 3:
        	pos = QEIPositionGet(QEI0_BASE);
            while((pos < 4069) && (pos> 3931)){

            	pos = QEIPositionGet(QEI0_BASE);
            	if(flag_escape==1){break;}

            }
            POSCNT =  69;
            timer1_valuemSeg = TimerValueGet(TIMER0_BASE,TIMER_A);

            break;
        case 4:
        	pos = QEIPositionGet(QEI0_BASE);
            while((pos < 4091) && (pos > 3909)){
            	pos = QEIPositionGet(QEI0_BASE);
            	if(flag_escape==1){break;}

            }
            POSCNT =  91;
            timer1_valuemSeg =TimerValueGet(TIMER0_BASE,TIMER_A);

            break;
        case 5:
        	pos = QEIPositionGet(QEI0_BASE);
            while((pos < 4114) && (pos> 3886)){
            	pos = QEIPositionGet(QEI0_BASE);
            	if(flag_escape==1){break;}
            }
            POSCNT =  114;
            timer1_valuemSeg =TimerValueGet(TIMER0_BASE,TIMER_A);

            break;

       case 6:// not working!
            while(((int)QEIPositionGet(QEI0_BASE) < 2048) && ((int)QEIPositionGet(QEI0_BASE) > -2048)){if(flag_escape==1){break;}}
            timer1_valuemSeg = TimerValueGet(TIMER0_BASE,TIMER_A);

            break;
    }
}

void TxData(void){

	uint32_t i;

	for(i=0; i<index_times;i++){

		sendvarToUART(times_ms[i],0); // manda todos los datos por el UART.

	}

}



