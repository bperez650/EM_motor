#include <asf.h>


void clock_setup(void);
void wait(volatile int d);
void port_setup(void);
void timer_setup_1(void);
void timer_setup_2(void);
void timer_setup_3(void);
void timer_setup_4(void);
void timer_setup_5(void);
void timer_setup_6(void);
void ADC_setup(void);
void convert(void);
void generate_PWM(void);

volatile int result;
volatile int ref_ADC_value = 1643;	//12 bit value
volatile int R1 = 1000;	//units (ohm)
volatile float mm_per_OHM = .09975; // units (mm/ohm)
volatile float mm_per_OHM1 = .025;//.02693;
volatile float distance;	// units (mm)
volatile float distance1;	// units (mm)
volatile int PWM_value1;

volatile float coil_len = 25.4;	//units (mm)
volatile float norm_factor = (25.4 * 25.4) / 127;
volatile int old_result;	//for direction of movement
volatile int PWM_value;

int main (void){
	
	system_init();
 	clock_setup();
	port_setup();
	timer_setup_1();
	timer_setup_2();
	timer_setup_3();
	timer_setup_4();
	timer_setup_5();
	timer_setup_6();
	ADC_setup();
	
	while(1){}
	
}

void clock_setup(void){

	SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET | SYSCTRL_INTFLAG_DFLLRDY;
	SYSCTRL->OSC8M.bit.PRESC = 3;	//divide the clock by 8	so 1MHz still
	SYSCTRL->OSC8M.bit.ONDEMAND = 1;	//clock is off is no peripheral request
	SYSCTRL->OSC8M.bit.RUNSTDBY = 0;	//clock is off in sleep mode
	SYSCTRL->OSC8M.reg |= 1<<1;	//enable clock
	//SYSCTRL->OSC8M.bit.FRANGE = 2;	//yet another way to control manipulate the clock freq	
	
	SYSCTRL->OSC32K.bit.STARTUP = 0;	//start up asap
	SYSCTRL->OSC32K.bit.ONDEMAND = 1;	//clock is off if no peripheral request
	SYSCTRL->OSC32K.bit.RUNSTDBY = 1;	//clock is on in sleep mode
	SYSCTRL->OSC32K.bit.EN32K = 1;	//enable output
	SYSCTRL->OSC32K.reg |= 1<<1;	//enable clock
	
	GCLK->CTRL.bit.SWRST = 1;	//reset the generators
	while (GCLK->STATUS.bit.SYNCBUSY){}	//waiting for the reset to complete  
		
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(0) | GCLK_GENDIV_DIV(1);	//divide generator0 by 1
	
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_OE |
	GCLK_GENCTRL_SRC_OSC32K | GCLK_GENCTRL_RUNSTDBY;
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_OE |
	GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_RUNSTDBY;
	while (GCLK->STATUS.bit.SYNCBUSY){}	//waiting for sync to complete  
		
	GCLK->CLKCTRL.reg |= 0<<14;	//disable clock
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC4_TC5 | GCLK_CLKCTRL_GEN_GCLK0 | 1<<14;	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC6_TC7 | GCLK_CLKCTRL_GEN_GCLK0 | 1<<14;	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3 | GCLK_CLKCTRL_GEN_GCLK0 | 1<<14;	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC0_TCC1 | GCLK_CLKCTRL_GEN_GCLK0 | 1<<14;	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC | GCLK_CLKCTRL_GEN_GCLK1 | 1<<14;	//setup genclk for ADC
	while (GCLK->STATUS.bit.SYNCBUSY==1){}	//waiting for sync to complete  
		
	PM->CPUSEL.bit.CPUDIV = 0;	//divide CPU clock by 1	pg 15
	PM->APBASEL.bit.APBADIV = 0;	//divide apba bus by 1
	PM->APBBSEL.bit.APBBDIV = 0;	//divide apbb bus by 1 
	PM->APBCSEL.bit.APBCDIV = 0;	//divide apbc bus by 1
	PM->APBAMASK.reg |= 1<<3;	
	PM->APBCMASK.reg |= PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7 | PM_APBCMASK_TCC0;	
	PM->APBCMASK.reg |= 1<<16;	//enable the ADC APB
}

void port_setup(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	PortGroup *porA = &(por->Group[0]);
	
	
	//the potentiometer
	porA->PMUX[2].bit.PMUXO = 1;	//mux the ADC to pin PA05 (5=2*n+1)	AIN[5]
	porA->PINCFG[5].bit.PMUXEN =1;	//enable the MUX
	
	//the coil1
	//porA->PMUX[5].bit.PMUXE = 1;	//mux the ADC to pin PA10 (10=2*n)	AIN[18]
	//porA->PINCFG[10].bit.PMUXEN =1;	//enable the MUX
	
	porA->PMUX[10].bit.PMUXE = 5;	//mux the TCC0 wavegen PA20	PWM output
	porA->PINCFG[20].bit.PMUXEN = 1;	//enable the mux
	porA->PMUX[10].bit.PMUXO = 5;	//mux the TCC0 wavegen PA21	PWM output
	porA->PINCFG[21].bit.PMUXEN = 1;	//enable the mux

	
	//the coil2
	//porB->PMUX[2].bit.PMUXO = 1;	//mux the ADC to pin PB05 (5=2*n+1)	AIN[13]
	//porB->PINCFG[5].bit.PMUXEN =1;	//enable the MUX
	
	porB->PMUX[5].bit.PMUXE = 4;	//mux the TC5 wavegen PB10	PWM output
	porB->PINCFG[10].bit.PMUXEN = 1;	//enable the mux
	porB->PMUX[5].bit.PMUXO = 4;	//mux the TC5 wavegen PB11	PWM output
	porB->PINCFG[11].bit.PMUXEN = 1;	//enable the mux
	
	//the coil3
	//porB->PMUX[2].bit.PMUXE = 1;	//mux the ADC to pin PB04 (4=2*n)	AIN[12]
	//porB>PINCFG[4].bit.PMUXEN =1;	//enable the MUX
	
	porA->PMUX[9].bit.PMUXE = 4;	//mux the TC3 wavegen PA18	PWM output
	porA->PINCFG[18].bit.PMUXEN = 1;	//enable the mux
	porA->PMUX[9].bit.PMUXO = 4;	//mux the TC3 wavegen PA19	PWM output
	porA->PINCFG[19].bit.PMUXEN = 1;	//enable the mux
	
	//the coil4
	//porB->PMUX[3].bit.PMUXO = 1;	//mux the ADC to pin PB07 (7=2*n+1)	AIN[15]
	//porB->PINCFG[7].bit.PMUXEN =1;	//enable the MUX
	
	porB->PMUX[1].bit.PMUXE = 4;	//mux the TC6 wavegen PB02	PWM output
	porB->PINCFG[2].bit.PMUXEN = 1;	//enable the mux
	porB->PMUX[1].bit.PMUXO = 4;	//mux the TC6 wavegen PB02	PWM output
	porB->PINCFG[3].bit.PMUXEN = 1;	//enable the mux
	
	//the coil5
	//porB->PMUX[3].bit.PMUXE = 1;	//mux the ADC to pin PB06 (6=2*n)	AIN[14]
	//porB->PINCFG[6].bit.PMUXEN =1;	//enable the MUX
	
	porB->PMUX[0].bit.PMUXE = 4;	//mux the TC7 wavegen PB00	PWM output
	porB->PINCFG[0].bit.PMUXEN = 1;	//enable the mux
	porB->PMUX[0].bit.PMUXO = 4;	//mux the TC7 wavegen PB01	PWM output
	porB->PINCFG[1].bit.PMUXEN = 1;	//enable the mux
	
	//the coil6
	//porA->PMUX[5].bit.PMUXO = 1;	//mux the ADC to pin PA11 (11=2*n+1)	AIN[19]
	//porA->PINCFG[11].bit.PMUXEN =1;	//enable the MUX
	
	porB->PMUX[6].bit.PMUXE = 4;	//mux the TC4 wavegen PB12	PWM output
	porB->PINCFG[12].bit.PMUXEN = 1;	//enable the mux
	porB->PMUX[6].bit.PMUXO = 4;	//mux the TC4 wavegen PB13	PWM output
	porB->PINCFG[13].bit.PMUXEN = 1;	//enable the mux
}

void wait(volatile int d){
	int count = 0;
	while (count < d*1000){
		count++;
	}
}

//coil3
void timer_setup_1(void){
	Tc *tc = TC3;
	TcCount8 *tc3 = &tc->COUNT8;
	tc3->CTRLA.reg = 0;	//disable the TC3
	while(tc3->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tc3->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tc3->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tc3->CTRLA.bit.MODE = 1;	//8 bit mode	
	tc3->PER.reg = 0xff;
	tc3->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tc3->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc3->CC[0].reg = 0x80;
	tc3->CC[1].reg = 0x80;
	tc3->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tc3->STATUS.bit.SYNCBUSY){}	//wait for sync to complete	
	tc3->CTRLA.reg |= 1<<1;	//enable the TC3
	while(tc3->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
}

//coil6
void timer_setup_2(void){
	Tc *tc = TC4;
	TcCount8 *tc4 = &tc->COUNT8;
	tc4->CTRLA.reg = 0;	//disable the TC4
	while(tc4->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tc4->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tc4->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tc4->CTRLA.bit.MODE = 1;	//8 bit mode
	tc4->PER.reg = 0xff;
	tc4->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tc4->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc4->CC[0].reg = 0x80;
	tc4->CC[1].reg = 0x80;
	tc4->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tc4->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc4->CTRLA.reg |= 1<<1;	//enable the TC4
	while(tc4->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
}

//coil2
void timer_setup_3(void){
	Tc *tc = TC5;
	TcCount8 *tc5 = &tc->COUNT8;
	tc5->CTRLA.reg = 0;	//disable the TC5
	while(tc5->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tc5->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tc5->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tc5->CTRLA.bit.MODE = 1;	//8 bit mode
	tc5->PER.reg = 0xff;
	tc5->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tc5->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc5->CC[0].reg = 0x80;
	tc5->CC[1].reg = 0x80;
	tc5->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tc5->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc5->CTRLA.reg |= 1<<1;	//enable the TC5
	while(tc5->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
}

//coil4
void timer_setup_4(void){
	Tc *tc = TC6;
	TcCount8 *tc6 = &tc->COUNT8;
	tc6->CTRLA.reg = 0;	//disable the TC6
	while(tc6->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tc6->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tc6->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tc6->CTRLA.bit.MODE = 1;	//8 bit mode
	tc6->PER.reg = 0xff;
	tc6->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tc6->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc6->CC[0].reg = 0x80;
	tc6->CC[1].reg = 0x80;
	tc6->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tc6->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc6->CTRLA.reg |= 1<<1;	//enable the TC6
	while(tc6->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
}

//coil5
void timer_setup_5(void){
	Tc *tc = TC7;
	TcCount8 *tc7 = &tc->COUNT8;
	tc7->CTRLA.reg = 0;	//disable the TC7
	while(tc7->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tc7->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tc7->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tc7->CTRLA.bit.MODE = 1;	//8 bit mode
	tc7->PER.reg = 0xff;
	tc7->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tc7->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc7->CC[0].reg = 0x80;
	tc7->CC[1].reg = 0x80;
	tc7->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tc7->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tc7->CTRLA.reg |= 1<<1;	//enable the TC7
	while(tc7->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
}
//coil1 using TCC0
void timer_setup_6(void){
	Tcc *tcc = TCC0;
	tcc->CTRLA.reg |= 0<<1;	//disable the TC7
	while(tcc->SYNCBUSY.reg){}	//wait for sync of disable
	//tcc->WAVE.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
	tcc->PER.reg = 0xff;
	tcc->DRVCTRL.bit.INVEN7 = 1;//invert channel 1
	while(tcc->SYNCBUSY.reg){}	//wait for sync of disable
	tcc->CC[0].reg |= 0xA0;
	tcc->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tcc->SYNCBUSY.reg){}	//wait for sync of disable
	tcc->CTRLA.reg |= 1<<1;	//enable the TC7
	while(tcc->SYNCBUSY.reg){}	//wait for sync of disable
}


/*
void TC4_Handler(void){
	Port *port = PORT;
	PortGroup *porA = &(port->Group[0]);
	porA->DIRSET.reg = 1<<10;
	porA->OUTSET.reg = PORT_PA10;	//turn on power to sensor
	Tc *tc = TC4;
	TcCount8 *tcc = &tc->COUNT8;
	tcc->INTFLAG.bit.OVF = 1;	//clear the interrupt
}
*/


void ADC_setup(void){
	ADC->CTRLA.reg = 0<<1;	//disable so that we can reset
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for disable to complete
	ADC->CTRLA.bit.SWRST = 1;	//reset
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for reset to complete
	ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;	//internal reference = .5VDDann
	//ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 | ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_FREERUN | 0<<0 | ADC_CTRLB_CORREN;
	ADC->AVGCTRL.bit.SAMPLENUM = 3;	//take 8 samples for averaging
	ADC->AVGCTRL.bit.ADJRES = 3;	//divsion needed for averaging
	ADC->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT | 0<<0;	//for averaging need 16bit
	ADC->CTRLB.bit.PRESCALER = 0;	//no division
	ADC->CTRLB.bit.FREERUN  = 0;	//single shot conversion on
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for sync to complete
	ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN5;	//pin5=AIN5=pot
	ADC->INPUTCTRL.bit.GAIN = 0xF;	//gain = 1/2
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for sync to complete
	
	ADC->SWTRIG.bit.START = 1;	//start conversion needed for first conversion in freerun mode
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for sync to complete
	ADC->INTENSET.reg = ADC_INTENSET_RESRDY;	//setup interrupt when reg is ready to be read
	ADC->CTRLA.reg |= 1<<1;	//enable ADC
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for enable to complete
	NVIC->ISER[0] |= 1<<23;	//enable the NVIC handler

}


					
//potentiometer
void ADC_Handler(void){
   	result = ADC->RESULT.reg;
	ADC->SWTRIG.bit.START = 1;	//start conversion needed for first conversion in freerun mode
	convert();
}

void convert(void){
	volatile float V_diff;
	volatile float ref_V;
	volatile float value_V;
	volatile int  ref_OHM;
	volatile int value_OHM;
	volatile int OHM_travel;
			
	ref_V = 3.3 * ref_ADC_value / 4095;
	ref_OHM = ref_V * R1 / (3.3 - ref_V);
	value_V = 3.3 * result / 4095;
	value_OHM = value_V * R1 / (3.3 - value_V);
	OHM_travel = value_OHM - ref_OHM;
	distance = OHM_travel *mm_per_OHM1; //units (mm) distance of 0 means furthest away form coil
	
	distance1 = OHM_travel *mm_per_OHM1; //units (mm) distance of 0 means furthest away form coil

	//PWM_value = ((coil_len - distance) * (coil_len - distance)) / norm_factor;
	//PWM_value1 = ((coil_len - distance1) * (coil_len - distance1)) / norm_factor;

 	generate_PWM();
	//old_result = result;
}

void generate_PWM(void){
	
	Tc *t3 = TC3;
	TcCount8 *tc3 = &t3->COUNT8;
	Tc *t4 = TC4;
	TcCount8 *tc4 = &t4->COUNT8;
	Tc *t5 = TC5;
	TcCount8 *tc5 = &t5->COUNT8;
	Tc *t6 = TC6;
	TcCount8 *tc6 = &t6->COUNT8;
	Tc *t7 = TC7;
	TcCount8 *tc7 = &t7->COUNT8;
	Tcc *tcc = TCC0;
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	PortGroup *porA = &(por->Group[0]);
	

		//c2<-		
		if (distance >= 48 && distance < 50.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0xA0;
			tc5->CC[1].reg = 0xA0;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x80;
			tc7->CC[1].reg = 0x80;
			
		}
		//c1->
		else if (distance >= 50.8 && distance < 72){	//no current, duty cycle is 50%	
			tcc->CC[0].bit.CC = 0x60;
			tcc->CC[0].bit.CC = 0x60;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0xA0;
			tc5->CC[1].reg = 0xA0;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x80;
			tc7->CC[1].reg = 0x80;	
		}
		//c5<-
		else if (distance >= 72 && distance < 74.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x60;
			tcc->CC[0].bit.CC = 0x60;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0xA0;
			tc5->CC[1].reg = 0xA0;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
			
		}
		//c4->
		else if (distance >=74.8 && distance < 96){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x60;
			tcc->CC[0].bit.CC = 0x60;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0xA0;
			tc5->CC[1].reg = 0xA0;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
			
		}
		//c3<-
		else if (distance >= 96 && distance < 98.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x60;
			tcc->CC[0].bit.CC = 0x60;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0xA0;
			tc5->CC[1].reg = 0xA0;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
			
		}
		//c2->
		else if (distance >=98.8 && distance < 101.6){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x60;
			tcc->CC[0].bit.CC = 0x60;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
		}
		//c1-X
		else if (distance >= 101.6 && distance < 120){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
		}
		//c6<-
		else if (distance >= 120 && distance < 122.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0xA0;
			tc4->CC[1].reg = 0xA0;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0xA0;
			tc7->CC[1].reg = 0xA0;
		}
		//c5->
		else if (distance >= 122.8 && distance < 126){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0xA0;
			tc4->CC[1].reg = 0xA0;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x60;
			tc6->CC[1].reg = 0x60;
			tc7->CC[0].reg = 0x60;
			tc7->CC[1].reg = 0x60;
		}
		//c4-X
		else if (distance >= 126 && distance < 146.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0xA0;	
			tc3->CC[1].reg = 0xA0;
			tc4->CC[0].reg = 0xA0;
			tc4->CC[1].reg = 0xA0;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x60;
			tc7->CC[1].reg = 0x60;
		}
		//c3-X
		else if (distance >= 146.8 && distance < 149.6){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0xA0;
			tc4->CC[1].reg = 0xA0;
			tc5->CC[0].reg = 0x60;
			tc5->CC[1].reg = 0x60;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x60;
			tc7->CC[1].reg = 0x60;
		}
		//c2-X
		else if (distance >= 149.6 && distance < 170.8){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0xA0;
			tc4->CC[1].reg = 0xA0;
			tc5->CC[0].reg = 0x80;
			tc5->CC[1].reg = 0x80;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x60;
			tc7->CC[1].reg = 0x60;
		}
		//c6-X
		else if (distance >= 170.8 && distance < 173.6){	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0x80;
			tc5->CC[1].reg = 0x80;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x60;
			tc7->CC[1].reg = 0x60;
		}
		//c5-X
		else {	//no current, duty cycle is 50%
			tcc->CC[0].bit.CC = 0x80;
			tcc->CC[0].bit.CC = 0x80;	
			tc3->CC[0].reg = 0x80;	
			tc3->CC[1].reg = 0x80;
			tc4->CC[0].reg = 0x80;
			tc4->CC[1].reg = 0x80;
			tc5->CC[0].reg = 0x80;
			tc5->CC[1].reg = 0x80;
			tc6->CC[0].reg = 0x80;
			tc6->CC[1].reg = 0x80;
			tc7->CC[0].reg = 0x80;
			tc7->CC[1].reg = 0x80;
			
		}
	
	
	//if (distance > 25.4 || distance < 0){	//no current, duty cycle is 50%
		//tcc->CC[0].reg = 0x80;	
		//tcc->CC[1].reg = 0x80;
	//}
	//
	//if (distance1 > 25.4 || distance1 < 0){	//no current, duty cycle is 50%
		////tcc->CC[0].reg = 0x80;
		//tcc->CC[1].reg = 0x80;
	//}
	//
	//else{	//coil moving in
		//tcc->CC[0].reg = 128 + PWM_value;
		//tcc->CC[1].reg = 128 + PWM_value1;
	//}
	
	//direction
	//else if(old_result - result > 1){	//coil moving in, duty cycle {128-255}
		//tcc->CC[0].reg = 128 + PWM_value;
		//tcc->CC[1].reg = 128 + PWM_value;
	//}
	//else if(result - old_result > 1){	//coil moving out, duty cycle {0-127}
		//tcc->CC[0].reg = 127 - PWM_value;
		//tcc->CC[1].reg = 127 - PWM_value;
	//}
}


