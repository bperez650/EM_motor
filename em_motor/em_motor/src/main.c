#include <asf.h>


void clock_setup(void);
void wait(volatile int d);
void port_setup(void);
void timer_setup(void);
void ADC_setup(void);
void convert(void);
void generate_PWM(void);

volatile int result;
volatile int ref_ADC_value = 2523;	//12 bit value
volatile int R1 = 1000;	//units (ohm)
volatile float mm_per_OHM = .09975; // units (mm/ohm)
volatile float distance;	// units (mm)
volatile float coil_len = 25.4;	//units (mm)
volatile float norm_factor = (25.4 * 25.4) / 127;
volatile int old_result;	//for direction of movement
volatile int PWM_value;

int main (void){
	
	system_init();
 	clock_setup();
	port_setup();
	timer_setup();
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
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TC4_TC5 | GCLK_CLKCTRL_GEN_GCLK0 | 1<<14;	//setup genCLK for TC4
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC | GCLK_CLKCTRL_GEN_GCLK1 | 1<<14;	//setup genclk for ADC
	while (GCLK->STATUS.bit.SYNCBUSY==1){}	//waiting for sync to complete  
		
	PM->CPUSEL.bit.CPUDIV = 0;	//divide CPU clock by 1	pg 15
	PM->APBASEL.bit.APBADIV = 0;	//divide apba bus by 1
	PM->APBBSEL.bit.APBBDIV = 0;	//divide apbb bus by 1 
	PM->APBCSEL.bit.APBCDIV = 0;	//divide apbc bus by 1
	PM->APBAMASK.reg |= 1<<3;	//enable the GCLK clock DONT THINK NECESSARY they should be enabled by default pg 159
	PM->APBCMASK.reg |= PM_APBCMASK_TC4;	//enable the TC4 APB
	PM->APBCMASK.reg |= 1<<16;	//enable the ADC APB
}

void port_setup(void){
	Port *por = PORT;
	PortGroup *porB = &(por->Group[1]);
	PortGroup *porA = &(por->Group[0]);
	
	porB->PMUX[6].bit.PMUXE = 4;	//mux the TC wavegen PB12	PWM output
	porB->PINCFG[12].bit.PMUXEN = 1;	//enable the mux
	//porB->PINCFG[9].bit.DRVSTR = 1;	//high drive on PWM 
	porB->PMUX[6].bit.PMUXO = 4;	//mux the TC wavegen PB13	PWM output
	porB->PINCFG[13].bit.PMUXEN = 1;	//enable the mux
		
	porA->PMUX[5].bit.PMUXO = 1;	//mux the ADC to pin PA11 (11=2*n+1)	AIN[19]
	porA->PINCFG[11].bit.PMUXEN =1;	//enable the MUX
}

void wait(volatile int d){
	int count = 0;
	while (count < d*1000){
		count++;
	}
}

void timer_setup(void){
	Tc *tc = TC4;
	TcCount8 *tcc = &tc->COUNT8;
	tcc->CTRLA.reg = 0;	//disable the TC4
	while(tcc->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
	tcc->CTRLA.bit.PRESCALER = 0;	//divide by 1;
	tcc->CTRLA.bit.WAVEGEN = 2;	//normal PWM frequency per=period, CC1/CC0=compare value
	tcc->CTRLA.bit.MODE = 1;	//8 bit mode	
	tcc->PER.reg = 0xff;
	tcc->CTRLC.bit.INVEN1 = 1;//invert channel 1
	while(tcc->STATUS.bit.SYNCBUSY){}	//wait for sync to complete
	tcc->CC[0].reg = 0x0;
	tcc->CC[1].reg = 0xff;
	tcc->CTRLBSET.bit.ONESHOT = 0;	//turn off one shot mode
	while(tcc->STATUS.bit.SYNCBUSY){}	//wait for sync to complete	
	tcc->CTRLA.reg |= 1<<1;	//enable the TC4
	while(tcc->STATUS.bit.SYNCBUSY){}	//wait for sync of disable
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
	ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN19;	//pin0=AIN0=PA02, pin4=AIN4=PA04
	ADC->INPUTCTRL.bit.GAIN = 0xF;	//gain = 1/2
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for sync to complete
	
	ADC->SWTRIG.bit.START = 1;	//start conversion needed for first conversion in freerun mode
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for sync to complete
	ADC->INTENSET.reg = ADC_INTENSET_RESRDY;	//setup interrupt when reg is ready to be read
	ADC->CTRLA.reg |= 1<<1;	//enable ADC
	while (ADC->STATUS.bit.SYNCBUSY==1){}	//wait for enable to complete
	NVIC->ISER[0] |= 1<<23;	//enable the NVIC handler
	//ADC->OFFSETCORR.reg = 0b000000110100;	//shift down by 52, 2's comp
	//ADC->GAINCORR.reg =   0b100010100000;	//when corren is enabled it enables gain comp too, fractional
}

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
	distance = OHM_travel *mm_per_OHM; //units (mm) distance of 0 means furthest away form coil
	PWM_value = ((coil_len - distance) * (coil_len - distance)) / norm_factor;
	generate_PWM();
	old_result = result;
}

void generate_PWM(void){
	Tc *tc = TC4;
	TcCount8 *tcc = &tc->COUNT8;
	
	
	if (distance > 25.4 | distance < 0){	//no current, duty cycle is 50%
		tcc->CC[0].reg = 0x80;	
		tcc->CC[1].reg = 0x80;
	}
	else if(old_result - result > 1){	//coil moving in, duty cycle {128-255}
		tcc->CC[0].reg = 128 + PWM_value;
		tcc->CC[1].reg = 128 + PWM_value;
	}
	else if(result - old_result > 1){	//coil moving out, duty cycle {0-127}
		tcc->CC[0].reg = 127 - PWM_value;
		tcc->CC[1].reg = 127 - PWM_value;
	}
}


