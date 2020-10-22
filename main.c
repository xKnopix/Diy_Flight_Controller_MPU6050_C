/*//////////PIN_CONNECTION///////////
+CH1 ==> PC1 ==> PCINT8
+CH2 ==> PC2 ==> PCINT9
+CH3 ==> PD3 ==> PCINT10
+CH4 ==> PD4 ==> PCINT11
+
+PWMM1 ==>PD5
+PWMM2 ==> PB6
+PWMM3 ==> PD7
+PWMM4 ==> PB0
+
+SDA ==> PC4
+SCL ==> PC5
+
///////////////////////////////////*/
//CPU Geschwindigkeit
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
///////////////////////////////////
/////////MAXANGLE FROM INPUT///////
//////////////////////////////////
#define MAXPITCHANGLE 12
#define MAXROLLANGLE 12 

///////////////////////////////////
/////////Serial////////////////////
//////////////////////////////////
#define BAUD 9600UL
#define UBRR0_VAL ((F_CPU/(16*BAUD))-1)

///////////////////////////////////
///////////TX Buffer///////////////
//////////////////////////////////
#define TX_BUFFER_SIZE (2^32)

//Variablen MPU6050
int16_t acc_x,acc_y,acc_z,temp,gyro_x,gyro_y,gyro_z;
int8_t acc_x_l,acc_y_l,acc_z_l, acc_x_h,acc_y_h,acc_z_h,temp_h, temp_l,
		gyro_x_h,gyro_y_h,gyro_z_h,gyro_x_l,gyro_y_l,gyro_z_l;
long acc_total_vector;
long looptimer;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc, angle_pitch_output,angle_roll_output;
int set_gyro_angles = 0;
//Variablen für empfanfen von PWM
unsigned long timems= 0;
unsigned long timer1 = 0, timer2 = 0, timer3 = 0, timer4 = 0, timer5 = 0, timer6 = 0;
unsigned long ch1pwm = 0, ch2pwm, ch3pwm, ch4pwm, ch5pwm, ch6pwm;
int state1 = 0,state2 = 0,state3 = 0,state4 = 0,state5 = 0,state6 =0;
//TX_Buffer_Variablen
char serialBuffer[TX_BUFFER_SIZE];
uint8_t serialReadPos = 0;
uint8_t serialWritePos = 0;
//SEND BUFFER
char INPUT[(2^32)];
//Variablen Gen PWM
int counterPWM = 0;
unsigned long timeGenPWMStart = 0;

//Variablen PID
//Roll
float pid_p_gain_roll = 0;
float pid_i_gain_roll = 0;
float pid_d_gain_roll = 0;
int pid_max_roll = 400;
//Pitch
float pid_p_gain_pitch = 0;
float pid_i_gain_pitch = 0;
float pid_d_gain_pitch = 0;
int pid_max_pitch = 400;
//Yaw
float pid_p_gain_yaw = 0;
float pid_i_gain_yaw = 0;
float pid_d_gain_yaw = 0;
int pid_max_yaw = 400;
//Funktionen
//Funktionen Serielle Kommunikation
void SetupBaud(void){
	UBRR0H = (UBRR0_VAL>>8);
	UBRR0L = UBRR0_VAL;
	UCSR0B = (1<<TXEN0)  | (1<<TXCIE0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
}
void Bitwise(void){
	PORTD |= ((1<<5) | (1<<6));  // setzen
	_delay_ms(1000);
	PORTD &= ~(1<<5); //löschen
	_delay_ms(1000);
	//^ exclisive or ==> bittoggeln PORTD ^= (1<<5);
}
void appendSerial(char c){ //add char to Buffer
	serialBuffer[serialWritePos]  =c;
	serialWritePos++;
	
	if(serialWritePos >= TX_BUFFER_SIZE){
		serialWritePos = 0;
	}
}
void serialWrite(char c[]){
	for (uint8_t i = 0; i < strlen(c); i++){
		appendSerial(c[i]);
	}
	if (UCSR0A & (1<<UDRE0)){
		UDR0 = 0;
	}
	
}
ISR(USART_TX_vect){
	if (serialReadPos != serialWritePos){
		UDR0 = serialBuffer[serialReadPos];
		serialReadPos++;
		
		if (serialReadPos >= TX_BUFFER_SIZE){
			serialReadPos = 0;
		}
	}
}

//Funktionen PWM Read
void PCI_Setup(void){
	PCICR |=  (1<<PCIE1); //aktiviert Pin Change interupts
				// CH4			CH3					CH2			CH1
	PCMSK1 |= ((1<<PCINT11) | (1<<PCINT10) | (1<<PCINT9) | (1<<PCINT8)); //Aktiviert PIN Change an PCINT 0 bis PCINT3

	serialWrite("PCI Complete");
	_delay_ms(100);
}
void Setup_Timer_1(void){//Timer 0 einstellen (Für PWM Reading)
	// reset
	TCCR1A = 0; // set TCCR1A register to 0
	TCCR1B = 0; // set TCCR1B register to 0
	TCNT1  = 0; // reset counter value
 
	 OCR1A = 1999; // compare match register 1999 Entspricht 1 MS
 
	// set prescaler to 8
	 TCCR1B |= (1 << CS11);
 
	TCCR1B |= (1 << WGM12); // turn on CTC mode
	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
 
	sei(); // allow interrupts
	serialWrite("Timer Complete");
	_delay_ms(100);
}
ISR(TIMER1_COMPA_vect) { // function which will be called when an interrupt occurs at timer 1
	timems++;
}
unsigned long mills(){
	return timems;
}
unsigned long micros(){
	return (timems*1000)+(TCNT1/2);
}
ISR(PCINT1_vect){ //Wird aufgerufen wenn ein Pin toggelt (PCINT 0 bis PCINT 3)
	//CH1
	unsigned long acc_mus = micros();
	if (state1 == 0 && PINC &(1<<0)){ // state abfragen und PINB0 abfragen
		state1 = 1;
		timer1 = acc_mus;
		}else if(state1 ==1 &&  !(PINC &(1<<0))){
		state1 = 0;
		ch1pwm = (acc_mus -timer1);
	}
	//CH2
	if (state2 == 0 && PINC &(1<<1)){ // state abfragen und PINB1 abfragen
		state2 = 1;
		timer2 = acc_mus;
		}else if(state2 ==1 && !(PINC &(1<<1))){
		state2 = 0;
		ch2pwm = (acc_mus -timer2);
	}
	//CH3
	if (state3 == 0 && PINC &(1<<2)){ // state abfragen und PINB1 abfragen
		state3 = 1;
		timer3 = acc_mus;
		}else if(state3 ==1 && !(PINC &(1<<2))){
		state3 = 0;
		ch3pwm = (acc_mus -timer3);
	}
	//CH4
	if (state4 == 0 && PINC &(1<<3)){ // state abfragen und PINB1 abfragen
		state4 = 1;
		timer4 = acc_mus;
		}else if(state4 ==1 && !(PINC &(1<<3))){
		state4 = 0;
		ch4pwm =(acc_mus -timer4);
	}

}

//Funktionen I2C to MPU6050
void MPU_Init(){
	TWIM_Start(0x68, 0); //0 ==> writing to slave
	TWIM_Write(0x6B);	 // Send request for starting register
	TWIM_Write(0x00);    //send to Register 0x6B value 0x00 ==> activates MPU6050
	TWIM_Stop();		 // end transfer
	//Setup Accelerometer (+/-8g)
	TWIM_Start(0x68, 0); //0 ==> writing to slave
	TWIM_Write(0x1C);	 // Send request for starting register
	TWIM_Write(0x10);    //send to Register 0x1C value 0x10
	TWIM_Stop();		 // end transfer
	//Setup Gyro (500dps full range)
	TWIM_Start(0x68, 0); //0 ==> writing to slave
	TWIM_Write(0x1B);	 // Send request for starting register
	TWIM_Write(0x08);    //send to Register 0x1B value 0x08
	TWIM_Stop();		 // end transfer
}
void MPU_Read(){
	
		TWIM_Start(0x68,0); //Send start sequence to MPU6050
		TWIM_Write(0x3B);   //Send starting register
		TWIM_Stop();		//stops communication
		TWIM_Start(0x68,1);  //Send request to read 1 ==> read		
		
		acc_x_h = (int)TWIM_ReadAck();
		acc_x_l = (int)TWIM_ReadAck();
		acc_y_h = (int)TWIM_ReadAck();
		acc_y_l = (int)TWIM_ReadAck();
		acc_z_h = (int)TWIM_ReadAck();
		acc_z_l = (int)TWIM_ReadAck();
		temp_h = (int)TWIM_ReadAck();
		temp_l = (int)TWIM_ReadAck();
		gyro_x_h = (int)TWIM_ReadAck();
		gyro_x_l = (int)TWIM_ReadAck();
		gyro_y_h = (int)TWIM_ReadAck();
		gyro_y_l = (int)TWIM_ReadAck();
		gyro_z_h = (int)TWIM_ReadAck();
		gyro_y_l = (int)TWIM_ReadNack();
		
		acc_x = (acc_x_h<<8)+acc_x_l;
		acc_y = (acc_y_h<<8)+acc_y_l;
		acc_z = (acc_z_h<<8)+acc_z_l;
		
		gyro_x = (gyro_x_h<<8)+gyro_x_l;
		gyro_y = (gyro_y_h<<8)+gyro_y_l;
		gyro_z = (gyro_z_h<<8)+gyro_z_l;
}
void Calc_MPU(){
	angle_pitch += (float)gyro_x * 0.0000611;
	angle_roll += (float)gyro_y * 0.0000611;
	
	angle_pitch += angle_roll * sin((float)gyro_z * 0.000001066);
	angle_roll -= angle_pitch * sin((float)gyro_z * 0.000001066);
	
	acc_total_vector = sqrt(((float)acc_x*(float)acc_x)+((float)acc_y*(float)acc_y)+((float)acc_z*(float)acc_z));
	angle_pitch_acc = asin(((float)acc_y/acc_total_vector))* 57.296;       //Calculate the pitch angle
	angle_roll_acc = asin(((float)acc_x/acc_total_vector))* -57.296;
	
	angle_pitch_acc -= -0.5;                                              //Accelerometer calibration value for pitch
	angle_roll_acc -= -0.4;
	
	if(set_gyro_angles){                                                 //If the IMU is already started
		angle_pitch = angle_pitch * 0.97 + angle_pitch_acc * 0.03;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
		angle_roll = angle_roll * 0.97 + angle_roll_acc * 0.03;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
	}
	else{                                                                //At first start
		angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
		angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
		set_gyro_angles = 1;  
		}
		
	angle_pitch_output = angle_pitch_output * 0.95 + angle_pitch * 0.05;   //Take 90% of the output pitch value and add 10% of the raw pitch value
	angle_roll_output = angle_roll_output * 0.95 + angle_roll * 0.05;                                          //Set the IMU started flag
}

//Funktionen PWM Generieren
void Gen_PWM(int M1_Val, int M2_Val, int M3_Val, int M4_Val){//Gernerates 50 Herz PWM signal 
	// Ein Loop == 4 ms ==> 50 HZ ==> 20ms ==> in jedem 5. lauf wird ein neues signal erzeugt
	counterPWM++;
	if(counterPWM >= 5){// in jedem 5. durchlauf ==> 50 HZ
		timeGenPWMStart = micros();
		PORTD |= ((1<<5) | (1<<6) | (1<<7)); // setzt bits PD5,PD6,PD7
		PORTB |= (1<<0); //setzt bit PB0
		while (micros() - timeGenPWMStart <=1000){//Hier kann man noch was machen max 1 ms
			
		}
		while (micros() - timeGenPWMStart <=2000){//die nächsten 2 ms macht er die Signale
			if(micros() - timeGenPWMStart >=M1_Val){
				PORTD &= ~(1<<5); //löscht bit an PD5
			}
			if (micros() - timeGenPWMStart >=M2_Val){
				PORTD &= ~(1<<6); //löscht bit an PD6
			}
			if (micros() - timeGenPWMStart >=M3_Val){
				PORTD &= ~(1<<7); //löscht bit an PD7
			}
			if (micros() - timeGenPWMStart >=M4_Val){
				PORTB &= ~(1<<0); //löscht bit an PB0
			}
		}
		counterPWM= 0;
	}
}

//Funktionen PID
int mapInput(int input_start, int input_end, int output_start, int output_end, int input){
	return (output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start));
}
void Calc_Pid(){
	int soll_roll = mapInput(1000,2000,-12,12,(int)ch1pwm);
	int soll_pitch = mapInput(1000,2000,-(MAXPITCHANGLE),MAXPITCHANGLE,(int)ch2pwm);
	itoa((int)soll_roll, INPUT, 10);
	serialWrite(INPUT);
	itoa((int)soll_pitch, INPUT, 10);
	serialWrite(INPUT);
	serialWrite("\n");
}

//Main
int main(void)
{
	//init Urat
	SetupBaud();
	//Init Pin Change
	PCI_Setup();
	//Setup Timer 0
	Setup_Timer_1();
	//Setup I2C
	TWIM_Init(400000);
	MPU_Init();
	
	//Ports I/O
	DDRB = 0xFF; //Setzt Port D auf Augang
	DDRD = 0xFF; // Setzt Port D auf 0 ==> Port D = Eingang
	DDRC = 0x00; //Setzt Port B auf 0 ==> input
	
	sei();		//Aktiviert Interrupts
	serialWrite("loop");
	looptimer = micros();
	while (1) {
		if (counterPWM <5){// Neuer Winkel wird nur dann berechnet wenn kein PWM erzeugt wird
		MPU_Read();
		Calc_MPU();
		}
		Calc_Pid();
		Gen_PWM(ch3pwm,ch3pwm,ch3pwm,ch3pwm);
		 while(micros() - looptimer <4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
		 looptimer = micros();
	}
}

