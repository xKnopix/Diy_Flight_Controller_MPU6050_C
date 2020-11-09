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
#define MAXPITCHANGLE 24.0
#define MAXROLLANGLE 24.0

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
int8_t bufferMPU[14];
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
//						Roll,Pitch,Yaw	0.0025				
float p_Settings [3] = {(1/102400),(1/102400),0.15};
float i_Settings [3] = {0,0,0.0001};
float d_Settings [3] = {217,217,0};
float max_Settings [3] = {400,400,400};
float i_Mem [3] ;
float pid_soll[3];
float pid_ist[3];
float d_last_error [3];
float pid_error[3];
float pid_final [3];

int pwm_Motor[4] = {1000,1000,1000,1000};

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
	
	for (int i = 0; i<=13; i++){
		if (i<13){
			bufferMPU[i] = (int)TWIM_ReadAck();		//bufferMPU[0] acc_x_h//bufferMPU[1] acc_x_l//bufferMPU[2] acc_y_h//bufferMPU[3] acc_y_l//bufferMPU[4] acc_z_h//bufferMPU[5] acc_z_l//bufferMPU[6] tmp_h//bufferMPU[7] tmp_l//bufferMPU[8] gyro_x_h//bufferMPU[9] gyro_x_l//bufferMPU[10] gyro_y_h//bufferMPU[11] gyro_y_l//bufferMPU[12] gyro_z_h	//bufferMPU[12] gyro_z_l
		}else{
			bufferMPU[i] = (int)TWIM_ReadNack();
		}
	}
			
	acc_x = (bufferMPU[0]<<8)+bufferMPU[1];
	acc_y = (bufferMPU[2]<<8)+bufferMPU[3];
	acc_z = (bufferMPU[4]<<8)+bufferMPU[5];
	
	gyro_x = (bufferMPU[8]<<8)+bufferMPU[9];
	gyro_y = (bufferMPU[10]<<8)+bufferMPU[11];
	gyro_z = (bufferMPU[12]<<8)+bufferMPU[13];
}
void Calc_MPU(){
	angle_pitch += (float)gyro_x * (1/(65.5*400));
	angle_roll += (float)gyro_y * (1/(65.5*400));
	
	angle_pitch += angle_roll * sin((float)gyro_z * (1/(65.5*400)));
	angle_roll -= angle_pitch * sin((float)gyro_z * (1/(65.5*400)));
	
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
	pid_ist[1] = pid_ist[1] * 0.95 + angle_pitch * 0.05;   //Take 90% of the output pitch value and add 10% of the raw pitch value
	pid_ist[0] = pid_ist[0] * 0.95 + angle_roll * 0.05;                                          //Set the IMU started flag
}


//Funktionen PID
float mapInput(int x, float input_from, float input_to, float output_from, float output_to){
	//			  m	*x +b
	float m = (output_to-(output_from))/(input_to-(input_from));
	float b = (output_from-m*input_from);
	return m*x + (b);
}
void Calc_Pid(){
	//Roll
	pid_soll[0] = mapInput((int)ch1pwm,1000,2000,-(MAXROLLANGLE),MAXROLLANGLE);
	//Pitch
	pid_soll[1] = mapInput((int)ch2pwm,1000,2000,MAXPITCHANGLE,-(MAXPITCHANGLE));//Inverted because if quadcopter is moving forward pitch will be 0 to -12
	//Yaw
	pid_soll[2] = ch4pwm;
	pid_ist[2] = mapInput((int)gyro_z,-32750,32750,1000,2000);
	if (pid_ist[2]>=1480 && pid_ist[2]<=1520){ //Zittern ausgleichen
		pid_ist[2] =1500;
	}
	//Loop 3x;  0 = Roll 1 =Pitch 2 =Yaw; Berechnung der PID Werte
	for (int i = 0; i<3; i++){
		pid_error[i] = pid_ist[i]*10 - pid_soll[i]*10; //ERROR berechnen
		//i_Mem[0] = Error speicher von Roll
		i_Mem[i] += i_Settings[i] * pid_error[i]; //I Roll berechnen
		//Begrenzt i
		if(i_Mem[i] > max_Settings[i])i_Mem[i] = max_Settings[i];
		else if(i_Mem[i] < (max_Settings[i] * -1)) i_Mem[i] = (max_Settings[i]*-1);
		//Berechne PID Wert
		pid_final[i] = p_Settings[i] * pid_error[i] + i_Mem[i] + d_Settings[i] * (pid_error[i] - d_last_error[i]);
		d_last_error[i] = pid_error[i];
		//Limitiere PID Wert
		if(pid_final[i] > max_Settings[i])pid_final[i] = max_Settings[i];
		else if(pid_final[i] < (max_Settings[i] * -1)) pid_final[i] = (max_Settings[i]*-1);
	}
}
void limit_Motor(void){
	//Verhindert dass die Motoren an gehen wenn sick auf 0 ist
	if(ch3pwm <1020){
		for (int i = 0; i<4;i++){
			pwm_Motor[i] = 1000;
		}
		}else{
		//limitiert den maximalen output auf 2000ms
		for(int i =0; i<4;i++){
			if(pwm_Motor[i]>2000){
				pwm_Motor[i] =2000;
			}
		}
		//Verhindert dass die motoren ausgehen wenn man am fliegen ist
		for (int i = 0; i <4;i++){
			if (pwm_Motor[i] <1100){
				pwm_Motor[i] =1100;
			}
		}
	}
}

void Calc_Motor_Speed(){
	//ch3pwm = speed; pid_final[0] = Pid Roll; pid_final[1] = Pid Pitch; pid_final[2] = Pid Yaw
	pwm_Motor[0] = ch3pwm - pid_final[0] - pid_final[1] - pid_final[2];
	pwm_Motor[1] = ch3pwm + pid_final[0] - pid_final[1] + pid_final[2];
	pwm_Motor[2] = ch3pwm + pid_final[0] + pid_final[1] - pid_final[2];
	pwm_Motor[3] = ch3pwm - pid_final[0] + pid_final[1] + pid_final[2];
	
	limit_Motor();
}


//Funktionen PWM Generieren
void Gen_PWM(int M1_Val, int M2_Val, int M3_Val, int M4_Val){
		timeGenPWMStart = micros();
		PORTD |= ((1<<5) | (1<<6) | (1<<7)); // setzt bits PD5,PD6,PD7
		PORTB |= (1<<0); //setzt bit PB0
		while (micros() - timeGenPWMStart <=1000){//Hier kann man noch was machen max 1 ms
			MPU_Read();
			Calc_MPU();
			Calc_Pid();
			Calc_Motor_Speed();
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
		PORTD &= ~((1<<5) | (1<<6)|(1<<7));
		PORTB &= ~(1<<0);
		
}

//Main
int main(void)
{
	SetupBaud();
	PCI_Setup();
	Setup_Timer_1();
	TWIM_Init(400000);
	MPU_Init();
	
	//Ports I/O ???? noch genauer definieren 
	DDRB = 0x00; //Setzt Port D auf Augang
	DDRD = 0x00; // Setzt Port D auf 0 ==> Port D = Eingang
	DDRC = 0x00; //Setzt Port C auf 0 ==> input
	
	looptimer = micros();
	while (1) {
		
		ltoa((long)pid_ist[1], INPUT,10);
		serialWrite(INPUT);
		serialWrite("\n");
		Gen_PWM(ch3pwm,ch3pwm,ch3pwm,ch3pwm);
		/*Gen_PWM((int)pwm_Motor[0],(int)pwm_Motor[1],(int)pwm_Motor[2],(int)pwm_Motor[3]);*/
		while(micros() - looptimer <2500);                                 //Wait until the loop_timer reaches 2500us (400Hz) before starting the next loop
		looptimer = micros();
	}
}

