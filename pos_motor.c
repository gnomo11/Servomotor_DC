//CONTROL DE POSCION DE UN MOTOR DC
//PWM

#include <18F4550.h>
#fuses HSPLL,NOWDT,NOPROTECT,NOLVP,NOBROWNOUT,NODEBUG,USBDIV,PLL2,CPUDIV1,VREGEN
#use delay(clock=48000000)
//#use rs232(baud=19200, parity = N, xmit=PIN_C6, rcv=PIN_C7, bits=8)

#define LCD_ENABLE_PIN PIN_B7 // Definición del RB5 para el E del LCD
#define LCD_RW_PIN PIN_B6 // Definición del RB6 para el R/W del LCD
#define LCD_RS_PIN PIN_B5 // Definición del RB7 para el RS del LCD
#define LCD_DATA4 PIN_D4 // Definición del RD4 para el DB4 del LCD
#define LCD_DATA5 PIN_D5 // Definición del RD5 para el DB5 del LCD
#define LCD_DATA6 PIN_D6 // Definición del RD6 para el DB6 del LCD
#define LCD_DATA7 PIN_D7 // Definición del RD7 para el DB7 del LCD
#include <lcd.c> // Se incluye al proyecto la librería lcd.c

#define gira_derecha PIN_B3
#define gira_izquierda PIN_B4
#define signal_A PIN_B0
#define signal_B PIN_B1

signed long long pulsos=0;
short a=0;
unsigned long sp_g=180, control=0;
signed long long grados=0, sp_pulsos=0;

signed long long grados_1=0;

float k=0.8, tao=100.5, theta=30.0;
float kp,ti,td,q0,q1,q2;
float e=0.0, e_1=0.0, e_2=0.0, u=0.0, u_1=0.0, T=1.0;
int32 inc=0;
short ida=0;
float TsMA,Wn,P1,P2;

void ziegler_nichols(void);

//void definir_giro(long vueltas);

//const unsigned long ppr=4192; //748

void main(void) {

	//setup_oscillator(OSC_8MHZ|OSC_INTRC);
	enable_interrupts(GLOBAL); // Habilitación Interrupción global;//HABILITAR INT_GLOBALES
	enable_interrupts(INT_EXT);//HABILITAR INT_EXT_0
	ext_int_edge(0,H_TO_L); //INT_EXT0:INT POR FLANCO DE BAJADA
	

	output_drive(gira_derecha);//Salida RB3
	output_drive(gira_izquierda);//Salidas RB4
	output_float(signal_A);//señal A del enconder
	output_float(signal_B);//señal B del encoder

	setup_timer_2(T2_DIV_BY_16,255,1); // PR2=224, Tpwm=720us
	setup_ccp1(CCP_PWM); // CCP1 en modo PWM
	set_pwm1_duty(0);
 
	ziegler_nichols();
	lcd_init(); //Inicializa LCD
	
//GRADOS --> PULSOS
	sp_pulsos=(sp_g*2096)/180;


	grados_1=0;

while(true) {
 		
//PULSOS --> GRADOS:

		grados=(pulsos*180)/2096; //desplegar en grados en LCD
		
		
		e=1*(sp_g - grados);

		if(e>0){output_high(gira_derecha);
  			output_low(gira_izquierda);}

	//	else if(e==0){u_1=0;}

		else if (e<=0) {
	output_low(gira_derecha);
  	output_high(gira_izquierda);
		}
	

	
	u = u_1 + (q0*e) + (q1*e_1) + (q2*e_2);

//	u= u_1 + (0.030*e_1) - (0.200*(grados-grados_1));
//30 -50
	
	if (u>=1000){u=1000;}
		

	if (u<=0){
	u=0;
	}

	control = u;

	e_2 = e_1;
	e_1 = e;
	u_1 = u;
//	grados_1=grados;

	set_pwm1_duty(control);	
/*
		if(inc==1000){
			
			inc=0;

			if (sp_g<=0){ida=1;}
			else if (sp_g>=180) {ida=0;}
		
			if(ida==1){
				sp_g=sp_g+20;
			}

			else {

			sp_g=sp_g-20;
			}
		}

*/
		
		lcd_gotoxy(1,1);
		printf(lcd_putc,"S.P: ");
		printf(lcd_putc,"%lu",sp_g);

		printf(lcd_putc," u: ");
		printf(lcd_putc,"%lu",control);

		lcd_gotoxy(1,2);
		printf(lcd_putc,"POS:");
   		printf(lcd_putc,"%ld",grados);

		printf(lcd_putc," e:");
   		printf(lcd_putc,"%2.2f",e);

		delay_ms(1);

	}
}



#int_ext 
void contar_pulsos(void){

	if(input(signal_B)==input(signal_A)){pulsos++;} //GIRO IZQ
    else{pulsos--;} //GIRO DER
        
        
//	Configurar para leer tanto en flanco de subida por flanco de bajada
	if(a==0){ext_int_edge(0,L_TO_H);a=1;}
	else {ext_int_edge(0,H_TO_L);a=0;}

        clear_interrupt(INT_EXT); //limpiar bandera de interrupción
}




void ziegler_nichols(void){
/*
	kp = (1.2*tao)/k*theta;
	ti = 2*theta;
	td = 0.5*theta;
	
*/


	TsMA=5*tao;                     //Tiempo deseado en Lazo Cerrado    
   Wn=3/(TsMA);               //Frecuencia natural del sistema
   
   //Ubicación de 2 Polos reales
   P1=Wn+Wn;
   P2=Wn*Wn;
   
  	kp=(P1*tao-1)/k;        //Calculo de Kc
    ti=(k*kp)/(P2*tao);     //Calculo de ti
    td=0;


	//kp=0.0931;
	//ki=0.0178;
 
	q0 = kp*(1+(T/(2*ti))+(td/T));
	q1 = -kp*(1-(T/(2*ti))+((2*td)/T));
	q2 = (kp*td)/T; 


}




	




