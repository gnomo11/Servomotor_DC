/*                                             CONTROL PID DE POSICIÓN DE UN MOTOR CON ENCODER                              */
// ********************************************** PINES UTILIZADOS ******************************************************************************************************
const byte    encA = 2;                  // Entrada de la señal A del encoder en pin D2 INT0
const byte    encB = 3;                  // Entrada de la señal B del encoder en pin D3
const byte    PWMA = 5;                  // Salida PWM en pin D5
const byte    PWMB = 6;                  // Salida PWM en pin D6
const byte    PWMC = 9;                  // Salida PWM en pin D9

const byte    ledok    = 13;             // LED integrado en el Pin 13 del ArduinoNano. Enciende cuando se alcanza Setpoint
// ************************************************** Variables PID ***************************************************************************************************
unsigned long lastTime = 0,   SampleTime = 0;                    // Variables del tiempo de muestreo
double        Input    = 0.0, Setpoint   = 0.0;                  // Variables de posición del motor (pulsos del encorder -> input) y posición deseada (Setpoint)
double        ITerm    = 0.0, dInput     = 0.0, lastInput = 0.0; //     "     de error integral, error derivativo y posición anterior del motor
double            kp   = 0.0,      ki    = 0.0,        kd = 0.0; // Constantes: proprocional, integral y derivativa.
double        outMin   = 0.0, outMax     = 0.0;                  // Límites para no sobrepasar la resolución del PWM.
double        error    = 0.0, grados     = 0.0;                                    // Desviación o error entre la posición real del motor y la posición designada.
//short         signalA  = 0,   signalB =0;                        // Variables de un bit que me indican el estado de las señales del encoder
// **************************************************** Otras Variables *******************************************************************************************
volatile long pulsos   =  0;             // Variable que lleva la cuenta de los pulsos del encoder
byte          cmd      =  0;             // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
byte          pwm      =  0;             // Variable de 8 bits para el pwm (0-255)
// ****************************************************************************************************************************************************************

void setup(void)                        // configurar los pines de entrada/salida y el terminal serie
{
  Serial.begin(115200);                 // Configurar la velocidad en baudios del terminal serie
  
  pinMode(PWMA, OUTPUT);                // Declarar las dos salidas PWM para el control del motor (pin 5)
  pinMode(PWMB, OUTPUT);                // y pin 6
  pinMode(PWMC, OUTPUT);
  
  digitalWrite(PWMA, LOW);              // Ambas salidas se inicializan a cero
  digitalWrite(PWMB, LOW);
  digitalWrite(PWMC, LOW);              //Salida PWM "C" comienza en nivel bajo

  
 // TCCR0B = TCCR0B & B11111000 | 1;   // Configuración de la frecuencia del PWM para los pines 5 y 6 a (32kHz)
                                     // Podemos variar la frecuencia del PWM con un número de 1 (32kHz) hasta 7 (32Hz). El número que pongamos es un divisor de frecuencia. Min.=7, Max.=1
                                     // El control de motores funciona mejor con frecuencias del pwm grandes
                                     //Visitar https://forum.arduino.cc/index.php?topic=328714.0 para más info.

  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to 1 for PWM frequency of 31.37255 kHz -- éste controla al PWM C

 
  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // INT0 EXT --> En cualquier flanco ascendente o descendente
  //attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE); // en los pines 2 y 3 actúa la interrupción.
  pinMode(encB,INPUT); //Configurar pin como entrada digital
  
  // Acotación máxima y mínima; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). El PWM se convertirá a la salida en un valor absoluto, nunca negativo.
  outMax =  255.0;                      // Límite máximo del controlador PID
  outMin = -outMax;                     // Límite mínimo del controlador PID
  
  SampleTime = 30;                      // Se le asigna el tiempo de muestreo en milisegundos
  
  kp = 2.0;                             // Constantes PID iniciales. PPR del encoder = 4200 (16pulsos del canal A * ambos flancos (2) * 131.25 [ gearbox 131.25:1])
  ki = 0.01;                            
  kd = 10.0;
  
//  Setpoint = (double)pulsos;          // Para evitar que haga cosas extrañas al ponerse en marcha o después de resetear, igualamos los dos valores para que comience estando quieto el motor.
  Setpoint = 0.0;
 // imprimir(3);                          // Muestra las constantes de sintonización, el tiempo de muestreo y la posición por el terminal serie 
}

void loop(void)
{
  double Out = Compute();               // Llama a la función "Compute()" -> [control PID] y el resultado lo carga en la variable 'Out'
  
  // *********************************************** Control del Motor *************************************************
  if (error == 0.0)                     // Si el error es 0, parar el motor
  {
    digitalWrite(PWMA, LOW);            // Pone a 0 los dos pines del puente en H
    digitalWrite(PWMB, LOW);
    //digitalWrite(PWMC, LOW);
    digitalWrite(ledok, HIGH);          // Se enciende el led (pin 13) porque ya está en la posición deseada
  }
  else                                  // Si aún no se llega al Setpoint... (el sentido de giro lo determina el signo que contiene Out)
  {
    pwm = abs(Out);                     // Transfiere a la variable pwm el valor absoluto de Out

    analogWrite(PWMC,pwm);
    
    if (Out > 0.0)                      // Gira el motor en un sentido con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMB, LOW);          // Pone a 0 el segundo pin del puente en H.
      digitalWrite(PWMA, HIGH);           // Por el primer pin sale la señal PWM.
    }
    else                                // Gira el motor en sentido contrario con el PWM correspondiente a su posición.
    {
      digitalWrite(PWMA, LOW);          // Pone a 0 el primer pin del puente en H.
      digitalWrite(PWMB, HIGH);           // Por el segundo pin sale la señal PWM.
    }
  }
  // Recepción de datos para posicionar el motor, o modificar las constantes PID, o el tiempo de muestreo. Admite posiciones relativas y absolutas.
  if (Serial.available() > 0)           // Comprueba si ha recibido algún dato por el terminal serie
  {
    cmd = 0;                            // Por seguridad "limpiamos" cmd
    cmd = Serial.read();                // "cmd" guarda el byte recibido.
    if (cmd > 31)
    {
      byte flags = 0;                                      // Borramos la bandera que decide lo que hay que imprimir.
      if (cmd >  'Z') cmd -= 32;                           // Si una letra entra en minúscula la covierte en mayúscula.
      /*
      if (cmd == 'W') { Setpoint += 250.0;   flags = 2; }  // Si (por ejemplo) es la letra 'W' mueve 5 pasos hacia delante. Estos son movimientos relativos.
      if (cmd == 'Q') { Setpoint -= 250.0;   flags = 2; }  // Aquí son esos 5 pasos pero hacia atrás si se pulsa la letra 'Q'.
      if (cmd == 'S') { Setpoint += 525.0;   flags = 2; }  // Se repite lo mismo en el resto de las teclas.
      if (cmd == 'A') { Setpoint -= 525.0;   flags = 2; }
      if (cmd == 'X') { Setpoint += 1050.0;  flags = 2; }
      if (cmd == 'Z') { Setpoint -= 1050.0;  flags = 2; }
      if (cmd == '2') { Setpoint += 2100.0;  flags = 2; }
      */
      if (cmd == '1') { Setpoint -= 2100.0;  flags = 2; }
      if (cmd == '0') { Setpoint = 0.0;      flags = 2; }  // Ir a Inicio.
      
      // Decodificador para modificar las constantes PID
      switch(cmd)                                          // Si ponemos en el terminal serie, por ejemplo "P2.5 I0.5 D40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
      {                                                    // También se puede poner individualmente, por ejemplo "P5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
        case 'P': kp  = Serial.parseFloat();        flags = 1; break; // Carga las constantes y presenta en el terminal serie los valores de las variables que hayan sido modificadas.
        case 'I': ki  = Serial.parseFloat();        flags = 1; break;
        case 'D': kd  = Serial.parseFloat();        flags = 1; break;
        case 'T': SampleTime = Serial.parseInt();   flags = 1; break;
        case 'G': Setpoint   = Serial.parseFloat(); flags = 2; break; // Esta línea permite introducir una posición absoluta. Ex: G23000 (y luego enter) e irá a esa posición.
        case 'K':                                   flags = 3; break;
      }
      if (flags == 2) digitalWrite(ledok, LOW); // Cuando entra una posición nueva se apaga el led y no se volverá a encender hasta que el motor llegue a la posición que le hayamos designado.
      
    //  imprimir(flags);
    }
  } 
 }

// Cálculo PID.
double Compute(void)
{
   unsigned long now = millis();                  // Toma el número total de milisegundos que hay en ese instante.
   unsigned long timeChange = (now - lastTime);   // Resta el tiempo actual con el último tiempo que se guardó (esto último se hace al final de esta función).
   
   if(timeChange >= SampleTime)                   // Si se cumple el tiempo de muestreo entonces calcula la salida.
   {
     Input  = (double)pulsos;                   // Lee el valor del encoder óptico. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
     grados = Input*(360.0/4200.0);
     Serial.print("\n");
     Serial.print(grados);
     Serial.print("\n");    //imprime el valor en grados de la flecha
     error  = (Setpoint - Input)  * kp;           // Calcula el error proporcional.
     dInput = (Input - lastInput) * kd;           // Calcula el error derivativo.
     
     // Esta línea permite dos cosas: 1) Suaviza la llegada a la meta. 2) El error integral se auto-ajusta a las circunstancias del motor.
     //if (dInput == 0.0)  ITerm += (error * ki); else ITerm -= (dInput * ki);
     // Acota el error integral para eliminar el "efecto windup".
    // if (ITerm > outMax) ITerm = outMax; else if (ITerm < outMin) ITerm = outMin;
      ITerm += (ki * error);
      if (abs(ITerm) > outMax) ITerm = 0.0; 
       
     double Output = error + ITerm - dInput;      // Suma todos los errores, es la salida del control PID.
     if (Output > outMax) Output = outMax; else if (Output < outMin) Output = outMin; // Acota la salida para que el PWM pueda estar entre outMin y outMax.
     
     lastInput = Input;                           // Se guarda la posición para convertirla en pasado.
     lastTime  = now;                             // Se guarda el tiempo   para convertirlo en pasado.
     
     return Output;                               // Devuelve el valor de salida PID.
   }
}

// INT0 Conteo de pulsos del encoder 4200 ppr
void encoder(void)
{
 if(digitalRead(encA) == digitalRead(encB)){pulsos++;} //giro izq
 else{ pulsos--; } //giro derecha
}
