/*
   *********  AUTOR: ALIENNEO  *********
   *********  DATE: 07/05/2013  ********
   ***********  VERSION: 1.1  **********
     ARDUINO MEGA, control de motores correcto. Falta modificar
     el PID para mejorarlo  *********
     TIMER4 Y TIMER5 PARA MOTORES Y TIMER3 PARA SERVO(MODIFICADO EN LA LIBRERIA SERVO.H)
     FUNCIONA CORRECTO FALTA CONTROL DE RAM
*/
#include <Wire.h>
#include <Servo.h> //libreria para los servos



#define DEBUG 1 //#0 nada #1 #1
#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0
#define RAD_TO_DEG 57.295779
#define BAUDIOS  115200
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define ZERO_SPEED 65536  //  65535 se trata de la frecuencia máxima en 8bits
#define MAXIMA_ACELERACION         580 // MAX RECOMMENDED VALUE: 8) (default:7)
#define MAX_ANG -5.8
#define VEL_MAX   500


bool Robot_tumbado = false;           // FLAG NO UTILICADO
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];
long tiempo_valor;
int16_t velocidad_Motor_A;
int16_t velocidad_Motor_B;
int8_t  dir_Motor_A;
int8_t  dir_Motor_B;          // Actual direction of steppers motors

float Kp =   5;
float Ki =   500;
float Kd =   0.5;
float Kps =  0.01;
float Kis =  100;
float Kds =  0.01;

float integralSum, anguloPID_errorSum;
float errorVelociadAnterior;
float vel1 , vel2;
float velocidadMedia;
float velocidad;
float giro;
int velocidadDeseada;
float tiempoActual, tiempo;
float anguloActual, anguloObjetivo;
float angObj;
float H = 0.5;
int V = 0;
float errorAnguloAnterior;
int anguloSH, anguloSV, speedServo;
char datosEntrantes;
Servo servoVertical;
Servo servoHorizontal;
String comandoRecibido;
String instruccion;

/***********       SETUP        ************* */
void setup() {
  pinMode(23, OUTPUT); // DIR MOTOR_A PORTA,1
  pinMode(22, OUTPUT); // STEP MOTOR_A  PORTA,0
  pinMode(25, OUTPUT); // DIR MOTOR_B PORTA,3
  pinMode(24, OUTPUT); // STEP MOTOR_B  PORTA,2
  pinMode(26, OUTPUT); // ENABLE MOTORS PORTA,4
  pinMode(13, OUTPUT); //LED

  servoVertical.attach(2);  //SERVO VERTICAL
  servoHorizontal.attach(3);//SERVO HORIZONTAL
  digitalWrite(9, HIGH);  // HIGH Deshabilitar motores

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(BAUDIOS); // Serial salida to console

  Serial.println("Rob-Bat by Alienneo v.1.1");
  Serial.println("Iniciando...");

  delay(500);
  Serial.println("Gyro calibracion!!  No mover en 10 segundos... ");
  delay(500);

  Serial.println("Iniciando motores...");
  servosFrente();

  //******************************************************
  cli();
  TCCR4A = 0;                                              // Timer4 CTC mode 4
  TCCR4B = (1 << WGM42) | (1 << CS41);                     // Prescaler=256
  OCR4A = ZERO_SPEED;                                             // Motor parado
  TCNT4 = 0;

  TCCR5A = 0;                                     // Timer5 CTC mode 4
  TCCR5B = (1 << WGM52) | (1 << CS51);                     // Prescaler=256
  OCR5A = ZERO_SPEED;                                             // Motor parado
  TCNT3 = 0;


  TIMSK4 |= (1 << OCIE4A);
  TIMSK5 |= (1 << OCIE5A);
  sei();

}  /*   ************ FINAL SETUP *************** */


/* ************  LOOP   ***************** */
void loop() {

  digitalWrite(13, HIGH);  //ENCENDER LED

  if (Serial.available()) {

    datosEntrantes = Serial.read();
    comandoRecibido += datosEntrantes;
    if (comandoRecibido.length() == 9) {
      instruccion = comandoRecibido.substring(0, 2);
      anguloSH = stringToNumber(comandoRecibido.substring(2, 5));
      anguloSV = stringToNumber(comandoRecibido.substring(5, 8));
      speedServo = stringToNumber(comandoRecibido.substring(8, 9));
      Serial.print(instruccion);
      Serial.print(" ");
      Serial.print(anguloSH);
      Serial.print(" ");
      Serial.println(anguloSV);
      // Serial.print(" ");
      //Serial.println(speedServo);

      // *********** ESTRUCTURA DE LA COMUNICACIÓN  *******************
      //
      //              ******   PARA MICROSERVOS:   **********
      // XX 000 000 0
      // INSTRUCCION  GRADOS_SH  GRADOS_SV VELOCIDAD
      // SV 180 180 0
      //
      //
      //              ******   PARA MOVIMIENTO MOTORES:  ********
      // XX 000 000 0
      // INSTRUCCION  PASOS_M_DERECHO  PASOS_M_IZQUIERDO  VELOCIDAD  **** HAY QUE PERFECCIONARLO ******

      if (!(instruccion.compareTo("SV"))) {
        moverServoVertical(anguloSV);
        comandoRecibido = "";
      }
      if (!(instruccion.compareTo("SH"))) {
        moverServoHorizontal(anguloSH);
        comandoRecibido = "";
      }
      if (!(instruccion.compareTo("FR"))) {
        servosFrente();
        comandoRecibido = "";
      }
      if (!(instruccion.compareTo("VE"))) {
        servoVector(anguloSH, anguloSV);
        comandoRecibido = "";
      }
    }
  }

  tiempo_valor = millis();

  /* ************   DATOS DEL ACELEROMETRO/GIROSCOPIO *********************** */
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  Acc[1] = atan(-1 * (AcX / A_R) / sqrt(pow((AcY / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;
  Acc[0] = atan((AcY / A_R) / sqrt(pow((AcX / A_R), 2) + pow((AcZ / A_R), 2))) * RAD_TO_DEG;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true);

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();

  Gy[0] = GyX / G_R;
  Gy[1] = GyY / G_R;

  Angle[0] = 0.98 * (Angle[0] + Gy[0] * 0.010) + 0.02 * Acc[0];
  Angle[1] = 0.98 * (Angle[1] + Gy[1] * 0.010) + 0.02 * Acc[1];

  //Serial.print("Angle X: "); Serial.print(Angle[0]); Serial.print("\n");
  //Serial.print("Angle Y: "); Serial.print(Angle[1]);  // Angulo necesario
  delay(2);

  float tiempoAnterior = tiempoActual;
  tiempoActual = micros();
  float tiempoPasado = (tiempoActual - tiempoAnterior);                 //tiempo pasado desde ciclo anterior en microsegundos
  tiempo  = tiempoPasado / 1000;                                        //tiempo del ciclo en milisegundos
  anguloActual = Angle[1];

  float velo = ((vel1 + vel2) / 2);
  velocidadMedia = velo;//0.95 * velocidadMedia + 0.05 * velo;
  angObj = -velocidadPID(tiempo, velocidadMedia, velocidadDeseada, Kps, Kis , Kds) ;
  anguloObjetivo = 0.02 * angObj + 0.98 * anguloObjetivo;
  anguloObjetivo = constrain (anguloObjetivo , -MAX_ANG, MAX_ANG);
  float resultadoanguloPID = anguloPID (anguloActual, anguloObjetivo, Kp, Kd, Ki, tiempo);

  if ((anguloActual < 55) && (anguloActual > -55)) // Esta el robot de pie????
  {
    if (anguloActual < (anguloObjetivo + H) && anguloActual > (anguloObjetivo - H) && (velocidad < V && velocidad > -V))
    {
      CLR(PORTA, 4);
      vel1 = 0 - giro ;
      vel2 = 0 + giro ;
      integralSum = 0;
      setSpeed_Motor_A(vel1);
      setSpeed_Motor_B(vel2);
    }  else
    {
      CLR(PORTA, 4);
      velocidad = resultadoanguloPID;
      vel1 = velocidad + giro  ;
      vel2 = velocidad - giro ;
      vel1 = constrain(vel1, -VEL_MAX, VEL_MAX);
      vel2 = constrain(vel2, -VEL_MAX, VEL_MAX);
      setSpeed_Motor_A(vel1);
      setSpeed_Motor_B(vel2);
    }
  }
  else
  {
    SET(PORTA, 4);
    setSpeed_Motor_A(0);
    setSpeed_Motor_B(0);
    integralSum = 0;
    anguloPID_errorSum = 0;
  }

}/* **************   FINAL LOOP   ******************** */

// TIMER 4 : STEPPER MOTOR_A SPEED CONTROL  *********   ISR  ****************
ISR(TIMER4_COMPA_vect) //16bits
{
  SET(PORTA, 0); // STEP MOTOR A
  delay_1us();
  delay_1us();
  CLR(PORTA, 0);
}
//TIMER 5 : STEPPER MOTOR_B SPEED CONTROL
ISR(TIMER5_COMPA_vect) //16bits
{
  SET(PORTA, 2); // STEP MOTOR B
  delay_1us();
  delay_1us();
  CLR(PORTA, 2);
}
/* ******************************************************************** */
/* ****************        setSpeed_Motor_A         ******************* */
/* ******************************************************************** */
void setSpeed_Motor_A(int16_t tspeed) {

  long timer_period;
  int16_t speed;

  if ((velocidad_Motor_A - tspeed) > MAXIMA_ACELERACION)
    velocidad_Motor_A -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_A - tspeed) < -MAXIMA_ACELERACION)
    velocidad_Motor_A += MAXIMA_ACELERACION;
  else
    velocidad_Motor_A = tspeed;

  speed = velocidad_Motor_A * 46; // Adjust factor from control output speed to real motor speed in steps/second

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_Motor_A = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_Motor_A = 1;
    SET(PORTA, 1); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_Motor_A = -1;
    CLR(PORTA, 1); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;
  //Serial.println(timer_period);
  OCR4A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT4 > OCR4A)
    TCNT4 = 0;
}
/* ******************************************************************** */
/* ****************        setSpeed_Motor_B         ******************* */
/* ******************************************************************** */

void setSpeed_Motor_B(int16_t tspeed) {

  long timer_period;
  int16_t speed;

  if ((velocidad_Motor_B - tspeed) > MAXIMA_ACELERACION)
    velocidad_Motor_B -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_B - tspeed) < -MAXIMA_ACELERACION)
    velocidad_Motor_B += MAXIMA_ACELERACION;
  else
    velocidad_Motor_B = tspeed;

  speed = velocidad_Motor_B * 46; // Adjust factor from control output speed to real motor speed in steps/second

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_Motor_B = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_Motor_B = 1;
    SET(PORTA, 3); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_Motor_B = -1;
    CLR(PORTA, 3); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;
  //Serial.println(timer_period);
  OCR5A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT5 > OCR5A)
    TCNT5 = 0;
}

/* ************************************************************************** */
/* ****************         CONTROL PID  (B-ROBOT)        ******************* */
/* ** modificar el los algoritmos para mejorar la estavilidad y velocidad  ** */
/* ************************************************************************** */

float anguloPID (float anguloActual, float anguloDeseado, float Kp, float Kd, float Ki, float tiempo) {

  float errorAnguloActual;
  float anguloCalculado;

  errorAnguloActual = anguloDeseado - anguloActual;

  float propAng =   Kp * errorAnguloActual;// * 0.1;
  float inteAng =   Ki * integralSum * tiempo;// * 0.001;
  float deriAng =   Kd * Gy[0] / tiempo ;


  anguloCalculado = propAng + inteAng + deriAng;
  errorAnguloAnterior = errorAnguloActual;

  return (anguloCalculado);
}

float velocidadPID(float DT, float velMedia, float velObjetivo,  float Kp, float Ki, float Kd) {

  float errorVelocidadActual;
  float velocidadCalculada;

  errorVelocidadActual = velObjetivo - velMedia;

  //  anguloPID_errorSum += constrain(errorVelocidadActual, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  //  anguloPID_errorSum = constrain(anguloPID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(anguloPID_errorSum);
  float propVel = Kp * errorVelocidadActual;
  float integVel = Ki * anguloPID_errorSum * DT * 0.001;
  float deriVel = Kd * (errorVelocidadActual - errorVelociadAnterior) / DT;

  velocidadCalculada = propVel + integVel + deriVel;
  errorVelociadAnterior = errorVelocidadActual;

  return (velocidadCalculada);
}

void moverServoVertical(int grados) {
  servoVertical.write(grados);
}
void moverServoHorizontal(int grados) {
  servoHorizontal.write(grados);
}
void servosFrente() {
  servoHorizontal.write(96);
  servoVertical.write(32);
  Serial.println("Servos al frente desde la funcion");
}
void servoVector(int shAngulo, int svAngulo) {
  servoHorizontal.write(shAngulo);
  servoVertical.write(svAngulo);
}

int stringToNumber(String thisString) {
  int i, value, length;
  length = thisString.length();
  char blah[(length + 1)];
  for (i = 0; i < length; i++) {
    blah[i] = thisString.charAt(i);
  }
  blah[i] = 0;
  value = atoi(blah);
  return value;
}

void delay_1us() {

  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}
