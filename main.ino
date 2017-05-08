/*
   *********  AUTOR: ALIENNEO  *********
   *********  DATE: 07/05/2013  ********
   ***********  VERSION: 1.1  **********
*/
#include <Wire.h>
#define MPU 0x68
#define A_R 16384.0
#define G_R 131.0
#define RAD_A_DEG = 57.295779


#define BAUDIOS  115200

#define LIMITE_ACCELERACION       650  //580
#define LIMITE_GIRO               0  //150
#define LIMITE_ANGULO_OBJETIVO    1  //12
#define KP                        0.18  //0.19 aceleracion de las ruedas
#define KD                        25  //28
#define KP_ACCELERACION           0.2  //0.07
#define KI_ACCELERACION           0.2  //0.04
#define MAX_CONTROL_OUTPUT        30  //500 maximo de velocidad de salida

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define ZERO_SPEED             0  //  65535 se trata de la frecuencia máxima en 16bits
#define MAXIMA_ACELERACION         10 // MAX RECOMMENDED VALUE: 8) (default:7)
#define MICROSTEPPING             16  // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)
#define ITERM_MAX_ERROR           9  // Iterm windup constants for PI control //40
#define ITERM_MAX               50  // 5000

bool Robot_tumbado = false;           // Robot tumbado flag => Out of
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];
long tiempo_Antiguo;
long tiempo_Valor;
float dt;
float angulo_recogido;
float angulo_recogido_antiguo;
float PID_errorSum;
float PID_errorAntiguo = 0;
float PID_errorAntiguo2 = 0;
float setPointAntiguo = 0;
float angulo_objetivo;
float acelerador;
float giro;
float control_salida;
int16_t motor_A;
int16_t motor_B;
int16_t velocidad_Motor_A;
int16_t velocidad_Motor_B;
int8_t  direccin_Motor_A = 1;
int8_t  direccin_Motor_B = 1;          // Actual direction of steppers motors
int16_t actual_robot_velocidad;
int16_t actual_robot_velocidad_Antiguo;
float velocidad_estimada_filtered;    // Estimated robot velocidad




float stabilityPDControl(float DT, float input, float setPoint,  float , float )
{
  float error;
  float salida;

  error = setPoint - input;

  salida = KP * error + (KD * (setPoint - setPointAntiguo)
                         - KD * (input - PID_errorAntiguo2)) / DT;

  PID_errorAntiguo2 = PID_errorAntiguo;
  PID_errorAntiguo = input;
  setPointAntiguo = setPoint;
  return (salida);
}
float velocidadPIControl(float DT, float input, float setPoint,  float , float Ki)
{
  float error;
  float salida;
  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);
  salida = KP * error + Ki * PID_errorSum * DT * 0.001; // DT is in miliseconds...
  return (salida);
}
// 16 single cycle instructions = 1us at 16Mhz
void delay_1us()
{
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




void setVelocidadMotor_A(int16_t tvelocidad)
{
  long timer_period;
  int16_t velocidad;

  // Limit max velocidad?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((velocidad_Motor_A - tvelocidad) > MAXIMA_ACELERACION)
    velocidad_Motor_A -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_A - tvelocidad) < -MAXIMA_ACELERACION)
    velocidad_Motor_A += MAXIMA_ACELERACION;
  else
    velocidad_Motor_A = tvelocidad;

#if MICROSTEPPING==16
  velocidad = velocidad_Motor_A * 46; // Adjust factor from control salida velocidad to real motor velocidad in steps/second
#else
  velocidad = velocidad_Motor_A * 23; // 1/8 Microstepping
#endif

  if (velocidad == 0)
  {
    timer_period = ZERO_SPEED;
    direccin_Motor_A = 0;
  }
  else if (velocidad > 0)
  {
    timer_period = 2000000 / velocidad; // 2Mhz timer
    direccin_Motor_A = 1;
    SET(PORTD, 7); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -velocidad;
    direccin_Motor_A = -1;
    CLR(PORTD, 7); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun velocidad (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}
// Set velocidad of Stepper Motor2
// tvelocidad could be positive or negative (reverse)
void setVelocidadMotor_B(int16_t tvelocidad)
{
  long timer_period;
  int16_t velocidad;

  // Limit max velocidad?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((velocidad_Motor_B - tvelocidad) > MAXIMA_ACELERACION)
    velocidad_Motor_B -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_B - tvelocidad) < -MAXIMA_ACELERACION)
    velocidad_Motor_B += MAXIMA_ACELERACION;
  else
    velocidad_Motor_B = tvelocidad;

#if MICROSTEPPING==16
  velocidad = velocidad_Motor_B * 46; // Adjust factor from control salida velocidad to real motor velocidad in steps/second
#else
  velocidad = velocidad_Motor_B * 23; // 1/8 Microstepping
#endif

  if (velocidad == 0)
  {
    timer_period = ZERO_SPEED;
    direccin_Motor_B = 0;
  }
  else if (velocidad > 0)
  {
    timer_period = 2000000 / velocidad; // 2Mhz timer
    direccin_Motor_B = 1;
    CLR(PORTD, 2);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -velocidad;
    direccin_Motor_B = -1;
    SET(PORTD, 2);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun velocidad (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR2A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT2 > OCR2A)
    TCNT2 = 0;
}
/***********       SETUP        ************* */
void setup() {


  pinMode(7, OUTPUT); // DIR MOTOR 1 PORTD,7
  pinMode(8, OUTPUT); // STEP MOTOR 1  PORTB,0
  pinMode(2, OUTPUT); // DIR MOTOR 2 PORTD,2
  pinMode(6, OUTPUT); // STEP MOTOR 2  PORTD,6
  pinMode(9, OUTPUT); // ENABLE MOTORS
  digitalWrite(9, HIGH);  // Deshabilitar motores

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(BAUDIOS); // Serial salida to console

  Serial.println("Rob-Bat by Alienneo v.1.1");
  Serial.println("Iniciando...");

  // Gyro calibration
  // The robot must be steady during initialization
  delay(500);
  Serial.println("Gyro calibracion!!  No mover en 10 segundos... ");
  delay(500);

  Serial.println("Iniciando motores...");
  noInterrupts();
  // MOTOR_A => TIMER1
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11); //(1 << CS12)segun b-robot
  OCR1A = ZERO_SPEED;        //??
  direccin_Motor_A = 0;
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A);
  //
  //MOTOR B => TIMER2 //segun manolo robot equilibrista
  TCCR2A = 0;
  TCCR2B = (1 << WGM22) | (1 << CS21);
  OCR2A = ZERO_SPEED; //según manolo robot equilibrista
  TCNT2 = 0;
  TIMSK2 |= (1 << OCIE2A);

  // MOTOR_B => TIMER3 //modificar para timer2 o esperar a LEONARDO
  // TCCR3A = 0;
  // TCCR3B = (1 << WGM32) | (1 << CS31);
  // OCR3A = ZERO_SPEED;
  // direccin_Motor_B = 0;
  //TCNT3 = 0;
  //TIMSK3 |= (1 << OCIE1A);

  interrupts();

  // Pequeña vibracion para indicar que el robot esta READY
  //  for (uint8_t k = 0; k < 5; k++)
  //  {
  //    setVelocidadMotor_A(50);
  //    // setVelocidadMotor_B(5);
  //    delay(200);
  //    setVelocidadMotor_A(-50);
  //    // setVelocidadMotor_B(-5);
  //    delay(200);
  //  }

  Serial.println("Preparado...");

  tiempo_Antiguo = millis();
  Robot_tumbado = false;
}  /*   ************ FINAL SETUP *************** */


/* ************  LOOP   ***************** */
void loop() {
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
  Serial.print("Angle Y: "); Serial.println(Angle[1]);  // Angulo necesario

  delay(20);

  /*   ************************************************************************
       *********   substituir por lo recogido en el puerto serial?? ***********
    //     ************************************************************************  */
  //OSC.MsgRead();  // Read UDP OSC messages
  //if (OSC.newMessage)
  //{
  //  OSC.newMessage = 0;
  //  if (OSC.page == 1)  // Get commands from user (PAGE1 are user commands: acelerador, giro...)
  //  {
  //    OSC.newMessage = 0;
  //    acelerador = (OSC.fadder1 - 0.5) * LIMITE_ACCELERACION;
  //    // We add some exponential on giro to smooth the center band
  //    giro = OSC.fadder2 - 0.5;
  //    if (giro > 0)
  //      giro = (giro * giro + 0.5 * giro) * LIMITE_GIRO;
  //    else
  //      giro = (-giro * giro + 0.5 * giro) * LIMITE_GIRO;

  // } // End new OSC message

  tiempo_Valor = millis();

  dt = (tiempo_Valor - tiempo_Antiguo);
  tiempo_Antiguo = tiempo_Valor;

  angulo_recogido_antiguo = angulo_recogido;

  angulo_recogido = Angle[1];
  Serial.print("angulo: ");
  Serial.println(angulo_recogido);

  actual_robot_velocidad_Antiguo = actual_robot_velocidad;

  actual_robot_velocidad = (velocidad_Motor_A + velocidad_Motor_B) / 2;

  int16_t velociad_angular = (angulo_recogido - angulo_recogido_antiguo) * 90.0;

  int16_t velocidad_estimada = -actual_robot_velocidad_Antiguo - velociad_angular;

  velocidad_estimada_filtered = velocidad_estimada_filtered * 0.95
                                + (float)velocidad_estimada * 0.05;

  angulo_objetivo = velocidadPIControl(dt, velocidad_estimada_filtered,
                                       acelerador, KP_ACCELERACION, KI_ACCELERACION);

  angulo_objetivo = constrain(angulo_objetivo,
                              -LIMITE_ANGULO_OBJETIVO,
                              LIMITE_ANGULO_OBJETIVO);

  control_salida += stabilityPDControl(dt, angulo_recogido, angulo_objetivo, KP, KD);

  control_salida = constrain(control_salida, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

  motor_A = control_salida + giro;
  motor_B = control_salida - giro;

  motor_A = constrain(motor_A, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
  motor_B = constrain(motor_B, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

  if ((angulo_recogido < 76) && (angulo_recogido > -76)) // Is robot ready (upright?)
  {
    digitalWrite(9, LOW);  // Motors enable
    setVelocidadMotor_A(motor_A);  //se intoduce la velocidad motor_A
    setVelocidadMotor_B(motor_B);  //se intoduce la velocidad motor_B
  } else {
    digitalWrite(9, HIGH);  // Disable motors
    setVelocidadMotor_A(0);
    setVelocidadMotor_B(0);
    PID_errorSum = 0;
  }
}/* **************   FINAL LOOP   ******************** */




// TIMER 1 : STEPPER MOTOR_A SPEED CONTROL  *********   ISR  ****************
ISR(TIMER1_COMPA_vect)
{
  if (direccin_Motor_A == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTB, 0); // STEP MOTOR 1
  // SET(PORTD, 6);
  delay_1us();
  CLR(PORTB, 0);
  // CLR(PORTD, 6);
}
// TIMER 2 : STEPPER MOTOR_B SPEED CONTROL
ISR(TIMER2_COMPA_vect)
{
  if (direccin_Motor_B == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTD, 6); // STEP MOTOR 2
  delay_1us();
  CLR(PORTD, 6);
}
