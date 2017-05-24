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


#define BAUDIOS  9600



#define LIMITE_ANGULO_OBJETIVO    3  //12

#define MAX_CONTROL_SALIDA        200  //500 maximo de velocidad de salida
#define MAXIMA_ACELERACION         6 // MAX RECOMMENDED VALUE: 8) (default:7)
#define ITERM_MAX_ERROR           25  // Iterm windup constants for PI control //40
#define ITERM_MAX                 8000  // 5000

#define MICROSTEPPING             16  // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

//
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535  //  65535 se trata de la frecuencia máxima en 8bits

// Default control terms
#define KP 0.20
#define KD 20
#define KP_THROTTLE 0.07
#define KI_THROTTLE 0.04


bool Robot_tumbado = false;           // Robot tumbado flag => Out of
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[2];
float Gy[2];
float Angle[2];
long tiempo_antiguo;
long tiempo_valor;
float dt;
float angulo_recogido;
float angulo_recogido_antiguo;
float PID_errorSuma;
float PID_errorAntiguo = 0;
float PID_errorAntiguo2 = 0;
float setPointAntiguo = 0;
float angulo_objetivo;
float acelerador;
float direccion;
float control_salida;


int16_t motor_A;
int16_t motor_B;


int16_t velocidad_Motor_A;
int16_t velocidad_Motor_B;
uint8_t loop_counter;
uint8_t slow_loop_counter;

float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;


int8_t  dir_Motor_A;
int8_t  dir_Motor_B;          // Actual direction of steppers motors
int16_t actual_robot_velocidad;
int16_t actual_robot_velocidad_antiguo;
float velocidad_estimada_filtered;    // Estimated robot velocidad




//// 16 single cycle instructions = 1us at 16Mhz
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

/***********       SETUP        ************* */
void setup() {


  pinMode(8, OUTPUT); // DIR MOTOR_A PORTB,4
  pinMode(11, OUTPUT); // STEP MOTOR_A  PORTB,7
  pinMode(7, OUTPUT); // DIR MOTOR_B PORTE,6
  pinMode(6, OUTPUT); // STEP MOTOR_B  PORTD,7
  
  pinMode(9, OUTPUT); // ENABLE MOTORS

  digitalWrite(9, HIGH);  // HIGH Deshabilitar motores

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

  //******************************************************
  cli();
  TCCR1A = 0;                                              // Timer1 CTC mode 4
  TCCR1B = (1 << WGM12) | (1 << CS11);                     // Prescaler=256
  OCR1A = ZERO_SPEED;                                             // Motor parado
  TCNT1 = 0;

  TCCR3A = 0;                                     // Timer2 CTC mode 4
  TCCR3B = (1 << WGM32) | (1 << CS31);                     // Prescaler=256
  OCR3A = ZERO_SPEED;                                             // Motor parado
  TCNT3 = 0;


  TIMSK1 |= (1 << OCIE1A);
  TIMSK3 |= (1 << OCIE3A);
  sei();

  //pequeña vibracion en los motores para comprobar que estan preparados
  for (uint8_t k = 0; k < 5; k++) {
    setSpeed_Motor_A(5);
    setSpeed_Motor_B(-5);
    delay(200);
    setSpeed_Motor_A(-5);
    setSpeed_Motor_B(5);
    delay(200);

  }
  setSpeed_Motor_A(0);
  setSpeed_Motor_B(0);


}  /*   ************ FINAL SETUP *************** */


/* ************  LOOP   ***************** */
void loop() {
  tiempo_valor=millis();
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
  //Serial.print("Angle Y: "); Serial.println(Angle[1]);  // Angulo necesario
  delay(2);




  loop_counter++;
  slow_loop_counter++;
  dt = (tiempo_valor - tiempo_antiguo);
  tiempo_antiguo = tiempo_valor;

  angulo_recogido_antiguo = angulo_recogido;

  angulo_recogido = Angle[1];

  //Serial.println(angulo_recogido);


  // We calculate the estimated robot speed:
  // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
  actual_robot_velocidad_antiguo = actual_robot_velocidad;
  actual_robot_velocidad = (velocidad_Motor_A + velocidad_Motor_B) / 2; // Positive: forward

  int16_t velocidad_angular = (angulo_recogido - angulo_recogido_antiguo) * 90.0; // 90 is an empirical extracted factor to adjust for real units
  int16_t velocidad_estimada = -actual_robot_velocidad_antiguo - velocidad_angular;     // We use robot_speed(t-1) or (t-2) to compensate the delay
  velocidad_estimada_filtered = velocidad_estimada_filtered * 0.95 + (float)velocidad_estimada * 0.05;  // low pass filter on estimated speed

  angulo_objetivo = controlVelocidadPI(dt, velocidad_estimada_filtered, acelerador, Kp_thr, Ki_thr);
  angulo_objetivo = constrain(angulo_objetivo, -LIMITE_ANGULO_OBJETIVO, LIMITE_ANGULO_OBJETIVO); // limited outpu



  control_salida += controlEstabilidadPD(dt, angulo_recogido, angulo_objetivo, Kp, Kd);
 control_salida = constrain(control_salida, -MAX_CONTROL_SALIDA, MAX_CONTROL_SALIDA); // Limit max output from control

  // The steering part from the user is injected directly on the output
  motor_A = control_salida + direccion;
  motor_B = control_salida - direccion;

  // Limit max speed (control output)
  motor_A = constrain(motor_A, -MAX_CONTROL_SALIDA, MAX_CONTROL_SALIDA);
  motor_B = constrain(motor_B, -MAX_CONTROL_SALIDA, MAX_CONTROL_SALIDA);


  if ((angulo_recogido < 76) && (angulo_recogido > -76)) // Is robot ready (upright?)
  {
    // NORMAL MODE
    digitalWrite(9, LOW);  // Motors enable
    setSpeed_Motor_A(-motor_A);
    setSpeed_Motor_B(motor_B);
    
  }


//Serial.print(",");
Serial.print("dt: ");
Serial.print(dt);
Serial.print("  kp: ");
Serial.print(Kp);
Serial.print("  kd:  ");
Serial.print(Kd);

Serial.print("  angulo_recogido: ");
Serial.print(angulo_recogido);
Serial.print("  angulo_objetivo: ");
Serial.print(angulo_objetivo);
Serial.print("  motor_A:  ");
Serial.print(motor_A);
Serial.print("  motor_B:  ");
Serial.print(motor_B);
Serial.print("  control_salida: ");
Serial.println(control_salida);
//Serial.println(motor_A);
//Serial.print(",");
//Serial.println(motor_B);




  //  if (velocidad < 50) {
  //    setSpeed_Motor_A(velocidad);//
  //    setSpeed_Motor_B(-velocidad);
  //    ++velocidad;
  //    Serial.println(velocidad);
  //    delay(10);
  //  }
  //  Serial.print("motor_A 16bits steps: ");
  //  Serial.println(cuenta_A);
  //  Serial.print("motor_B  8bits steps: ");
  //  Serial.println(cuenta_B);

}/* **************   FINAL LOOP   ******************** */




// TIMER 1 : STEPPER MOTOR_A SPEED CONTROL  *********   ISR  ****************
ISR(TIMER1_COMPA_vect) //16bits
{

  SET(PORTB, 7); // STEP MOTOR A
  delay_1us();
  delay_1us();
  CLR(PORTB, 7);


}
//TIMER 3 : STEPPER MOTOR_B SPEED CONTROL
ISR(TIMER3_COMPA_vect) //16bits
{

  SET(PORTD, 7); // STEP MOTOR B
  delay_1us();
  delay_1us();
  CLR(PORTD, 7);


}
/* ******************************************************************** */
/* ****************        setSpeed_Motor_A         ******************* */
/* ******************************************************************** */
void setSpeed_Motor_A(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((velocidad_Motor_A - tspeed) > MAXIMA_ACELERACION)
    velocidad_Motor_A -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_A - tspeed) < -MAXIMA_ACELERACION)
    velocidad_Motor_A += MAXIMA_ACELERACION;
  else
    velocidad_Motor_A = tspeed;

#if MICROSTEPPING==16
  speed = velocidad_Motor_A * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = velocidad_Motor_A * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_Motor_A = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_Motor_A = 1;
    SET(PORTB, 4); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_Motor_A = -1;
    CLR(PORTB, 4); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;
  //Serial.println(timer_period);
  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;

/* ******************************************************************** */
/* ****************        setSpeed_Motor_B         ******************* */
/* ******************************************************************** */
} void setSpeed_Motor_B(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((velocidad_Motor_B - tspeed) > MAXIMA_ACELERACION)
    velocidad_Motor_B -= MAXIMA_ACELERACION;
  else if ((velocidad_Motor_B - tspeed) < -MAXIMA_ACELERACION)
    velocidad_Motor_B += MAXIMA_ACELERACION;
  else
    velocidad_Motor_B = tspeed;

#if MICROSTEPPING==16
  speed = velocidad_Motor_B * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = velocidad_Motor_B * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_Motor_B = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_Motor_B = 1;
    SET(PORTE, 6); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_Motor_B = -1;
    CLR(PORTE, 6); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;
  //Serial.println(timer_period);
  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}

/* ******************************************************************** */
/* ****************     CONTROL PID  (B-ROBOT)      ******************* */
/* ******************************************************************** */

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float controlEstabilidadPD(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointAntiguo) - Kd * (input - PID_errorAntiguo2)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorAntiguo2 = PID_errorAntiguo;
  PID_errorAntiguo = input;  // error for Kd is only the input component
  setPointAntiguo = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT is in miliseconds
float controlVelocidadPI(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PID_errorSuma += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSuma = constrain(PID_errorSuma, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSuma * DT * 0.001; // DT is in miliseconds...
  return (output);
}

