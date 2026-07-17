#include <Arduino.h>
#include <math.h>

// ==========================================
// 1. PINES 
#define ENC_L_A 25
#define ENC_L_B 26
#define PULSO_L_A 32    // PWM IN1
#define PULSO_L_B 33    // PWM IN2

#define ENC_R_A 4
#define ENC_R_B 13
#define PULSO_R_A 27    // PWM IN3
#define PULSO_R_B 14    // PWM IN4

// ==========================================
// 2. CONSTANTES FÍSICAS
const float R = 0.0325;        
const float L = 0.197;         
const float N_VUELTA = 1680.0; 
const float Ts = 0.01;         

// CALIBRACIÓN LAZO ABIERTO
const float K_CONVERSION = 2300.0; 

// ==========================================
// 3. VARIABLES DE ESTADO
unsigned long tiempo_inicio_estado = 0;
bool robot_activo = false;
bool estado_recien_iniciado = true;

double comando_lineal_v = 0.0;
double comando_angular_w = 0.0;

volatile bool flag_control = false;
hw_timer_t *timer = NULL;

// ==========================================
// 4. FUNCIONES DE MOTOR
void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (abs(pwm) < 40) pwm = 0; 
  if (pwm >= 0) { ledcWrite(pinA, (int)pwm); ledcWrite(pinB, 0); }
  else { ledcWrite(pinA, 0); ledcWrite(pinB, (int)-pwm); }
}

// ==========================================
// 5. LÓGICA TAREA 3 
void ejecutarControl() {
  if (!robot_activo) return;

  if (estado_recien_iniciado) {
    Serial.println(">>> EJECUTANDO TAREA 3: Círculo de 10 segundos");
    tiempo_inicio_estado = millis();
    
    // Cálculo de velocidades para un círculo
    // w_ref: Velocidad para dar una vuelta completa (2*PI) en 10 segundos
    comando_angular_w = (2.0 * PI) / 10.0; 
    // v_ref: Define el radio del círculo (w * radio)
    comando_lineal_v = comando_angular_w * 0.4; // Radio de 0.4 metros
    
    estado_recien_iniciado = false;
  }

  // Comprobar si han pasado los 10 segundos
  if (millis() - tiempo_inicio_estado > 10000) { 
    setMotor(PULSO_L_A, PULSO_L_B, 0);
    setMotor(PULSO_R_A, PULSO_R_B, 0);
    robot_activo = false;
    Serial.println(">>> TAREA 3 COMPLETADA");
    return;
  }

  // CINEMÁTICA INVERSA 
  // Calculamos la velocidad diferencial para que el robot gire
  double v_der = comando_lineal_v + (comando_angular_w * L / 2.0); 
  double v_izq = comando_lineal_v - (comando_angular_w * L / 2.0); 

  // ACTUACIÓN EN LAZO ABIERTO 
  // Traducimos la velocidad física (m/s) a potencia PWM
  double pwm_L = v_izq * K_CONVERSION;
  double pwm_R = v_der * K_CONVERSION;

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

void IRAM_ATTR onTimer() { flag_control = true; }

// ==========================================
// 6. SETUP
void setup() {
  Serial.begin(115200);
  ledcAttach(PULSO_L_A, 1000, 10); ledcAttach(PULSO_L_B, 1000, 10);
  ledcAttach(PULSO_R_A, 1000, 10); ledcAttach(PULSO_R_B, 1000, 10);
  
  delay(5000);
  
  robot_activo = true;
  estado_recien_iniciado = true;

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0);
}

void loop() {
  if (flag_control) {
    flag_control = false;
    ejecutarControl();
  }
}