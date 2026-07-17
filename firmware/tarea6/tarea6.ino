#include <math.h>
#include <Arduino.h> 

// ==========================================
// 1. VARIABLES DE POSICIÓN 
double pos_x_robot = 0.0;
double pos_y_robot = 0.0;
double angulo_theta_robot = 0.0;

// ==========================================
// 2. ESTRUCTURA MOTORES 
struct InfoMotor {
  int pin_fase_A; 
  int pin_fase_B;      
  int pin_sensor_A; 
  int pin_sensor_B;    
  volatile long ticks_actuales; 
  long ticks_anteriores_vel; 
  long ticks_anteriores_odom;
  double velocidad_bruta; 
  double velocidad_suavizada;  
  double error_anterior; 
  double error_acumulado; 
  double derivada_suavizada; 
  double potencia_pwm;         
  double ganancia_Kp; 
  double ganancia_Ki; 
  double ganancia_Kd;
  unsigned long tiempo_inicio_bloqueo; 
  bool esta_bloqueado;
};

// ==========================================
// 3. CONSTANTES ROBOT 
const double RADIO_RUEDA = 0.0325;      
const double DISTANCIA_ENTRE_RUEDAS = 0.19;   
const double TICKS_POR_VUELTA = 1680.0;

// VARIABLES DE CONTROL
double objetivo_vel_motor1 = 0; 
double objetivo_vel_motor2 = 0; 
double comando_lineal_v = 0.0; 
double comando_angular_w = 0.0; 

// GESTIÓN DE OBSTÁCULOS 
unsigned long tiempo_ultimo_obstaculo = 0; 
bool en_pausa_obstaculo = false;
bool esperando_reinicio = false;

// Motor 1 (Derecho)
const int PIN_M1_IN1 = 32;  
const int PIN_M1_IN2 = 33;  
const int PIN_M1_ENC_A = 26; 
const int PIN_M1_ENC_B = 25; 

// Motor 2 (Izquierdo)
const int PIN_M2_IN3 = 27;  
const int PIN_M2_IN4 = 14;  
const int PIN_M2_ENC_A = 4;  
const int PIN_M2_ENC_B = 13; 

// PINES SENSORES 
const int PIN_SENSOR_1 = 34; 
const int PIN_SENSOR_2 = 35; 
const int PIN_SENSOR_3 = 39; 
const int UMBRAL_OBSTACULO = 300; 

// PID PARAMETERS
const double VALOR_KP = 0.2;  
const double VALOR_KI = 0.5;  
const double VALOR_KD = 0.005;

// Inicialización
InfoMotor motor_derecho   = {PIN_M1_IN1, PIN_M1_IN2, PIN_M1_ENC_A, PIN_M1_ENC_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, 0, false};
InfoMotor motor_izquierdo = {PIN_M2_IN3, PIN_M2_IN4, PIN_M2_ENC_A, PIN_M2_ENC_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, 0, false};

const int frecuencia_pwm = 1000; 
const int resolucion_pwm = 10;
double tiempo_muestreo_seg = 0.01; 
double factor_suavizado_vel = 0.7; 
double factor_suavizado_derivada = 0.85;  
hw_timer_t *temporizador = NULL;
volatile bool hay_datos_nuevos = false;

// ==========================================
// 4. INTERRUPCIONES 
void IRAM_ATTR ISR_MD_A() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales--; else motor_derecho.ticks_actuales++; }
void IRAM_ATTR ISR_MD_B() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales++; else motor_derecho.ticks_actuales--; }
void IRAM_ATTR ISR_MI_A() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales++; else motor_izquierdo.ticks_actuales--; }
void IRAM_ATTR ISR_MI_B() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales--; else motor_izquierdo.ticks_actuales++; }

// ==========================================
// 5. CONTROL PID 
void calcular_PID(InfoMotor &m, int vel_obj) {
  if(m.esta_bloqueado) { m.potencia_pwm = 0; return; }
  
  double error = vel_obj - m.velocidad_suavizada;
  double deriv = (error - m.error_anterior) / tiempo_muestreo_seg;
  
  m.derivada_suavizada = factor_suavizado_derivada * m.derivada_suavizada + (1.0 - factor_suavizado_derivada) * deriv;
  m.error_acumulado += error * tiempo_muestreo_seg;
  
  // Anti-windup más estricto
  if (m.error_acumulado > 2000) m.error_acumulado = 2000;
  else if (m.error_acumulado < -2000) m.error_acumulado = -2000;
  
  if (vel_obj == 0) m.error_acumulado = 0;

  double out = m.ganancia_Kp * error + m.ganancia_Ki * m.error_acumulado + m.ganancia_Kd * m.derivada_suavizada;
  m.error_anterior = error;
  
  if (out > 1023) out = 1023; else if (out < -1023) out = -1023;
  m.potencia_pwm = out;
}

void mover_motor(InfoMotor &m) {
  if(m.esta_bloqueado) { ledcWrite(m.pin_fase_A, 0); ledcWrite(m.pin_fase_B, 0); return; }
  
  if (abs(m.potencia_pwm) < 50) {
    ledcWrite(m.pin_fase_A, 0); 
    ledcWrite(m.pin_fase_B, 0);
    return;
  }

  if (m.potencia_pwm >= 0) { 
    ledcWrite(m.pin_fase_A, m.potencia_pwm); 
    ledcWrite(m.pin_fase_B, 0); 
  } else { 
    ledcWrite(m.pin_fase_B, -1 * m.potencia_pwm); 
    ledcWrite(m.pin_fase_A, 0); 
  }
}

void medir_velocidad(InfoMotor &m) {
  double dif = m.ticks_actuales - m.ticks_anteriores_vel;
  m.velocidad_bruta = dif / tiempo_muestreo_seg;
  m.velocidad_suavizada = factor_suavizado_vel * m.velocidad_suavizada + (1.0 - factor_suavizado_vel) * m.velocidad_bruta;
  m.ticks_anteriores_vel = m.ticks_actuales;
}

void IRAM_ATTR Interrupcion_Temporizador() {
  medir_velocidad(motor_derecho);   
  calcular_PID(motor_derecho, objetivo_vel_motor1);   
  mover_motor(motor_derecho);

  medir_velocidad(motor_izquierdo); 
  calcular_PID(motor_izquierdo, objetivo_vel_motor2); 
  mover_motor(motor_izquierdo);
  
  hay_datos_nuevos = true; 
}

// ==========================================
// 6. FUNCIÓN DE SEGURIDAD OBSTÁCULOS
bool hay_obstaculo() {
  int val1 = analogRead(PIN_SENSOR_1);
  int val2 = analogRead(PIN_SENSOR_2);
  int val3 = analogRead(PIN_SENSOR_3);

  if (val1 < UMBRAL_OBSTACULO || val2 < UMBRAL_OBSTACULO || val3 < UMBRAL_OBSTACULO) {
    return true;
  }
  return false;
}

// ==========================================
// 7. SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("--- INICIANDO TAREA 6: CORREGIDA ---");
  
  ledcAttach(motor_derecho.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_derecho.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  ledcAttach(motor_izquierdo.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_izquierdo.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  
  pinMode(motor_derecho.pin_sensor_A, INPUT_PULLUP); pinMode(motor_derecho.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_A), ISR_MD_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_B), ISR_MD_B, CHANGE);
  
  pinMode(motor_izquierdo.pin_sensor_A, INPUT_PULLUP); pinMode(motor_izquierdo.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_A), ISR_MI_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_B), ISR_MI_B, CHANGE);

  pinMode(PIN_SENSOR_1, INPUT); pinMode(PIN_SENSOR_2, INPUT); pinMode(PIN_SENSOR_3, INPUT);

  temporizador = timerBegin(1000000);
  timerAttachInterrupt(temporizador, &Interrupcion_Temporizador);
  timerAlarm(temporizador, tiempo_muestreo_seg * 1000000, true, 0);

  delay(2000); 
}

void loop() {
  bool obstaculo_presente = hay_obstaculo();

  if (obstaculo_presente) {
    if (!en_pausa_obstaculo) {
      Serial.println("!!! PARANDO POR OBSTÁCULO !!!");
      en_pausa_obstaculo = true;
      esperando_reinicio = false;
    }
    comando_lineal_v = 0.0;
    comando_angular_w = 0.0;
  } 
  else {
    if (en_pausa_obstaculo && !esperando_reinicio) {
      Serial.println("... Esperando 3s ...");
      tiempo_ultimo_obstaculo = millis();
      esperando_reinicio = true;
    }

    if (esperando_reinicio) {
      comando_lineal_v = 0.0;
      if (millis() - tiempo_ultimo_obstaculo > 3000) {
        Serial.println(">>> AVANZANDO");
        esperando_reinicio = false;
        en_pausa_obstaculo = false;
      }
    } 
    else {
      // MOVIMIENTO NORMAL
      comando_lineal_v = 0.15;
      comando_angular_w = 0.0;
    }
  }

  // Conversión a ticks
  double v_der = comando_lineal_v + (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
  double v_izq = comando_lineal_v - (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
  
  objetivo_vel_motor1 = (v_der / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;
  objetivo_vel_motor2 = (v_izq / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;

  static unsigned long last_print = 0;
  if(millis() - last_print > 500){
    Serial.print("Obj: "); Serial.print(objetivo_vel_motor1);
    Serial.print(" | Vel Real Der: "); Serial.print(motor_derecho.velocidad_suavizada);
    Serial.print(" | PWM Der: "); Serial.println(motor_derecho.potencia_pwm);
    last_print = millis();
  }
  
  delay(10);
}