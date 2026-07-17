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

// AJUSTE DE VELOCIDAD 
const float K_OPEN_LOOP = 2200.0;

// ==========================================
// 3. VARIABLES DE CONTROL
unsigned long tiempo_inicio_estado = 0;
bool robot_activo = false;
bool estado_recien_iniciado = true;

double comando_lineal_v = 0.0;
double comando_angular_w = 0.0;

volatile bool flag_control = false;
hw_timer_t *timer = NULL;

// ==========================================
// 4. FUNCIONES DE ACTUACIÓN
void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (abs(pwm) < 40) pwm = 0; 

  if (pwm >= 0) {
    ledcWrite(pinA, (int)pwm);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, (int)-pwm);
  }
}

// ==========================================
// 5. LÓGICA DE LA TAREA 1
void ejecutarControl() {
  if (!robot_activo) return;

  // --- TAREA 1: RECTA 20 SEGUNDOS ---
  if (estado_recien_iniciado) {
    Serial.println(">>> INICIANDO TAREA 1: 0.1 m/s por 20 segundos");
    tiempo_inicio_estado = millis();
    
    // Introducimos la velocidad objetivo física
    comando_lineal_v = 0.1; 
    comando_angular_w = 0.0;
    
    estado_recien_iniciado = false;
  }

  // COMPROBAR TIEMPO (20 SEGUNDOS)
  // Matematicamente: 0.1 m/s * 20 s = 2.0 metros
  if (millis() - tiempo_inicio_estado > 20000) { 
    setMotor(PULSO_L_A, PULSO_L_B, 0);
    setMotor(PULSO_R_A, PULSO_R_B, 0);
    robot_activo = false;
    Serial.println(">>> TAREA 1 FINALIZADA <<<");
    return;
  }

  // CONVERSIÓN DE VELOCIDAD A PWM 
  // v_izq = v - (w*L/2) | v_der = v + (w*L/2)
  double v_izq = comando_lineal_v - (comando_angular_w * L / 2.0);
  double v_der = comando_lineal_v + (comando_angular_w * L / 2.0);

  // Mapeo directo a PWM usando el factor de calibración
  double pwm_L = v_izq * K_OPEN_LOOP;
  double pwm_R = v_der * K_OPEN_LOOP;

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

void IRAM_ATTR onTimer() { flag_control = true; }

// ==========================================
// 6. SETUP
void setup() {
  Serial.begin(115200);

  // Configurar PWM
  ledcAttach(PULSO_L_A, 1000, 10); ledcAttach(PULSO_L_B, 1000, 10);
  ledcAttach(PULSO_R_A, 1000, 10); ledcAttach(PULSO_R_B, 1000, 10);

  // Espera para colocar el robot
  delay(5000);

  robot_activo = true;
  estado_recien_iniciado = true;

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0); // 10ms
}

void loop() {
  if (flag_control) {
    flag_control = false;
    ejecutarControl();
  }
}