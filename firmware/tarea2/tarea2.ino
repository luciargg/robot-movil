#include <Arduino.h>
#include <math.h>

// ==========================================
// 1. PINES 
#define ENC_L_A 25
#define ENC_L_B 26
#define PULSO_L_A 32
#define PULSO_L_B 33

#define ENC_R_A 4
#define ENC_R_B 13
#define PULSO_R_A 27
#define PULSO_R_B 14

// ==========================================
// 2. CONSTANTES 
const float R = 0.0325;        
const float L = 0.197;         
const float N_VUELTA = 1680.0; 
const float Ts = 0.01;         
const float K_CONVERSION = 2100.0; // Factor de calibración (Lazo Abierto)

// VARIABLES ODOMETRÍA 
volatile long counts_L = 0;
volatile long counts_R = 0;
long odom_prev_L = 0, odom_prev_R = 0;
double x_pos = 0.0, y_pos = 0.0, theta = 0.0;

// VARIABLES ESTADO 
bool robot_activo = false;
bool estado_recien_iniciado = true;
double start_x = 0, start_y = 0;

volatile bool flag_control = false;
hw_timer_t *timer = NULL;

// ==========================================
// 3. INTERRUPCIONES 
void IRAM_ATTR isr_L_A() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L++; else counts_L--; }
void IRAM_ATTR isr_L_B() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L--; else counts_L++; }
void IRAM_ATTR isr_R_A() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R++; else counts_R--; }
void IRAM_ATTR isr_R_B() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R--; else counts_R++; }

void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (abs(pwm) < 40) pwm = 0; 
  if (pwm >= 0) { ledcWrite(pinA, (int)pwm); ledcWrite(pinB, 0); }
  else { ledcWrite(pinA, 0); ledcWrite(pinB, (int)-pwm); }
}

// ==========================================
// 4. LÓGICA TAREA 2 
void ejecutarControl() {
  // 1. CALCULAR ODOMETRÍA
  long dL = counts_L - odom_prev_L;
  long dR = counts_R - odom_prev_R;
  odom_prev_L = counts_L; odom_prev_R = counts_R;

  double dsL = (2.0 * PI * R * dL) / N_VUELTA;
  double dsR = (2.0 * PI * R * dR) / N_VUELTA;
  double ds = (dsR + dsL) / 2.0;
  x_pos += ds; 

  if (!robot_activo) return;

  if (estado_recien_iniciado) {
    Serial.println(">>> EJECUTANDO TAREA 2: 2 metros con frenado");
    start_x = x_pos; 
    estado_recien_iniciado = false;
  }

  double distancia_recorrida = abs(x_pos - start_x);
  
  // AJUSTE: Paramos un poco antes para compensar el deslizamiento
  if (distancia_recorrida >= 1.75) {
    
    // FRENADO ACTIVO (Pulso atrás de 50ms)
    setMotor(PULSO_L_A, PULSO_L_B, -400); 
    setMotor(PULSO_R_A, PULSO_R_B, -400);
    delay(50); // Frenado
    
    setMotor(PULSO_L_A, PULSO_L_B, 0);
    setMotor(PULSO_R_A, PULSO_R_B, 0);
    
    robot_activo = false;
    Serial.printf(">>> TAREA 2 COMPLETADA. Distancia: %.2f m\n", distancia_recorrida);
    return;
  }

  // Velocidad normal
  double comando_lineal_v = 0.2; 
  double pwm = comando_lineal_v * K_CONVERSION;

  setMotor(PULSO_L_A, PULSO_L_B, pwm);
  setMotor(PULSO_R_A, PULSO_R_B, pwm);
}

void IRAM_ATTR onTimer() { flag_control = true; }

// ==========================================
// 5. SETUP
void setup() {
  Serial.begin(115200);
  ledcAttach(PULSO_L_A, 1000, 10); ledcAttach(PULSO_L_B, 1000, 10);
  ledcAttach(PULSO_R_A, 1000, 10); ledcAttach(PULSO_R_B, 1000, 10);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_L_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_R_B, CHANGE);

  delay(5000);
  robot_activo = true;
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0);
}

void loop() {
  if (flag_control) { flag_control = false; ejecutarControl(); }
}