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

// ==========================================
// 3. PID 
float Kp_vel = 0.17;   
float Ki_vel = 2.0;   
float Kd_vel = 0.005;

// Variables internas PID
float err_sum_L = 0, last_err_L = 0;
float err_sum_R = 0, last_err_R = 0;
// Filtro derivativo
float derivada_filtrada_L = 0;
float derivada_filtrada_R = 0;
float alpha = 0.85;

// ==========================================
// 4. VARIABLES GLOBALES
volatile long counts_L = 0;
volatile long counts_R = 0;
long prev_counts_L = 0;
long prev_counts_R = 0;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

// Trayectoria Cuadrado 0.5x0.5m
float pathX[] = {0.0, 0.5, 0.5, 0.0, 0.0}; 
float pathY[] = {0.0, 0.0, 0.5, 0.5, 0.0};
int currentPoint = 1; 
int totalPoints = 5;

bool robot_activo = false; 
volatile bool flag_control = false; 
hw_timer_t *timer = NULL;
const int MAX_PWM = 1023;

// ==========================================
// 5. INTERRUPCIONES
void IRAM_ATTR isr_L_A() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L++; else counts_L--;
}
void IRAM_ATTR isr_L_B() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L--; else counts_L++;
}
void IRAM_ATTR isr_R_A() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R++; else counts_R--;
}
void IRAM_ATTR isr_R_B() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R--; else counts_R++;
}

// ==========================================
// 6. FUNCIONES AUXILIARES
double normalizeAngle(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  
  // Zona muerta
  if (abs(pwm) < 40) pwm = 0;

  if (pwm >= 0) {
    ledcWrite(pinA, (int)pwm);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, (int)-pwm);
  }
}

// PID con filtro en la derivada
double computePID(float ref, float med, float &err_sum, float &last_err, float &der_filt) {
  float error = ref - med;
  
  // Derivada filtrada
  float dev = (error - last_err) / Ts;
  der_filt = alpha * der_filt + (1.0 - alpha) * dev;

  // Integral con anti-windup
  err_sum += error * Ts;
  if (err_sum > 4000) err_sum = 4000;   
  if (err_sum < -4000) err_sum = -4000;
  
  double u = Kp_vel * error + Ki_vel * err_sum + Kd_vel * der_filt;
  
  last_err = error;
  return u;
}

// ==========================================
// 7. CONTROL PRINCIPAL
void ejecutarControl() {
  if (!robot_activo) return;

  // ODOMETRÍA 
  long d_counts_L = counts_L - prev_counts_L;
  long d_counts_R = counts_R - prev_counts_R;
  prev_counts_L = counts_L;
  prev_counts_R = counts_R;

  double ds_L = (2.0 * PI * R * d_counts_L) / N_VUELTA;
  double ds_R = (2.0 * PI * R * d_counts_R) / N_VUELTA;
  double ds = (ds_R + ds_L) / 2.0;
  double dtheta = (ds_R - ds_L) / L;

  x_pos += ds * cos(theta + dtheta / 2.0);
  y_pos += ds * sin(theta + dtheta / 2.0);
  theta += dtheta;
  theta = normalizeAngle(theta);

  // NAVEGACIÓN 
  float v_ref = 0.0;
  float w_ref = 0.0;

  if (currentPoint < totalPoints) {
    double dx = pathX[currentPoint] - x_pos;
    double dy = pathY[currentPoint] - y_pos;
    double dist_error = sqrt(dx*dx + dy*dy);

    if (dist_error < 0.05) { // Llegada al punto (5cm tolerancia)
      currentPoint++; 
      // Resetear integrales
      err_sum_L = 0; err_sum_R = 0; 
      // Parada breve
      setMotor(PULSO_L_A, PULSO_L_B, 0);
      setMotor(PULSO_R_A, PULSO_R_B, 0);
      return; 
    } 
    
    double angle_target = atan2(dy, dx);
    double angle_error = normalizeAngle(angle_target - theta);

    if (abs(angle_error) > 0.2) { 
       v_ref = 0.0; 
       // Giro proporcional
       w_ref = 3.0 * angle_error; 
       
       // Saturar giro
       if (w_ref > 2.0) w_ref = 2.0;
       if (w_ref < -2.0) w_ref = -2.0;
       
    } else {
       // AVANZAR Y CORREGIR SUAVEMENTE
       v_ref = 0.20; 
       w_ref = 1.5 * angle_error; 
    }
  } else {
    // FIN
    setMotor(PULSO_L_A, PULSO_L_B, 0);
    setMotor(PULSO_R_A, PULSO_R_B, 0);
    robot_activo = false;
    return;
  }

  // CINEMÁTICA 
  double v_wheel_R = v_ref + w_ref * (L / 2.0);
  double v_wheel_L = v_ref - w_ref * (L / 2.0);

  // REFERENCIA EN CUENTAS/SEGUNDO
  double ref_counts_R = (v_wheel_R / (2.0 * PI * R)) * N_VUELTA;
  double ref_counts_L = (v_wheel_L / (2.0 * PI * R)) * N_VUELTA;

  // MEDIDA REAL EN CUENTAS/SEGUNDO
  double vel_real_L = d_counts_L / Ts;
  double vel_real_R = d_counts_R / Ts;
  
  // LLAMADA A PID
  double pwm_L = computePID(ref_counts_L, vel_real_L, err_sum_L, last_err_L, derivada_filtrada_L);
  double pwm_R = computePID(ref_counts_R, vel_real_R, err_sum_R, last_err_R, derivada_filtrada_R);

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

void IRAM_ATTR onTimer() {
  flag_control = true; 
}

// ==========================================
// 8. SETUP
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

  Serial.println("ESPERANDO 3 SEGUNDOS...");
  delay(3000); 

  noInterrupts();
  counts_L = 0; counts_R = 0; 
  prev_counts_L = 0; prev_counts_R = 0;
  interrupts();
  
  robot_activo = true; 
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