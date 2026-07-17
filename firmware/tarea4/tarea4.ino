#include <Arduino.h>
#include <math.h>

// ==========================================
// 1. CONFIGURACIÓN DE PINES
#define ENC_L_A 25
#define ENC_L_B 26
#define PULSO_L_A 32
#define PULSO_L_B 33
#define ENC_R_A 4
#define ENC_R_B 13
#define PULSO_R_A 27
#define PULSO_R_B 14

// ==========================================
// 2. CONSTANTES FÍSICAS
const float R = 0.0325;
const float L = 0.197;
const float N_VUELTA = 1680.0;
const float Ts = 0.01;

// ==========================================
// 3. TUNING PID (TAREA 5)
float Kp_vel = 0.17;
float Ki_vel = 2.0;
float Kd_vel = 0.005;
float alpha = 0.85;
const float Kp_orientacion = 2.2;

float err_sum_L = 0, last_err_L = 0, der_filt_L = 0;
float err_sum_R = 0, last_err_R = 0, der_filt_R = 0;

// ==========================================
// 4. VARIABLES DE ESTADO
volatile long counts_L = 0, counts_R = 0;
long prev_counts_L = 0, prev_counts_R = 0;
long odom_prev_L = 0, odom_prev_R = 0;

double x_pos = 0.0, y_pos = 0.0, theta = 0.0;
double start_x = 0.0, start_y = 0.0, theta_ref = 0.0;

bool robot_activo = false;
bool estado_recien_iniciado = true;
volatile bool flag_control = false;
hw_timer_t *timer = NULL;

// ==========================================
// 5. INTERRUPCIONES
void IRAM_ATTR isr_L_A() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L++; else counts_L--; }
void IRAM_ATTR isr_L_B() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L--; else counts_L++; }
void IRAM_ATTR isr_R_A() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R++; else counts_R--; }
void IRAM_ATTR isr_R_B() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R--; else counts_R++; }
void IRAM_ATTR onTimer() { flag_control = true; }

// ==========================================
// 6. FUNCIONES AUXILIARES
double normalizeAngle(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > 1023) pwm = 1023; if (pwm < -1023) pwm = -1023;
  if (abs(pwm) < 45) pwm = 0;
  if (pwm >= 0) { ledcWrite(pinA, (int)pwm); ledcWrite(pinB, 0); }
  else { ledcWrite(pinA, 0); ledcWrite(pinB, (int)-pwm); }
}

double computePID(float ref, float med, float &err_sum, float &last_err, float &der_filt) {
  float error = ref - med;
  float dev = (error - last_err) / Ts;
  der_filt = alpha * der_filt + (1.0 - alpha) * dev;
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
  long d_counts_L = counts_L - odom_prev_L;
  long d_counts_R = counts_R - odom_prev_R;
  odom_prev_L = counts_L; odom_prev_R = counts_R;

  double ds_L = (2.0 * PI * R * d_counts_L) / N_VUELTA;
  double ds_R = (2.0 * PI * R * d_counts_R) / N_VUELTA;
  double ds = (ds_R + ds_L) / 2.0;
  double dtheta = (ds_R - ds_L) / L;

  x_pos += ds * cos(theta + dtheta / 2.0);
  y_pos += ds * sin(theta + dtheta / 2.0);
  theta = normalizeAngle(theta + dtheta);

  if (!robot_activo) return;

  if (estado_recien_iniciado) {
    start_x = x_pos; start_y = y_pos;
    theta_ref = theta;
    err_sum_L = 0; err_sum_R = 0;
    prev_counts_L = counts_L; prev_counts_R = counts_R;
    estado_recien_iniciado = false;
  }

  double distancia = sqrt(pow(x_pos - start_x, 2) + pow(y_pos - start_y, 2));
  
  if (distancia >= 1.75) { 
    // FRENADO MÁS SUAVE
    setMotor(PULSO_L_A, PULSO_L_B, -350); 
    setMotor(PULSO_R_A, PULSO_R_B, -350);
    delay(40); 
    
    setMotor(PULSO_L_A, PULSO_L_B, 0);
    setMotor(PULSO_R_A, PULSO_R_B, 0);
    robot_activo = false;
    Serial.printf(">>> LLEGADA. Distancia Odometría: %.2f m\n", distancia);
    return;
  }

  float v_ref = 0.20; 
  float w_ref = Kp_orientacion * normalizeAngle(theta_ref - theta);

  double v_wheel_R = v_ref + w_ref * (L / 2.0);
  double v_wheel_L = v_ref - w_ref * (L / 2.0);

  double ref_L = (v_wheel_L / (2.0 * PI * R)) * N_VUELTA;
  double ref_R = (v_wheel_R / (2.0 * PI * R)) * N_VUELTA;

  double vel_real_L = (counts_L - prev_counts_L) / Ts;
  double vel_real_R = (counts_R - prev_counts_R) / Ts;
  prev_counts_L = counts_L; prev_counts_R = counts_R;
  
  double pwm_L = computePID(ref_L, vel_real_L, err_sum_L, last_err_L, der_filt_L);
  double pwm_R = computePID(ref_R, vel_real_R, err_sum_R, last_err_R, der_filt_R);

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

// ==========================================
// 8. SETUP
void setup() {
  Serial.begin(115200);
  ledcAttach(PULSO_L_A, 1000, 10); ledcAttach(PULSO_L_B, 1000, 10);
  ledcAttach(PULSO_R_A, 1000, 10); ledcAttach(PULSO_R_B, 1000, 10);

  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_L_B, CHANGE);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_R_B, CHANGE);

  delay(4000);
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