#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ==========================================
// 1. CONFIGURACIÓN WIFI
const char* ssid = "labrob";
const char* password = "";              

WebServer server(80); 

// ==========================================
// 2. HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Control Robot</title>
  <style>
    body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px; background-color: #222; color: white; }
    .button { display: inline-block; padding: 15px 25px; font-size: 24px; cursor: pointer; text-align: center; text-decoration: none; outline: none; color: #fff; background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; margin: 10px; width: 120px; -webkit-user-select: none; }
    .button:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }
    .stop { background-color: #f44336; width: 80%; }
    .nav { background-color: #2196F3; }
  </style>
</head>
<body>
  <h1>Control Robot Movil</h1>
  <p>ESTADO: <span id="estado">CONECTADO</span></p>
  
  <button class="button nav" onmousedown="send('/F')" onmouseup="send('/S')" ontouchstart="send('/F')" ontouchend="send('/S')">AVANZA</button><br>
  
  <div style="display:flex; justify-content:center;">
    <button class="button nav" onmousedown="send('/L')" onmouseup="send('/S')" ontouchstart="send('/L')" ontouchend="send('/S')">IZQ</button>
    <button class="button nav" onmousedown="send('/R')" onmouseup="send('/S')" ontouchstart="send('/R')" ontouchend="send('/S')">DER</button>
  </div>
  
  <button class="button nav" onmousedown="send('/B')" onmouseup="send('/S')" ontouchstart="send('/B')" ontouchend="send('/S')">ATRAS</button><br><br>
  
  <button class="button stop" onclick="send('/S')">STOP DE EMERGENCIA</button>

  <script>
    function send(cmd) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", cmd, true);
      xhr.send();
    }
  </script>
</body>
</html>
)rawliteral";

// ==========================================
// 3. PINES Y VARIABLES
#define ENC_L_A 25
#define ENC_L_B 26
#define PULSO_L_A 32    
#define PULSO_L_B 33    
#define ENC_R_A 4
#define ENC_R_B 13
#define PULSO_R_A 27    
#define PULSO_R_B 14    

const int PIN_SENSOR_1 = 34; 
const int PIN_SENSOR_2 = 35; 
const int PIN_SENSOR_3 = 39; 
const int UMBRAL_OBSTACULO = 300; 

const float R_RUEDA = 0.0325;   
const float L_EJE = 0.197;      
const float N_VUELTA = 1680.0;  
const float Ts = 0.01;          

float Kp_vel = 0.17; float Ki_vel = 2.0; float Kd_vel = 0.005;
float err_sum_L = 0, last_err_L = 0, der_filt_L = 0;
float err_sum_R = 0, last_err_R = 0, der_filt_R = 0;
float alpha = 0.85; 

volatile long counts_L = 0; volatile long counts_R = 0;
long prev_counts_L = 0; long prev_counts_R = 0;


float v_ref_usuario = 0.0; 
float w_ref_usuario = 0.0; 
float v_actual_suavizada = 0.0; // Velocidad real filtrada
float w_actual_suavizada = 0.0; // Giro real filtrado

volatile bool flag_control = false; 
hw_timer_t *timer = NULL;
const int MAX_PWM = 1023;

// ==========================================
// 4. FUNCIONES WEB (Velocidades más bajas)
void handleRoot() { server.send(200, "text/html", index_html); }
void handleF() { v_ref_usuario = 0.18; w_ref_usuario = 0; server.send(200, "text/plain", "OK"); }
void handleB() { v_ref_usuario = -0.18; w_ref_usuario = 0; server.send(200, "text/plain", "OK"); }
void handleL() { v_ref_usuario = 0; w_ref_usuario = 1.5; server.send(200, "text/plain", "OK"); }
void handleR() { v_ref_usuario = 0; w_ref_usuario = -1.5; server.send(200, "text/plain", "OK"); }
void handleS() { v_ref_usuario = 0; w_ref_usuario = 0; server.send(200, "text/plain", "OK"); }

// ==========================================
// 5. CONTROL
void IRAM_ATTR isr_L_A() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L++; else counts_L--; }
void IRAM_ATTR isr_L_B() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L--; else counts_L++; }
void IRAM_ATTR isr_R_A() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R++; else counts_R--; }
void IRAM_ATTR isr_R_B() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R--; else counts_R++; }
void IRAM_ATTR onTimer() { flag_control = true; }

void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > MAX_PWM) pwm = MAX_PWM; if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  if (abs(pwm) < 40) pwm = 0; 
  if (pwm >= 0) { ledcWrite(pinA, (int)pwm); ledcWrite(pinB, 0); } 
  else { ledcWrite(pinA, 0); ledcWrite(pinB, (int)-pwm); }
}

double computePID(float ref, float med, float &err_sum, float &last_err, float &der_filt) {
  float error = ref - med;
  float dev = (error - last_err) / Ts;
  der_filt = alpha * der_filt + (1.0 - alpha) * dev;
  err_sum += error * Ts;
  if (err_sum > 4000) err_sum = 4000; if (err_sum < -4000) err_sum = -4000;
  if (ref == 0 && abs(error) < 5) err_sum = 0;
  double u = Kp_vel * error + Ki_vel * err_sum + Kd_vel * der_filt;
  last_err = error;
  return u;
}

bool hayObstaculo() {
  if (analogRead(PIN_SENSOR_1) < UMBRAL_OBSTACULO || 
      analogRead(PIN_SENSOR_2) < UMBRAL_OBSTACULO || 
      analogRead(PIN_SENSOR_3) < UMBRAL_OBSTACULO) return true;
  return false;
}

void ejecutarControl() {
  // 
  bool obstaculo = hayObstaculo();
  float v_target = v_ref_usuario;
  
  if (v_ref_usuario > 0 && obstaculo) {
      v_target = 0.0; 
  }

  // FILTRO DE SUAVIZADO (RAMPA)
  float factor_rampa = 0.05; 
  
  v_actual_suavizada = (v_actual_suavizada * (1.0 - factor_rampa)) + (v_target * factor_rampa);
  w_actual_suavizada = (w_actual_suavizada * (1.0 - factor_rampa)) + (w_ref_usuario * factor_rampa);

  if(abs(v_target) < 0.01 && abs(v_actual_suavizada) < 0.01) v_actual_suavizada = 0;

  // --- Cinemática usando la velocidad suavizada ---
  double v_wheel_R = v_actual_suavizada + w_actual_suavizada * (L_EJE / 2.0);
  double v_wheel_L = v_actual_suavizada - w_actual_suavizada * (L_EJE / 2.0);
  
  double ref_counts_R = (v_wheel_R / (2.0 * PI * R_RUEDA)) * N_VUELTA;
  double ref_counts_L = (v_wheel_L / (2.0 * PI * R_RUEDA)) * N_VUELTA;

  long d_counts_L = counts_L - prev_counts_L;
  long d_counts_R = counts_R - prev_counts_R;
  prev_counts_L = counts_L; prev_counts_R = counts_R;

  double vel_real_L = d_counts_L / Ts;
  double vel_real_R = d_counts_R / Ts;

  double pwm_L = computePID(ref_counts_L, vel_real_L, err_sum_L, last_err_L, der_filt_L);
  double pwm_R = computePID(ref_counts_R, vel_real_R, err_sum_R, last_err_R, der_filt_R);

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

// ==========================================
// 6. SETUP & LOOP
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  
  Serial.begin(115200);

  WiFi.setTxPower(WIFI_POWER_11dBm); 
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP: "); Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/F", handleF); server.on("/B", handleB);
  server.on("/L", handleL); server.on("/R", handleR);
  server.on("/S", handleS);
  server.begin();

  ledcAttach(PULSO_L_A, 1000, 10); ledcAttach(PULSO_L_B, 1000, 10);
  ledcAttach(PULSO_R_A, 1000, 10); ledcAttach(PULSO_R_B, 1000, 10);
  ledcWrite(PULSO_L_A, 0); ledcWrite(PULSO_L_B, 0);
  ledcWrite(PULSO_R_A, 0); ledcWrite(PULSO_R_B, 0);

  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_L_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_R_B, CHANGE);
  
  pinMode(PIN_SENSOR_1, INPUT); pinMode(PIN_SENSOR_2, INPUT); pinMode(PIN_SENSOR_3, INPUT);

  noInterrupts(); counts_L = 0; counts_R = 0; interrupts();
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0);
}

void loop() {
  server.handleClient();
  if (flag_control) {
    flag_control = false;
    ejecutarControl();
  }
}