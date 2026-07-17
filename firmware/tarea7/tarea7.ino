#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <math.h>

// ==========================================
// 1. CONFIGURACIÓN WIFI Y AGENTE
char ssid[] = "iPhone";   
char password[] = "pass1234word";     
char agent_ip[] = "172.20.10.4";    
size_t agent_port = 8888;

// ==========================================
// 2. PINES
#define ENC_L_A 25
#define ENC_L_B 26
#define PULSO_L_A 32    // PWM IN1
#define PULSO_L_B 33    // PWM IN2

#define ENC_R_A 4
#define ENC_R_B 13
#define PULSO_R_A 27    // PWM IN3
#define PULSO_R_B 14    // PWM IN4

const float R = 0.0325;        
const float L = 0.197;         
const float N_VUELTA = 1680.0; 
const float Ts = 0.01;        

// ==========================================
// 4. PID 
float Kp_vel = 0.17;   
float Ki_vel = 2.0;   
float Kd_vel = 0.005;

// Variables internas PID
float err_sum_L = 0, last_err_L = 0, der_filt_L = 0;
float err_sum_R = 0, last_err_R = 0, der_filt_R = 0;
float alpha = 0.85;

// ==========================================
// 5. VARIABLES GLOBALES ROS Y ROBOT
// Odometría
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

// Encoders
volatile long counts_L = 0;
volatile long counts_R = 0;
long prev_counts_L = 0;
long prev_counts_R = 0;

// Referencias de velocidad
float target_v = 0.0;
float target_w = 0.0;

// Objetos micro-ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
rcl_publisher_t publisher;
geometry_msgs__msg__Pose2D msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// PWM Config
const int freq = 1000;
const int resolution = 10;
const int MAX_PWM = 1023;

// ==========================================
// 6. INTERRUPCIONES ENCODER
void IRAM_ATTR isr_L_A() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L++; else counts_L--; }
void IRAM_ATTR isr_L_B() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) counts_L--; else counts_L++; }
void IRAM_ATTR isr_R_A() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R++; else counts_R--; }
void IRAM_ATTR isr_R_B() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) counts_R--; else counts_R++; }

// ==========================================
// 7. FUNCIONES AUXILIARES
double normalizeAngle(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void setMotor(int pinA, int pinB, double pwm) {
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;
  if (abs(pwm) < 40) pwm = 0; // Tu zona muerta

  if (pwm >= 0) {
    ledcWrite(pinA, (int)pwm);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, (int)-pwm);
  }
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
// 8. CALLBACKS DE MICRO-ROS

// Recibimos velocidad del PC (cmd_vel)
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  target_v = msg->linear.x;  
  target_w = msg->angular.z; 
}

// Bucle de Control (Se ejecuta cada 0.01s - 100Hz)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  // --- A. ODOMETRÍA ---
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

  // --- B. PUBLICAR POSICIÓN A ROS ---
  msg_pub.x = x_pos;
  msg_pub.y = y_pos;
  msg_pub.theta = theta;
  rcl_publish(&publisher, &msg_pub, NULL);

  // --- C. CINEMÁTICA INVERSA ---
  double v_wheel_R = target_v + target_w * (L / 2.0);
  double v_wheel_L = target_v - target_w * (L / 2.0);

  double ref_counts_R = (v_wheel_R / (2.0 * PI * R)) * N_VUELTA;
  double ref_counts_L = (v_wheel_L / (2.0 * PI * R)) * N_VUELTA;

  // --- D. PID ---
  double vel_real_L = d_counts_L / Ts;
  double vel_real_R = d_counts_R / Ts;

  double pwm_L = computePID(ref_counts_L, vel_real_L, err_sum_L, last_err_L, der_filt_L);
  double pwm_R = computePID(ref_counts_R, vel_real_R, err_sum_R, last_err_R, der_filt_R);

  setMotor(PULSO_L_A, PULSO_L_B, pwm_L);
  setMotor(PULSO_R_A, PULSO_R_B, pwm_R);
}

// ==========================================
// 9. SETUP Y LOOP
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- DEBUG INICIAL ---
  Serial.println();
  Serial.println("==================================");
  Serial.println("INICIANDO ESP32 ROBOT...");
  Serial.print("Intentando conectar a WiFi: "); Serial.println(ssid);
  Serial.print("Buscando Agente en: "); Serial.println(agent_ip);
  Serial.println("==================================");

  // Pines y PWM
  ledcAttach(PULSO_L_A, freq, resolution); ledcAttach(PULSO_L_B, freq, resolution);
  ledcAttach(PULSO_R_A, freq, resolution); ledcAttach(PULSO_R_B, freq, resolution);
  
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_L_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_L_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_R_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_R_B, CHANGE);

  // Conexión micro-ROS
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  Serial.println("--> WiFi Conectado! Intentando conectar con Agente ROS...");

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot", "", &support);

  Serial.println("--> AGENTE ENCONTRADO! Creando Publishers/Subscribers...");

  // SUBSCRIBER
  rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // PUBLISHER
  rclc_publisher_init_default(
    &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "turtlebot_pose");

  // TIMER PID
  rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(10), timer_callback);

  // EXECUTOR
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
  
  Serial.println("--> TODO LISTO. ESPERANDO COMANDOS...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}