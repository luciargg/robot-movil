# Proyecto Robot de Tracción Diferencial
### Grado en Ingeniería Electrónica, Robótica y Mecatrónica
**Universidad de Málaga (UMA)**  
**Departamento:** Ingeniería de Sistemas y Automática 

---

## 📝 Descripción del Proyecto
Este repositorio contiene el conjunto de software y firmware desarrollado para el diseño, control y navegación de un robot móvil de tracción diferencial basado en el microcontrolador **ESP32**. El proyecto se estructura en diferentes tareas, que van incrementando en dificultad, abarcando desde el modelado cinemático básico en bucle abierto hasta la integración en entornos distribuidos **ROS 2** mediante **micro-ROS** y el control web.

---

## 🔌 Arquitectura de Hardware y Conexionado

El núcleo físico del sistema se compone de un chasis diferencial, un controlador de potencia y una matriz de sensores periféricos coordinados por el ESP32.

### ⚙️ Conexiones del Sistema por Motor
| Componente Origen | Pin/Salida | Componente Destino | Función Técnica |
| :--- | :--- | :--- | :--- |
| **Driver L298N** | IN1 / IN2 | ESP32 - **PIN 32 / PIN 33** | Señales PWM control Motor 1 (Izq) |
| **Driver L298N** | IN3 / IN4 | ESP32 - **PIN 27 / PIN 14** | Señales PWM control Motor 2 (Der) |
| **Driver L298N** | OUT1 / OUT2 | Motor 1 | Blanco y Rojo (Alimentación Motor 1)|
| **Driver L298N** | OUT3 / OUT4 | Motor 2 | Rojo y Blanco (Alimentación Motor 2) |
| **Encoder Motor 1**| Amarillo / Verde | ESP32 - **PIN 26 / PIN 25** | Fase A / Fase B (Interrupciones)|
| **Encoder Motor 2**| Amarillo / Verde | ESP32 - **PIN 4 / PIN 13** | Fase A / Fase B (Interrupciones) |

### 🔋 Conexiones de Alimentación y Sensores
* **Alimentación de Potencia:** Bornes `UPS+` acoplados a los **12V** del driver L298N mediante un interruptor general de seguridad.
* **Referencia Común:** Interconexión obligatoria de `UPS-`, `GND` del ESP32, `GND` de encoders y `GND` del driver L298N para asegurar coherencia en los niveles lógicos.
* **Sensores de Proximidad Infrarrojos:**
  | Sensor Izquierda | Sensor Centro | Sensor Derecha |
  | :---: | :---: | :---: |
  | **PIN 35** | **PIN 39** | **PIN 34** |

---

## 📐 Fundamentos Teóricos y Modelo Cinemático

Para la traslación de comandos del mundo físico al plano discreto del microcontrolador se adoptan los siguientes parámetros físicos base[cite: 2]:
* **Radio de la rueda ($R$):** $0.0325 \text{ m}$
* **Distancia entre ruedas ($L$):** $0.197 \text{ m}$
* **Resolución del Encoder ($N$):** $1680 \text{ pulsos/vuelta}$

### 🔄 Cinemática Inversa Diferencial
A partir de las velocidades deseadas de la plataforma (lineal $v$ y angular $\omega$), se determinan las consignas individuales de cada rueda[cite: 2]:

$$v_{izq} = v - \frac{\omega \cdot L}{2}$$

$$v_{der} = v + \frac{\omega \cdot L}{2}$$

### 🧭 Odometría y Estimación de Pose
En cada periodo de muestreo ($T_s = 10\text{ ms}$), se evalúa la variación discreta de pulsos de los encoders ($\Delta \text{counts}$). El desplazamiento lineal diferencial ($\Delta s$) e integración de la pose se calculan mediante aproximación de Runge-Kutta de orden inferior:

$$\Delta s_i = \frac{2\pi R \cdot \Delta \text{counts}_i}{N} \quad (i = L, R)$$

$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}$$

$$x_{k+1} = x_k + \Delta s \cdot \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

$$y_{k+1} = y_k + \Delta s \cdot \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

$$\theta_{k+1} = \theta_k + \Delta \theta$$

---

## 📂 Estructura de Tareas y Desarrollo del Software

El repositorio está organizado jerárquicamente siguiendo la evolución metodológica de la práctica:

### [Tarea 1](./proyecto_final/tarea1) & [Tarea 3](./proyecto_final/tarea3): Control en Bucle Abierto y Temporización
* **Objetivo 1:** Movimiento rectilíneo uniforme a $0.1\text{ m/s}$ durante $20\text{ s}$ ($2.0\text{ m}$ teóricos).
* **Objetivo 3:** Trayectoria circular de radio $R_{giro} = 0.4\text{ m}$ a velocidad angular $\omega = \frac{2\pi}{10}\text{ rad/s}$ durante $10\text{ s}$.
* **Estrategia:** Uso de constantes de calibración empíricas (`K_OPEN_LOOP = 2200.0`)[cite: 2]. Control de estados mediante ventanas temporales estrictas con `millis()`.

### [Tarea 2](./proyecto_final/tarea2): Odometría Lineal y Frenado Activo
* **Objetivo:** Recorrer exactamente $2\text{ metros}$ lineales interrumpiendo el bucle por distancia acumulada y no por tiempo[cite: 2].
* **Mitigación de Inercia:** Debido al deslizamiento inherente del lazo abierto, se implementaron dos técnicas avanzadas[cite: 2]:
  1. *Compensación de Umbral:* Parada a los $1.75\text{ m}$ (absorbiendo $25\text{ cm}$ de inercia).
  2. *Contramarcha Activa:* Inversión instantánea de polaridad (`PWM = -400`) durante un pulso crítico de $50\text{ ms}$ para clavar el robot en el punto objetivo.

### 🎯 [Tarea 4](./proyecto_final/tarea4): Lazo Cerrado de Velocidad y Orientación
* **Control de Orientación:** Se almacena el ángulo inicial $\theta_{ref}$. Un controlador Proporcional ($K_{p\_orientacion} = 2.2$) regula el error angular normalizado entre $[-\pi, \pi]$ para corregir desviaciones transversales en tiempo real.
* **Control de Velocidad:** Implementación de lazos **PID** discretos de bajo nivel de cuentas por segundo ($K_p = 0.17$, $K_i = 2.0$, $K_d = 0.005$) con frenado suavizado predictivo (`PWM = -350` por $40\text{ ms}$).

### 🗺️ [Tarea 5](./proyecto_final/tarea5): Guiado Autónomo "Follow the Carrot"
* **Lógica de Control:** Navegación por un array de waypoints que definen un perfil cuadrado de $0.5 \times 0.5\text{ m}$.
* **Máquina de Estados Cinemática:**
  * **Giro Puro:** Si el error angular $|e_\theta| > 0.2\text{ rad}$, el robot se detiene en traslación ($v=0$) y ejecuta rotación pura sobre su eje mitrada por saturación estricta a $\pm 2.0\text{ rad/s}$.
  * **Avance con Corrección:** Cuando se alinea, avanza a $0.20\text{ m/s}$ aplicando correcciones cinemáticas suaves hacia el objetivo.
* **PID Robusto:** Incorporación de **filtro derivativo de primer orden** y algoritmos **anti-windup** en las integrales de velocidad para anular picos de sobrecorriente y saturación ante transitorios bruscos.

### 🛡️ [Tarea 6](./proyecto_final/tarea6): Concurrencia y Seguridad Reactiva
* **Lectura:** Muestreo analógico de los 3 sensores infrarrojos buscando umbrales críticos de tensión ajustados por calibración (`UMBRAL_OBSTACULO = 300`).
* **Rutina de Emergencia:** Ante detección positiva, se inhiben las consignas de guiado, forzando de inmediato $(v=0, \omega=0)$ y activando la bandera `pausa_obstaculo`. Al desaparecer la amenaza, el sistema ejecuta un estado de retardo de seguridad ininterrumpido de $3\text{ segundos}$ mediante temporizadores antes de reanudar la marcha.

### 🌐 [Tarea 7](./proyecto_final/tarea7): Capa de Middleware (micro-ROS y ROS 2)
* **Infraestructura:** Conexión inalámbrica UDP a través de la pila Wi-Fi nativa del ESP32 hacia un agente local (`IP 172.20.10.4`).
* **Grafo de ROS 2:**
  * **Suscriptor:** Canalizado al topic `/cmd_vel` (`geometry_msgs/msg/Twist`) para la captura dinámica de comandos cinemáticos de teleoperación o planificadores de alto nivel[cite: 2].
  * **Publicador:** Canalizado al topic `/turtlebot_pose` (`geometry_msgs/msg/Pose2D`) inyectando la telemetría de odometría discretizada a frecuencias estrictas de $100\text{ Hz}$ ($T_s = 10\text{ ms}$)[cite: 2].

### 📱 [Tarea Opcional](./proyecto_final/tareaOpcional): Estación de Control Embebida Web
* **Arquitectura:** Configuración del ESP32 en modo Punto de Acceso (AP) bajo el SSID `"labrob"` levantando un servidor web HTTP autónomo sin dependencias externas[cite: 2].
* **Lógica Asíncrona:** Rutas asíncronas de bajo impacto HTTP GET (`/F`, `/B`, `/L`, `/R`, `/S`) asociadas a botones de la interfaz móvil para control en tiempo real e inclusión de un botón dominante de **STOP DE EMERGENCIA**.
* **Suavizado Exponencial (Perfil de Rampa):** Para proteger la mecánica de transitorios bruscos de par, el firmware filtra la consigna objetivo mediante una rampa exponencial:
  
  $$v_{actual} = v_{actual} \cdot (1 - \alpha) + v_{target} \cdot \alpha \quad (\alpha = 0.05)$$

---

## 📱 Interfaz de Usuario Remota

<p align="center">
  <img src="https://images.unsplash.com/photo-1546776310-eef45dd6d63c?q=80&w=300" alt="Ejemplo GUI Robot" width="280">
  <br>
  <i>La aplicación embebida proporciona control cinemático, diagnóstico y parada de emergencia instantánea[cite: 2].</i>
</p>

---

## 🛠️ Requisitos e Instalación

1. **Instalación de Dependencias en Arduino IDE / VS Code:**
   * Soporte de tarjetas ESP32 de Expressif.
   * Librería `micro_ros_arduino` (para la compilación de la Tarea 7)[cite: 2].
2. **Despliegue del Agente micro-ROS (PC con ROS 2 Humble/Iron):**
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888