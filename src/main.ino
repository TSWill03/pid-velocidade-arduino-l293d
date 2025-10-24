/* =========================================================
   Controle de Velocidade por PID (Arduino UNO + L293D)
   Autores: Wícolly Pedro Alcântara; Fernando Frazão Moreira Nunes
   =========================================================
   Hardware:
   - L293D: EN12->D5 (PWM), EN34->D6 (PWM), IN1->D2, IN2->D9, IN3->D4, IN4->D7
   - Tacômetro (IR/Hall): D3 (INT1, RISING), com INPUT_PULLUP
   - PING))) (um fio): D8 (SIG único, toggle OUTPUT/INPUT na leitura)
   - Buzzer D13, LED vermelho D12, LED verde D11
   - GND da fonte de motor em COMUM com GND do Arduino
   ========================================================= */



/* ================= PINOUT ================= */
const int BUZZ   = 13;
const int LED_R  = 12;
const int LED_G  = 11;

const int ULTRA_SIG = 8;   // PING))) SIG (um fio)

const int IN1   = 2;       // direção motor A
const int IN2   = 9;       // direção motor A
const int EN12  = 5;       // PWM motor A (enable)

const int IN3   = 4;       // direção motor B
const int IN4   = 7;       // direção motor B
const int EN34  = 6;       // PWM motor B (enable)

const int TAC_PIN = 3;     // Tacômetro em INT1 (D3)

/* ================ PARÂMETROS ================ */
// Segurança por ultrassom
#define USE_SAFETY_ULTRA 1
const int   FREIO_CM     = 35;     // <= freia por completo (short-brake)
const int   DIST_OK_CM   = 100;    // acima disso, sem limitação de PWM

// PID de velocidade (RPM)
const int   SETPOINT_RPM = 1200;   // alvo de velocidade (ajuste conforme)
const float Kp = 0.8f;
const float Ki = 0.15f;
const float Kd = 0.02f;
const float beta  = 1.0f;          // ponderação do setpoint (2-DOF)
const float alpha = 0.90f;         // filtro do termo D (0<alpha<1)
const float Kt    = 0.20f;         // anti-windup (back-calculation)

const uint16_t Ts_ms = 50;         // amostragem do PID (20 Hz)
const int PWM_MAX    = 255;

const int  PULSOS_POR_VOLTA = 1;   // ajuste conforme disco/ímã (1 marca = 1)

/* ================ VARIÁVEIS ================ */
// Estados do PID
float Iterm = 0.0f, d_f = 0.0f, y_prev = 0.0f;

// Tacômetro (protegido por 'volatile' e cópias atômicas)
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long periodMicros    = 0;

// Temporização principal
unsigned long tLoop = 0;

/* ============ PROTÓTIPOS DE FUNÇÕES ============ */
float medirDistanciaCM();                 // PING)))
void  setDirecaoFrente();
void  aplicarPWM(int u);
void  freioAtivo();
void  pararMotores();
float rpmMedida();
int   pidStep(float r, float y);
void  onPulse();
int   pwmCapFromDist(float d);

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZ, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);

  pinMode(EN12, OUTPUT);
  pinMode(EN34, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ULTRA_SIG será alternado na função de medida (OUTPUT->INPUT). Não fixar aqui.

  // Tacômetro (entrada com pullup — típico em sensores Hall open-collector)
  pinMode(TAC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TAC_PIN), onPulse, RISING);

  pararMotores();
  digitalWrite(BUZZ, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, LOW);

  // Cabeçalho do logger (CSV)
  Serial.println(F("t_ms,set_rpm,rpm,pwm,dist_cm,pwm_cap"));
  tLoop = millis();
}

/* ================== LOOP ================== */
void loop() {
  if (millis() - tLoop < Ts_ms) return;
  tLoop += Ts_ms;

  // Medida de velocidade (RPM)
  float y = rpmMedida();

  // Segurança por distância
  float dist = 999.0f;
#if USE_SAFETY_ULTRA
  dist = medirDistanciaCM();

  // Emergência: short-brake abaixo/igual ao FREIO_CM
  if (dist <= FREIO_CM) {
    freioAtivo();
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R, HIGH);
    digitalWrite(BUZZ, HIGH);

    // zera integrador para evitar “arrasto” ao sair da saturação
    Iterm = 0.0f;

    // log e retorna
    Serial.print(millis()); Serial.print(',');
    Serial.print(SETPOINT_RPM); Serial.print(',');
    Serial.print(y); Serial.print(',');
    Serial.print(0); Serial.print(',');
    Serial.print(dist); Serial.print(',');
    Serial.println(0); // pwm_cap = 0
    return;
  }
#endif

  // Controle por PID no setpoint desejado
  int u = pidStep((float)SETPOINT_RPM, y);

  // Cap de segurança progressivo pela distância
  int cap = pwmCapFromDist(dist);
  u = min(u, cap);

  aplicarPWM(u);

  // Indicadores
  digitalWrite(BUZZ, LOW);
  if (dist >= DIST_OK_CM) { digitalWrite(LED_G, HIGH); digitalWrite(LED_R, LOW); }
  else                    { digitalWrite(LED_G, LOW);  digitalWrite(LED_R, HIGH); }

  // Logger CSV
  Serial.print(millis()); Serial.print(',');
  Serial.print(SETPOINT_RPM); Serial.print(',');
  Serial.print(y); Serial.print(',');
  Serial.print(u); Serial.print(',');
  Serial.print(dist); Serial.print(',');
  Serial.println(cap);
}

/* ============= MEDIÇÃO DE DISTÂNCIA (PING))) ============= */
float medirDistanciaCM() {
  // Pulso de trigger (modo OUTPUT)
  pinMode(ULTRA_SIG, OUTPUT);
  digitalWrite(ULTRA_SIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_SIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRA_SIG, LOW);

  // Leitura do echo (modo INPUT)
  pinMode(ULTRA_SIG, INPUT);
  unsigned long dur = pulseIn(ULTRA_SIG, HIGH, 30000UL); // timeout 30 ms (~5 m)
  if (dur == 0) return 999.0f;
  return dur / 58.0f; // us->cm (aprox.)
}

/* ============= MOTORES / L293D ============= */
void setDirecaoFrente() {
  // Ajuste LOW/HIGH aqui se algum motor estiver invertido
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void aplicarPWM(int u) {
  u = constrain(u, 0, PWM_MAX);
  setDirecaoFrente();
  analogWrite(EN12, u);
  analogWrite(EN34, u);
}

void pararMotores() {
  analogWrite(EN12, 0);
  analogWrite(EN34, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void freioAtivo() {
  // Short-brake REAL: EN alto e entradas iguais
  analogWrite(EN12, PWM_MAX);
  analogWrite(EN34, PWM_MAX);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

/* ============= TACÔMETRO (RPM) ============= */
// ISR: mede período entre pulsos
void onPulse() {
  unsigned long now = micros();
  periodMicros = now - lastPulseMicros;
  lastPulseMicros = now;
}

// Função de leitura com timeout e filtro simples
float rpmMedida() {
  unsigned long Tcopy, lastCopy;
  noInterrupts();
  Tcopy    = periodMicros;
  lastCopy = lastPulseMicros;
  interrupts();

  // Sem pulsos recentes -> 0 RPM
  if ((micros() - lastCopy) > 400000UL) return 0.0f; // 0,4 s sem pulso

  if (Tcopy == 0) return 0.0f;

  float rpm = (60.0e6f / (float)Tcopy) / (float)PULSOS_POR_VOLTA;

  // Suavização (EWMA) para reduzir jitter
  static float rpm_f = 0.0f;
  const float a = 0.2f; // 0<a<=1
  rpm_f = (1.0f - a) * rpm_f + a * rpm;
  return rpm_f;
}

/* ============= PASSO DO PID (2-DOF + D filtrado + anti-windup) ============= */
int pidStep(float r, float y) {
  // Derivada sobre a MEDIÇÃO, filtrada 1a ordem
  float Ts = Ts_ms / 1000.0f;
  float dy = (y - y_prev) / Ts;
  d_f = alpha * d_f + (1.0f - alpha) * dy;

  // Pré-saturação
  float v = Kp * (beta * r - y) + Iterm - Kd * d_f;

  // Saturação 0..255
  float u = constrain(v, 0.0f, (float)PWM_MAX);

  // Anti-windup (back-calculation)
  Iterm += Ki * Ts * (r - y) + Kt * (u - v);

  // Limitar Iterm a uma faixa segura
  if (Iterm > PWM_MAX)  Iterm = PWM_MAX;
  if (Iterm < -PWM_MAX) Iterm = -PWM_MAX;

  y_prev = y;
  return (int)u;
}

/* ============= CAP DE PWM POR DISTÂNCIA (Rampa) ============= */
// DIST_OK_CM -> 255 ; FREIO_CM -> 0 ; interpolação linear no meio
int pwmCapFromDist(float d) {
  if (d >= DIST_OK_CM) return PWM_MAX;
  if (d <= FREIO_CM)   return 0;
  float f = (d - FREIO_CM) / (float)(DIST_OK_CM - FREIO_CM); // 0..1
  int cap = (int)(f * PWM_MAX);
  return constrain(cap, 0, PWM_MAX);
}
