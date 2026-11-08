#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================== PINOS ==============================
constexpr uint8_t PIN_PIR       = 27;
constexpr uint8_t PIN_LED_G     = 25;
constexpr uint8_t PIN_LED_Y     = 26;
constexpr uint8_t PIN_LED_R     = 33;
constexpr uint8_t PIN_BUZZER    = 32;
constexpr uint8_t PIN_SDA       = 21;
constexpr uint8_t PIN_SCL       = 22;
constexpr uint8_t PIN_BTN_PASS  = 4;    // botão de teste: +1 passagem
constexpr uint8_t PIN_BTN_CLEAR = 18;   // botão de teste: zera janela
constexpr uint8_t LCD_ADDR      = 0x27; // use 0x3F se o módulo real for PCF8574A

// ============================== REDE ===============================
const char* WIFI_SSID  = "Wokwi-GUEST";
const char* WIFI_PASS  = "";
const char* MQTT_HOST  = "broker.emqx.io";      
constexpr uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID  = "esp32-conecta-01";
const char* TOPIC_TELE = "conectabrasil/iot/sensores";
const char* TOPIC_STAT = "conectabrasil/iot/status";   // online/offline (LWT, retained)

// =========================== TEMPORIZAÇÃO ==========================
// (mantidos exatamente como estavam)
constexpr uint32_t PIR_REFRACTORY_MS = 500UL;    // intervalo mínimo entre ativações do PIR
constexpr uint32_t BTN_LOCKOUT_MS    = 30UL;     // anti-repique do botão (dentro da ISR)
constexpr uint32_t CONFIRM_UP_MS     = 1000UL;   // subir de estado
constexpr uint32_t CONFIRM_DOWN_MS   = 3000UL;   // descer de estado
constexpr uint32_t TELEMETRY_MS      = 5000UL;   // envio periódico
constexpr uint32_t TELE_MIN_GAP_MS   = 1000UL;   // envio por evento (no máx. 1/s)
constexpr uint32_t SNAPSHOT_MS       = 5000UL;   // resumo no Serial
constexpr uint32_t LCD_PERIOD_MS     = 250UL;
constexpr uint32_t WIFI_RETRY_MS     = 10000UL;
constexpr uint32_t MQTT_RETRY_MS     = 5000UL;
constexpr uint16_t MQTT_KEEPALIVE_S  = 30;
constexpr uint32_t BEEP_ON_MS        = 150UL;
constexpr uint32_t BEEP_PERIOD_MS    = 3000UL;
constexpr uint16_t BEEP_FREQ_HZ      = 2000;
constexpr bool     LOG_JSON          = true;     // mostra o JSON enviado no Serial

// ================= PERFIL DE CALIBRAÇÃO (janela + limiares) ==================
// 0 = PONTO DE ÔNIBUS (realista, PADRÃO)   |   1 = ESPECIFICAÇÃO DA DISCIPLINA (60 s, 0-2 / 3-5 / 6+)
#define PERFIL 0

#if PERFIL == 0
  // Modelo (valores de planejamento, calibrar com dados reais): ônibus a cada ~5 min e ~3 pessoas/min
  // chegando no pico => ~15 ativações por ciclo de ônibus (mais quem já estava no ponto: ~20-25 no total).
  //   BAIXA    0-5   (<= 1 pessoa/min)
  //   MODERADA 6-14  (1 a 3 pessoas/min)
  //   ALTA     15+   (>= 3 pessoas/min)
  constexpr uint32_t WINDOW_MS        = 300000UL;  // janela deslizante: 5 min
  constexpr uint8_t  TH_UP_MODERADA   = 6;
  constexpr uint8_t  TH_UP_ALTA       = 15;
  constexpr uint8_t  TH_DOWN_MODERADA = 3;         // histerese: MODERADA -> BAIXA só com <= 3
  constexpr uint8_t  TH_DOWN_ALTA     = 12;        // histerese: ALTA -> MODERADA só com <= 12
  constexpr uint8_t  BAR_SCALE        = 20;        // fundo de escala das barras (LCD e Serial)
#else
  constexpr uint32_t WINDOW_MS        = 60000UL;   // janela da especificação: 60 s
  constexpr uint8_t  TH_UP_MODERADA   = 3;
  constexpr uint8_t  TH_UP_ALTA       = 6;
  constexpr uint8_t  TH_DOWN_MODERADA = 1;
  constexpr uint8_t  TH_DOWN_ALTA     = 4;
  constexpr uint8_t  BAR_SCALE        = 12;
#endif
constexpr unsigned long WINDOW_S = WINDOW_MS / 1000UL;
constexpr uint8_t BAR_LCD_CELLS    = 12;             // cabe em 16 colunas: "[" + 12 + "]" + 2 dígitos
constexpr uint8_t BAR_SERIAL_CELLS = 20;

enum State : uint8_t { ST_BAIXA = 0, ST_MODERADA = 1, ST_ALTA = 2 };
const char* STATUS_NAME[] = { "BAIXA", "MODERADA", "ALTA" };

// ============================== OBJETOS ============================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// ================= JANELA DESLIZANTE (buffer circular) =============
constexpr uint8_t MAX_EVENTS = 200;          // comporta o pico da janela de 5 min
uint32_t evTimes[MAX_EVENTS];
uint8_t  evHead = 0, evCount = 0;

void purgeOldEvents(uint32_t now) {
  while (evCount > 0 && (uint32_t)(now - evTimes[evHead]) >= WINDOW_MS) {
    evHead = (evHead + 1) % MAX_EVENTS;
    evCount--;
  }
}
void pushEvent(uint32_t now) {
  if (evCount == MAX_EVENTS) { evHead = (evHead + 1) % MAX_EVENTS; evCount--; }
  evTimes[(evHead + evCount) % MAX_EVENTS] = now;
  evCount++;
}

// ============================ ESTADO GLOBAL ========================
State    currentState = ST_BAIXA, candidateState = ST_BAIXA;
bool     candidateActive = false;
uint32_t candidateSince = 0;

// ---- Fila de eventos preenchida por interrupção (produtor: ISR | consumidor: loop) ----
constexpr uint8_t SRC_PIR = 1, SRC_PASS = 2, SRC_CLEAR = 3;
constexpr uint8_t ISR_QUEUE = 16;
volatile uint32_t isrTime[ISR_QUEUE];
volatile uint8_t  isrSrc[ISR_QUEUE];
volatile uint8_t  isrHead = 0, isrTail = 0;
volatile uint32_t isrLastPir = 0, isrLastPass = 0, isrLastClear = 0;

bool     buzzerOn = false;
uint32_t beepEndAt = 0, nextBeepAt = 0;

bool     wifiWasUp = false, mqttWasUp = false, telePending = false;
uint32_t tLastTele = 0, tLastLcd = 0, tLastSnap = 0, tLastWifi = 0, tLastMqtt = 0;
char     clientId[40];
char     lcdPrev1[17] = "", lcdPrev2[17] = "";

// =============================== UTIL ==============================
void stamp(char* out, size_t n, uint32_t ms) {          // "[mm:ss]"
  uint32_t s = ms / 1000UL;
  snprintf(out, n, "[%02lu:%02lu]", (unsigned long)(s / 60UL), (unsigned long)(s % 60UL));
}
void buildBar(char* out, uint8_t cells, uint8_t n, char full, char empty) {   // barra proporcional ao fundo de escala
  uint16_t filled = ((uint16_t)n * cells + BAR_SCALE / 2) / BAR_SCALE;
  if (n > 0 && filled == 0) filled = 1;                    // qualquer ativação aparece na barra
  if (filled > cells) filled = cells;
  for (uint8_t i = 0; i < cells; i++) out[i] = (i < filled) ? full : empty;
  out[cells] = '\0';
}
const char* windowText() {                                  // "5min" ou "60s"
  static char b[12];
  if (WINDOW_S % 60UL == 0) snprintf(b, sizeof b, "%lumin", WINDOW_S / 60UL);
  else                      snprintf(b, sizeof b, "%lus", WINDOW_S);
  return b;
}
const char* mqttRcText(int rc) {
  switch (rc) {
    case -4: return "timeout";
    case -3: return "conexao perdida";
    case -2: return "nao abriu conexao (broker/porta/DNS)";
    case -1: return "desconectado";
    case 1:  return "protocolo recusado";
    case 2:  return "client id recusado";
    case 3:  return "broker indisponivel";
    case 4:  return "credenciais invalidas";
    case 5:  return "nao autorizado";
    default: return "erro desconhecido";
  }
}

// ============================== BUZZER =============================
void buzzerBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, BEEP_FREQ_HZ, 8);
#else
  ledcSetup(0, BEEP_FREQ_HZ, 8);
  ledcAttachPin(PIN_BUZZER, 0);
#endif
}
void buzzerTone(uint16_t f) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, f);
#else
  ledcWriteTone(0, f);
#endif
}
void updateBuzzer(uint32_t now) {            // bipe curto, periódico, não bloqueante (só em ALTA)
  if (currentState != ST_ALTA) {
    if (buzzerOn) { buzzerTone(0); buzzerOn = false; }
    return;
  }
  if (!buzzerOn) {
    if ((int32_t)(now - nextBeepAt) >= 0) {
      buzzerTone(BEEP_FREQ_HZ); buzzerOn = true; beepEndAt = now + BEEP_ON_MS;
    }
  } else if ((int32_t)(now - beepEndAt) >= 0) {
    buzzerTone(0); buzzerOn = false; nextBeepAt = now + BEEP_PERIOD_MS;
  }
}

void applyLeds(State s) {
  digitalWrite(PIN_LED_G, s == ST_BAIXA);
  digitalWrite(PIN_LED_Y, s == ST_MODERADA);
  digitalWrite(PIN_LED_R, s == ST_ALTA);
}

// ======================= ENTRADAS (PIR + BOTÕES) ===================
void registerActivation(uint32_t t, const char* src) {   // t = instante REAL do evento
  purgeOldEvents(t);
  pushEvent(t);
  telePending = true;                        // envia telemetria por evento
  char ts[12]; stamp(ts, sizeof ts, t);
  Serial.printf("%s >> PASSAGEM detectada (%s) | %02u ativ/%s na janela\n", ts, src, evCount, windowText());
}

// ---- ISRs: só guardam (instante, origem) na fila. Sem Serial, sem I2C, sem rede aqui dentro. ----
void IRAM_ATTR isrPush(uint8_t src, uint32_t t) {
  uint8_t next = (isrHead + 1) % ISR_QUEUE;
  if (next == isrTail) return;               // fila cheia: descarta (muito improvável)
  isrTime[isrHead] = t;
  isrSrc[isrHead]  = src;
  isrHead = next;
}
void IRAM_ATTR isrPir() {                    // borda de subida do PIR + período refratário
  uint32_t t = millis();
  if ((uint32_t)(t - isrLastPir) >= PIR_REFRACTORY_MS) { isrLastPir = t; isrPush(SRC_PIR, t); }
}
void IRAM_ATTR isrPass() {                   // botão verde: borda de descida
  uint32_t t = millis();
  if ((uint32_t)(t - isrLastPass) >= BTN_LOCKOUT_MS) { isrLastPass = t; isrPush(SRC_PASS, t); }
}
void IRAM_ATTR isrClear() {                  // botão preto: borda de descida
  uint32_t t = millis();
  if ((uint32_t)(t - isrLastClear) >= BTN_LOCKOUT_MS) { isrLastClear = t; isrPush(SRC_CLEAR, t); }
}

void clearWindow(uint32_t now);                // definida mais abaixo

void processInputEvents(uint32_t now) {      // consome a fila no loop principal
  while (isrTail != isrHead) {
    uint32_t t   = isrTime[isrTail];
    uint8_t  src = isrSrc[isrTail];
    isrTail = (isrTail + 1) % ISR_QUEUE;
    if (src == SRC_PIR)        registerActivation(t, "PIR");
    else if (src == SRC_PASS)  registerActivation(t, "botao");
    else if (src == SRC_CLEAR) clearWindow(now);
  }
}

// ============================= DECISÃO =============================
State targetState(State cur, uint8_t n) {
  switch (cur) {
    case ST_BAIXA:
      if (n >= TH_UP_ALTA)     return ST_ALTA;
      if (n >= TH_UP_MODERADA) return ST_MODERADA;
      return ST_BAIXA;
    case ST_MODERADA:
      if (n >= TH_UP_ALTA)       return ST_ALTA;
      if (n <= TH_DOWN_MODERADA) return ST_BAIXA;
      return ST_MODERADA;
    case ST_ALTA:
      if (n <= TH_DOWN_ALTA) return ST_MODERADA;   // desce um nível por vez
      return ST_ALTA;
  }
  return cur;
}

void setState(State s, uint32_t now) {
  char ts[12]; stamp(ts, sizeof ts, now);
  Serial.printf("%s ##### ESTADO: %s -> %s  (%02u ativ/%s) #####\n",
                ts, STATUS_NAME[currentState], STATUS_NAME[s], evCount, windowText());
  if (s == ST_ALTA) Serial.printf("%s !!!!! ALERTA: FLUXO ALTO NO PONTO !!!!!\n", ts);
  currentState = s; candidateActive = false;
  applyLeds(s);
  if (s == ST_ALTA) nextBeepAt = now;          // primeiro bipe imediato
  tLastLcd = 0;                                // força redesenho do LCD
  telePending = true;
}

void updateState(uint32_t now) {
  State t = targetState(currentState, evCount);
  if (t == currentState) { candidateActive = false; return; }
  if (!candidateActive || candidateState != t) {
    candidateState = t; candidateSince = now; candidateActive = true; return;
  }
  uint32_t need = (t > currentState) ? CONFIRM_UP_MS : CONFIRM_DOWN_MS;
  if ((now - candidateSince) >= need) setState(t, now);
}

void clearWindow(uint32_t now) {             // atalho de demonstração: zera tudo
  evHead = 0; evCount = 0; candidateActive = false;
  if (buzzerOn) { buzzerTone(0); buzzerOn = false; }
  currentState = ST_BAIXA; applyLeds(currentState);
  tLastLcd = 0; telePending = true;
  char ts[12]; stamp(ts, sizeof ts, now);
  Serial.printf("%s -- janela zerada (botao LIMPAR) -> BAIXA\n", ts);
}

// ================================ LCD ==============================
// Linha 1: estado (em ALTA alterna "Fluxo: ALTO" / "!!! ALERTA !!!")
// Linha 2: barra proporcional [█████-------]NN (NN = ativações na janela); a cada 6 s mostra Wi-Fi/MQTT por 2 s
void updateLcd(uint32_t now) {
  if ((now - tLastLcd) < LCD_PERIOD_MS) return;
  tLastLcd = now;

  char t1[40], t2[40], l1[17], l2[17], bar[BAR_LCD_CELLS + 1];
  bool phase = ((now / 1000UL) % 2UL) == 1UL;

  switch (currentState) {
    case ST_BAIXA:    snprintf(t1, sizeof t1, "Fluxo: Tranquilo"); break;
    case ST_MODERADA: snprintf(t1, sizeof t1, "Fluxo: Moderado");  break;
    default:          snprintf(t1, sizeof t1, phase ? " !!! ALERTA !!! " : "Fluxo: ALTO");  break;
  }

  if (((now / 1000UL) % 6UL) >= 4UL) {
    snprintf(t2, sizeof t2, "Wi-Fi:%s MQTT:%s",
             WiFi.status() == WL_CONNECTED ? "OK" : "--", mqtt.connected() ? "OK" : "--");
  } else {
    buildBar(bar, BAR_LCD_CELLS, evCount, '\x01', '-');
    snprintf(t2, sizeof t2, "[%s]%02u", bar, evCount > 99 ? 99 : evCount);
  }
  snprintf(l1, sizeof l1, "%-16.16s", t1);
  snprintf(l2, sizeof l2, "%-16.16s", t2);

  if (strcmp(l1, lcdPrev1)) { lcd.setCursor(0, 0); lcd.print(l1); strcpy(lcdPrev1, l1); }
  if (strcmp(l2, lcdPrev2)) { lcd.setCursor(0, 1); lcd.print(l2); strcpy(lcdPrev2, l2); }
}

// ======================= CONECTIVIDADE (não bloqueante) ============
void manageWifi(uint32_t now) {
  bool up = (WiFi.status() == WL_CONNECTED);
  if (up != wifiWasUp) {
    wifiWasUp = up;
    char ts[12]; stamp(ts, sizeof ts, now);
    if (up) Serial.printf("%s [REDE] Wi-Fi conectado | IP %s\n", ts, WiFi.localIP().toString().c_str());
    else    Serial.printf("%s [REDE] Wi-Fi caiu -> modo OFFLINE local (alertas continuam)\n", ts);
  }
  if (up) return;
  if (tLastWifi == 0 || (now - tLastWifi) >= WIFI_RETRY_MS) {
    tLastWifi = now ? now : 1;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  }
}

void manageMqtt(uint32_t now) {
  bool up = mqtt.connected();
  if (up != mqttWasUp) {
    mqttWasUp = up;
    char ts[12]; stamp(ts, sizeof ts, now);
    if (up) { Serial.printf("%s [REDE] MQTT conectado em %s\n", ts, MQTT_HOST); telePending = true; }
    else      Serial.printf("%s [REDE] MQTT desconectado -> telemetria pausada\n", ts);
  }
  if (WiFi.status() != WL_CONNECTED) return;
  if (up) { mqtt.loop(); return; }
  if (tLastMqtt != 0 && (now - tLastMqtt) < MQTT_RETRY_MS) return;
  tLastMqtt = now ? now : 1;
  if (mqtt.connect(clientId, nullptr, nullptr, TOPIC_STAT, 0, true, "offline")) {
    mqtt.publish(TOPIC_STAT, "online", true);
  } else {
    char ts[12]; stamp(ts, sizeof ts, now);
    Serial.printf("%s [REDE] MQTT falhou (rc=%d: %s) - nova tentativa em %lus\n",
                  ts, mqtt.state(), mqttRcText(mqtt.state()), (unsigned long)(MQTT_RETRY_MS / 1000UL));
  }
}

// ============================= TELEMETRIA ==========================
// Periódica (5 s) + por evento (nova ativação / mudança de estado, no máx. 1 por segundo).
void publishTelemetry(uint32_t now) {
  bool periodic = (now - tLastTele) >= TELEMETRY_MS;
  bool onEvent  = telePending && (now - tLastTele) >= TELE_MIN_GAP_MS;
  if (!periodic && !onEvent) return;
  tLastTele = now; telePending = false;
  if (!mqtt.connected()) return;

  char payload[160];
  snprintf(payload, sizeof payload,
           "{\"device_id\":\"%s\",\"activations_pm\":%u,\"status\":\"%s\",\"uptime_s\":%lu}",
           DEVICE_ID, evCount, STATUS_NAME[currentState], (unsigned long)(now / 1000UL));
  bool ok = mqtt.publish(TOPIC_TELE, payload);
  if (LOG_JSON) Serial.printf("          MQTT%s %s\n", ok ? ">" : " (FALHOU)>", payload);
}

void printSnapshot(uint32_t now) {           // 1 linha clara a cada 5 s
  if ((now - tLastSnap) < SNAPSHOT_MS) return;
  tLastSnap = now;
  char ts[12], bar[BAR_SERIAL_CELLS + 1];
  stamp(ts, sizeof ts, now);
  buildBar(bar, BAR_SERIAL_CELLS, evCount, '#', '-');
  Serial.printf("%s FLUXO %-8s | %02u ativ/%s [%s] | Wi-Fi %s | MQTT %s\n",
                ts, STATUS_NAME[currentState], evCount, windowText(), bar,
                WiFi.status() == WL_CONNECTED ? "OK " : "OFF", mqtt.connected() ? "OK " : "OFF");
}

// ============================== SETUP/LOOP =========================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BTN_PASS, INPUT_PULLUP);
  pinMode(PIN_BTN_CLEAR, INPUT_PULLUP);
  attachInterrupt(PIN_PIR,       isrPir,   RISING);    // movimento novo (LOW -> HIGH)
  attachInterrupt(PIN_BTN_PASS,  isrPass,  FALLING);   // botão apertado (pull-up: HIGH -> LOW)
  attachInterrupt(PIN_BTN_CLEAR, isrClear, FALLING);
  pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_Y, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  buzzerBegin(); buzzerTone(0);
  applyLeds(currentState);

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init(); lcd.backlight();
  uint8_t block[8] = { 31, 31, 31, 31, 31, 31, 31, 31 };   // caractere "bloco cheio"
  lcd.createChar(1, block);
  lcd.setCursor(0, 0); lcd.print(" CONECTA BRASIL ");
  lcd.setCursor(0, 1); lcd.print("Iniciando...    ");

  snprintf(clientId, sizeof clientId, "%s-%04lX", DEVICE_ID, (unsigned long)random(0xFFFF));  // ID único: evita derrubar sessão no broker público
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setSocketTimeout(1);   // limita o bloqueio do connect()/publish()

  Serial.println();
  Serial.println("==================================================================");
  Serial.println(" CONECTA BRASIL  |  Monitor de Ocupacao de Fluxo  |  ESP32 EDGE  |  v2.4");
  Serial.printf (" ativ/%s = chegadas (ativacoes do PIR) nos ULTIMOS %s - janela deslizante\n", windowText(), windowText());
  Serial.printf (" Estados : BAIXA 0-%u | MODERADA %u-%u | ALTA %u+  (ativacoes na janela)\n",
                 (unsigned)(TH_UP_MODERADA - 1), (unsigned)TH_UP_MODERADA, (unsigned)(TH_UP_ALTA - 1), (unsigned)TH_UP_ALTA);
  Serial.println(" Teste   : botao VERDE = +1 passagem | botao PRETO = limpar");
  Serial.printf (" Broker  : %s:%u | topico %s\n", MQTT_HOST, MQTT_PORT, TOPIC_TELE);
  Serial.println("==================================================================");
}

void loop() {
  uint32_t now = millis();

  // ---- Núcleo da borda (independe de rede) ----
  processInputEvents(now);
  purgeOldEvents(now);
  updateState(now);
  updateBuzzer(now);
  updateLcd(now);

  // ---- Conectividade e telemetria ----
  manageWifi(now);
  manageMqtt(now);
  publishTelemetry(now);
  printSnapshot(now);
}
