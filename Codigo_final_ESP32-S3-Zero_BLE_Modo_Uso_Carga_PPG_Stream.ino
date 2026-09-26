#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================================================================
#define MODO_PLOTTER_PPG 1

// ================================================================
// OLED
// ================================================================
#define ANCHO 128
#define ALTO   64
Adafruit_SSD1306 display(ANCHO, ALTO, &Wire, -1);

// ================================================================
// Corazón 16x16 para animación en pantalla BPM
// ================================================================
const unsigned char PROGMEM corazon_16_chico[] = {
  0b00000000, 0b00000000,
  0b00000000, 0b00000000,
  0b00011000, 0b00110000,
  0b00111100, 0b01111000,
  0b01111110, 0b11111100,
  0b01111111, 0b11111100,
  0b00111111, 0b11111000,
  0b00011111, 0b11110000,
  0b00001111, 0b11100000,
  0b00000111, 0b11000000,
  0b00000011, 0b10000000,
  0b00000001, 0b00000000,
  0b00000000, 0b00000000,
  0b00000000, 0b00000000,
  0b00000000, 0b00000000,
  0b00000000, 0b00000000
};

const unsigned char PROGMEM corazon_16_grande[] = {
  0b00000000, 0b00000000,
  0b00011000, 0b00110000,
  0b00111100, 0b01111000,
  0b01111110, 0b11111100,
  0b11111111, 0b11111110,
  0b11111111, 0b11111110,
  0b01111111, 0b11111100,
  0b00111111, 0b11111000,
  0b00011111, 0b11110000,
  0b00001111, 0b11100000,
  0b00000111, 0b11000000,
  0b00000011, 0b10000000,
  0b00000001, 0b00000000,
  0b00000000, 0b00000000,
  0b00000000, 0b00000000,
  0b00000000, 0b00000000
};

// El corazón se activa con el latido real; si el algoritmo pierde
// un latido usa el BPM como reloj predictivo de respaldo.
unsigned long ultimoLatidoAnim          = 0;
const unsigned long DURACION_CORAZON_GRANDE = 220; // ms

// ================================================================
// Botón GPIO4 — navegación temporal entre pantallas
// ================================================================
const int  BOTON_PIN       = 4;
const byte TOTAL_PANTALLAS = 3;
byte pantallaActual        = 0;

// El encendido/apagado real se realiza con el switch físico de batería.
// GPIO4 se conserva únicamente para cambiar de pantalla mientras se
// desarrolla la navegación futura por BLE/APK.
const unsigned long DEBOUNCE_BOTON_MS = 40;


bool botonEstabaPresionado = false;
unsigned long ultimoEventoBoton = 0;

// ================================================================
// BLE — PROTOCOLO APP + STREAM PPG
// ================================================================
// Servicio tipo Nordic UART (NUS):
// RX = teléfono -> ESP32 (WRITE)
// TX = ESP32 -> teléfono (NOTIFY)
//
// TX comparte tres tipos de mensajes compactos:
//   1) Estado a 2 Hz: B:078 A:0 C:4 P:084
//   2) Onda PPG a 20 Hz: S:-123.4
//   3) ACK/ERR de comandos: ACK:MODE:USO, ACK:MODE:CARGA, etc.
//
// El stream S: utiliza acFiltrado, salida Butterworth 0,5–3,5 Hz.
// El MAX30102 y todo el procesamiento PPG/BPM continúan a 100 Hz.
// Solo la transmisión BLE de la onda se desacopla a 20 Hz, suficiente
// para visualizar una señal cuyo contenido útil está limitado a 3,5 Hz.
// En MODO_CARGA no se transmiten muestras S:.
const char* BLE_NOMBRE = "PPG-Monitor-S3";
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* servidorBLE = nullptr;
BLECharacteristic* caracteristicaBLE_TX = nullptr;
BLECharacteristic* caracteristicaBLE_RX = nullptr;

volatile bool bleConectado = false;

// Estado general BLE: 2 Hz.
unsigned long ultimoBLENotify = 0;
const unsigned long INTERVALO_BLE_NOTIFY = 500;

// Stream de onda PPG para la APK: 20 Hz (una muestra cada 50 ms).
// El procesamiento interno del MAX30102 sigue intacto a 100 Hz.
unsigned long ultimoBLEPPGNotify = 0;
const unsigned long INTERVALO_BLE_PPG_STREAM = 50;

// El callback BLE corre en otra tarea del sistema.
// No cambiamos pantallaActual directamente desde el callback:
// dejamos una solicitud pendiente y el loop principal la aplica.
volatile int8_t pantallaSolicitadaBLE = -1;
volatile bool ackComandoBLEPendiente = false;

// ================================================================
// MODO DEL DISPOSITIVO
// ================================================================
// El ESP32 arranca siempre en MODO_USO.
// La APK será responsable de recordar el modo deseado y reenviarlo
// después de una reconexión/reinicio del sistema.
enum ModoDispositivo : uint8_t {
  MODO_USO = 0,
  MODO_CARGA = 1
};

ModoDispositivo modoActual = MODO_USO;

// Solicitud escrita por el callback BLE y aplicada en el loop principal.
// -1 = sin solicitud, 0 = USO, 1 = CARGA.
volatile int8_t modoSolicitadoBLE = -1;

class CallbacksServidorBLE : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleConectado = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    bleConectado = false;
    // Volver a anunciar el dispositivo para permitir reconexión.
    pServer->startAdvertising();
  }
};

class CallbacksComandosBLE : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String comando = pCharacteristic->getValue();

    // Limpiar posibles terminadores enviados por distintas apps.
    comando.replace("\r", "");
    comando.replace("\n", "");
    comando.trim();
    comando.toUpperCase();

    if (comando == "SCREEN:0") {
      pantallaSolicitadaBLE = 0;
      ackComandoBLEPendiente = true;
    }
    else if (comando == "SCREEN:1") {
      pantallaSolicitadaBLE = 1;
      ackComandoBLEPendiente = true;
    }
    else if (comando == "SCREEN:2") {
      pantallaSolicitadaBLE = 2;
      ackComandoBLEPendiente = true;
    }
    else if (comando == "MODE:USO" || comando == "MODE:NORMAL") {
      modoSolicitadoBLE = MODO_USO;
    }
    else if (comando == "MODE:CARGA" || comando == "MODE:CHARGE") {
      modoSolicitadoBLE = MODO_CARGA;
    }
  }
};

void iniciarBLE() {
  BLEDevice::init(BLE_NOMBRE);

  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new CallbacksServidorBLE());

  BLEService* servicio = servidorBLE->createService(BLE_SERVICE_UUID);

  caracteristicaBLE_TX = servicio->createCharacteristic(
    BLE_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  caracteristicaBLE_TX->addDescriptor(new BLE2902());

  caracteristicaBLE_RX = servicio->createCharacteristic(
    BLE_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  caracteristicaBLE_RX->setCallbacks(new CallbacksComandosBLE());

  servicio->start();

  BLEAdvertising* advertising = servidorBLE->getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
}



// ================================================================
// BATERÍA — divisor resistivo en GPIO2
// ================================================================
// Conexión de PRUEBA actual (alimentación por USB-C):
//   3V3 ESP32 -> R1 95 kOhm -> nodo -> R2 95 kOhm -> GND
//                                      |
//                                    GPIO2
//
// Conexión FINAL con batería:
//   B+ batería -> R1 95 kOhm -> nodo -> R2 95 kOhm -> GND
//                                      |
//                                    GPIO2
//
// Como R1 = R2, el ADC recibe aproximadamente la mitad de la tensión
// aplicada a R1. El firmware es el mismo para la prueba con 3V3 y para B+.
// La calibración 1.0156 surge de la prueba de banco:
//   multímetro = 3.280 V | ESP32 reconstruido promedio = ~3.230 V.
// Puede reajustarse más adelante con la batería real si se desea.
const int   PIN_BATERIA = 2;
const float R1_BATERIA  = 95000.0f;
const float R2_BATERIA  = 95000.0f;
const float FACTOR_DIVISOR_BATERIA = (R1_BATERIA + R2_BATERIA) / R2_BATERIA; // 2.000
const float FACTOR_CALIBRACION_BATERIA = 1.0156f;

// Muestreo no bloqueante: la batería cambia muy lentamente y no debe
// interferir con el PPG a 100 Hz. Se toman 32 muestras espaciadas,
// se descartan 4 mínimas y 4 máximas y se promedia el centro.
const byte BAT_NUM_MUESTRAS   = 32;
const byte BAT_DESCARTE_EXTREMOS = 4;
const unsigned long INTERVALO_MUESTRA_BATERIA = 25; // ms -> un lote cada ~0,8 s
uint16_t muestrasBateria_mV[BAT_NUM_MUESTRAS];
byte indiceMuestraBateria = 0;
unsigned long ultimaMuestraBateria = 0;

float voltajeGPIO2Bateria = 0.0f;
float voltajeBateria      = 0.0f;
int   porcentajeBateria   = 0;
bool  bateriaInicializada = false;
bool  bateriaDetectada    = false;

// Suavizado entre lotes para evitar saltos de algunos milivoltios.
const float ALPHA_BATERIA = 0.25f;

// ================================================================
// MAX30102
// ================================================================
MAX30105 sensor;

long irActual    = 0;
long redActual   = 0;
long varIRActual = 0;

// ================================================================
// Reintentos de arranque I2C
// ================================================================
bool esperarDispositivoI2C(uint8_t direccion, byte intentos,
                           unsigned long esperaMs) {
  for (byte intento = 0; intento < intentos; intento++) {
    Wire.beginTransmission(direccion);
    if (Wire.endTransmission() == 0) return true;
    delay(esperaMs);
  }
  return false;
}

bool iniciarMAX30102ConReintentos(byte intentos,
                                 unsigned long esperaMs) {
  for (byte intento = 0; intento < intentos; intento++) {
    if (sensor.begin(Wire, I2C_SPEED_FAST)) return true;
    delay(esperaMs);
  }
  return false;
}

// ================================================================
// MPU6050
// ================================================================
const int MPU_ADDR = 0x68;

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;
int16_t Tmp;

float ax, ay, az;
float gx, gy, gz;
float acelTotal;

// ----------------------------------------------------------------
// Deteccion de movimiento independiente de la orientacion
// ----------------------------------------------------------------
// El codigo anterior comparaba ax/ay/az contra una orientacion fija
// (AX_BASE, AY_BASE, AZ_BASE). Por eso, al girar el dispositivo y dejarlo
// quieto, la gravedad cambiaba de eje y podia interpretarse como movimiento.
//
// Ahora se estima continuamente el vector de gravedad mediante un filtro
// pasa-bajos y se lo resta de la aceleracion medida. El resultado es la
// componente dinamica de aceleracion, que tiende a cero cuando el dispositivo
// queda quieto, independientemente de su orientacion.
//
// A 20 Hz (INTERVALO_MPU = 50 ms), ALPHA_GRAVEDAD = 0.90 hace que el
// estimador se adapte a una nueva orientacion en aproximadamente 1-2 s.
float gravedadX = 0.0f;
float gravedadY = 0.0f;
float gravedadZ = 0.0f;
bool gravedadInicializada = false;

const float ALPHA_GRAVEDAD = 0.90f;

// cambioEjes conserva su nombre para no alterar el resto del programa,
// pero ahora representa la magnitud de la aceleracion DINAMICA en g.
float cambioEjes = 0.0f;

// giroTotal se conserva solo como dato diagnostico.
// El giroscopio YA NO participa en la clasificacion de actividad.
float giroTotal = 0.0f;

// Umbrales aplicados exclusivamente a la aceleracion dinamica.
const float UMBRAL_REPOSO_EJES   = 0.05f;
const float UMBRAL_MOV_LEVE_EJES = 0.18f;

String estadoActividad = "Reposo";

// ================================================================
// BPM
// RATE_SIZE = 4: ventana corta para obtener respuesta rápida.
// La estabilidad se refuerza con media recortada, rechazo progresivo de outliers
// y validación por etapas temporales.
// BPM_RAPIDO_MIN queda como referencia histórica; la lógica actual usa BPM_ESTABLE_MIN.
// ================================================================
const byte RATE_SIZE      = 4;   // ventana robusta interna corta: aparece rapido y luego estabiliza
const byte BPM_RAPIDO_MIN = 1;   // desde el primer intervalo valido ya se muestra BPM
const byte BPM_ESTABLE_MIN = 3;  // desde aca se endurece progresivamente la validacion

byte  rates[RATE_SIZE];
byte  rateSpot       = 0;
long  lastBeat       = 0;
float beatsPerMinute = 0;
int   beatAvg        = 0;
byte  latidosValidos = 0;

// ================================================================
// Detector rápido de latidos por componente AC
// ================================================================
float acFiltrado = 0;   // FILTRADA = salida Butterworth pasa-banda 0,5–3,5 Hz.
float acAnterior = 0;
float acMax      = 0;
float acMin      = 0;

// ================================================================
// Filtro Butterworth pasa-banda 0,5–3,5 Hz @ 100 Hz
// Base conceptual: síntesis Claude + coeficientes precisos ChatGPT/Gemini.
//
// Diseño:
//   HP Butterworth 2.º orden 0,5 Hz + LP Butterworth 2.º orden 3,5 Hz
//   en cascada = orden 4 efectivo.
//
// Implementación:
//   Direct Form II Transposed. Usa solo 2 estados por sección.
//
// Uso en este código:
//   - IR_AC se conserva SOLO para visualización/comparación en Plotter/Spyder.
//   - FILTRADA/acFiltrado es la salida Butterworth y se usa para detección BPM.
//   - El Butterworth trabaja sobre IR crudo; el HP interno elimina DC/deriva.
//   - Fs asumida = 100 Hz, asociada a INTERVALO_PPG = 10 ms.
//   - Si cambia Fs, hay que recalcular estos coeficientes.
// ================================================================

// Coeficientes HP Butterworth — fc = 0,5 Hz, Fs = 100 Hz
const float HP_B0 =  0.97803048f;
const float HP_B1 = -1.95606096f;
const float HP_B2 =  0.97803048f;
const float HP_A1 = -1.95557824f;
const float HP_A2 =  0.95654368f;

// Coeficientes LP Butterworth — fc = 3,5 Hz, Fs = 100 Hz
const float LP_B0 =  0.01043241f;
const float LP_B1 =  0.02086483f;
const float LP_B2 =  0.01043241f;
const float LP_A1 = -1.69099638f;
const float LP_A2 =  0.73272603f;

// Estados internos — Direct Form II Transposed
float hp_z1 = 0.0f, hp_z2 = 0.0f;
float lp_z1 = 0.0f, lp_z2 = 0.0f;

// Estimador DC lento solo para Plotter/Spyder.
// Permite construir IR_AC = IR - DC para comparación visual.
// NO se usa como filtro principal ni para detectar BPM.
float irDC_plot  = 0.0f;
float irACActual = 0.0f;

// Biquad genérico — Direct Form II Transposed:
//   y  = b0*x + z1
//   z1 = b1*x - a1*y + z2
//   z2 = b2*x - a2*y
float biquadDF2T(float x, float b0, float b1, float b2,
                 float a1, float a2,
                 float &z1, float &z2) {
  float y = b0 * x + z1;
  z1 = b1 * x - a1 * y + z2;
  z2 = b2 * x - a2 * y;
  return y;
}

// Filtro pasa-banda completo: IR crudo -> HP -> LP -> FILTRADA.
float aplicarFiltroBPF(float x) {
  float salidaHP = biquadDF2T(x, HP_B0, HP_B1, HP_B2, HP_A1, HP_A2, hp_z1, hp_z2);
  return biquadDF2T(salidaHP, LP_B0, LP_B1, LP_B2, LP_A1, LP_A2, lp_z1, lp_z2);
}

void resetearButterworthPPG() {
  hp_z1 = 0.0f;
  hp_z2 = 0.0f;
  lp_z1 = 0.0f;
  lp_z2 = 0.0f;
}

bool detectorRapidoInicializado = false;
bool pulsoArmado                = true;
bool enPico                     = false;   // v9: detección de máximo local

unsigned long ultimoLatidoRapido           = 0;
unsigned long ultimoLatidoRegistrado       = 0;
unsigned long ultimaVezDedoDetectado       = 0;
unsigned long ultimoLatidoRapidoRegistrado = 0;  // v9: solo del detector rápido
unsigned long ultimoResetTimeout           = 0;  // v9: evita resets AC repetidos

// Rango fisiológico aceptado para el prototipo: 35–180 lpm.
// Se acota el rango para reducir falsos picos por artefactos, pero sin
// bloquear valores plausibles en reposo bajo o movimiento moderado.
const float BPM_MIN_VALIDO = 35.0f;
const float BPM_MAX_VALIDO = 180.0f;

// Límites equivalentes de IBI según BPM = 60000 / IBI_ms:
// 180 lpm -> 333 ms | 35 lpm -> 1715 ms.
const unsigned long MIN_IBI_MS          = 333;   // 180 lpm maximo permitido
const unsigned long MAX_IBI_MS          = 1715;  // 35 lpm minimo permitido
const unsigned long TIEMPO_PERDIDA_DEDO = 700;
const unsigned long TIMEOUT_RESET_AC    = 6000;  // evita reset prematuro en 40-50 BPM

// ================================================================
// Robustez BPM — v9 PROGRESIVO
// ================================================================

// Outlier: tolerancia variable según cuántos latidos válidos hay.
//   < 3 latidos → ±50 % (muy permisivo, promedio aún inestable)
//   3–5 latidos → ±40 % (transición)
//   ≥ 6 latidos → ±30 % (estricto, promedio ya confiable)
const float OUTLIER_TOLERANCIA_BAJA  = 0.50f;
const float OUTLIER_TOLERANCIA_MEDIA = 0.40f;
const float OUTLIER_TOLERANCIA_ALTA  = 0.30f;
const float OUTLIER_TOLERANCIA_ARRANQUE = 0.70f; // primer tramo: no encerrar el promedio temprano

// Recuperacion de promedio falso alto:
// Si el algoritmo arranca mal cerca de 100 lpm, puede rechazar como outliers
// los valores reales bajos de una persona con FC de 40-50 lpm.
// Esta logica permite salir de esa trampa si aparecen valores bajos coherentes.
const int BPM_PROMEDIO_ALTO_SOSPECHOSO = 80;
const int BPM_BAJO_REAL_MIN            = 35;
const int BPM_BAJO_REAL_MAX            = 65;
const byte BAJOS_CONSECUTIVOS_RESET    = 2;
byte candidatosBajosConsecutivos       = 0;

// MIN_IBI dinámico: 60 % del intervalo promedio como período refractario.
// Solo se activa cuando el promedio ya tiene RATE_SIZE latidos válidos.
const float FACTOR_REFRACTARIO_DINAMICO = 0.60f;

// Ventana progresiva en tres etapas temporales.
const unsigned long ETAPA0_MS = 4000;  // 0–4 s: muy permisivo
const unsigned long ETAPA1_MS = 8000;  // 4–8 s: transición
                                       // >8 s:  modo normal estricto

float amplitudACActual   = 0;
float umbralRapidoActual = 0;

// ================================================================
// Calidad de señal PPG para interfaz
// ================================================================
// Antes la calidad dependía casi solo de amplitudACActual. Eso hacía
// que el indicador quedara siempre al máximo si la envolvente AC era alta,
// aunque el dedo estuviera torcido, moviéndose o sin latidos recientes.
// Ahora se combina: contacto óptico, movimiento, amplitud útil,
// latido reciente y estabilidad del promedio.
int calidadSenalActual = 0;

// ================================================================
// Fallback BPM por cruce de cero
// ================================================================
unsigned long tiempoInicioLectura = 0;
unsigned long ultimoCruceAC       = 0;
int           bpmFallback         = 0;

// ================================================================
// SpO2 — flujo específico a 25 Hz para el algoritmo Maxim
// ================================================================
// El MAX30102 y el camino PPG/BPM siguen trabajando a 100 Hz.
// Para SpO2 se toma 1 de cada 4 muestras, obteniendo 25 Hz.
// La ventana del algoritmo mantiene 100 muestras cronológicas (~4 s).
// Después de la primera ventana, se reemplazan 25 muestras por segundo,
// siguiendo la estructura temporal esperada por spo2_algorithm.
const int  TAM_BUFFER = 100;
const byte SPO2_DECIMACION = 4;          // 100 Hz / 4 = 25 Hz
const byte SPO2_BLOQUE_NUEVO = 25;       // 1 s de datos a 25 Hz

uint32_t irBuffer[TAM_BUFFER];
uint32_t redBuffer[TAM_BUFFER];
uint32_t irNuevasSpO2[SPO2_BLOQUE_NUEVO];
uint32_t redNuevasSpO2[SPO2_BLOQUE_NUEVO];

int  bufferIndex = 0;                    // progreso inicial 0..100
byte contadorDecimacionSpO2 = 0;
byte indiceNuevasSpO2 = 0;
bool hayMuestrasSpO2 = false;

int32_t spo2               = 0;
int8_t  validSPO2          = 0;
int32_t heartRateSPO2      = 0;
int8_t  validHeartRateSPO2 = 0;


// ================================================================
// BLE — notificación de estado (ubicada después de las variables globales)
// ================================================================
void notificarEstadoBLE(unsigned long ahora) {
  if (!bleConectado || caracteristicaBLE_TX == nullptr) return;
  if (ahora - ultimoBLENotify < INTERVALO_BLE_NOTIFY) return;

  ultimoBLENotify = ahora;

  int actividadCodigo = 3;
  if (estadoActividad == "Reposo") actividadCodigo = 0;
  else if (estadoActividad == "Mov. leve") actividadCodigo = 1;
  else if (estadoActividad == "Mov. alto") actividadCodigo = 2;

  int bpmBLE = (beatAvg > 0) ? beatAvg : bpmFallback;
  int calidadBLE = constrain(calidadSenalActual, 0, 4);
  int bateriaBLE = (bateriaDetectada) ? constrain(porcentajeBateria, 0, 100) : 0;

  // 19 bytes como máximo:
  // B:078 A:0 C:4 P:084
  // Cabe dentro del payload ATT clásico de 20 bytes.
  char paquete[24];
  snprintf(
    paquete, sizeof(paquete),
    "B:%03d A:%d C:%d P:%03d",
    constrain(bpmBLE, 0, 999),
    actividadCodigo,
    calidadBLE,
    bateriaBLE
  );

  caracteristicaBLE_TX->setValue((uint8_t*)paquete, strlen(paquete));
  caracteristicaBLE_TX->notify();
}


// ================================================================
// BLE — stream de onda PPG filtrada para la APK
// ================================================================
// Formato compacto:
//   S:-123.4
//
// S = muestra instantánea de acFiltrado, es decir, la salida del
// Butterworth 0,5–3,5 Hz que ya usa el firmware para detectar BPM.
//
// Se envía a ~20 Hz. El procesamiento PPG interno permanece a 100 Hz.
// Esto reduce carga BLE y sigue siendo suficiente para representar
// visualmente una señal limitada a 3,5 Hz.
//
// En MODO_CARGA el stream se detiene por completo.
// ================================================================
void notificarPPGBLE(unsigned long ahora) {
  if (!bleConectado || caracteristicaBLE_TX == nullptr) return;
  if (modoActual != MODO_USO) return;
  if (ahora - ultimoBLEPPGNotify < INTERVALO_BLE_PPG_STREAM) return;

  ultimoBLEPPGNotify = ahora;

  float muestra = acFiltrado;

  // Protección defensiva ante un valor no finito durante un transitorio.
  if (!isfinite(muestra)) muestra = 0.0f;

  // Limitar únicamente el valor transmitido para asegurar un paquete
  // corto y evitar cadenas anormalmente largas por un transitorio extremo.
  if (muestra >  999999.0f) muestra =  999999.0f;
  if (muestra < -999999.0f) muestra = -999999.0f;

  // Máximo esperado: "S:-999999.0" -> ampliamente menor a 20 bytes.
  char paquete[20];
  snprintf(paquete, sizeof(paquete), "S:%.1f", muestra);

  caracteristicaBLE_TX->setValue((uint8_t*)paquete, strlen(paquete));
  caracteristicaBLE_TX->notify();
}


// ================================================================
// actualizarSpO2_25Hz
// ================================================================
// Mantiene el algoritmo de SpO2 separado del camino PPG/BPM:
// - PPG/BPM: 100 Hz sin cambios.
// - SpO2: una muestra de cada cuatro -> 25 Hz.
// - Primera estimación: 100 muestras (~4 s).
// - Luego: conserva 75 muestras y agrega 25 nuevas (~1 s).
// Si la ventana deja de ser válida por movimiento, se reinicia para no
// mezclar datos separados temporalmente.
void actualizarSpO2_25Hz(uint32_t ir, uint32_t red, bool permitirSpO2) {
  if (!permitirSpO2) {
    if (bufferIndex > 0 || hayMuestrasSpO2 || indiceNuevasSpO2 > 0) {
      resetearSpO2();
    } else {
      contadorDecimacionSpO2 = 0;
    }
    return;
  }

  contadorDecimacionSpO2++;
  if (contadorDecimacionSpO2 < SPO2_DECIMACION) return;
  contadorDecimacionSpO2 = 0;

  // Primera ventana cronológica de 100 muestras.
  if (!hayMuestrasSpO2) {
    irBuffer[bufferIndex]  = ir;
    redBuffer[bufferIndex] = red;
    bufferIndex++;

    if (bufferIndex >= TAM_BUFFER) {
      bufferIndex = TAM_BUFFER;
      hayMuestrasSpO2 = true;

      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, TAM_BUFFER, redBuffer,
        &spo2, &validSPO2, &heartRateSPO2, &validHeartRateSPO2
      );

#if !MODO_PLOTTER_PPG
      Serial.print("[SpO2] spo2="); Serial.print(spo2);
      Serial.print(" valid=");      Serial.println(validSPO2);
#endif
    }
    return;
  }

  // Acumular un segundo nuevo: 25 muestras a 25 Hz.
  irNuevasSpO2[indiceNuevasSpO2]  = ir;
  redNuevasSpO2[indiceNuevasSpO2] = red;
  indiceNuevasSpO2++;

  if (indiceNuevasSpO2 < SPO2_BLOQUE_NUEVO) return;

  // Mantener una ventana cronológica de 4 s:
  // se conservan los últimos 75 valores y se anexan los 25 nuevos.
  for (int i = 0; i < TAM_BUFFER - SPO2_BLOQUE_NUEVO; i++) {
    irBuffer[i]  = irBuffer[i + SPO2_BLOQUE_NUEVO];
    redBuffer[i] = redBuffer[i + SPO2_BLOQUE_NUEVO];
  }

  for (byte i = 0; i < SPO2_BLOQUE_NUEVO; i++) {
    int destino = (TAM_BUFFER - SPO2_BLOQUE_NUEVO) + i;
    irBuffer[destino]  = irNuevasSpO2[i];
    redBuffer[destino] = redNuevasSpO2[i];
  }

  indiceNuevasSpO2 = 0;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, TAM_BUFFER, redBuffer,
    &spo2, &validSPO2, &heartRateSPO2, &validHeartRateSPO2
  );

#if !MODO_PLOTTER_PPG
  Serial.print("[SpO2] spo2="); Serial.print(spo2);
  Serial.print(" valid=");      Serial.println(validSPO2);
#endif
}


// ================================================================
// Umbrales MAX30102
// ================================================================
const long UMBRAL_DEDO                 = 8000;
const long UMBRAL_MOVIMIENTO_IR        = 7000;  // Sensible a micro-movimientos/artefactos IR
const byte LIMITE_MOVIMIENTO_SOSTENIDO = 8;

long irAnterior             = 0;
bool movimientoPPG          = false;
bool movimientoPPGSostenido = false;
byte contadorMovimientoPPG  = 0;

// ================================================================
// Timing
// ================================================================
unsigned long ultimaMuestraPPG      = 0;
unsigned long ultimaMuestraMPU      = 0;
unsigned long ultimoRefreshOLED     = 0;
unsigned long ultimoSerialPrint     = 0;   // Debug textual a 2 Hz
unsigned long ultimoSerialPlotter   = 0;   // Plotter/Spyder a 20 Hz
uint32_t secuenciaTelemetria      = 0;   // Secuencia para detectar paquetes perdidos en Spyder/BLE
unsigned long ultimoReporteFsReal   = 0;   // Medición interna de Fs real

const unsigned long INTERVALO_PPG        = 10;   // 100 Hz nominales
const unsigned long INTERVALO_MPU        = 50;   // 20 Hz
const unsigned long INTERVALO_OLED       = 250;  // 4 Hz
const unsigned long INTERVALO_SERIAL     = 500;  // Debug textual a 2 Hz
const unsigned long INTERVALO_PLOTTER    = 50;   // Plotter/Spyder a 20 Hz
const unsigned long INTERVALO_FS_REAL    = 1000; // Reporte interno de Fs cada 1 s

unsigned int muestrasPPGEnUltimoSegundo  = 0;
float fsRealPPG                          = 0.0f;


// ================================================================
// BLE — aplicación segura de comandos recibidos
// ================================================================
void procesarComandosBLE() {
  int8_t solicitada = pantallaSolicitadaBLE;

  if (solicitada >= 0 && solicitada < TOTAL_PANTALLAS) {
    pantallaActual = (byte)solicitada;
    pantallaSolicitadaBLE = -1;

    // Forzar refresco OLED inmediato en el próximo paso del loop.
    ultimoRefreshOLED = 0;
  }

  // ACK corto por TX para confirmar que el WRITE de pantalla llegó.
  if (ackComandoBLEPendiente && bleConectado && caracteristicaBLE_TX != nullptr) {
    ackComandoBLEPendiente = false;

    char ack[10];
    snprintf(ack, sizeof(ack), "ACK:S%d", pantallaActual);

    caracteristicaBLE_TX->setValue((uint8_t*)ack, strlen(ack));
    caracteristicaBLE_TX->notify();
  }

  // --------------------------------------------------------------
  // Aplicar solicitud de modo en el loop principal, nunca dentro
  // del callback BLE.
  // --------------------------------------------------------------
  int8_t modoSolicitado = modoSolicitadoBLE;

  if (modoSolicitado == MODO_CARGA) {
    modoSolicitadoBLE = -1;

    entrarModoCarga();

    if (bleConectado && caracteristicaBLE_TX != nullptr) {
      const char* ack = "ACK:MODE:CARGA";
      caracteristicaBLE_TX->setValue((uint8_t*)ack, strlen(ack));
      caracteristicaBLE_TX->notify();
    }
  }
  else if (modoSolicitado == MODO_USO) {
    modoSolicitadoBLE = -1;

    bool ok = entrarModoUso();

    if (bleConectado && caracteristicaBLE_TX != nullptr) {
      const char* respuesta = ok ? "ACK:MODE:USO" : "ERR:MODE:USO";
      caracteristicaBLE_TX->setValue((uint8_t*)respuesta, strlen(respuesta));
      caracteristicaBLE_TX->notify();
    }
  }
}

// Warm-up inicial: durante este tiempo se filtra y se actualiza la envolvente,
// pero NO se registran nuevos latidos. Reduce falsos picos por transitorio.
const unsigned long WARMUP_BUTTER_MS = 1500;

// Gating suave por movimiento: no borra el BPM anterior, pero impide que
// movimiento alto alimente nuevos latidos espurios.
bool bpmBloqueadoPorMovimiento = false;

// Control post-warm-up:
// durante el warm-up se filtra, pero al salir se reinicia la envolvente
// para que un artefacto inicial no deje umbrales mal calibrados.
bool warmupButterActivo = false;

// ================================================================
// Animación de puntos suspensivos
// ================================================================
String puntosAnimados() {
  int fase = (millis() / 500) % 3;
  switch (fase) {
    case 0: return ".";
    case 1: return "..";
    default: return "...";
  }
}


// ================================================================
// Spinner circular de carga para OLED
// ================================================================
void dibujarSpinner(int cx, int cy, int r) {
  // Círculo base + punto que gira alrededor.
  // No usa delay; se anima con millis().
  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  int fase = (millis() / 120) % 12;
  float angulo = fase * (2.0f * PI / 12.0f);

  int px = cx + (int)(cos(angulo) * r);
  int py = cy + (int)(sin(angulo) * r);

  display.fillCircle(px, py, 2, SSD1306_WHITE);
}


// ================================================================
// Helpers UI v18: estética tipo wearable
// ================================================================
void dibujarIconoBateriaCabecera(int x, int y) {
  // Ícono compacto: cuerpo 11x7 + terminal de 2x3.
  display.drawRect(x, y, 11, 7, SSD1306_WHITE);
  display.fillRect(x + 11, y + 2, 2, 3, SSD1306_WHITE);

  // El relleno interno representa aproximadamente el porcentaje.
  if (bateriaInicializada && bateriaDetectada) {
    int relleno = map(constrain(porcentajeBateria, 0, 100), 0, 100, 0, 9);
    if (relleno > 0) {
      display.fillRect(x + 1, y + 1, relleno, 5, SSD1306_WHITE);
    }
  }
}

void dibujarCabecera(const char* titulo, bool mostrarBateria) {
  display.setTextSize(1);
  display.setFont(NULL);
  display.setCursor(0, 0);
  display.print(titulo);

  if (mostrarBateria) {
    // Reservar la esquina superior derecha para batería.
    // x=88 deja espacio suficiente incluso para "100%".
    dibujarIconoBateriaCabecera(88, 2);
    display.setCursor(104, 0);
    if (bateriaInicializada && bateriaDetectada) {
      display.print(porcentajeBateria);
      display.print("%");
    } else {
      display.print("--%");
    }
  }

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

int calcularCalidadSenalInstantanea(bool dedoDetectado) {
  if (!dedoDetectado) return 0;

  // Penalización fuerte por movimiento. Si la mano se mueve o el PPG cambia
  // bruscamente, puede haber mucho IR y mucha amplitud, pero la señal no es confiable.
  if (movimientoPPGSostenido || estadoActividad == "Mov. alto") return 1;

  int nivel = 0;

  // 1) Contacto óptico.
  // IR muy cerca del umbral = dedo detectado, pero contacto todavía pobre.
  if (irActual > UMBRAL_DEDO + 2000) nivel++;
  if (irActual > UMBRAL_DEDO + 12000) nivel++;

  // 2) Amplitud útil de la componente AC.
  // Rangos más exigentes que antes: evita que cualquier amplitud alta
  // deje la señal clavada en 4 barras.
  if (amplitudACActual > 20) nivel++;
  if (amplitudACActual > 70) nivel++;

  // 3) Latido reciente. Este es el criterio más importante:
  // si no se registran latidos hace varios segundos, la calidad no puede ser máxima.
  bool latidoReciente = false;
  if (ultimoLatidoRegistrado > 0) {
    unsigned long limiteReciente = 3500;
    if (beatAvg > 0) {
      unsigned long ibi = 60000UL / (unsigned long)beatAvg;
      limiteReciente = 2UL * ibi;
      if (limiteReciente < 2500) limiteReciente = 2500;
      if (limiteReciente > 4500) limiteReciente = 4500;
    }
    latidoReciente = ((millis() - ultimoLatidoRegistrado) <= limiteReciente);
  }

  if (!latidoReciente) {
    // Puede haber dedo y AC, pero si no hay latidos registrados, no mostramos calidad máxima.
    if (nivel > 2) nivel = 2;
  } else {
    nivel++;
  }

  // 4) Estabilidad: si ya hay varios latidos válidos, se permite llegar a 4 barras.
  // Si no, se limita para que durante el arranque no parezca "señal perfecta".
  if (latidosValidos >= RATE_SIZE && beatAvg > 0) {
    nivel++;
  } else if (nivel > 3) {
    nivel = 3;
  }

  // 5) Movimiento puntual por IR. No siempre llega a sostenido, pero ensucia la lectura.
  if (movimientoPPG && nivel > 2) nivel = 2;

  if (nivel < 0) nivel = 0;
  if (nivel > 4) nivel = 4;
  return nivel;
}

int suavizarCalidadSenal(int nuevaCalidad) {
  // Sube de a una barra para que no "salte" a máximo de golpe.
  // Baja más rápido para reflejar enseguida dedo torcido/movimiento/pérdida de contacto.
  if (nuevaCalidad < calidadSenalActual) {
    calidadSenalActual = nuevaCalidad;
  } else if (nuevaCalidad > calidadSenalActual) {
    calidadSenalActual++;
  }
  return calidadSenalActual;
}

int calcularCalidadSenal(bool dedoDetectado) {
  return suavizarCalidadSenal(calcularCalidadSenalInstantanea(dedoDetectado));
}

void dibujarBarrasSenal(int x, int y, int nivel) {
  // 4 barras ascendentes, estilo indicador de señal.
  for (byte i = 0; i < 4; i++) {
    int h = 3 + i * 2;
    int bx = x + i * 5;
    int by = y + (9 - h);
    display.drawRoundRect(bx, by, 4, h, 1, SSD1306_WHITE);
    if (nivel > i) {
      display.fillRoundRect(bx + 1, by + 1, 2, h - 2, 1, SSD1306_WHITE);
    }
  }
}



void dibujarPuntoActividad(int x, int y, bool lleno) {
  const int r = 7;
  if (lleno) {
    display.fillCircle(x, y, r, SSD1306_WHITE);
  } else {
    display.drawCircle(x, y, r, SSD1306_WHITE);
  }
}

void dibujarIconoActividad(int cx, int cy, String estado) {
  // Indicador abstracto de intensidad:
  // Reposo:    ● ○ ○
  // Mov. leve: ● ● ○
  // Mov. alto: ● ● ●
  byte nivel = 1;
  if (estado == "Mov. leve") nivel = 2;
  else if (estado == "Mov. alto") nivel = 3;

  const int separacion = 24;
  int x1 = cx - separacion;
  int x2 = cx;
  int x3 = cx + separacion;

  dibujarPuntoActividad(x1, cy, nivel >= 1);
  dibujarPuntoActividad(x2, cy, nivel >= 2);
  dibujarPuntoActividad(x3, cy, nivel >= 3);
}

String etiquetaActividadUsuario() {
  // Cambio exclusivamente textual para la interfaz.
  // La clasificación interna sigue usando "Reposo", "Mov. leve" y "Mov. alto".
  if (estadoActividad == "Reposo") return "Reposo";
  if (estadoActividad == "Mov. leve") return "Leve";
  if (estadoActividad == "Mov. alto") return "Moderado";
  return estadoActividad;
}

byte nivelConfianzaBPM(bool dedoDetectado) {
  // 0 = sin señal/buscando, 1 = baja/bloqueada, 2 = media, 3 = alta.
  if (!dedoDetectado) return 0;
  if (bpmBloqueadoPorMovimiento) return 1;
  if (movimientoPPGSostenido || movimientoPPG) return 1;
  if (estadoActividad == "Mov. leve") return 2;
  if (latidosValidos >= RATE_SIZE && beatAvg > 0) return 3;
  if (beatAvg > 0) return 2;
  return 0;
}

String etiquetaConfianzaBPM(bool dedoDetectado) {
  // Confianza cualitativa del BPM: no modifica el cálculo,
  // solo ayuda a interpretar si el valor debe considerarse estable,
  // cauteloso o bloqueado por movimiento.
  if (!dedoDetectado) return "Sin senal";
  if (bpmBloqueadoPorMovimiento) return "Bloq";

  byte nivel = nivelConfianzaBPM(dedoDetectado);
  if (nivel >= 3) return "Alta";
  if (nivel == 2) return "Media";
  if (nivel == 1) return "Baja";
  return "Buscando";
}

void imprimirTextoCentrado(String texto, int y) {
  display.setTextSize(1);
  display.setFont(NULL);
  int ancho = texto.length() * 6;
  int x = (128 - ancho) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(texto);
}

// ================================================================
// Estado "Leyendo/Reconocido" por pantalla
// ================================================================
String estadoLecturaBPM() {
  // v15: no mostrar nunca el mensaje de ajuste manual del dedo.
  // Mientras no haya BPM, mantiene el estado de lectura normal.
  // Cuando hay BPM, el texto principal de P1 decide si está nivelando o estable.
  if (beatAvg > 0) return "Reconocido.";
  return "Leyendo" + puntosAnimados();
}





// ================================================================
// mostrarOLED — solo para boot y errores
// ================================================================
void mostrarOLED(String l1, String l2, String l3, String l4) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,  0); display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 32); display.println(l3);
  display.setCursor(0, 48); display.println(l4);
  display.display();
}

// ================================================================
// leerBoton — GPIO4 solo cambia de pantalla
// ================================================================
void leerBoton() {
  bool presionado = (digitalRead(BOTON_PIN) == LOW);
  unsigned long ahora = millis();

  // Registrar pulsación con debounce.
  if (presionado && !botonEstabaPresionado) {
    if (ahora - ultimoEventoBoton >= DEBOUNCE_BOTON_MS) {
      botonEstabaPresionado = true;
      ultimoEventoBoton = ahora;
    }
    return;
  }

  // Cambiar una sola pantalla al liberar el botón.
  if (!presionado && botonEstabaPresionado) {
    if (ahora - ultimoEventoBoton >= DEBOUNCE_BOTON_MS) {
      botonEstabaPresionado = false;
      ultimoEventoBoton = ahora;

      pantallaActual++;
      if (pantallaActual >= TOTAL_PANTALLAS) pantallaActual = 0;

#if !MODO_PLOTTER_PPG
      Serial.print("Pantalla -> ");
      Serial.println(pantallaActual);
#endif
    }
  }
}

// ================================================================
// Reset detector rápido
// ================================================================
void resetearDetectorRapido() {
  irDC_plot         = 0;
  irACActual        = 0;
  acFiltrado        = 0;
  acAnterior        = 0;
  acMax             = 0;
  acMin             = 0;
  amplitudACActual  = 0;
  umbralRapidoActual = 0;

  detectorRapidoInicializado     = false;
  pulsoArmado                    = true;
  enPico                         = false;

  ultimoLatidoRapido             = 0;
  ultimoLatidoRegistrado         = 0;
  ultimoLatidoRapidoRegistrado   = 0;
  ultimoResetTimeout             = 0;

  tiempoInicioLectura = 0;
  ultimoCruceAC       = 0;
  bpmFallback         = 0;
  bpmBloqueadoPorMovimiento = false;
  calidadSenalActual = 0;

  resetearButterworthPPG();
}

// ================================================================
// registrarLatido — v9 PROGRESIVO
// ================================================================
bool registrarLatido(unsigned long ahora, float bpmNuevo) {

  // --- Rango fisiológico absoluto ---
  if (bpmNuevo < BPM_MIN_VALIDO || bpmNuevo > BPM_MAX_VALIDO) return false;

  // ------------------------------------------------------------------
  // 1) Período refractario
  //    Dinámico solo cuando el promedio ya es confiable (≥ RATE_SIZE).
  //    Antes de eso usa el mínimo fijo para no bloquear el arranque.
  // ------------------------------------------------------------------
  unsigned long intervaloMinimo = MIN_IBI_MS;

  if (beatAvg > 0 && latidosValidos >= RATE_SIZE) {
    unsigned long intervaloPromedio = 60000UL / (unsigned long)beatAvg;
    intervaloMinimo = (unsigned long)(intervaloPromedio * FACTOR_REFRACTARIO_DINAMICO);
    if (intervaloMinimo < MIN_IBI_MS) intervaloMinimo = MIN_IBI_MS;
    if (intervaloMinimo > 1200)       intervaloMinimo = 1200;
  }

  if (ultimoLatidoRegistrado > 0 &&
      (ahora - ultimoLatidoRegistrado) < intervaloMinimo)
    return false;

  // ------------------------------------------------------------------
  // 2) Recuperacion de promedio falso alto
  // ------------------------------------------------------------------
  // Caso observado en pruebas: el sistema acepta al inicio valores ~100 lpm
  // por doble deteccion, y luego rechaza valores reales de 35-60 lpm como
  // outliers. Si el promedio actual esta alto pero empiezan a aparecer
  // BPM bajos coherentes, reiniciamos SOLO la ventana de promedio y dejamos
  // que el valor bajo actual entre como nuevo arranque.
  bool promedioAltoSospechoso = (beatAvg >= BPM_PROMEDIO_ALTO_SOSPECHOSO && latidosValidos >= 2);
  bool bpmBajoCoherente = (bpmNuevo >= BPM_BAJO_REAL_MIN && bpmNuevo <= BPM_BAJO_REAL_MAX);

  if (promedioAltoSospechoso && bpmBajoCoherente) {
    candidatosBajosConsecutivos++;

#if !MODO_PLOTTER_PPG
    Serial.print("[Candidato bajo] ");
    Serial.print((int)bpmNuevo);
    Serial.print(" lpm | prom alto=");
    Serial.print(beatAvg);
    Serial.print(" | count=");
    Serial.println(candidatosBajosConsecutivos);
#endif

    if (candidatosBajosConsecutivos < BAJOS_CONSECUTIVOS_RESET) {
      // Primer valor bajo: lo observamos, pero no reseteamos todavia.
      // Esto evita que un unico pico perdido destruya un promedio correcto.
      return false;
    }

#if !MODO_PLOTTER_PPG
    Serial.println("[RECUPERACION] Promedio alto falso. Reiniciando ventana BPM.");
#endif

    beatAvg = 0;
    beatsPerMinute = 0;
    rateSpot = 0;
    latidosValidos = 0;
    for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;

    candidatosBajosConsecutivos = 0;
    // No hacemos return: este bpmNuevo se registra abajo como nuevo inicio.
  } else {
    // Si aparece un valor que no confirma el patron bajo, se cancela la recuperacion.
    candidatosBajosConsecutivos = 0;
  }

  // ------------------------------------------------------------------
  // 3) Rechazo de outliers progresivo
  //    La tolerancia se endurece a medida que el promedio es más confiable.
  // ------------------------------------------------------------------
  if (latidosValidos >= 3 && beatAvg > 0) {

    float tolerancia;
    if      (latidosValidos < BPM_ESTABLE_MIN) tolerancia = OUTLIER_TOLERANCIA_ARRANQUE; // ±70 %
    else if (latidosValidos < RATE_SIZE)       tolerancia = OUTLIER_TOLERANCIA_MEDIA;    // ±40 %
    else                                       tolerancia = OUTLIER_TOLERANCIA_ALTA;     // ±30 %

    float variacion = fabs(bpmNuevo - (float)beatAvg) / (float)beatAvg;

    if (variacion > tolerancia) {
#if !MODO_PLOTTER_PPG
      Serial.print("[Outlier] rechazado: ");
      Serial.print((int)bpmNuevo);
      Serial.print(" lpm | prom=");
      Serial.print(beatAvg);
      Serial.print(" | tol=+/-");
      Serial.print((int)(tolerancia * 100));
      Serial.println("%");
#endif
      return false;
    }
  }

  // ------------------------------------------------------------------
  // 4) Registro del latido valido
  // ------------------------------------------------------------------
  ultimoLatidoRegistrado = ahora;
  ultimoLatidoAnim       = ahora;
  beatsPerMinute         = bpmNuevo;

  rates[rateSpot++] = (byte)bpmNuevo;
  rateSpot %= RATE_SIZE;

  if (latidosValidos < RATE_SIZE) latidosValidos++;

  // ------------------------------------------------------------------
  // 5) Media recortada para beatAvg
  //    Con ≥4 valores válidos: descartar mayor y menor, promediar el resto.
  //    Con <4 valores: promedio simple (no hay suficiente para recortar).
  //    Más robusto que la media simple ante un outlier que haya escapado.
  // ------------------------------------------------------------------
  byte sorted[RATE_SIZE];
  byte n = 0;
  for (byte x = 0; x < RATE_SIZE; x++) {
    if (rates[x] > 0) sorted[n++] = rates[x];
  }

  // Ordenar por burbuja (array pequeño, costo negligible)
  for (byte i = 0; i < n - 1; i++)
    for (byte j = i + 1; j < n; j++)
      if (sorted[i] > sorted[j]) {
        byte t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
      }

  int suma   = 0;
  byte cuenta = 0;

  if (n >= 4) {
    // Recortar el mínimo (índice 0) y el máximo (índice n-1)
    for (byte i = 1; i < n - 1; i++) { suma += sorted[i]; cuenta++; }
  } else {
    for (byte i = 0; i < n; i++)     { suma += sorted[i]; cuenta++; }
  }

  if (cuenta > 0) beatAvg = suma / cuenta;

  return true;
}

// ================================================================
// detectarLatidoRapido — v9 PROGRESIVO + máximo local
// ================================================================
bool detectarLatidoRapido(long irValue, unsigned long ahora) {

  // ---- Inicialización al primer contacto ----
  if (!detectorRapidoInicializado) {
    // Inicialización en estado estacionario para el nivel DC actual.
    // Reduce el spike inicial del pasa-altos cuando se apoya el dedo.
    // Derivado de la condición de estado estacionario del HP con entrada DC:
    //   y_ss = 0, z2_ss = HP_B2 * x, z1_ss = HP_B1 * x + z2_ss.
    float fIR = (float)irValue;
    hp_z2 = HP_B2 * fIR;
    hp_z1 = HP_B1 * fIR + hp_z2;
    lp_z1 = 0.0f;
    lp_z2 = 0.0f;

    irDC_plot         = fIR;
    irACActual        = 0;
    acFiltrado        = 0;
    acAnterior        = 0;
    acMax             = 0;
    acMin             = 0;
    amplitudACActual  = 0;
    umbralRapidoActual = 0;

    detectorRapidoInicializado = true;
    pulsoArmado                = true;
    enPico                     = false;
    ultimoLatidoRapido         = 0;
    ultimoCruceAC              = 0;
    bpmBloqueadoPorMovimiento = false;
    warmupButterActivo = true;
    return false;
  }

  // ---- Determinar etapa según tiempo de contacto y latidos válidos ----
  // Una vez que hay RATE_SIZE latidos, se pasa a modo normal directamente,
  // independientemente del tiempo transcurrido.
  unsigned long tiempoContacto = (tiempoInicioLectura > 0)
    ? (ahora - tiempoInicioLectura) : 0;

  int etapa;
  if      (latidosValidos >= RATE_SIZE)    etapa = 2;  // promedio estable
  else if (tiempoContacto < ETAPA0_MS)     etapa = 0;  // 0–4 s: muy permisivo
  else if (tiempoContacto < ETAPA1_MS)     etapa = 1;  // 4–8 s: transición
  else                                     etapa = 2;  // >8 s: normal

  // ---- Filtros ----
  // IR_AC: solo para visualización/comparación en Plotter/Spyder.
  // Esta estimación DC lenta no participa en la detección de BPM.
  irDC_plot  = 0.99f * irDC_plot + 0.01f * (float)irValue;
  irACActual = (float)irValue - irDC_plot;

  // FILTRADA: salida Butterworth pasa-banda 0,5–3,5 Hz.
  // El filtro trabaja sobre IR crudo; el pasa-altos interno elimina DC/deriva.
  // Esta es la señal usada por el detector de picos/BPM.
  acFiltrado = aplicarFiltroBPF((float)irValue);

  // Decaimiento de la envolvente: más rápido al inicio para converger antes
  float factorDecaimiento;
  switch (etapa) {
    case 0:  factorDecaimiento = 0.985f; break;
    case 1:  factorDecaimiento = 0.990f; break;
    default: factorDecaimiento = 0.995f; break;
  }

  if (acFiltrado > acMax) acMax = acFiltrado;
  else                    acMax *= factorDecaimiento;

  if (acFiltrado < acMin) acMin = acFiltrado;
  else                    acMin *= factorDecaimiento;

  amplitudACActual = acMax - acMin;

  // ---- Umbrales adaptativos progresivos por etapa ----
  float minUmbralAlto, minUmbralBajo, minAmplitudLatido;
  switch (etapa) {
    case 0:                            // 0–4 s: arranque rapido, pero controlado
      minUmbralAlto     = 28.0f;
      minUmbralBajo     = 10.0f;
      minAmplitudLatido = 12.0f;
      break;
    case 1:                            // 4–8 s: transición
      minUmbralAlto     = 40.0f;
      minUmbralBajo     = 15.0f;
      minAmplitudLatido = 18.0f;
      break;
    default:                           // >8 s o promedio estable: normal robusto
      minUmbralAlto     = 55.0f;
      minUmbralBajo     = 22.0f;
      minAmplitudLatido = 28.0f;
      break;
  }

  float umbralAlto = max(minUmbralAlto, amplitudACActual * 0.35f);
  float umbralBajo = max(minUmbralBajo, amplitudACActual * 0.12f);
  umbralRapidoActual = umbralAlto;

  // ---- Warm-up inicial del Butterworth ----
  // Se procesa la señal y se actualiza la envolvente, pero se bloquea el registro
  // de picos durante los primeros WARMUP_BUTTER_MS desde que se detecta contacto.
  // Esto evita que pequeños transitorios del filtro o cambios de presión iniciales
  // generen BPM falsos.
  if (tiempoContacto < WARMUP_BUTTER_MS) {
    pulsoArmado = true;
    enPico = false;
    ultimoCruceAC = 0;
    acAnterior = acFiltrado;
    return false;
  }

  // Limpieza post-warm-up:
  // se reinicia la envolvente justo al terminar el asentamiento del filtro.
  // Esto evita que una presión inicial o un golpe al apoyar el dedo deje
  // acMax/acMin contaminados y genere umbrales incorrectos.
  if (warmupButterActivo) {
    acMax = 0;
    acMin = 0;
    amplitudACActual = 0;
    umbralRapidoActual = 0;
    pulsoArmado = true;
    enPico = false;
    ultimoCruceAC = 0;
    acAnterior = acFiltrado;
    warmupButterActivo = false;
    return false;
  }

  // ---- Gating suave por movimiento alto ----
  // El Butterworth no puede distinguir un pulso real de un artefacto rítmico
  // de muñeca si ambos caen en 0,5–3,5 Hz. Por eso, con movimiento alto
  // se congela el registro de nuevos latidos sin borrar el BPM previo.
  bpmBloqueadoPorMovimiento = (estadoActividad == "Mov. alto" || movimientoPPGSostenido);
  if (bpmBloqueadoPorMovimiento) {
    pulsoArmado = true;
    enPico = false;
    ultimoCruceAC = 0;
    acAnterior = acFiltrado;
    return false;
  }

  // ---- Reset por silencio prolongado (>3 s sin pico) ----
  // Si la señal dejó de pulsar (dedo presionado, movimiento brusco, etc.),
  // reiniciamos la envolvente para evitar que umbralAlto quede obsoleto
  // y produzca falsos positivos o bloqueos cuando la señal vuelve.
  if (ultimoLatidoRapido > 0 &&
      (ahora - ultimoLatidoRapido) > TIMEOUT_RESET_AC &&
      (ahora - ultimoResetTimeout) > TIMEOUT_RESET_AC) {
    acMax            = 0;
    acMin            = 0;
    pulsoArmado      = true;
    enPico           = false;
    ultimoResetTimeout = ahora;
#if !MODO_PLOTTER_PPG
    Serial.println("[Reset AC] Envolvente reiniciada por silencio prolongado");
#endif
  }

  bool latidoRegistrado = false;

  // ---- Fallback por cruce de cero del AC ----
  // Estima BPM a partir de la periodicidad de la señal, sin necesidad
  // de detectar picos individuales. Solo se usa para mostrar en pantalla
  // mientras el detector principal no ha enganchado aún.
  if (acAnterior < 0.0f && acFiltrado >= 0.0f && amplitudACActual > 12.0f) {
    unsigned long delta = ahora - ultimoCruceAC;
    if (ultimoCruceAC > 0 && delta >= MIN_IBI_MS && delta <= MAX_IBI_MS) {
      bpmFallback = (int)(60000.0f / delta);

      // En el arranque, el cruce de cero tambien puede alimentar el promedio.
      // Esto evita esperar 20-25 s cuando el maximo local tarda en enganchar.
      // registrarLatido() mantiene el refractario y la validacion progresiva.
      if (latidosValidos < BPM_ESTABLE_MIN) {
        registrarLatido(ahora, (float)bpmFallback);
      }
    }
    ultimoCruceAC = ahora;
  }

  // ---- Detección de máximo local (v9) ----
  //
  // Reemplaza el cruce de umbral ascendente del v8.
  // El IBI ahora se mide siempre desde el mismo punto fisiológico
  // de la onda (el pico sistólico), independientemente de la amplitud
  // o la forma de la onda en ese ciclo.
  //
  // Lógica en 4 pasos:
  //   Paso 1 — Entrar en zona de pico cuando acFiltrado supera umbralAlto
  //            y la amplitud mínima garantiza que no es ruido.
  //   Paso 2 — Detectar el máximo: cuando estamos en pico y la señal
  //            empieza a descender (acFiltrado < acAnterior).
  //   Paso 3 — Registrar el latido en ese punto y resetear estado.
  //   Paso 4 — Re-armar el detector cuando la señal cae bajo umbralBajo.

  // Paso 1: entrar en zona de pico
  if (!enPico && pulsoArmado &&
      acFiltrado > umbralAlto &&
      amplitudACActual > minAmplitudLatido) {
    enPico = true;
  }

  // Paso 2+3: máximo local → registrar latido
  if (enPico && acFiltrado < acAnterior) {
    enPico      = false;
    pulsoArmado = false;

    unsigned long delta = ahora - ultimoLatidoRapido;

    if (ultimoLatidoRapido == 0) {
      // Primer pico visto: solo fijar la referencia temporal, no calcular BPM
      ultimoLatidoRapido = ahora;

    } else if (delta >= MIN_IBI_MS && delta <= MAX_IBI_MS) {
      // IBI dentro de rango fisiológico → calcular BPM e intentar registrar
      float bpmRapido = 60000.0f / delta;
      latidoRegistrado = registrarLatido(ahora, bpmRapido);
      if (latidoRegistrado) ultimoLatidoRapidoRegistrado = ahora;
      ultimoLatidoRapido = ahora;

    } else if (delta > MAX_IBI_MS) {
      // Demasiado tiempo desde el último pico: resincronizar sin registrar
      ultimoLatidoRapido = ahora;
    }
    // Si delta < MIN_IBI_MS: pico duplicado o rebote → ignorar silenciosamente
  }

  // Paso 4: re-armar cuando la señal baja bajo el umbral bajo
  if (!pulsoArmado && acFiltrado < umbralBajo) {
    pulsoArmado = true;
  }

  acAnterior = acFiltrado;
  return latidoRegistrado;
}

// ================================================================
// Reset BPM
// ================================================================
void resetearBPM() {
  beatsPerMinute   = 0;
  beatAvg          = 0;
  rateSpot         = 0;
  lastBeat         = 0;
  ultimoLatidoAnim = 0;
  latidosValidos   = 0;
  candidatosBajosConsecutivos = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
  resetearDetectorRapido();
}

// ================================================================
// Reset SpO2
// ================================================================
void resetearSpO2() {
  bufferIndex              = 0;
  contadorDecimacionSpO2   = 0;
  indiceNuevasSpO2         = 0;
  hayMuestrasSpO2          = false;
  spo2                     = 0;
  validSPO2                = 0;
  heartRateSPO2            = 0;
  validHeartRateSPO2       = 0;

  for (int i = 0; i < TAM_BUFFER; i++) {
    irBuffer[i]  = 0;
    redBuffer[i] = 0;
  }

  for (byte i = 0; i < SPO2_BLOQUE_NUEVO; i++) {
    irNuevasSpO2[i]  = 0;
    redNuevasSpO2[i] = 0;
  }
}

// ================================================================
// MPU6050
// ================================================================
void iniciarMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);
}

bool leerMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) return false;

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  ax = AcX / 16384.0f;
  ay = AcY / 16384.0f;
  az = AcZ / 16384.0f;
  gx = GyX / 131.0f;
  gy = GyY / 131.0f;
  gz = GyZ / 131.0f;

  acelTotal = sqrt(ax*ax + ay*ay + az*az);
  return true;
}

String clasificarMovimientoMPU(float ax, float ay, float az,
                                float gx, float gy, float gz) {

  // El giroscopio se calcula unicamente como dato diagnostico.
  // NO interviene en la decision Reposo / Mov. leve / Mov. alto.
  giroTotal = sqrt(gx*gx + gy*gy + gz*gz);

  // Primera lectura valida: inicializar el estimador de gravedad con la
  // orientacion real en la que se encuentre el dispositivo al arrancar.
  // Esto evita imponer una posicion "correcta" de la muneca o de la placa.
  if (!gravedadInicializada) {
    gravedadX = ax;
    gravedadY = ay;
    gravedadZ = az;
    gravedadInicializada = true;
    cambioEjes = 0.0f;
    return "Reposo";
  }

  // Estimacion lenta del vector de gravedad/orientacion.
  gravedadX = ALPHA_GRAVEDAD * gravedadX + (1.0f - ALPHA_GRAVEDAD) * ax;
  gravedadY = ALPHA_GRAVEDAD * gravedadY + (1.0f - ALPHA_GRAVEDAD) * ay;
  gravedadZ = ALPHA_GRAVEDAD * gravedadZ + (1.0f - ALPHA_GRAVEDAD) * az;

  // Aceleracion dinamica = aceleracion medida - componente cuasi-estatica
  // debida principalmente a la gravedad y a la orientacion.
  float aDynX = ax - gravedadX;
  float aDynY = ay - gravedadY;
  float aDynZ = az - gravedadZ;

  cambioEjes = sqrt(aDynX*aDynX + aDynY*aDynY + aDynZ*aDynZ);

  // Clasificacion basada SOLO en aceleracion dinamica.
  // Si se gira el dispositivo, durante el giro habra movimiento.
  // Cuando queda quieto en la nueva orientacion, el estimador de gravedad
  // converge y cambioEjes vuelve hacia cero -> Reposo.
  if (cambioEjes < UMBRAL_REPOSO_EJES)
    return "Reposo";
  else if (cambioEjes < UMBRAL_MOV_LEVE_EJES)
    return "Mov. leve";
  else
    return "Mov. alto";
}

// ================================================================
// MODO CARGA / MODO USO — control de sensores
// ================================================================

// Poner MPU6050 en sleep.
// PWR_MGMT_1 (0x6B), bit SLEEP = 1.
void dormirMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x40);
  Wire.endTransmission(true);
}

// Limpiar por completo los estados de adquisición.
// Se usa tanto al entrar en carga como al regresar a uso para evitar
// mezclar filtros, latidos, SpO2 o estimaciones de gravedad entre sesiones.
void resetearEstadoAdquisicionModo() {
  resetearBPM();
  resetearSpO2();

  irActual = 0;
  redActual = 0;
  varIRActual = 0;
  irAnterior = 0;

  movimientoPPG = false;
  movimientoPPGSostenido = false;
  contadorMovimientoPPG = 0;

  ultimaVezDedoDetectado = 0;
  tiempoInicioLectura = 0;
  bpmBloqueadoPorMovimiento = false;
  warmupButterActivo = false;
  calidadSenalActual = 0;

  gravedadX = 0.0f;
  gravedadY = 0.0f;
  gravedadZ = 0.0f;
  gravedadInicializada = false;
  cambioEjes = 0.0f;
  giroTotal = 0.0f;

  ultimaMuestraPPG = 0;
  ultimaMuestraMPU = 0;
  ultimoReporteFsReal = 0;
  muestrasPPGEnUltimoSegundo = 0;
  fsRealPPG = 0.0f;
}

// Entrar en modo carga.
// NO duerme al ESP32: BLE, OLED y lectura de batería siguen activos.
void entrarModoCarga() {
  if (modoActual == MODO_CARGA) return;

  // Apagar LEDs antes del shutdown para evitar emisión residual.
  sensor.setPulseAmplitudeRed(0x00);
  sensor.setPulseAmplitudeIR(0x00);
  sensor.shutDown();

  dormirMPU6050();

  resetearEstadoAdquisicionModo();
  estadoActividad = "Pausado";

  modoActual = MODO_CARGA;
  ultimoRefreshOLED = 0;
}

// Volver al funcionamiento normal.
// Devuelve false si alguno de los sensores no responde por I2C.
bool entrarModoUso() {
  if (modoActual == MODO_USO) return true;

  Wire.setClock(400000);

  // En shutdown el MAX30102 sigue respondiendo por I2C.
  if (!esperarDispositivoI2C(0x57, 3, 50)) {
    return false;
  }

  if (!esperarDispositivoI2C(MPU_ADDR, 3, 50)) {
    return false;
  }

  // Reactivar y reconfigurar exactamente con los parámetros del firmware final.
  sensor.wakeUp();
  delay(5);
  sensor.setup(0x24, 1, 2, 100, 411, 4096);
  sensor.setPulseAmplitudeRed(0x24);
  sensor.setPulseAmplitudeIR(0x24);

  // iniciarMPU6050() escribe PWR_MGMT_1=0 y restablece rangos.
  iniciarMPU6050();

  resetearEstadoAdquisicionModo();
  estadoActividad = "Reposo";

  modoActual = MODO_USO;
  ultimoRefreshOLED = 0;

  return true;
}

// ================================================================
// Corazón predictivo 16x16
// ================================================================
void dibujarCorazonPromedio16(int x, int y) {
  // v14: el corazón aparece apenas hay un BPM disponible, aunque todavía
  // el promedio esté en etapa de estabilización. Antes se mostraba recién
  // cuando latidosValidos >= RATE_SIZE, por eso parecía que no latía al inicio.
  if (beatAvg <= 0 && bpmFallback <= 0) return;

  int bpmVisual = (beatAvg > 0) ? beatAvg : bpmFallback;
  if (bpmVisual < (int)BPM_MIN_VALIDO) bpmVisual = (int)BPM_MIN_VALIDO;
  if (bpmVisual > (int)BPM_MAX_VALIDO) bpmVisual = (int)BPM_MAX_VALIDO;

  unsigned long ahora     = millis();
  unsigned long intervalo = 60000UL / (unsigned long)bpmVisual;

  if (intervalo < MIN_IBI_MS) intervalo = MIN_IBI_MS;
  if (intervalo > MAX_IBI_MS) intervalo = MAX_IBI_MS;

  if (ultimoLatidoAnim == 0) {
    ultimoLatidoAnim = ahora;
  } else if ((ahora - ultimoLatidoAnim) > intervalo) {
    ultimoLatidoAnim = ahora;
  }

  bool corazonGrande = ((ahora - ultimoLatidoAnim) < DURACION_CORAZON_GRANDE);

  display.drawBitmap(x, y,
    corazonGrande ? corazon_16_grande : corazon_16_chico,
    16, 16, SSD1306_WHITE);
}


// ================================================================
// Batería — estimación de porcentaje por curva Li-ion 1S
// ================================================================
// Es una estimación orientativa: la tensión de una Li-ion no se relaciona
// linealmente con el porcentaje y además varía con carga, temperatura y uso.
int calcularPorcentajeBateria(float v) {
  struct PuntoBat { float v; int pct; };

  static const PuntoBat curva[] = {
    {4.20f, 100},
    {4.15f,  95},
    {4.10f,  90},
    {4.05f,  85},
    {4.00f,  80},
    {3.95f,  75},
    {3.90f,  70},
    {3.85f,  60},
    {3.80f,  50},
    {3.75f,  40},
    {3.70f,  30},
    {3.65f,  20},
    {3.60f,  12},
    {3.50f,   6},
    {3.40f,   3},
    {3.30f,   1},
    {3.20f,   0}
  };

  const int n = sizeof(curva) / sizeof(curva[0]);

  if (v >= curva[0].v) return 100;
  if (v <= curva[n - 1].v) return 0;

  for (int i = 0; i < n - 1; i++) {
    if (v <= curva[i].v && v >= curva[i + 1].v) {
      float fraccion = (v - curva[i + 1].v) / (curva[i].v - curva[i + 1].v);
      float pct = curva[i + 1].pct + fraccion * (curva[i].pct - curva[i + 1].pct);
      return constrain((int)roundf(pct), 0, 100);
    }
  }

  return 0;
}

// ================================================================
// actualizarBateriaNoBloqueante
// ================================================================
// Usa analogReadMilliVolts() para aprovechar la calibración disponible del ADC.
// El promedio recortado elimina valores extremos y luego se aplica una EMA
// suave entre lotes. No se usa delay(), para no alterar el muestreo PPG.
void actualizarBateriaNoBloqueante(unsigned long ahora) {
  if (ahora - ultimaMuestraBateria < INTERVALO_MUESTRA_BATERIA) return;
  ultimaMuestraBateria = ahora;

  uint32_t mV = analogReadMilliVolts(PIN_BATERIA);
  if (mV > 5000) mV = 5000; // protección lógica ante una lectura anómala

  muestrasBateria_mV[indiceMuestraBateria++] = (uint16_t)mV;

  if (indiceMuestraBateria < BAT_NUM_MUESTRAS) return;

  // Ordenamiento simple: son solo 32 muestras y se ejecuta ~1 vez por segundo.
  for (byte i = 0; i < BAT_NUM_MUESTRAS - 1; i++) {
    for (byte j = i + 1; j < BAT_NUM_MUESTRAS; j++) {
      if (muestrasBateria_mV[i] > muestrasBateria_mV[j]) {
        uint16_t tmp = muestrasBateria_mV[i];
        muestrasBateria_mV[i] = muestrasBateria_mV[j];
        muestrasBateria_mV[j] = tmp;
      }
    }
  }

  uint32_t suma = 0;
  byte cantidad = 0;
  for (byte i = BAT_DESCARTE_EXTREMOS; i < BAT_NUM_MUESTRAS - BAT_DESCARTE_EXTREMOS; i++) {
    suma += muestrasBateria_mV[i];
    cantidad++;
  }

  float promedio_mV = (cantidad > 0) ? ((float)suma / cantidad) : 0.0f;
  float nuevoVoltajeGPIO2 = promedio_mV / 1000.0f;
  float nuevoVoltajeBateria = nuevoVoltajeGPIO2 * FACTOR_DIVISOR_BATERIA * FACTOR_CALIBRACION_BATERIA;

  if (!bateriaInicializada) {
    voltajeGPIO2Bateria = nuevoVoltajeGPIO2;
    voltajeBateria = nuevoVoltajeBateria;
    bateriaInicializada = true;
  } else {
    voltajeGPIO2Bateria = (1.0f - ALPHA_BATERIA) * voltajeGPIO2Bateria + ALPHA_BATERIA * nuevoVoltajeGPIO2;
    voltajeBateria      = (1.0f - ALPHA_BATERIA) * voltajeBateria      + ALPHA_BATERIA * nuevoVoltajeBateria;
  }

  // Una Li-ion 1S útil debería quedar dentro de este rango amplio.
  // Si GPIO2 queda a 0 V por falta de batería, no se muestra un porcentaje falso.
  bateriaDetectada = (voltajeBateria >= 2.50f && voltajeBateria <= 4.35f);
  porcentajeBateria = bateriaDetectada ? calcularPorcentajeBateria(voltajeBateria) : 0;

  indiceMuestraBateria = 0;
}

// ================================================================
// renderizarPantallaModoCarga
// ================================================================
void renderizarPantallaModoCarga() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setFont(NULL);

  imprimirTextoCentrado("MODO CARGA", 4);

  display.drawRoundRect(4, 18, 120, 28, 5, SSD1306_WHITE);

  if (bateriaInicializada && bateriaDetectada) {
    display.setTextSize(2);

    String bateriaTexto = String(porcentajeBateria) + "%";
    int ancho = bateriaTexto.length() * 12;
    int x = (128 - ancho) / 2;
    if (x < 0) x = 0;

    display.setCursor(x, 25);
    display.print(bateriaTexto);
    display.setTextSize(1);
  } else {
    imprimirTextoCentrado("Bateria no detectada", 29);
  }

  imprimirTextoCentrado("Medicion pausada", 50);

  display.setCursor(0, 58);
  display.print(bleConectado ? "BLE conectado" : "BLE disponible");

  display.display();
}

// ================================================================
// renderizarPantalla
// ================================================================
void renderizarPantalla(bool dedoDetectado) {

  // En MODO CARGA las tres pantallas normales quedan temporalmente
  // sustituidas por una pantalla única de carga.
  if (modoActual == MODO_CARGA) {
    renderizarPantallaModoCarga();
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setFont(NULL);

  switch (pantallaActual) {

    // ----------------------------------------------------------
    // PANTALLA 0 — BPM / frecuencia cardíaca
    // ----------------------------------------------------------
    case 0: {
      // Cabecera abreviada para separar el título del nombre completo
      // de la variable fisiológica mostrada sobre el valor.
      dibujarCabecera("BPM", true);

      if (!dedoDetectado) {
        imprimirTextoCentrado("Sensor sin contacto", 27);
        break;
      }

      int calidad = calcularCalidadSenal(dedoDetectado);

      bool bpmDisponible = (beatAvg > 0);
      // Si todavía no existe un BPM utilizable, el mensaje depende de la
      // calidad óptica. Esto es SOLO presentación: no modifica el detector.
      if (!bpmDisponible) {
        if (calidad <= 1) {
          imprimirTextoCentrado("Mejorar contacto", 27);
        } else {
          imprimirTextoCentrado("Midiendo...", 20);
          dibujarSpinner(64, 40, 9);
        }

        display.setCursor(0, 55);
        display.print("Senal");
        dibujarBarrasSenal(35, 53, calidad);
        break;
      }

      // Si ya existe un BPM, se muestra aunque la confianza/calidad sea baja.
      // La confianza se conserva en esta pantalla para contextualizar el valor.
      imprimirTextoCentrado("Frecuencia cardiaca", 14);

      // Valor principal grande, estilo smartwatch.
      display.setTextSize(3);

      int digitosBPM = (beatAvg >= 100) ? 3 : 2;
      int anchoNumero = digitosBPM * 18;
      int anchoCorazon = 16;
      int separacion = 5;
      int anchoGrupo = anchoNumero + separacion + anchoCorazon;
      int xNumero = (128 - anchoGrupo) / 2;
      int yNumero = 29;
      int xCorazon = xNumero + anchoNumero + separacion;
      int yCorazon = yNumero + 3;

      display.setCursor(xNumero, yNumero);
      display.print(beatAvg);

      display.setTextSize(1);
      dibujarCorazonPromedio16(xCorazon, yCorazon);

      display.setCursor(0, 55);
      display.print("Senal");
      dibujarBarrasSenal(35, 53, calidad);
      display.setCursor(75, 55);
      display.print("C:");
      display.print(etiquetaConfianzaBPM(dedoDetectado));

      break;
    }

    // ----------------------------------------------------------
    // PANTALLA 1 — Actividad interpretada
    // ----------------------------------------------------------
    case 1: {
      dibujarCabecera("Actividad", true);

      String etiqueta = etiquetaActividadUsuario();

      // Tres puntos grandes de intensidad:
      // Reposo = 1 punto, Leve = 2 puntos, Moderado = 3 puntos.
      dibujarIconoActividad(64, 32, estadoActividad);
      imprimirTextoCentrado(etiqueta, 53);

      break;
    }

    // ----------------------------------------------------------
    // PANTALLA 2 — Resumen
    // ----------------------------------------------------------
    case 2: {
      dibujarCabecera("Resumen", false);

      // La estructura del resumen se conserva siempre visible.
      // Si una variable todavía no es válida se representa con "--".

      // Tarjeta BPM
      display.drawRoundRect(0, 14, 61, 16, 3, SSD1306_WHITE);
      display.setCursor(5, 19);
      display.print("BPM:");
      display.setCursor(34, 19);
      if (dedoDetectado && beatAvg > 0) display.print(beatAvg);
      else                              display.print("--");

      // SpO2 se conserva únicamente como variable exploratoria del resumen.
      display.drawRoundRect(67, 14, 61, 16, 3, SSD1306_WHITE);
      display.setCursor(72, 19);
      display.print("O2:");
      display.setCursor(97, 19);
      if (dedoDetectado &&
          hayMuestrasSpO2 && validSPO2 == 1 && spo2 > 70 && spo2 <= 100) {
        display.print(spo2);
        display.print("%");
      } else {
        display.print("--");
      }

      // Movimiento. La clasificación interna no cambia:
      // "Mov. alto" se presenta al usuario como "Moderado".
      display.drawRoundRect(0, 34, 128, 18, 3, SSD1306_WHITE);
      display.setCursor(4, 40);
      display.print("Movimiento:");
      display.setCursor(76, 40);
      display.print(etiquetaActividadUsuario());

      // Señal PPG + batería. La confianza se elimina del resumen y
      // permanece únicamente en la pantalla 1.
      int calidadResumen = calcularCalidadSenal(dedoDetectado);

      display.setCursor(0, 56);
      display.print("Senal");
      dibujarBarrasSenal(35, 54, calidadResumen);

      // Ícono real de batería en lugar de la abreviatura "B:".
      dibujarIconoBateriaCabecera(80, 55);
      display.setCursor(96, 56);
      if (bateriaInicializada && bateriaDetectada) {
        display.print(porcentajeBateria);
        display.print("%");
      } else {
        display.print("--%");
      }

      break;
    }
  }

  display.display();
}

// ================================================================
// Setup
// ================================================================
void setup() {
#if MODO_PLOTTER_PPG != 2
  Serial.begin(460800);  // Modos 0/1: salida de laboratorio.
#endif

  pinMode(BOTON_PIN, INPUT_PULLUP);
  delay(20);

  // ADC de batería en GPIO2. En este core ESP32 la atenuación de 11 dB
  // se expresa como ADC_11db. La entrada máxima esperada del divisor
  // es ~2,10 V con una Li-ion a 4,20 V.
  pinMode(PIN_BATERIA, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATERIA, ADC_11db);

  // Lecturas de descarte para que el ADC se estabilice al arrancar.
  for (byte i = 0; i < 8; i++) {
    analogReadMilliVolts(PIN_BATERIA);
    delay(2);
  }

  Wire.begin(5, 6);
  Wire.setClock(400000);  // Kimi refinada: I2C rápido para reducir bloqueos

#if !MODO_PLOTTER_PPG
  Serial.println("Monitor PPG final Butterworth iniciando...");
#endif

  // OLED: confirmar ACK I2C con hasta 3 intentos antes de inicializar.
  if (!esperarDispositivoI2C(0x3C, 3, 150) ||
      !display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
#if !MODO_PLOTTER_PPG
    Serial.println("ERROR: OLED");
#endif
    while (1) delay(100);
  }
  mostrarOLED("PPG Butter", "OLED OK", "Filtro BPF", "Iniciando...");
  delay(1000);

  // MAX30102: hasta 3 intentos antes de declarar una falla permanente.
  if (!iniciarMAX30102ConReintentos(3, 150)) {
#if !MODO_PLOTTER_PPG
    Serial.println("ERROR: MAX30102");
#endif
    mostrarOLED("ERROR", "MAX30102 ausente", "Revisar cables", "VIN GND SDA SCL");
    while (1) delay(100);
  }
#if !MODO_PLOTTER_PPG
  Serial.println("MAX30102 OK");
#endif

  Wire.setClock(400000);  // asegurar 400 kHz después de iniciar MAX30102

  // Configuración PPG principal: permanece exactamente a 100 Hz.
  sensor.setup(0x24, 1, 2, 100, 411, 4096);
  sensor.setPulseAmplitudeRed(0x24);
  sensor.setPulseAmplitudeIR(0x24);

  // MPU6050: hasta 3 ACK antes de declarar una falla permanente.
  if (!esperarDispositivoI2C(MPU_ADDR, 3, 150)) {
#if !MODO_PLOTTER_PPG
    Serial.println("ERROR: MPU6050");
#endif
    mostrarOLED("ERROR", "MPU6050 ausente", "Revisar cables", "VCC GND SDA SCL");
    while (1) delay(100);
  }
  iniciarMPU6050();
#if !MODO_PLOTTER_PPG
  Serial.println("MPU6050 OK");
#endif

  resetearBPM();
  resetearSpO2();

  // El sistema siempre arranca en uso normal.
  modoActual = MODO_USO;
  estadoActividad = "Reposo";

  // BLE se inicia después de validar el hardware principal.
  iniciarBLE();

  mostrarOLED("Sistema OK", "BLE: PPG-Monitor-S3", "Modo: USO", "Bateria: GPIO2");
  delay(1200);
}

// ================================================================
// Loop principal
// ================================================================
void loop() {
  unsigned long ahora = millis();

  // GPIO4 se conserva únicamente para navegación local entre pantallas.
  leerBoton();

  // ---- Batería: ADC GPIO2, muestreo no bloqueante ----
  // Permanece activa tanto en MODO USO como en MODO CARGA.
  actualizarBateriaNoBloqueante(ahora);

  // Aplicar de forma segura los comandos recibidos por BLE.
  // Se procesa antes de leer sensores para que MODE:CARGA detenga
  // la adquisición en este mismo ciclo del loop.
  procesarComandosBLE();

  // ==============================================================
  // ADQUISICIÓN NORMAL
  // ==============================================================
  if (modoActual == MODO_USO) {

  // ---- MPU6050 a 20 Hz ----
  if (ahora - ultimaMuestraMPU >= INTERVALO_MPU) {
    ultimaMuestraMPU = ahora;
    if (leerMPU6050())
      estadoActividad = clasificarMovimientoMPU(ax, ay, az, gx, gy, gz);
    else
      estadoActividad = "MPU error";
  }

  // ---- MAX30102 a 100 Hz nominales ----
  // Temporización con acumulación para reducir deriva: si una iteración tarda
  // más de 10 ms, no reiniciamos el reloj desde "ahora" sino desde la próxima
  // muestra planificada.
  if (ultimaMuestraPPG == 0) ultimaMuestraPPG = ahora;
  if ((long)(ahora - ultimaMuestraPPG) >= (long)INTERVALO_PPG) {
    ultimaMuestraPPG += INTERVALO_PPG;

    // Si el sistema se atrasó mucho (OLED/Serial/I2C), resincronizar
    // para no intentar "ponerse al día" con ráfagas de muestras tardías.
    if ((long)(ahora - ultimaMuestraPPG) > (long)(5 * INTERVALO_PPG)) {
      ultimaMuestraPPG = ahora;
    }

    // Lectura directa del MAX30102.
    // Nota para defensa: no se usa sensor.check()/sensor.available() en esta versión
    // para no cambiar la arquitectura antes de presentar. La regularidad se documenta
    // con fsRealPPG. Como mejora futura, puede migrarse a FIFO/check().
    irActual  = sensor.getIR();
    redActual = sensor.getRed();

    muestrasPPGEnUltimoSegundo++;
    if (ultimoReporteFsReal == 0) ultimoReporteFsReal = ahora;
    if (ahora - ultimoReporteFsReal >= INTERVALO_FS_REAL) {
      fsRealPPG = (muestrasPPGEnUltimoSegundo * 1000.0f) / (float)(ahora - ultimoReporteFsReal);
      muestrasPPGEnUltimoSegundo = 0;
      ultimoReporteFsReal = ahora;
    }

    varIRActual = abs(irActual - irAnterior);
    irAnterior  = irActual;

    movimientoPPG          = false;
    movimientoPPGSostenido = false;

    if (irActual > UMBRAL_DEDO && varIRActual > UMBRAL_MOVIMIENTO_IR) {
      movimientoPPG = true;
      if (contadorMovimientoPPG < 255) contadorMovimientoPPG++;
    } else {
      if (contadorMovimientoPPG > 0) contadorMovimientoPPG--;
    }
    if (contadorMovimientoPPG >= LIMITE_MOVIMIENTO_SOSTENIDO)
      movimientoPPGSostenido = true;

    bool dedoCrudo = (irActual >= UMBRAL_DEDO);
    if (dedoCrudo) ultimaVezDedoDetectado = ahora;

    bool dedoDetectado = dedoCrudo ||
                         (ultimaVezDedoDetectado > 0 &&
                          (ahora - ultimaVezDedoDetectado) < TIEMPO_PERDIDA_DEDO);

    if (!dedoDetectado) {
      resetearBPM();
      resetearSpO2();
      lastBeat = 0;  // reset explícito del respaldo SparkFun al perder contacto
      contadorMovimientoPPG = 0;
      tiempoInicioLectura   = 0;
      bpmBloqueadoPorMovimiento = false;

    } else {
      if (tiempoInicioLectura == 0) tiempoInicioLectura = ahora;

      bool actividadAlta = (estadoActividad == "Mov. alto");

      // ---- Detector rápido — siempre activo con dedo ----
      detectarLatidoRapido(irActual, ahora);

      // ---- checkForBeat (SparkFun) — respaldo condicional ----
      //
      // Se llama en CADA muestra para mantener su estado interno actualizado.
      // Su resultado solo se usa si el detector rápido lleva más de
      // 2 IBIs esperados sin registrar nada, lo que indica que la señal
      // es difícil de capturar con el método de máximo local.
      // Esto elimina la competencia entre detectores en condiciones normales.
      //
      bool sparkFunDetecto = checkForBeat(irActual);  // siempre llamar

      bool usarSparkFun;
      if (latidosValidos < BPM_ESTABLE_MIN || ultimoLatidoRapidoRegistrado == 0) {
        // Hasta tener un promedio usable-estable, SparkFun ayuda activamente.
        usarSparkFun = true;
      } else {
        unsigned long ibiEsperado = (beatAvg > 0)
          ? (60000UL / (unsigned long)beatAvg)
          : 1000UL;
        usarSparkFun = ((ahora - ultimoLatidoRapidoRegistrado) > ibiEsperado);
      }

      if (bpmBloqueadoPorMovimiento) {
        usarSparkFun = false;
      }

      if (usarSparkFun && sparkFunDetecto) {
        long delta = ahora - lastBeat;

        if (lastBeat == 0) {
          // Primer disparo de SparkFun: solo referencia temporal
          lastBeat = ahora;

        } else if (delta > 0) {
          float bpmSpark = 60000.0f / delta;
          bool aceptado  = registrarLatido(ahora, bpmSpark);

          // Actualizar lastBeat solo si fue aceptado o si pasó demasiado
          // tiempo (para evitar que SparkFun quede bloqueado en un delta enorme)
          if (aceptado || (bpmSpark >= BPM_MIN_VALIDO && bpmSpark <= BPM_MAX_VALIDO) || delta > (long)MAX_IBI_MS) {
            lastBeat = ahora;
          }
        }
      }

      // ---- SpO2 ----
      // PPG/BPM continúa a 100 Hz. Solo el camino SpO2 se desacopla a 25 Hz.
      bool permitirSpO2 = !movimientoPPGSostenido && !actividadAlta;
      actualizarSpO2_25Hz((uint32_t)irActual, (uint32_t)redActual, permitirSpO2);
    }

#if MODO_PLOTTER_PPG == 1
    // ============================================================
    // TELEMETRIA UNIFICADA PARA SPYDER / FUTURO BLE
    // ============================================================
    // Un único paquete TSV cada 50 ms (~20 Hz).
    // El procesamiento PPG sigue a 100 Hz: solo se desacopla la transmisión.
    //
    // Formato:
    // DATA  T_MS  SEQ  PPG  FILTRADA  BPM  MOV  ACT  CAL
    //       SPO2  SPO2_OK  BATV  BATP  BAT_OK  CONTACTO  FS
    //
    // ACT: 0=Reposo, 1=Leve, 2=Moderado, 3=Error/otro
    // CAL: 0..4 barras de calidad (misma variable usada por OLED)
    //
    // Se transmite también sin contacto para que Spyder no "congele"
    // la interfaz cuando la pulsera no está apoyada.
    if (ahora - ultimoSerialPlotter >= INTERVALO_PLOTTER) {
      ultimoSerialPlotter = ahora;

      int actividadCodigo = 3;
      if (estadoActividad == "Reposo") actividadCodigo = 0;
      else if (estadoActividad == "Mov. leve") actividadCodigo = 1;
      else if (estadoActividad == "Mov. alto") actividadCodigo = 2;

      int bpmTelemetria = (beatAvg > 0) ? beatAvg : bpmFallback;

      Serial.print("DATA");
      Serial.print("\tT_MS:");      Serial.print(ahora);
      Serial.print("\tSEQ:");       Serial.print(secuenciaTelemetria++);
      Serial.print("\tPPG:");       Serial.print(irACActual, 2);
      Serial.print("\tFILTRADA:");  Serial.print(acFiltrado, 2);
      Serial.print("\tBPM:");       Serial.print(bpmTelemetria);
      Serial.print("\tMOV:");       Serial.print(cambioEjes, 5);
      Serial.print("\tACT:");       Serial.print(actividadCodigo);
      Serial.print("\tCAL:");       Serial.print(calidadSenalActual);
      Serial.print("\tSPO2:");      Serial.print(spo2);
      Serial.print("\tSPO2_OK:");   Serial.print(validSPO2 ? 1 : 0);
      Serial.print("\tBATV:");      Serial.print(bateriaDetectada ? voltajeBateria : 0.0f, 3);
      Serial.print("\tBATP:");      Serial.print(bateriaDetectada ? porcentajeBateria : 0);
      Serial.print("\tBAT_OK:");    Serial.print(bateriaDetectada ? 1 : 0);
      Serial.print("\tCONTACTO:");  Serial.print(dedoDetectado ? 1 : 0);
      Serial.print("\tFS:");        Serial.println(fsRealPPG, 1);
    }
#endif

#if !MODO_PLOTTER_PPG
    // ---- Serial a 2 Hz (v9) ----
    // Limitar a 2 Hz evita que el buffer serie frene el loop a 100 Hz.
    if (ahora - ultimoSerialPrint >= INTERVALO_SERIAL) {
      ultimoSerialPrint = ahora;
      Serial.print("IR_Cruda=");     Serial.print(irActual);
      Serial.print(" IR_AC=");        Serial.print(irACActual);
      Serial.print(" Filtrada=");     Serial.print(acFiltrado);
      Serial.print(" BPMi=");         Serial.print((int)beatsPerMinute);
      Serial.print(" BPMp=");         Serial.print(beatAvg);
      Serial.print(" Lat=");          Serial.print(latidosValidos);
      Serial.print(" SpO2=");         Serial.print(spo2);
      Serial.print(" Movimiento=");   Serial.print(cambioEjes);
      Serial.print(" BPM_bloq_mov="); Serial.print(bpmBloqueadoPorMovimiento ? 1 : 0);
      Serial.print(" Conf=");         Serial.print(etiquetaConfianzaBPM(dedoDetectado));
      Serial.print(" Fs_real=");      Serial.print(fsRealPPG, 1);
      Serial.print(" BatV=");         Serial.print(bateriaDetectada ? voltajeBateria : 0.0f, 3);
      Serial.print(" Bat%=");         Serial.print(bateriaDetectada ? porcentajeBateria : 0);
      Serial.print(" Act=");          Serial.print(estadoActividad);
      Serial.print(" Pantalla=");     Serial.println(pantallaActual);
    }
#endif
  }

  } // fin MODO_USO

  // ---- OLED ----
  // Las tres pantallas se refrescan a 4 Hz.
  if (ahora - ultimoRefreshOLED >= INTERVALO_OLED) {
    ultimoRefreshOLED = ahora;

    bool dedoCrudoOLED     = (irActual >= UMBRAL_DEDO);
    bool dedoDetectadoOLED = dedoCrudoOLED ||
                             (ultimaVezDedoDetectado > 0 &&
                              (ahora - ultimaVezDedoDetectado) < TIEMPO_PERDIDA_DEDO);

    renderizarPantalla(dedoDetectadoOLED);
  }

  // BLE: onda PPG filtrada a ~20 Hz.
  // Solo se transmite en MODO USO. El procesamiento interno sigue a 100 Hz.
  notificarPPGBLE(ahora);

  // BLE: estado corto a 2 Hz.
  // En MODO CARGA envía BPM=0, ACT=3 y calidad=0, por lo que la APK
  // no registra muestras fisiológicas mientras la medición está pausada.
  // La batería continúa actualizándose si está físicamente conectada.
  notificarEstadoBLE(ahora);
}
