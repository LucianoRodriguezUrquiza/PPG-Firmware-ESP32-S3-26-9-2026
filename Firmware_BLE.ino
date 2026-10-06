// B19 PA-BLE + validated ColdStartFix; PA binary transport preserved from B19_PA_BLE(3).ino.
// B19 PA-BLE - derivado estrictamente de B18_BLE validado.
// Target: Waveshare ESP32-S3-LCD-1.47B, Arduino-ESP32 3.3.12, PSRAM OPI.
// B18/B17 siguen siendo duenos exclusivos de sensores/filtros/BPM/SpO2/PRV/PA. BLE consume snapshots.
// NUS y contrato HELLO:2/H/B/S/O/R/D permanecen byte-compatible con B18.
// B19 agrega UNICAMENTE transporte binario de la ventana BP15 (700 IR crudas @100 Hz)
// por una caracteristica Notify separada. No calcula PA en la ESP32.
// Los comentarios B17/B18 que siguen documentan la base historica.
// PA binary packet format is documented in B19_PA_BLE_TRANSPORT.md.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <esp_system.h>
#include <esp_arduino_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdarg.h>

// Optional diagnostics must never wait for the USB host to consume a full buffer.
// PA data writes/parsers are NOT routed through this best-effort logger.
class DiagnosticoInicio18 {
 public:
  void printf(const char *fmt, ...) {
    if(!Serial)return;
    char text[512];va_list args;va_start(args,fmt);
    int n=vsnprintf(text,sizeof(text),fmt,args);va_end(args);
    if(n<=0 || n>=(int)sizeof(text) || Serial.availableForWrite()<n)return;
    Serial.write((const uint8_t*)text,(size_t)n);
  }
  void println(const char *text){printf("%s\r\n",text);}
};
DiagnosticoInicio18 diagnosticoInicio18;
#if !defined(CONFIG_NIMBLE_ENABLED)
#error "B18 targets bundled NimBLE in ESP32-S3 Arduino-ESP32 3.3.12 (no external BLE library)."
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(3,3,12)
#warning "B18 was checked against Arduino-ESP32 3.3.12; verify BLE APIs on other versions."
#endif

// B17 - ETAPA PRV EXPLORATORIA + UI v8 ESTABLE.
// Base: V16 / ETAPA 1 aprobada. La interfaz LCD NO se modifica.
// El Butterworth 0.5-3.5 Hz, detector BPM, V9, MAX30102, SpO2 y PA se conservan.
// PRV17 observa de forma PASIVA los pulsos temporales del detector rapido DESPUES del Butterworth.
// No escribe beatAvg, rates, estados HP/LP, umbrales, ultimoLatidoRapido ni decisiones del BPM.
// Metricas exploratorias: PP medio, SDNN, RMSSD, pNN50, CVNN en ventana movil de 60 s.
// Patron ectopico sospechoso: secuencia corto-largo repetida; NO diagnostica extrasistoles/arritmias.
// PRV solo se considera util en reposo, con contacto, IMU valida y señal fresca.
// No se muestra PRV en LCD. Queda disponible en memoria y por log Serie para futura Etapa BLE.
// -----------------------------------------------------------------------------
// ETAPA 1 UI v8 ESTABLE: interfaz portrait 172x320 para wearable.
// Base funcional: Prueba_MAX30102(2).ino. El detector/filtro BPM se conserva byte por byte.
// Cambios v8: mantiene BPM/filtro original, corrige resincronizacion visual de contacto,
// anima de forma determinista los puntos de INICIALIZANDO y mueve buffers diagnosticos grandes a PSRAM.
// Bateria real por BAT_ADC cada 10 s; porcentaje continuo estimado 0..100%; Bluetooth centrado.
// BLE NO integrado en esta etapa: el indicador Bluetooth permanece OFF hasta Etapa 2.
// PA por PC sigue siendo una rama experimental OPCIONAL y no bloquea BPM/SpO2/IMU/LCD.
// V15: PA con modelo externo en PC. NO AUTONOMO PARA PA. BPM y SpO2 SI son autonomos en ESP32.
// V14: uso continuo BPM/SpO2. PA SOLO REFERENCIA HISTORICA, NO ESTIMACION.
// V13: historial temporal de ventanas PA; BPM/SpO2 y secuencia K M T C conservados.
// V13 corrige retrocesos de UI. Referencias sin onda util no entrenan el modelo.
// V11: calibracion guiada, ventana fijada con T, monitor silencioso durante ingreso.
// PPG Waveshare V10 EXPLORATORIA (basada en Prueba_MAX30102(1).ino, V9).
// 10 s contacto estable + 120 s medicion + 10 s retirada. NO es firmware clinico.
// Filtro, detector, confirmacion BPM y arranque MAX/IMU proceden de la V9.
// SpO2 usa la biblioteca SparkFun existente; PA es regresion propia calibrable.
// Sin coeficientes entrenados NO se muestra una PA numerica; no hay valores ficticios.
// Lee las instrucciones C/F/X incluidas mas abajo. Fuente original V9 conservada.
// PPG Waveshare V9 - ultimo BPM validado independiente del detector.
// Prueba: 10 s contacto estable, 120 s medicion, 10 s sin dedo. G inicia, D exporta traza.
// Sin BLE. V10 agrega SpO2 independiente; no modifica Butterworth ni umbrales de BPM.
// La retencion no es una medicion nueva. Contador VALID9 avanza solo al promover.
// Calidad ALTA/MEDIA/BAJA: heuristica de coherencia/movimiento, no exactitud clinica.
// Verificacion pendiente en placa. VALID9=presentacion; GATE9=motivos acumulados.
// V9 no cambia filtro ni detector: confirma 3 aceptaciones al inicio/rearme,
// tolera separaciones de hasta 6 s y actualiza cada aceptacion coherente posterior.
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <math.h>
#include <esp_heap_caps.h>
#include "MAX30105.h"
#include "heartRate.h"
struct Registro {
 uint32_t n=0,contacto=0,conBPM=0,caidas=0,cortesBPM=0,aceptados=0,ibiN=0;
 uint32_t ibiMin=0xFFFFFFFF,ibiMax=0; uint64_t ibiSuma=0; float bpmMin=1000,bpmMax=0,bpmSuma=0; };
void registrarDiagnostico(Registro &r,bool hayBPM,bool caida,bool corte,bool aceptado,uint32_t ibi);
void imprimirRegistro(const Registro &r);
// Captura a 25 Hz; filtro y detector siguen a 100 Hz.
// Flags acumulados en cada grupo de 4 muestras: contacto=1, maximo local=2,
// entrada en pico=4, aceptacion=8, rearme=16, movimiento=32, sin BPM vigente=64.
struct Traza { uint32_t ms,ir; float filtrada,umbral; uint8_t flags; };
const uint16_t CAP_TRAZA=3100;
Traza *trazas=nullptr;uint16_t nTraza=0,indiceEnvio=0;
uint8_t flagsTraza=0,divisionTraza=0;bool enviandoTraza=false;
uint32_t trazaOmitida=0,contactoDesde=0; bool trazaSubiendo=false;
void enviarTraza();
MAX30105 sensor; bool maxOK=false,imuOK=false;
uint32_t ir=0,rojo=0,nPPG=0,conContacto=0,ceros=0,minIR=0xFFFFFFFF,maxIR=0; uint64_t sumaIR=0;
const unsigned long duraciones[]={0,10000,120000,10000}; TwoWire WireIMU(1); uint8_t QMI_ADDR=0x6B;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ; float ax=0,ay=0,az=0,gx=0,gy=0,gz=0,acelTotal=0;
float gravedadX=0,gravedadY=0,gravedadZ=0,dinamica=0,giro=0;
bool gravedadLista=false,listo=false,lecturaOK=false; uint8_t fase=0,nivel=0;
unsigned long inicio=0,ultimoIMU=0,ultimoInforme=0,ultimoLCD=0;
uint32_t validas=0,errores=0,conteos[3]={0,0,0},saturadas=0; float sumaD=0,maxD=0,sumaG=0,maxG=0;
const char *niveles[]={"Reposo","Leve","Moderado"};
const char *fases[]={"LISTO","APOYA Y ESTABILIZA","BPM + SpO2 + PA","RETIRA EL DEDO"};
const int LCD_MOSI=45, LCD_SCLK=40, LCD_CS=42; const int LCD_DC=41, LCD_RST=39, LCD_BL=46;

// Screen-only timeout: keep LCD updates, sensors, processing and BLE running.
// BOOT is active LOW. Each debounced press restarts the 45-second interval.
const int SCREEN_BOOT_PIN=0;
const uint32_t SCREEN_ON_MS=45000, SCREEN_DEBOUNCE_MS=30;
uint32_t screenLastWake=0, screenButtonChanged=0;
bool screenBacklightOn=true;
int screenButtonRaw=HIGH, screenButtonStable=HIGH;

void iniciarAhorroPantalla(){
 pinMode(SCREEN_BOOT_PIN,INPUT_PULLUP);
 screenButtonRaw=screenButtonStable=digitalRead(SCREEN_BOOT_PIN);
 screenLastWake=screenButtonChanged=millis();
 screenBacklightOn=true;digitalWrite(LCD_BL,HIGH);
}

void actualizarAhorroPantalla(){
 const uint32_t now=millis();
 const int raw=digitalRead(SCREEN_BOOT_PIN);
 if(raw!=screenButtonRaw){screenButtonRaw=raw;screenButtonChanged=now;}
 if(raw!=screenButtonStable && (uint32_t)(now-screenButtonChanged)>=SCREEN_DEBOUNCE_MS){
  screenButtonStable=raw;
  if(raw==LOW){
   screenLastWake=now;
   if(!screenBacklightOn){digitalWrite(LCD_BL,HIGH);screenBacklightOn=true;}
  }
 }
 if(screenBacklightOn && (uint32_t)(now-screenLastWake)>=SCREEN_ON_MS){
  digitalWrite(LCD_BL,LOW);screenBacklightOn=false;
 }
}
const uint32_t LCD_SPI_HZ=40000000; const uint8_t LCD_ROTACION=0; SPIClass spiLCD(FSPI);
Adafruit_ST7789 lcd(&spiLCD, LCD_CS, LCD_DC, LCD_RST); bool pruebaTerminada=false,pausaEstadisticas=false;
void adquirir();
uint32_t recuperacionesBus=0,rearmes=0,rechazoRango=0,rechazoCercano=0,rechazoCambio=0;
uint32_t ultimoRearme=0,ultimoIntentoBus=0;
float ultimoBPMVisible=0; unsigned long instanteBPMVisible=0;
char cacheLCD[6][27]={{0}}; uint16_t colorCache[6]={0};
// Fuente fija opaca: sobrescribe solo caracteres distintos, sin borrar la banda.
void campoLCD(uint8_t campo,int x,int y,uint8_t tam,uint8_t ancho,const char *txt,uint16_t color) {
 size_t n=strlen(txt);lcd.setTextSize(tam);lcd.setTextColor(color,ST77XX_BLACK);
 for(uint8_t i=0;i<ancho;i++) {
  char c=i<n ? txt[i] : ' ';
  if(cacheLCD[campo][i]!=c || colorCache[campo]!=color) {
   lcd.setCursor(x+i*6*tam,y);lcd.write((uint8_t)c);adquirir();cacheLCD[campo][i]=c;
  }
 } colorCache[campo]=color;
}
void rectLCD(int x,int y,int w,int h,uint16_t color) {
  for(int fila=0;fila<h;fila+=4) {
    lcd.fillRect(x,y+fila,w,min(4,h-fila),color); adquirir();
  }
}
void iniciarLCD() { pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, LOW);
  spiLCD.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS); lcd.init(172, 320, SPI_MODE0); lcd.setSPISpeed(LCD_SPI_HZ);
    const uint8_t b0[] = {0x00, 0xF0}; // Adafruit_GFX envia RGB565 MSB primero: RAMCTRL en big-endian.
    const uint8_t b2[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    const uint8_t b7[] = {0x35}, bb[] = {0x35}, c0[] = {0x2C};
    const uint8_t c2[] = {0x01}, c3[] = {0x13}, c4[] = {0x20}, c6[] = {0x0F};
    const uint8_t d0[] = {0xA4, 0xA1}, d6[] = {0xA1};
    const uint8_t e0[] = {0xF0,0x00,0x04,0x04,0x04,0x05,0x29,0x33,0x3E,0x38,0x12,0x12,0x28,0x30};
    const uint8_t e1[] = {0xF0,0x07,0x0A,0x0D,0x0B,0x07,0x28,0x33,0x3E,0x36,0x14,0x14,0x29,0x32};
    lcd.sendCommand(0xB0,b0,sizeof(b0)); lcd.sendCommand(0xB2,b2,sizeof(b2));
    lcd.sendCommand(0xB7,b7,sizeof(b7)); lcd.sendCommand(0xBB,bb,sizeof(bb));
    lcd.sendCommand(0xC0,c0,sizeof(c0)); lcd.sendCommand(0xC2,c2,sizeof(c2));
    lcd.sendCommand(0xC3,c3,sizeof(c3)); lcd.sendCommand(0xC4,c4,sizeof(c4));
    lcd.sendCommand(0xC6,c6,sizeof(c6)); lcd.sendCommand(0xD0,d0,sizeof(d0));
    lcd.sendCommand(0xD6,d6,sizeof(d6)); lcd.sendCommand(0xE0,e0,sizeof(e0));
    lcd.sendCommand(0xE1,e1,sizeof(e1)); lcd.invertDisplay(true); lcd.setRotation(LCD_ROTACION);
    lcd.fillScreen(ST77XX_BLACK); lcd.setTextWrap(false);
  digitalWrite(LCD_BL, HIGH); }
void textoLCD(int y, const char *texto, uint8_t tam, uint16_t color) {
  lcd.setTextSize(tam); lcd.setTextColor(color, ST77XX_BLACK); int x=(lcd.width()-(int)strlen(texto)*6*tam)/2;
  lcd.setCursor(x<0 ? 0 : x, y);
  while(*texto) {char letra[2]={*texto++,0};lcd.print(letra);adquirir();} }

// Pantalla de arranque compacta. Se usa antes de que MAX/IMU esten listos, por lo que
// no depende de adquirir() ni del monitor serie. Los puntos avanzan 1 -> 2 -> 3.
bool pantallaInicializandoActiva=false;
uint8_t pasoInicializandoUI=0;
uint32_t ultimoPasoInicializandoUI=0;
const uint32_t INIT_PUNTOS_MS=180;

// Animacion determinista 1 -> 2 -> 3 -> 1. No depende de la fase absoluta de millis(),
// por lo que nunca queda visualmente "clavada" en dos o tres puntos durante el arranque.
void tickInicializandoUI(bool forzar=false){
 if(!pantallaInicializandoActiva)return;
 uint32_t ahora=millis();
 if(!forzar && ahora-ultimoPasoInicializandoUI<INIT_PUNTOS_MS)return;
 ultimoPasoInicializandoUI=ahora;
 pasoInicializandoUI=(pasoInicializandoUI%3)+1;
 lcd.setFont(NULL);lcd.setTextWrap(false);lcd.setTextSize(2);
 lcd.fillRect(58,171,56,20,ST77XX_BLACK);
 char p[4]={0};for(uint8_t i=0;i<pasoInicializandoUI;i++)p[i]='.';
 int x=(lcd.width()-(int)strlen(p)*12)/2;
 lcd.setTextColor(ST77XX_WHITE,ST77XX_BLACK);lcd.setCursor(x,173);lcd.print(p);
}

void dibujarInicializandoUI(){
 pantallaInicializandoActiva=true;pasoInicializandoUI=0;ultimoPasoInicializandoUI=0;
 lcd.fillScreen(ST77XX_BLACK);lcd.setFont(NULL);lcd.setTextWrap(false);
 lcd.setTextSize(2);lcd.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
 const char *t="Inicializando";
 int x=(lcd.width()-(int)strlen(t)*12)/2;
 lcd.setCursor(max(0,x),146);lcd.print(t);
 tickInicializandoUI(true);
}

void esperarInicializacionUI(uint32_t ms){
 uint32_t t0=millis();
 while(millis()-t0<ms){tickInicializandoUI();delay(8);}
}

// ===== ETAPA 1: estado de la interfaz principal =====
// No se usa framebuffer ni Canvas: todos los componentes se dibujan directamente
// sobre ST7789 y solo se redibuja la region cuyo estado cambia.
const int BAT_ADC_PIN=1;               // Waveshare BAT_ADC, divisor 200k/100k -> VBAT ~= 3*VADC.
const uint32_t BAT_UPDATE_MS=10000;
bool bluetoothActivoUI=false;          // Etapa 1: BLE no se integra; queda preparado para Etapa 2.
float bateriaVoltiosUI=NAN;
uint8_t bateriaEstadoUI=0;              // Porcentaje estimado 0..100. Color y barras se derivan de este valor.
unsigned long ultimaBateriaUI=0;
bool interfazBaseDibujada=false;

void actualizarFrecuenciaCardiacaUI();
void actualizarContactoUI();
void actualizarMovimientoUI();
void actualizarBluetoothUI(bool activo);
void actualizarBateriaUI();
void animarCorazonUI();
void dibujarBaseUI();
void informarMemoriaUI();
bool escribirRegistroQMI(uint8_t reg, uint8_t valor) {
  WireIMU.beginTransmission(QMI_ADDR); WireIMU.write(reg); WireIMU.write(valor);
  return WireIMU.endTransmission(true) == 0; }
bool leerRegistrosQMI(uint8_t reg, uint8_t* datos, uint8_t cantidad) {
  WireIMU.beginTransmission(QMI_ADDR); WireIMU.write(reg);
  if (WireIMU.endTransmission(false) != 0) return false;
  if (WireIMU.requestFrom(QMI_ADDR, cantidad, true) != cantidad) {
    while (WireIMU.available()) WireIMU.read();
    return false; }
  for (uint8_t i=0; i<cantidad; ++i) datos[i] = WireIMU.read();
  return true; } bool detectarQMI8658() {
  const uint8_t direcciones[] = {0x6B, 0x6A};
  for (uint8_t intento=0; intento<3; ++intento) {
    for (uint8_t direccion : direcciones) {
      QMI_ADDR = direccion; uint8_t who = 0;
      if (leerRegistrosQMI(0x00, &who, 1) && who == 0x05) return true;
    } delay(50); } return false; }
bool iniciarQMI8658() {
  if (!detectarQMI8658()) return false;
  if (!escribirRegistroQMI(0x02, 0x40) ||
      !escribirRegistroQMI(0x08, 0x00) ||
      !escribirRegistroQMI(0x03, 0x06) ||
      !escribirRegistroQMI(0x04, 0x46) ||
      !escribirRegistroQMI(0x06, 0x00) ||
      !escribirRegistroQMI(0x07, 0x00) ||
      !escribirRegistroQMI(0x08, 0x43)) return false;
  delay(25); uint8_t ctrl2=0, ctrl3=0, ctrl7=0;
  return leerRegistrosQMI(0x03, &ctrl2, 1) && ctrl2 == 0x06 &&
         leerRegistrosQMI(0x04, &ctrl3, 1) && ctrl3 == 0x46 &&
         leerRegistrosQMI(0x08, &ctrl7, 1) && ctrl7 == 0x43; } bool leerQMI8658() { uint8_t datos[12];
  if (!leerRegistrosQMI(0x35, datos, sizeof(datos))) return false;
  AcX = (int16_t)((uint16_t)datos[0] | ((uint16_t)datos[1]<<8));
  AcY = (int16_t)((uint16_t)datos[2] | ((uint16_t)datos[3]<<8));
  AcZ = (int16_t)((uint16_t)datos[4] | ((uint16_t)datos[5]<<8));
  GyX = (int16_t)((uint16_t)datos[6] | ((uint16_t)datos[7]<<8));
  GyY = (int16_t)((uint16_t)datos[8] | ((uint16_t)datos[9]<<8));
  GyZ = (int16_t)((uint16_t)datos[10] | ((uint16_t)datos[11]<<8));
  ax = AcX / 16384.0f; ay = AcY / 16384.0f; az = AcZ / 16384.0f;
  gx = GyX / 128.0f; gy = GyY / 128.0f; gz = GyZ / 128.0f; acelTotal = sqrt(ax*ax + ay*ay + az*az);
  return true; } void limpiar() {
  nPPG=conContacto=ceros=0; sumaIR=0; minIR=0xFFFFFFFF; maxIR=0; validas=errores=saturadas=0;
  sumaD=maxD=sumaG=maxG=0;
  for(int i=0;i<3;i++) conteos[i]=0;
} void procesar() { if(!gravedadLista) {
    gravedadX=ax; gravedadY=ay; gravedadZ=az; gravedadLista=true; } gravedadX=0.9f*gravedadX+0.1f*ax;
  gravedadY=0.9f*gravedadY+0.1f*ay; gravedadZ=0.9f*gravedadZ+0.1f*az;
  float dx=ax-gravedadX,dy=ay-gravedadY,dz=az-gravedadZ; dinamica=sqrtf(dx*dx+dy*dy+dz*dz);
  giro=sqrtf(gx*gx+gy*gy+gz*gz); nivel=dinamica<0.05f ? 0 : (dinamica<0.18f ? 1 : 2);
  if(fase>0 && !pausaEstadisticas) {
    validas++; conteos[nivel]++; sumaD+=dinamica; sumaG+=giro;
    if(dinamica>maxD) maxD=dinamica;
    if(giro>maxG) maxG=giro;
    if(abs((int)AcX)>32000 || abs((int)AcY)>32000 || abs((int)AcZ)>32000 ||
       abs((int)GyX)>32000 || abs((int)GyY)>32000 || abs((int)GyZ)>32000) saturadas++;
  } } const byte RATE_SIZE      = 4; const byte BPM_ESTABLE_MIN = 3;
// V7: conservar decimales de cada estimacion; beatAvg sigue siendo entero SOLO para interfaz/logica heredada.
float rates[RATE_SIZE]; byte  rateSpot       = 0; long  lastBeat       = 0; float beatsPerMinute = 0;
int   beatAvg        = 0; byte  latidosValidos = 0; float acFiltrado = 0; float acAnterior = 0;
float acMax      = 0; float acMin      = 0;
const float HP_B0 =  0.97803048f; const float HP_B1 = -1.95606096f;
const float HP_B2 =  0.97803048f; const float HP_A1 = -1.95557824f; const float HP_A2 =  0.95654368f;
const float LP_B0 =  0.01043241f; const float LP_B1 =  0.02086483f;
const float LP_B2 =  0.01043241f; const float LP_A1 = -1.69099638f; const float LP_A2 =  0.73272603f;
float hp_z1 = 0.0f, hp_z2 = 0.0f; float lp_z1 = 0.0f, lp_z2 = 0.0f;
float irDC_plot  = 0.0f; float irACActual = 0.0f;
float biquadDF2T(float x, float b0, float b1, float b2,
                 float a1, float a2,
                 float &z1, float &z2) {
  float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2;
  z2 = b2 * x - a2 * y; return y; }
float aplicarFiltroBPF(float x) {
  float salidaHP = biquadDF2T(x, HP_B0, HP_B1, HP_B2, HP_A1, HP_A2, hp_z1, hp_z2);
  return biquadDF2T(salidaHP, LP_B0, LP_B1, LP_B2, LP_A1, LP_A2, lp_z1, lp_z2);
} void resetearButterworthPPG() {
  hp_z1 = 0.0f; hp_z2 = 0.0f;
  lp_z1 = 0.0f; lp_z2 = 0.0f; }
bool detectorRapidoInicializado = false; bool pulsoArmado                = true;
bool enPico                     = false; unsigned long ultimoLatidoRapido           = 0;
unsigned long ultimoLatidoRegistrado       = 0; unsigned long ultimaVezDedoDetectado       = 0;
unsigned long ultimoLatidoRapidoRegistrado = 0; unsigned long ultimoResetTimeout           = 0;
const float BPM_MIN_VALIDO = 35.0f;
const float BPM_MAX_VALIDO = 180.0f; const unsigned long MIN_IBI_MS          = 333; const unsigned long MAX_IBI_MS          = 1715;
const unsigned long TIEMPO_PERDIDA_DEDO = 700; const unsigned long TIMEOUT_RESET_AC    = 6000;
const float OUTLIER_TOLERANCIA_MEDIA = 0.40f; const float OUTLIER_TOLERANCIA_ALTA  = 0.30f;
const float OUTLIER_TOLERANCIA_ARRANQUE = 0.70f; const int BPM_PROMEDIO_ALTO_SOSPECHOSO = 80;
const int BPM_BAJO_REAL_MIN            = 35; const int BPM_BAJO_REAL_MAX            = 65;
const byte BAJOS_CONSECUTIVOS_RESET    = 2; byte candidatosBajosConsecutivos       = 0;
const float FACTOR_REFRACTARIO_DINAMICO = 0.60f;
const unsigned long ETAPA0_MS = 4000; const unsigned long ETAPA1_MS = 8000; float amplitudACActual   = 0;
float umbralRapidoActual = 0;
unsigned long ultimoLatidoAnim=0,tiempoInicioLectura=0,ultimoCruceAC=0; const unsigned long WARMUP_BUTTER_MS=1500;
float bpmFallback=0.0f; int calidadSenalActual=0;
bool bpmBloqueadoPorMovimiento=false,movimientoPPGSostenido=false,warmupButterActivo=false;
String estadoActividad="Reposo";
float filtrada=0,bpm=0,sumaBPM=0,minBPM=1000,maxBPM=0,minFiltro=1e9,maxFiltro=-1e9;
bool contacto=false,contactoCrudo=false,datosVistos=false; uint32_t maxPausaMs=0;
uint32_t relojMuestra=0,perdidas=0,latidos=0,nBPM=0,reiniciosDatos=0;
unsigned long ultimoDato=0,inicioFisicoContacto=0,primerBPMms=0;
// V7: tiempos de vigencia/retencion explicitos. No alteran el detector ni aceptan latidos nuevos.
const unsigned long BPM_VIGENTE_MS = 3000;
const unsigned long RETENCION_BPM_VISIBLE_MS = 8000;
// Separacion entre aceptaciones: diagnostico, no PRV ni IBI fisiologico validado.
// Diagnostico independiente: nunca modifica el filtro ni sus decisiones.
Registro bloque,total; uint32_t ensayo=0,siguienteBloque=10000,inicioBloque=0,basePerdidas=0,baseReinicios=0;
uint32_t ultimoAceptadoReal=0,ultimoIBI=0,sinDatosEventos=0,baseSinDatos=0;
bool previoContacto=false,previoBPM=false,previaMuestra=false,alarmaDatos=false;

// V9: capa posterior; nunca escribe beatAvg, rates, estados del filtro ni relojMuestra.
struct EstadoBPM8 {
 bool valido=false; float ultimo=0; uint32_t fecha=0,secuencia=0;
 float candidatos[3]={0,0,0}; uint8_t n=0,pos=0,fuente=0;
 uint32_t ultimoCandidato=0,promovidos=0,noPromovidos=0;
 uint32_t motivos[6]={0}; // condiciones, espera, incoherencia, salto, separacion (reservado), rango
 float promedios[3]={0}; uint8_t motivo=0;
 uint32_t reiniciosConfirmacion=0;
} bpm8;
uint8_t fuente8=0; // 1=maximo local, 2=cruce, 3=SparkFun.
void confirmarDeNuevo8() {
 bpm8.n=bpm8.pos=0;bpm8.ultimoCandidato=0;
}
void invalidar8() {
 bpm8.valido=false;bpm8.ultimo=0;bpm8.fecha=0;confirmarDeNuevo8();
}
// V9: confirmar el arranque; luego actualizar cada aceptacion coherente.
// El detector conserva su propio refractario. No duplicarlo con millis().
// El historial de confirmacion tolera hasta 6 s; rearme real tambien lo limpia.
const uint32_t CONFIRMACION_MAX_MS=6000;
const char *motivos9[]={"CONDICIONES","ESPERA_3","INCOHERENCIA","SALTO","RESERVADO","RANGO","ACEPTADO"};
void rechazar9(uint8_t motivo) {
 bpm8.motivo=motivo;bpm8.motivos[motivo]++;bpm8.noPromovidos++;
}
void aceptarCandidato8(float candidato,uint8_t fuente) {
 uint32_t t=millis();
 if(!contactoCrudo || !datosVistos || t-ultimoDato>=500 || !lecturaOK || nivel>=2 || warmupButterActivo) {
  confirmarDeNuevo8();rechazar9(0);return;
 }
 if(!isfinite(candidato) || candidato<BPM_MIN_VALIDO || candidato>BPM_MAX_VALIDO ||
    beatAvg<BPM_MIN_VALIDO || beatAvg>BPM_MAX_VALIDO) {rechazar9(5);return;}
 if(bpm8.n && t-bpm8.ultimoCandidato>CONFIRMACION_MAX_MS) {
  confirmarDeNuevo8();bpm8.reiniciosConfirmacion++;
 }
 bpm8.ultimoCandidato=t;
 bpm8.candidatos[bpm8.pos]=candidato;bpm8.promedios[bpm8.pos]=beatAvg;
 bpm8.pos=(bpm8.pos+1)%3;if(bpm8.n<3)bpm8.n++;
 bool continuidad=bpm8.valido && bpm8.n>=3 &&
   fabsf(beatAvg-bpm8.ultimo)<=0.25f*bpm8.ultimo &&
   fabsf(candidato-bpm8.ultimo)<=0.30f*bpm8.ultimo;
 if(!continuidad) {
  if(bpm8.n<3){rechazar9(1);return;}
  float lo=bpm8.promedios[0],hi=lo,media=0;
  for(int i=0;i<3;i++){lo=min(lo,bpm8.promedios[i]);hi=max(hi,bpm8.promedios[i]);media+=bpm8.promedios[i]/3.0f;}
  if(hi-lo>0.25f*media){rechazar9(2);return;}
  // Un salto grande no se confirma solamente con promedios suavizados.
  if(bpm8.valido && fabsf(beatAvg-bpm8.ultimo)>0.25f*bpm8.ultimo) {
   for(int i=0;i<3;i++)if(fabsf(bpm8.candidatos[i]-beatAvg)>0.20f*beatAvg){rechazar9(3);return;}
  }
 }
 bpm8.valido=true;bpm8.ultimo=beatAvg;bpm8.fecha=t;bpm8.secuencia++;
 bpm8.promovidos++;bpm8.fuente=fuente;bpm8.motivo=6;
}
void vigilarEstado8() {
 if(!datosVistos || millis()-ultimoDato>=500){invalidar8();return;}
 // La pantalla oculta inmediatamente sin contacto crudo. Tras 700 ms el detector
 // confirma perdida y reiniciarDetector invalida para no arrastrar a otro contacto.
 if(!lecturaOK || nivel>=2)confirmarDeNuevo8();
 else if(bpm8.n && millis()-bpm8.ultimoCandidato>CONFIRMACION_MAX_MS) {
  confirmarDeNuevo8();bpm8.reiniciosConfirmacion++;
 }
}
const char *calidad8() {
 if(!datosVistos || millis()-ultimoDato>=500 || !contactoCrudo || !lecturaOK || nivel>=2 || warmupButterActivo || bpm8.n<3)return "BAJA";
 float lo=bpm8.candidatos[0],hi=lo,media=0;
 for(int i=0;i<3;i++){lo=min(lo,bpm8.candidatos[i]);hi=max(hi,bpm8.candidatos[i]);media+=bpm8.candidatos[i]/3.0f;}
 if(media<=0 || hi-lo>0.20f*media)return "BAJA";
 return nivel==0 && hi-lo<=0.10f*media ? "ALTA" : "MEDIA";
}
const char *estado8() {
 if(!datosVistos || millis()-ultimoDato>=500)return "SIN DATOS";
 if(!contactoCrudo)return "SIN CONTACTO";
 if(fase==1)return "ESTABILIZANDO";
 if(!bpm8.valido)return "CALCULANDO";
 uint32_t edad=millis()-bpm8.fecha;
 if(edad>=10000)return "ULTIMO / RECALCULANDO";
 if(edad>=5000 || !lecturaOK || nivel>=2)return "RETENIDO";
 return "ACTUALIZADO";
}
bool mostrarBPM8() {return fase!=1 && datosVistos && millis()-ultimoDato<500 && contactoCrudo && bpm8.valido;}
void informarEstado8() {
 vigilarEstado8();
 diagnosticoInicio18.printf("VALID9: BPM=%.0f Estado=%s Calidad=%s Edad_ms=%ld Sec=%lu Fuente=%u Promociones=%lu No_promovidos=%lu\n",
 mostrarBPM8()?bpm8.ultimo:0,estado8(),calidad8(),bpm8.valido?(long)(millis()-bpm8.fecha):-1L,
 (unsigned long)bpm8.secuencia,bpm8.fuente,(unsigned long)bpm8.promovidos,(unsigned long)bpm8.noPromovidos);
 adquirir();
 diagnosticoInicio18.printf("GATE9: ultimo=%s pendientes=%u rechazos(cond/espera/coherencia/salto/rango)=%lu/%lu/%lu/%lu/%lu reinicios_confirmacion=%lu\n",
 motivos9[bpm8.motivo],bpm8.n,(unsigned long)bpm8.motivos[0],(unsigned long)bpm8.motivos[1],
 (unsigned long)bpm8.motivos[2],(unsigned long)bpm8.motivos[3],(unsigned long)bpm8.motivos[5],(unsigned long)bpm8.reiniciosConfirmacion);
}
void registrarDiagnostico(Registro &r,bool hayBPM,bool caida,bool corte,bool aceptado,uint32_t ibi) {
 r.n++;if(contactoCrudo)r.contacto++;if(caida)r.caidas++;if(corte)r.cortesBPM++;
 if(hayBPM){r.conBPM++;r.bpmSuma+=bpm;r.bpmMin=min(r.bpmMin,bpm);r.bpmMax=max(r.bpmMax,bpm);}
 if(aceptado)r.aceptados++;
 if(ibi){r.ibiN++;r.ibiSuma+=ibi;r.ibiMin=min(r.ibiMin,ibi);r.ibiMax=max(r.ibiMax,ibi);}
}
// Coberturas por muestra de la fase de 120 s, independientes del criterio V7.
uint32_t n9=0,visible9=0,reciente9=0,antiguo9=0,maxEdad9=0,primero9=0;
void observarMuestra(unsigned long latidoAntes) {
 if(fase==2 && !pausaEstadisticas) {
  n9++;
  if(mostrarBPM8()) {
   visible9++;uint32_t edad=millis()-bpm8.fecha;maxEdad9=max(maxEdad9,edad);
   if(edad<5000)reciente9++;if(edad>=10000)antiguo9++;
   if(!primero9)primero9=millis()-inicio+1;
  }
 }
 bool hayBPM=contactoCrudo && bpm>0; bool caida=previaMuestra && previoContacto && !contactoCrudo;
 bool corte=previaMuestra && previoBPM && contactoCrudo && !hayBPM;
 bool aceptado=ultimoLatidoRegistrado && ultimoLatidoRegistrado!=latidoAntes;
 uint32_t ibi=aceptado && latidoAntes ? ultimoLatidoRegistrado-latidoAntes : 0;
 if(aceptado){flagsTraza|=8;ultimoAceptadoReal=millis();ultimoIBI=ibi;}
 if(fase && !pausaEstadisticas){registrarDiagnostico(total,hayBPM,caida,corte,aceptado,ibi);
  if(fase==2){registrarDiagnostico(bloque,hayBPM,caida,corte,aceptado,ibi);
   if(contactoCrudo)flagsTraza|=1;if(bpmBloqueadoPorMovimiento)flagsTraza|=32;if(!hayBPM)flagsTraza|=64;
   if(++divisionTraza==4){divisionTraza=0;
    if(trazas && nTraza<CAP_TRAZA)trazas[nTraza++]={(uint32_t)(millis()-inicio),ir,filtrada,umbralRapidoActual,flagsTraza};
    else trazaOmitida++;
    flagsTraza=0;
   }
  }}
 previoContacto=contactoCrudo;previoBPM=hayBPM;previaMuestra=true;
}

// ===== B17: PRV (Pulse Rate Variability) EXPLORATORIA =====
// Estrategia:
// - NO crea un segundo filtro PPG.
// - Toma el tiempo de los picos de ultimoLatidoRapido, generado por el detector existente
//   sobre la salida del Butterworth ya validado.
// - No usa beatAvg para construir los PP: evita que el promedio de BPM aplaste la variabilidad.
// - Mantiene dos conceptos separados:
//      a) PP crudos plausibles -> sirven para detectar irregularidad/patron corto-largo.
//      b) PP "limpios" -> sirven para SDNN/RMSSD/pNN50/CVNN.
// - Movimiento/contacto pobre/cortes de datos rompen la continuidad; no se interpolan latidos.
//
// IMPORTANTE: PRV derivada de PPG NO equivale a HRV de ECG y estas reglas NO diagnostican
// arritmias ni extrasistoles. El indicador "patronEctopico" es solo una bandera exploratoria.

const uint16_t PRV17_CAP = 200;              // >= 60 s incluso cerca de 180 pulsos/min
const uint32_t PRV17_VENTANA_MS = 60000;     // metrica movil operativa
const uint32_t PRV17_EVENTOS_MS = 30000;     // ventana para patron ectopico/irregularidad
const uint32_t PRV17_CALC_MS = 5000;         // recalculo/log cada 5 s
const uint16_t PRV17_MIN_PPI = 333;          // coherente con 180 lpm
const uint16_t PRV17_MAX_PPI = 1715;         // coherente con 35 lpm
const uint8_t PRV17_MIN_NN = 20;             // minimo pragmatico para salida exploratoria
const uint32_t PRV17_MIN_SPAN_MS = 30000;    // no declarar valida una ventana demasiado corta

const uint8_t PRV17_F_LIMPIO   = 0x01;
const uint8_t PRV17_F_IRREG    = 0x02;
const uint8_t PRV17_F_ECTOPIA  = 0x04;
const uint8_t PRV17_F_CORTE    = 0x08;

struct PRV17Intervalo {
  uint32_t t;      // tiempo logico de muestra (relojMuestra), no millis() de la UI
  uint16_t ppi;    // intervalo pulso-pulso [ms]
  uint8_t flags;
};

struct PRV17Estado {
  bool valido=false;
  float ppMedio=NAN;
  float bpmMedio=NAN;
  float sdnn=NAN;
  float rmssd=NAN;
  float pnn50=NAN;
  float cvnn=NAN;
  uint16_t nTotal=0;
  uint16_t nNN=0;
  uint16_t paresNN=0;
  uint8_t porcentajeLimpio=0;
  uint8_t irregulares30s=0;
  uint8_t patronesEctopicos30s=0;
  bool patronEctopico=false;
  uint32_t spanMs=0;
  uint32_t fecha=0;
} prv17Estado;

PRV17Intervalo prv17Buf[PRV17_CAP];
uint16_t prv17Pos=0,prv17N=0;
uint32_t prv17FuenteVista=0,prv17UltimoPulso=0;
uint32_t prv17UltCalc=0,prv17MalaDesde=0,prv17ReiniciosVistos=0;
bool prv17CortePendiente=true;
bool prv17PrevShort=false;
uint16_t prv17PrevShortPPI=0;
float prv17PrevShortBase=0;
uint32_t prv17IrregularesTotal=0,prv17PatronesEctopicosTotal=0,prv17ReiniciosVentana=0;

uint16_t prv17IndiceCronologico(uint16_t j){
  uint16_t inicio=(prv17Pos + PRV17_CAP - prv17N)%PRV17_CAP;
  return (inicio+j)%PRV17_CAP;
}

void prv17Ordenar(float *v,uint8_t n){
  for(uint8_t i=0;i<n;i++)for(uint8_t j=i+1;j<n;j++)if(v[j]<v[i]){
    float z=v[i];v[i]=v[j];v[j]=z;
  }
}

float prv17Mediana(float *v,uint8_t n){
  if(!n)return NAN;
  prv17Ordenar(v,n);
  if(n&1)return v[n/2];
  return 0.5f*(v[n/2-1]+v[n/2]);
}

uint8_t prv17BaseRobusta(float &med,float &mad){
  float v[9];uint8_t n=0;
  for(int j=(int)prv17N-1;j>=0 && n<9;j--){
    const PRV17Intervalo &e=prv17Buf[prv17IndiceCronologico((uint16_t)j)];
    if(e.flags & PRV17_F_LIMPIO)v[n++]=e.ppi;
  }
  if(n<5){med=mad=NAN;return n;}
  med=prv17Mediana(v,n);
  float d[9];
  for(uint8_t i=0;i<n;i++)d[i]=fabsf(v[i]-med);
  mad=prv17Mediana(d,n);
  return n;
}

bool prv17CalidadInstantanea(){
  return fase==2 && datosVistos && (millis()-ultimoDato<500) &&
         contactoCrudo && lecturaOK && nivel==0 && giro<20.0f &&
         !warmupButterActivo && ir>=8000 && ir<260000;
}

void reiniciarPRV17(bool total){
  prv17Pos=prv17N=0;
  prv17FuenteVista=0;
  prv17UltimoPulso=0;
  prv17CortePendiente=true;
  prv17PrevShort=false;prv17PrevShortPPI=0;prv17PrevShortBase=0;
  prv17Estado=PRV17Estado();
  prv17MalaDesde=0;
  if(total){
    prv17IrregularesTotal=0;
    prv17PatronesEctopicosTotal=0;
    prv17ReiniciosVentana=0;
  }else{
    prv17ReiniciosVentana++;
  }
}

void prv17Guardar(uint32_t t,uint16_t ppi,uint8_t flags){
  prv17Buf[prv17Pos]={t,ppi,flags};
  prv17Pos=(prv17Pos+1)%PRV17_CAP;
  if(prv17N<PRV17_CAP)prv17N++;
}

// Se llama una vez por muestra DESPUES de procesarPPG().
// Observa ultimoLatidoRapido sin modificarlo.
void muestraPRV17(){
  uint32_t pulso=ultimoLatidoRapido;
  if(!pulso || pulso==prv17FuenteVista)return;

  // Siempre sincronizar la fuente para que un tramo rechazado no cree un PPI gigante despues.
  uint32_t previo=prv17FuenteVista;
  prv17FuenteVista=pulso;
  prv17UltimoPulso=pulso;

  if(!previo){
    prv17CortePendiente=true;
    return;
  }

  uint32_t d=pulso-previo;
  if(!prv17CalidadInstantanea()){
    prv17CortePendiente=true;
    prv17PrevShort=false;
    return;
  }

  if(d<PRV17_MIN_PPI || d>PRV17_MAX_PPI){
    // Pulso fuera del rango fisiologico admitido por el propio detector BPM:
    // no se fuerza ni recorta. Se rompe continuidad para RMSSD/pNN50.
    prv17CortePendiente=true;
    prv17PrevShort=false;
    return;
  }

  uint16_t ppi=(uint16_t)d;
  uint8_t flags=0;
  float med=NAN,mad=NAN;
  uint8_t nBase=prv17BaseRobusta(med,mad);

  bool limpio=true,irregular=false,shortBeat=false,longBeat=false,ectopia=false;

  if(nBase>=5 && isfinite(med) && med>0){
    float delta=fabsf((float)ppi-med);
    // Criterio robusto: 5 sigma robustas (1.4826*MAD), con piso 120 ms
    // y techo 25% de la mediana. Evita que un MAD inflado acepte cambios enormes.
    float limiteRobusto=5.0f*1.4826f*mad;
    float limite=max(120.0f,limiteRobusto);
    limite=min(limite,0.25f*med);

    irregular=delta>0.20f*med;
    limpio=delta<=limite;
    shortBeat=(float)ppi<0.80f*med;
    longBeat =(float)ppi>1.20f*med;

    // Patron PPG exploratorio: intervalo prematuro seguido de pausa larga.
    // Se exige compensacion aproximada del par (~2 ciclos basales) y continuidad.
    if(!prv17CortePendiente && prv17PrevShort && longBeat){
      float base=(prv17PrevShortBase>0)?0.5f*(prv17PrevShortBase+med):med;
      float suma=(float)prv17PrevShortPPI+(float)ppi;
      if(suma>=1.70f*base && suma<=2.30f*base)ectopia=true;
    }
  }

  if(limpio)flags|=PRV17_F_LIMPIO;
  if(irregular){flags|=PRV17_F_IRREG;prv17IrregularesTotal++;}
  if(ectopia){flags|=PRV17_F_ECTOPIA;prv17PatronesEctopicosTotal++;}
  if(prv17CortePendiente)flags|=PRV17_F_CORTE;

  prv17Guardar(pulso,ppi,flags);

  if(nBase>=5 && isfinite(med)){
    if(shortBeat){
      prv17PrevShort=true;prv17PrevShortPPI=ppi;prv17PrevShortBase=med;
    }else{
      // El largo ya fue evaluado; cualquier otro intervalo cierra el candidato corto.
      prv17PrevShort=false;
    }
  }else{
    prv17PrevShort=false;
  }

  prv17CortePendiente=false;
}

void calcularPRV17(){
  PRV17Estado r;
  if(!prv17N){prv17Estado=r;return;}

  uint32_t ahora=relojMuestra;
  uint32_t desde=(ahora>PRV17_VENTANA_MS)?ahora-PRV17_VENTANA_MS:0;
  uint32_t desdeEvt=(ahora>PRV17_EVENTOS_MS)?ahora-PRV17_EVENTOS_MS:0;

  double suma=0,suma2=0,sumaDif2=0;
  uint16_t pares=0,nn50=0;
  bool previoLimpio=false;
  float previoPPI=0;
  uint32_t primero=0;

  for(uint16_t j=0;j<prv17N;j++){
    const PRV17Intervalo &e=prv17Buf[prv17IndiceCronologico(j)];
    if(e.t<desde)continue;
    if(!primero)primero=e.t;
    r.nTotal++;

    if(e.t>=desdeEvt){
      if((e.flags&PRV17_F_IRREG) && r.irregulares30s<255)r.irregulares30s++;
      if((e.flags&PRV17_F_ECTOPIA) && r.patronesEctopicos30s<255)r.patronesEctopicos30s++;
    }

    bool limpio=e.flags&PRV17_F_LIMPIO;
    if(limpio){
      double x=e.ppi;suma+=x;suma2+=x*x;r.nNN++;
      if(previoLimpio && !(e.flags&PRV17_F_CORTE)){
        float dif=(float)e.ppi-previoPPI;
        sumaDif2+=(double)dif*dif;
        if(fabsf(dif)>50.0f)nn50++;
        pares++;
      }
      previoPPI=e.ppi;previoLimpio=true;
    }else{
      previoLimpio=false;
    }
  }

  r.spanMs=primero?ahora-primero:0;
  r.paresNN=pares;
  r.porcentajeLimpio=r.nTotal?(uint8_t)min(100,(int)lroundf(100.0f*r.nNN/r.nTotal)):0;
  r.patronEctopico=r.patronesEctopicos30s>=3; // bandera conservadora, no diagnostico

  if(r.nNN>=2){
    double media=suma/r.nNN;
    double var=(suma2-r.nNN*media*media)/(r.nNN-1);
    if(var<0)var=0;
    r.ppMedio=(float)media;
    r.bpmMedio=media>0?(float)(60000.0/media):NAN;
    r.sdnn=(float)sqrt(var);
    r.cvnn=media>0?(float)(100.0*sqrt(var)/media):NAN;
  }
  if(pares){
    r.rmssd=(float)sqrt(sumaDif2/pares);
    r.pnn50=100.0f*nn50/pares;
  }

  r.valido = r.nNN>=PRV17_MIN_NN &&
             r.paresNN>=10 &&
             r.spanMs>=PRV17_MIN_SPAN_MS &&
             r.porcentajeLimpio>=80 &&
             prv17CalidadInstantanea() &&
             isfinite(r.rmssd) && isfinite(r.sdnn);
  r.fecha=millis();
  prv17Estado=r;
}

void informarPRV17(){
  if(!Serial)return;
  Serial.printf(
    "PRV17: valido=%d ventana_ms=%lu PP_NN=%u/%u limpio=%u%% PPmedio=%.1f ms "
    "BPMmedio=%.1f SDNN=%.1f ms RMSSD=%.1f ms pNN50=%.1f%% CVNN=%.2f%% "
    "irreg30=%u patron_corto_largo30=%u sospecha_ectopia=%d reinicios=%lu\n",
    prv17Estado.valido,(unsigned long)prv17Estado.spanMs,
    prv17Estado.nNN,prv17Estado.nTotal,prv17Estado.porcentajeLimpio,
    prv17Estado.ppMedio,prv17Estado.bpmMedio,prv17Estado.sdnn,
    prv17Estado.rmssd,prv17Estado.pnn50,prv17Estado.cvnn,
    prv17Estado.irregulares30s,prv17Estado.patronesEctopicos30s,
    prv17Estado.patronEctopico,(unsigned long)prv17ReiniciosVentana);
}

// Gestion de calidad fuera del detector. Si la señal deja de ser apta de forma sostenida,
// se inicia una ventana nueva; nunca se reinicia Butterworth/BPM desde PRV.
void tickPRV17(){
  uint32_t t=millis();

  if(reiniciosDatos!=prv17ReiniciosVistos){
    prv17ReiniciosVistos=reiniciosDatos;
    reiniciarPRV17(false);
  }

  bool calidad=prv17CalidadInstantanea();
  if(!calidad){
    if(!prv17MalaDesde)prv17MalaDesde=t;
    if(t-prv17MalaDesde>=500 && (prv17N || prv17FuenteVista))reiniciarPRV17(false);
  }else{
    prv17MalaDesde=0;
    if(prv17UltimoPulso && relojMuestra-prv17UltimoPulso>2500){
      reiniciarPRV17(false);
    }
  }

  if(!prv17UltCalc || t-prv17UltCalc>=PRV17_CALC_MS){
    prv17UltCalc=t;
    calcularPRV17();
    informarPRV17();
  }
}

// Accesores simples preparados para la futura etapa BLE.
// No envian nada todavia.
bool prvValido17(){return prv17Estado.valido;}
float prvRMSSD17(){return prv17Estado.rmssd;}
float prvSDNN17(){return prv17Estado.sdnn;}
float prvPNN5017(){return prv17Estado.pnn50;}
float prvPPMedio17(){return prv17Estado.ppMedio;}
bool prvPatronEctopico17(){return prv17Estado.patronEctopico;}


long edadPulso() {return ultimoAceptadoReal ? (long)(millis()-ultimoAceptadoReal) : -1;}
void imprimirRegistro(const Registro &r) {
 Serial.printf("Contacto=%.1f%% Caidas_contacto=%lu Cortes_BPM_con_contacto=%lu\n",r.n?100.0f*r.contacto/r.n:0,(unsigned long)r.caidas,(unsigned long)r.cortesBPM);adquirir();
 Serial.printf("Cobertura_BPM=%.1f%% Actualizaciones=%lu ",r.n?100.0f*r.conBPM/r.n:0,(unsigned long)r.aceptados);
 if(r.conBPM)Serial.printf("BPM_detector min=%.1f media=%.1f max=%.1f\n",r.bpmMin,r.bpmSuma/r.conBPM,r.bpmMax);
 else Serial.println("BPM: --");adquirir();
 if(r.ibiN)Serial.printf("Separacion_aceptados_ms min=%lu media=%.1f max=%lu\n",(unsigned long)r.ibiMin,(double)r.ibiSuma/r.ibiN,(unsigned long)r.ibiMax);
 else Serial.println("Separacion_aceptados_ms: sin pares consecutivos");adquirir();
}
void informeBloque() {
 // Copia y vacia SOLO estadisticas; las nuevas muestras pertenecen al siguiente bloque.
 Registro copia=bloque;bloque=Registro(); uint32_t ahora=millis(),dt=ahora-inicioBloque;inicioBloque=ahora;
 uint32_t perd=perdidas-basePerdidas,rein=reiniciosDatos-baseReinicios,sd=sinDatosEventos-baseSinDatos;
 basePerdidas=perdidas;baseReinicios=reiniciosDatos;baseSinDatos=sinDatosEventos;
 unsigned long marca=siguienteBloque/1000;siguienteBloque+=10000;
 float valor=(datosVistos && ahora-ultimoDato<500 && contactoCrudo)?bpm:0; long edad=edadPulso();
 Serial.printf("\n=== ENSAYO %lu BLOQUE %lu-%lu s (%.3f s reales) ===\n",(unsigned long)ensayo,marca-10,marca,dt/1000.0f);adquirir();
 Serial.printf("PPG=%.1f Hz Perdidas=%lu Reinicios=%lu Sin_datos=%lu\n",dt?1000.0f*copia.n/dt:0,(unsigned long)perd,(unsigned long)rein,(unsigned long)sd);adquirir();
 imprimirRegistro(copia);
 Serial.printf("COMPARAR t=%lu s Detector_V7=%.0f BPM Edad_pulso=%ld ms; anota OXIMETRO\n",marca,valor,edad);adquirir();
}
void resetearDetectorRapido() {
  irDC_plot         = 0; irACActual        = 0; acFiltrado        = 0; acAnterior        = 0;
  acMax             = 0; acMin             = 0; amplitudACActual  = 0; umbralRapidoActual = 0;
  detectorRapidoInicializado     = false; pulsoArmado                    = true;
  enPico                         = false; ultimoLatidoRapido             = 0;
  ultimoLatidoRegistrado         = 0; ultimoLatidoRapidoRegistrado   = 0; ultimoResetTimeout             = 0;
  tiempoInicioLectura = 0; ultimoCruceAC       = 0; bpmFallback         = 0.0f;
  bpmBloqueadoPorMovimiento = false; calidadSenalActual = 0;
  resetearButterworthPPG(); }
bool registrarLatido(unsigned long ahora, float bpmNuevo) {
  if (!isfinite(bpmNuevo) || bpmNuevo < BPM_MIN_VALIDO || bpmNuevo > BPM_MAX_VALIDO) {rechazoRango++;return false;}
  unsigned long intervaloMinimo = MIN_IBI_MS;
  if (beatAvg > 0 && latidosValidos >= RATE_SIZE) {
    unsigned long intervaloPromedio = 60000UL / (unsigned long)beatAvg;
    intervaloMinimo = (unsigned long)(intervaloPromedio * FACTOR_REFRACTARIO_DINAMICO);
    if (intervaloMinimo < MIN_IBI_MS) intervaloMinimo = MIN_IBI_MS;
    if (intervaloMinimo > 1200)       intervaloMinimo = 1200;
  } if (ultimoLatidoRegistrado > 0 &&
      (ahora - ultimoLatidoRegistrado) < intervaloMinimo)
    {rechazoCercano++;return false;}
  bool promedioAltoSospechoso = (beatAvg >= BPM_PROMEDIO_ALTO_SOSPECHOSO && latidosValidos >= 2);
  bool bpmBajoCoherente = (bpmNuevo >= BPM_BAJO_REAL_MIN && bpmNuevo <= BPM_BAJO_REAL_MAX);
  if (promedioAltoSospechoso && bpmBajoCoherente) {
    candidatosBajosConsecutivos++;
    if (candidatosBajosConsecutivos < BAJOS_CONSECUTIVOS_RESET) {
      return false; } beatAvg = 0;
    beatsPerMinute = 0; rateSpot = 0; latidosValidos = 0; for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0.0f;
    candidatosBajosConsecutivos = 0; } else {
    candidatosBajosConsecutivos = 0; }
  if (latidosValidos >= 3 && beatAvg > 0) {
    float tolerancia; if      (latidosValidos < BPM_ESTABLE_MIN) tolerancia = OUTLIER_TOLERANCIA_ARRANQUE;
    else if (latidosValidos < RATE_SIZE)       tolerancia = OUTLIER_TOLERANCIA_MEDIA;
    else                                       tolerancia = OUTLIER_TOLERANCIA_ALTA;
    float variacion = fabs(bpmNuevo - (float)beatAvg) / (float)beatAvg;
    if (variacion > tolerancia) {
      rechazoCambio++;return false; } }
  ultimoLatidoRegistrado = ahora; ultimoLatidoAnim       = ahora; beatsPerMinute         = bpmNuevo;
  // V7: no truncar bpmNuevo a byte. Se conserva precision sub-BPM dentro de la ventana.
  rates[rateSpot++] = bpmNuevo; rateSpot %= RATE_SIZE;
  if (latidosValidos < RATE_SIZE) latidosValidos++;
  float sorted[RATE_SIZE]; byte n = 0;
  for (byte x = 0; x < RATE_SIZE; x++) {
    if (rates[x] > 0.0f) sorted[n++] = rates[x];
  } for (byte i = 0; i < n - 1; i++)
    for (byte j = i + 1; j < n; j++)
      if (sorted[i] > sorted[j]) {
        float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
  float suma = 0.0f;
  byte cuenta = 0; if (n >= 4) {
    for (byte i = 1; i < n - 1; i++) { suma += sorted[i]; cuenta++; }
  } else {
    for (byte i = 0; i < n; i++)     { suma += sorted[i]; cuenta++; }
  }
  // La interfaz/protocolo siguen usando BPM entero, pero ahora se redondea una sola vez.
  if (cuenta > 0) beatAvg = (int)lroundf(suma / (float)cuenta);
  aceptarCandidato8(bpmNuevo,fuente8);
  return true; }
bool detectarLatidoRapido(long irValue, unsigned long ahora) {
  if (!detectorRapidoInicializado) {
    float fIR = (float)irValue; hp_z2 = HP_B2 * fIR;
    hp_z1 = HP_B1 * fIR + hp_z2;
    lp_z1 = 0.0f; lp_z2 = 0.0f;
    irDC_plot         = fIR;
    irACActual        = 0; acFiltrado        = 0;
    acAnterior        = 0; acMax             = 0;
    acMin             = 0; amplitudACActual  = 0;
    umbralRapidoActual = 0;
    detectorRapidoInicializado = true;
    pulsoArmado                = true;
    enPico                     = false;
    ultimoLatidoRapido         = 0;
    ultimoCruceAC              = 0;
    bpmBloqueadoPorMovimiento = false;
    warmupButterActivo = true; return false; }
  unsigned long tiempoContacto = (tiempoInicioLectura > 0)
    ? (ahora - tiempoInicioLectura) : 0; int etapa;
  if      (latidosValidos >= RATE_SIZE)    etapa = 2;
  else if (tiempoContacto < ETAPA0_MS)     etapa = 0;
  else if (tiempoContacto < ETAPA1_MS)     etapa = 1;
  else                                     etapa = 2;
  irDC_plot  = 0.99f * irDC_plot + 0.01f * (float)irValue;
  irACActual = (float)irValue - irDC_plot;
  acFiltrado = aplicarFiltroBPF((float)irValue);
  float factorDecaimiento; switch (etapa) {
    case 0:  factorDecaimiento = 0.985f; break;
    case 1:  factorDecaimiento = 0.990f; break;
    default: factorDecaimiento = 0.995f; break;
  } if (acFiltrado > acMax) acMax = acFiltrado;
  else                    acMax *= factorDecaimiento;
  if (acFiltrado < acMin) acMin = acFiltrado;
  else                    acMin *= factorDecaimiento;
  amplitudACActual = acMax - acMin;
  float minUmbralAlto, minUmbralBajo, minAmplitudLatido;
  switch (etapa) { case 0:
      minUmbralAlto     = 28.0f;
      minUmbralBajo     = 10.0f;
      minAmplitudLatido = 12.0f; break; case 1:
      minUmbralAlto     = 40.0f;
      minUmbralBajo     = 15.0f;
      minAmplitudLatido = 18.0f; break; default:
      minUmbralAlto     = 55.0f;
      minUmbralBajo     = 22.0f;
      minAmplitudLatido = 28.0f; break; }
  float umbralAlto = max(minUmbralAlto, amplitudACActual * 0.35f);
  float umbralBajo = max(minUmbralBajo, amplitudACActual * 0.12f);
  umbralRapidoActual = umbralAlto;
  if (tiempoContacto < WARMUP_BUTTER_MS) {
    pulsoArmado = true;
    enPico = false; ultimoCruceAC = 0;
    acAnterior = acFiltrado;
    return false; } if (warmupButterActivo) {
    acMax = 0; acMin = 0; amplitudACActual = 0;
    umbralRapidoActual = 0; pulsoArmado = true;
    enPico = false; ultimoCruceAC = 0;
    acAnterior = acFiltrado;
    warmupButterActivo = false; return false; }
  bpmBloqueadoPorMovimiento = (estadoActividad == "Mov. alto" || movimientoPPGSostenido);
  if (bpmBloqueadoPorMovimiento) {
    pulsoArmado = true;
    enPico = false; ultimoCruceAC = 0;
    acAnterior = acFiltrado; return false; }
  if (ultimoLatidoRapido > 0 &&
      (ahora - ultimoLatidoRapido) > TIMEOUT_RESET_AC &&
      (ahora - ultimoResetTimeout) > TIMEOUT_RESET_AC) {
    acMax            = 0; acMin            = 0;
    pulsoArmado      = true;
    enPico           = false;
    ultimoResetTimeout = ahora; }
  bool latidoRegistrado = false;
  if(trazaSubiendo && acAnterior>0 && acFiltrado<acAnterior)flagsTraza|=2;
  trazaSubiendo=acFiltrado>acAnterior;
  if (acAnterior < 0.0f && acFiltrado >= 0.0f && amplitudACActual > 12.0f) {
    unsigned long delta = ahora - ultimoCruceAC;
    if (ultimoCruceAC > 0 && delta >= MIN_IBI_MS && delta <= MAX_IBI_MS) {
      bpmFallback = 60000.0f / (float)delta;
      if (latidosValidos < BPM_ESTABLE_MIN) {
        registrarLatido(ahora, bpmFallback);
      } } ultimoCruceAC = ahora; }
  if (!enPico && pulsoArmado &&
      acFiltrado > umbralAlto &&
      amplitudACActual > minAmplitudLatido) {
    enPico = true; flagsTraza|=4; }
  if (enPico && acFiltrado < acAnterior) {
    enPico      = false; pulsoArmado = false;
    unsigned long delta = ahora - ultimoLatidoRapido;
    if (ultimoLatidoRapido == 0) {
      ultimoLatidoRapido = ahora;
    } else if (delta >= MIN_IBI_MS && delta <= MAX_IBI_MS) {
      float bpmRapido = 60000.0f / delta;
      fuente8=1;latidoRegistrado = registrarLatido(ahora, bpmRapido);
      if (latidoRegistrado) ultimoLatidoRapidoRegistrado = ahora;
      ultimoLatidoRapido = ahora;
    } else if (delta > MAX_IBI_MS) {
      ultimoLatidoRapido = ahora; } }
  if (!pulsoArmado && acFiltrado < umbralBajo) {
    pulsoArmado = true; } acAnterior = acFiltrado;
  return latidoRegistrado;
} void resetearBPM() { beatsPerMinute   = 0;
  beatAvg          = 0; rateSpot         = 0;
  lastBeat         = 0; ultimoLatidoAnim = 0;
  latidosValidos   = 0;
  candidatosBajosConsecutivos = 0;
  for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0.0f;
  resetearDetectorRapido(); }
void reiniciarDetector() {invalidar8();ultimoRearme=relojMuestra;resetearBPM();contacto=false;filtrada=bpm=0;ultimaVezDedoDetectado=0;inicioFisicoContacto=0;}
void limpiarFase() {
 total=Registro();sinDatosEventos=0;limpiar();maxPausaMs=0;perdidas=latidos=nBPM=reiniciosDatos=0;
 sumaBPM=0;minBPM=1000;maxBPM=0;minFiltro=1e9;maxFiltro=-1e9;
} void procesarPPG() { relojMuestra+=10;
 unsigned long ahora=relojMuestra;
 // Si no hay aceptaciones por 6 s, liberar el historial de BPM y los picos.
 // Conserva coeficientes y estados HP/LP: no crea otro transitorio del filtro.
 unsigned long referencia=ultimoLatidoRegistrado ? ultimoLatidoRegistrado : tiempoInicioLectura;
 if(contacto && contactoCrudo && lecturaOK && nivel<2 && referencia &&
    ahora-referencia>=6000 && ahora-ultimoRearme>=6000 && !warmupButterActivo) {
  beatAvg=0;latidosValidos=rateSpot=0;beatsPerMinute=0;candidatosBajosConsecutivos=0;
  for(byte i=0;i<RATE_SIZE;i++)rates[i]=0.0f;
  lastBeat=0;ultimoLatidoRapido=ultimoLatidoRapidoRegistrado=ultimoCruceAC=0;
  acMax=acMin=0;pulsoArmado=true;enPico=false;ultimoRearme=ahora;rearmes++;flagsTraza|=16;confirmarDeNuevo8();
 }
 bool crudo=ir>=8000; contactoCrudo=crudo;
 if(crudo)ultimaVezDedoDetectado=ahora;
 contacto=crudo || (ultimaVezDedoDetectado && ahora-ultimaVezDedoDetectado<TIEMPO_PERDIDA_DEDO);
 if(!contacto){reiniciarDetector();return;}
 if(!inicioFisicoContacto)inicioFisicoContacto=millis();
 if(!tiempoInicioLectura)tiempoInicioLectura=ahora;
 estadoActividad=(!lecturaOK || nivel==2) ? "Mov. alto" : (nivel==1 ? "Mov. leve" : "Reposo");
 unsigned long previo=ultimoLatidoRegistrado;
 detectarLatidoRapido(ir,ahora);
 bool spark=checkForBeat(ir);
 bool usar=latidosValidos<BPM_ESTABLE_MIN || ultimoLatidoRapidoRegistrado==0;
 if(!usar)usar=ahora-ultimoLatidoRapidoRegistrado>(beatAvg>0 ? 60000UL/beatAvg : 1000UL);
 if(usar && spark && !warmupButterActivo && !bpmBloqueadoPorMovimiento) {
   long delta=ahora-lastBeat;
   if(!lastBeat)lastBeat=ahora; else if(delta>0) {
     float v=60000.0f/delta;fuente8=3;bool aceptado=registrarLatido(ahora,v);
     if(aceptado || (v>=BPM_MIN_VALIDO && v<=BPM_MAX_VALIDO) || delta>(long)MAX_IBI_MS)lastBeat=ahora;
   } } filtrada=acFiltrado;
 bpm=(!bpmBloqueadoPorMovimiento && ultimoLatidoRegistrado && ahora-ultimoLatidoRegistrado<BPM_VIGENTE_MS) ? beatAvg : 0;
 if(bpm>0 && !primerBPMms)primerBPMms=millis()-inicioFisicoContacto;
 if(fase && !pausaEstadisticas) {
   minFiltro=min(minFiltro,filtrada);maxFiltro=max(maxFiltro,filtrada);
   if(ultimoLatidoRegistrado!=previo) {
     latidos++;if(bpm>0){nBPM++;sumaBPM+=bpm;minBPM=min(minBPM,bpm);maxBPM=max(maxBPM,bpm);}
   } } }
// El contacto visible depende SOLO del IR crudo reciente, nunca del filtro.
bool datosFrescos() {return datosVistos && millis()-ultimoDato<500;}
const char *estadoPPG() {
 if(!datosFrescos())return "SIN DATOS";
 if(!contactoCrudo)return "SIN CONTACTO";
 if(!lecturaOK)return "REVISAR IMU";
 if(nivel==2)return "MOVIMIENTO";
 if(warmupButterActivo)return "ESTABILIZANDO";
 return bpm>0 ? "CON BPM" : "CALCULANDO";
}
// ===== V10: ramas exploratorias independientes del detector V9 =====
// SpO2: MAXREFDES117 (Maxim), implementacion incluida en SparkFun.
// Se promedian 4 muestras SOLO en esta rama: 100 Hz -> 25 Hz, ventana 4 s.
// NO se utiliza el BPM devuelto por maxim_heart_rate_and_oxygen_saturation.
// PA: adaptacion propia de PWA + regresion, NO replica del modelo de un paper.
// Caracteristica W50 (anchura a media amplitud), estudiada por Teng/Zhang 2003:
// DOI 10.1109/IEMBS.2003.1280811. Ajuste OLS individual: PA = a + b*W50_ms.
// Los coeficientes se APRENDEN con referencias C sistolica diastolica; no hay
// coeficientes universales, ni garantia de relacion W50/PA en cada persona.
// Haddad et al. DOI 10.1109/JBHI.2021.3128229 usa 27 rasgos + MLR entrenada;
// este firmware NO reproduce esos 27 rasgos, pesos, calibracion ni resultados.
// No cambiar sujeto, dedo o montaje sin X. Calibracion solo RAM: reset la borra.
// C: captura referencia; F: ajusta/congela modelo; X: borra calibracion;
// E: exporta referencias. R: exporta rojo/IR 100 Hz al finalizar. D: traza V9.
#include "spo2_algorithm.h"
static_assert(FreqS==25 && BUFFER_SIZE==100,"Se requiere algoritmo Maxim a 25 Hz / 100 muestras");
void reiniciarExploratorio();
void muestraExploratoria();
void actualizarExploratorio();
void informeExploratorio();
void resumenExploratorio();
void comandoExploratorio(const char *linea);
bool serialExploratorio();
void exportarCrudo();
const uint16_t EXP_N=1000,RAW_CAP=12050;
uint32_t oxIR[100],oxRed[100],oxI[100],oxR[100];
uint8_t oxPos=0,oxN=0,oxDiv=0;uint32_t oxSI=0,oxSR=0;
float ondaPA[EXP_N],copiaPA[EXP_N];uint16_t paPos=0,paN=0;
float exHP1=0,exHP2=0,exLP1=0,exLP2=0;bool exInit=false;
uint32_t exConsecutivas=0,exGeneracion=0,exUltOx=0,exUltPA=0;
bool oxValida=false,rasgoValido=false,paValida=false;
float oxValor=NAN,oxCorr=NAN,oxPI=NAN,w50=NAN,w50CV=NAN,periodoPA=NAN;
float paSis=NAN,paDia=NAN;uint32_t oxFecha=0,rasgoFecha=0,paFecha=0;
uint16_t paPulsos=0;const char *oxMotivo="ESPERANDO",*paMotivo="SIN CALIBRAR";
uint32_t oxAceptadas=0,oxRechazadas=0,paAceptadas=0,paRechazadas=0,exMaxUs=0;
float oxSuma=0,oxMin=101,oxMax=0;
struct Crudo10 {uint32_t ms,ir,rojo;};
Crudo10 *crudos10=nullptr;uint16_t rawN=0,rawEnvio=0;uint32_t rawOmitidas=0;
bool enviandoCrudo=false;
bool buffersGrandesPSRAMOK=false;

// Los dos buffers historicos mas grandes (traza y crudo) no participan del calculo de BPM.
// En V7 ocupaban ~207 kB de DRAM estatica y eran la causa principal del aviso
// "Low memory available". La S3R8 dispone de 8 MB de PSRAM: se alojan alli sin
// alterar filtro, detector, SpO2 ni frecuencia de muestreo.
void inicializarBuffersGrandesPSRAM(){
 trazas=(Traza*)heap_caps_malloc((size_t)CAP_TRAZA*sizeof(Traza),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
 crudos10=(Crudo10*)heap_caps_malloc((size_t)RAW_CAP*sizeof(Crudo10),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
 buffersGrandesPSRAMOK=(trazas!=nullptr && crudos10!=nullptr);
 if(Serial){
  diagnosticoInicio18.printf("BUFFERS_PSRAM: traza=%s (%u B) crudo=%s (%u B) PSRAM_libre=%lu\n",
   trazas?"OK":"ERROR",(unsigned)(CAP_TRAZA*sizeof(Traza)),
   crudos10?"OK":"ERROR",(unsigned)(RAW_CAP*sizeof(Crudo10)),
   (unsigned long)ESP.getFreePsram());
 }
}
struct Cal10 {float x,s,d;uint32_t ensayo,ms;};
Cal10 cal10[24];uint8_t calN=0;bool modeloPA=false;
float aSis=0,bSis=0,aDia=0,bDia=0,calXmin=0,calXmax=0,r2Sis=0,r2Dia=0;
uint32_t ultimaCalFecha=0;bool huboCal=false;

// V11: asistente separado del detector. K inicia una referencia, G valida.
uint8_t guia=0; // 1 estabilizar, 2 listo, 3 manguito, 4 ingresar, 5 guardado, 6 ajuste
bool pedirK=false;
bool capturaUtil=false;
const char *motivoCaptura="Sin captura";
uint32_t guiaInicio=0,guiaFin=0,guiaRasgoFecha=0,guiaPerdidas=0;
float guiaW50=0;
const char *guiaAviso="";
// Ventanas de 10 s, evaluadas cada 5 s; solapadas, NO referencias independientes.
struct VentanaPA13 {uint32_t fin,generacion;float ancho,cv;uint16_t pulsos;};
VentanaPA13 ventanas13[32];uint8_t nVentanas13=0,posVentanas13=0;
uint32_t ultimaVentana13=0;uint16_t intentos13=0,fallosVentana13=0,fallosBPM13=0,fallosForma13=0;
uint8_t usadas13=0;uint32_t edadSeleccion13=0,generacionSeleccion13=0;
void limpiarVentanas13(){nVentanas13=posVentanas13=usadas13=0;ultimaVentana13=0;intentos13=fallosVentana13=fallosBPM13=fallosForma13=0;}
void registrarVentana13(){
 if(guia!=3)return;
 intentos13++;
 if(!rasgoValido){
  if(paN<EXP_N)fallosVentana13++;
  else if(!bpm8.valido || millis()-bpm8.fecha>=5000)fallosBPM13++;
  else fallosForma13++;
  return;
 }
 // La ventana completa debe estar dentro de M..T. No duplicar un calculo.
 if((int32_t)(rasgoFecha-guiaInicio)<10000 || rasgoFecha==ultimaVentana13)return;
 ultimaVentana13=rasgoFecha;
 ventanas13[posVentanas13]={rasgoFecha,exGeneracion,w50,w50CV,paPulsos};
 posVentanas13=(posVentanas13+1)%32;if(nVentanas13<32)nVentanas13++;
}
void seleccionarVentanas13(){
 capturaUtil=false;usadas13=0;edadSeleccion13=UINT32_MAX;generacionSeleccion13=exGeneracion;
 double suma=0,suma2=0;uint32_t finUlt=0;
 for(uint8_t i=0;i<nVentanas13;i++){
  const VentanaPA13 &v=ventanas13[i];uint32_t edad=guiaFin-v.fin;
  // Politica experimental explicita: ultimos 30 s, mismo segmento sin reinicios.
  if(edad>30000 || v.generacion!=exGeneracion)continue;
  suma+=v.ancho;suma2+=(double)v.ancho*v.ancho;usadas13++;
  if(edad<edadSeleccion13){edadSeleccion13=edad;finUlt=v.fin;}
 }
 if(usadas13<2){motivoCaptura="Menos de 2 ventanas";return;}
 if(edadSeleccion13>15000){motivoCaptura="Ventanas antiguas >15s";return;}
 double media=suma/usadas13,cv=sqrt(fmax(0.0,suma2/usadas13-media*media))/media;
 if(!isfinite(media) || !isfinite(cv) || cv>0.20){motivoCaptura="W50 entre ventanas varia";return;}
 guiaW50=media;guiaRasgoFecha=finUlt-inicio;capturaUtil=true;motivoCaptura="Ventanas utiles";
}
void informeVentanas13(){
 Serial.printf("VENTANAS_PA: guardadas=%u seleccionadas=%u edad_ultima_ms=%lu intentos=%u sin_ventana=%u sin_BPM_reciente=%u forma_rechazada=%u\n",nVentanas13,usadas13,(unsigned long)edadSeleccion13,intentos13,fallosVentana13,fallosBPM13,fallosForma13);adquirir();
 Serial.println("VENTANA: inicio_desde_M_ms,fin_desde_M_ms,W50_ms,CV,pulsos,edad_al_T_ms,seleccionada");
 for(uint8_t j=0;j<nVentanas13;j++){
  uint8_t i=nVentanas13==32?(posVentanas13+j)%32:j;
  const VentanaPA13 &v=ventanas13[i];uint32_t fin=v.fin-guiaInicio,edad=guiaFin-v.fin;
  Serial.printf("V13,%lu,%lu,%.3f,%.3f,%u,%lu,%d\n",(unsigned long)(fin-10000),(unsigned long)fin,v.ancho,v.cv,v.pulsos,(unsigned long)edad,(edad<=30000 && v.generacion==generacionSeleccion13));adquirir();
 }
}
void cancelarGuia() {guia=0;fase=0;pruebaTerminada=true;Serial.println("Referencia cancelada. K reintenta; G prueba.");}
bool rasgoGuiaOK() {
 return condicionesExploratorias() && rasgoValido && millis()-rasgoFecha<=5500;
}
// Los pasos nunca retroceden automaticamente por calidad ni por tiempo.
void actualizarGuia() {
 if(guia==1 && fase==2 && (!huboCal || millis()-ultimaCalFecha>=60000)) {
  guia=2;guiaAviso="";
  Serial.println("PASO 2/4: Envia M y enseguida inicia tensiometro en el OTRO brazo.");
 }
}
void finalizarManguito() {
 guiaFin=millis();capturaUtil=false;
 // Congela la lista antes de imprimir; el procesamiento PPG sigue activo.
 guia=4;
 if(guiaFin-guiaInicio<10000)motivoCaptura="Medicion muy corta";
 else if(!datosFrescos())motivoCaptura="Sin datos del sensor";
 else if(!contactoCrudo)motivoCaptura="Sin contacto al terminar";
 else if(!lecturaOK)motivoCaptura="IMU sin datos";
 else if(!condicionesExploratorias())motivoCaptura="Movimiento o saturacion";
 else seleccionarVentanas13();
 guiaAviso="";ultimoLCD=0;
 Serial.println("PASO 4/4: INGRESA PA EN SERIE: C sistolica diastolica + Enter (sin barra).");
 Serial.printf("CAPTURA: util=%d motivo=%s W50=%.2f duracion_M_T_ms=%lu perdidas_desde_M=%lu\n",capturaUtil,motivoCaptura,capturaUtil?guiaW50:NAN,(unsigned long)(guiaFin-guiaInicio),(unsigned long)(perdidas-guiaPerdidas));adquirir();
 informeVentanas13();
}
void pantallaGuia() {
 char b[27];
 campoLCD(0,4,5,2,26,"CALIBRACION PA - V13",ST77XX_CYAN);
 if(guia==1){
  campoLCD(1,4,33,2,26,"Apoya dedo, queda quieto",ST77XX_WHITE);
  if(huboCal && millis()-ultimaCalFecha<60000)snprintf(b,sizeof(b),"Pausa: %lu s",(unsigned long)((60000-(millis()-ultimaCalFecha)+999)/1000));
  else snprintf(b,sizeof(b),"Preparando lectura...");
  campoLCD(2,4,61,2,26,b,ST77XX_YELLOW);
  campoLCD(3,4,89,2,26,"Aun NO inicies manguito",ST77XX_WHITE);
 } else if(guia==2){
  campoLCD(1,4,33,2,26,"2/4 ENVIA M EN SERIE",ST77XX_WHITE);
  campoLCD(2,4,61,2,26,"Luego inicia tensiometro",ST77XX_WHITE);
  campoLCD(3,4,89,2,26,"Manguito en OTRO brazo",ST77XX_YELLOW);
 } else if(guia==3){
  campoLCD(1,4,33,2,26,"3/4 MEDICION EN CURSO",ST77XX_WHITE);
  campoLCD(2,4,61,2,26,"Manten dedo quieto",ST77XX_WHITE);
  campoLCD(3,4,89,2,26,"Al terminar: envia T",ST77XX_YELLOW);
 } else if(guia==4){
  campoLCD(1,4,33,2,26,"4/4 INGRESA PA EN SERIE",ST77XX_YELLOW);
  campoLCD(2,4,61,2,26,"C sistolica diastolica",ST77XX_WHITE);
  campoLCD(3,4,89,2,26,"Ej: C 120 60 + Enter",ST77XX_WHITE);
 } else {
  campoLCD(1,4,33,2,26,guia==7?"Referencia NO util":guia==5?"Referencia guardada":modeloPA?"Modelo ajustado*":"Ajuste insuficiente",ST77XX_YELLOW);
  campoLCD(2,4,61,2,26,"K: otra referencia",ST77XX_WHITE);
  campoLCD(3,4,89,2,26,modeloPA?"G: prueba independiente":guia==7?"Q: salir del asistente":"F: ajustar (min 8)",ST77XX_WHITE);
 }
 if(guia==3)snprintf(b,sizeof(b),"Ventanas guardadas: %u",nVentanas13);else snprintf(b,sizeof(b),"Referencias: %u/8 minimo",calN);
 campoLCD(4,4,117,2,26,b,ST77XX_CYAN);
 campoLCD(5,4,145,2,26,*guiaAviso?guiaAviso:"Q cancela | E exporta",ST77XX_YELLOW);
}

void reiniciarExploratorio() {
 oxPos=oxN=oxDiv=0;oxSI=oxSR=0;paPos=paN=0;exConsecutivas=0;
 exInit=false;exHP1=exHP2=exLP1=exLP2=0;
 oxValida=rasgoValido=paValida=false;oxValor=w50=paSis=paDia=NAN;
 oxCorr=oxPI=w50CV=periodoPA=NAN;paPulsos=0;
 oxMotivo="ESPERANDO";paMotivo=modeloPA?"ESPERANDO":"SIN CALIBRAR";
 exGeneracion++;exUltOx=exUltPA=millis();
}
bool condicionesExploratorias() {
 return datosFrescos() && contactoCrudo && lecturaOK && nivel==0 && giro<20;
}
bool oxVisible() {return condicionesExploratorias() && oxValida && millis()-oxFecha<2500;}
bool paVisible() {return condicionesExploratorias() && paValida && modeloPA && millis()-paFecha<6500;}
void muestraExploratoria() {
 if(fase==2 && !pausaEstadisticas) {
  if(crudos10 && rawN<RAW_CAP)crudos10[rawN++]={(uint32_t)(millis()-inicio),ir,rojo};else rawOmitidas++;
 }
 // Saturacion, falta de contacto o movimiento invalidan TODA la ventana auxiliar.
 // No reinician el Butterworth ni el detector de frecuencia cardiaca.
 if(!fase || ir<8000 || rojo<1000 || ir>=260000 || rojo>=260000 || !lecturaOK || nivel!=0 || giro>=20) {
  if(exConsecutivas || exInit)reiniciarExploratorio();return;
 }
 if(!exInit) {
  // Estado estacionario del pasaaltos para evitar escalon de contacto.
  exHP2=0.98675978f*ir;exHP1=-1.97351956f*ir+exHP2;exInit=true;
 }
 float h=biquadDF2T((float)ir,0.98675978f,-1.97351956f,0.98675978f,-1.97334425f,0.97369487f,exHP1,exHP2);
 float y=biquadDF2T(h,0.04613180f,0.09226360f,0.04613180f,-1.30728503f,0.49181224f,exLP1,exLP2);
 exConsecutivas++;
 if(exConsecutivas<=300)return; // asentamiento auxiliar 3 s, independiente de V9.
 ondaPA[paPos]=y;paPos=(paPos+1)%EXP_N;if(paN<EXP_N)paN++;
 oxSI+=ir;oxSR+=rojo;
 if(++oxDiv==4) {
  oxIR[oxPos]=oxSI/4;oxRed[oxPos]=oxSR/4;oxPos=(oxPos+1)%100;
  if(oxN<100)oxN++;oxDiv=0;oxSI=oxSR=0;
 }
}
// Calidad auxiliar: RMS tras retirar una recta de deriva, correlacion rojo/IR.
// Cortes HEURISTICOS para esta prueba; no representan validacion clinica.
void calcularOx() {
 if(oxN<100)return;
 double mi=0,mr=0,ti=0,tr=0;uint32_t loI=UINT32_MAX,hiI=0,loR=UINT32_MAX,hiR=0;
 for(int j=0;j<100;j++) {
  int k=(oxPos+j)%100;oxI[j]=oxIR[k];oxR[j]=oxRed[k];
  mi+=oxI[j];mr+=oxR[j];ti+=(j-49.5)*oxI[j];tr+=(j-49.5)*oxR[j];
  loI=min(loI,oxI[j]);hiI=max(hiI,oxI[j]);loR=min(loR,oxR[j]);hiR=max(hiR,oxR[j]);
 }
 mi/=100;mr/=100;ti/=83325.0;tr/=83325.0;
 double vi=0,vr=0,co=0;
 for(int j=0;j<100;j++){double u=oxI[j]-mi-ti*(j-49.5),v=oxR[j]-mr-tr*(j-49.5);vi+=u*u;vr+=v*v;co+=u*v;}
 oxPI=mi>0?100*sqrt(vi/100)/mi:0;
 oxCorr=vi>0 && vr>0?co/sqrt(vi*vr):0;
 oxValida=false;oxMotivo="CALIDAD";
 // Protege tambien las multiplicaciones int32 de la implementacion original.
 bool rangoSeguro=(uint64_t)(hiI-loI)*hiR<1500000000ULL && (uint64_t)(hiR-loR)*hiI<1500000000ULL;
 if(!rangoSeguro || oxPI<0.02f || oxPI>5.0f || oxCorr<0.80f || vr<=0){oxRechazadas++;return;}
 int32_t s=-999,hr=-999;int8_t vs=0,vhr=0;
 maxim_heart_rate_and_oxygen_saturation(oxI,100,oxR,&s,&vs,&hr,&vhr);
 // El hr auxiliar se descarta: el UNICO BPM mostrado proviene de la V9.
 if(!vs || s<70 || s>100){oxMotivo="FUERA DE RANGO";oxRechazadas++;return;}
 oxValor=s;oxValida=true;oxFecha=millis();oxMotivo="EXPLORATORIA";
 if(fase==2 && !pausaEstadisticas){oxAceptadas++;oxSuma+=s;oxMin=min(oxMin,(float)s);oxMax=max(oxMax,(float)s);}
}
// Extrae W50 entre valles alrededor de picos principales del pulso.
// Banda auxiliar 0.3-8 Hz causal: adaptacion propia, no replica de Teng/Haddad.
void calcularRasgoPA() {
 rasgoValido=paValida=false;
 if(paN<EXP_N){paMotivo="VENTANA INCOMPLETA";return;}
 if(!bpm8.valido){paMotivo="SIN BPM VALIDADO";return;}
 if(millis()-bpm8.fecha>=5000){paMotivo="BPM SIN ACTUALIZAR";return;}
 memcpy(copiaPA,ondaPA+paPos,(EXP_N-paPos)*sizeof(float));
 if(paPos)memcpy(copiaPA+EXP_N-paPos,ondaPA,paPos*sizeof(float));
 float lo=copiaPA[0],hi=lo;
 for(int i=1;i<EXP_N;i++){lo=min(lo,copiaPA[i]);hi=max(hi,copiaPA[i]);}
 if(!isfinite(lo) || !isfinite(hi) || hi-lo<30){paMotivo="ONDA DEBIL";return;}
 int peaks[40],np=0;int distancia=(int)(0.60f*6000.0f/bpm8.ultimo);
 distancia=max(20,distancia);
 for(int i=1;i<EXP_N-1;i++)if(copiaPA[i]>copiaPA[i-1] && copiaPA[i]>=copiaPA[i+1] && copiaPA[i]>lo+0.45f*(hi-lo)) {
  if(np && i-peaks[np-1]<distancia){if(copiaPA[i]>copiaPA[peaks[np-1]])peaks[np-1]=i;}
  else if(np<40)peaks[np++]=i;
 }
 double sw=0,sw2=0,sp=0;int n=0;
 for(int k=1;k<np-1;k++) {
  int p=peaks[k],l=peaks[k-1],r=p;
  for(int i=peaks[k-1];i<p;i++)if(copiaPA[i]<copiaPA[l])l=i;
  for(int i=p;i<peaks[k+1];i++)if(copiaPA[i]<copiaPA[r])r=i;
  int periodo=r-l;if(periodo<33 || periodo>172 || p<=l || r<=p)continue;
  float base=max(copiaPA[l],copiaPA[r]),amp=copiaPA[p]-base;
  if(amp<20 || fabsf(copiaPA[l]-copiaPA[r])>0.5f*amp)continue;
  float th=base+amp*0.5f;int a=p,b=p;
  while(a>l && copiaPA[a]>th)a--;
  while(b<r && copiaPA[b]>th)b++;
  if(a>=p || b<=p || copiaPA[a+1]==copiaPA[a] || copiaPA[b-1]==copiaPA[b])continue;
  float asc=a+(th-copiaPA[a])/(copiaPA[a+1]-copiaPA[a]);
  float des=b-1+(copiaPA[b-1]-th)/(copiaPA[b-1]-copiaPA[b]);
  float w=10*(des-asc);
  if(w<50 || w>0.8f*periodo*10)continue;
  sw+=w;sw2+=w*w;sp+=periodo*10;n++;
 }
 if(n<4){paMotivo="POCOS PULSOS";return;}
 float media=sw/n,cv=sqrt(max(0.0,sw2/n-(sw/n)*(sw/n)))/media;
 if(!isfinite(cv) || cv>0.20f){paMotivo="ONDA VARIABLE";return;}
 w50=media;w50CV=cv;periodoPA=sp/n;paPulsos=n;rasgoValido=true;rasgoFecha=millis();
 if(!modeloPA){paMotivo="SIN CALIBRAR";return;}
 if(w50<calXmin || w50>calXmax){paMotivo="FUERA CALIBRACION";paRechazadas++;return;}
 float s=aSis+bSis*w50,d=aDia+bDia*w50;
 if(!isfinite(s) || !isfinite(d) || s<60 || s>240 || d<30 || d>150 || s-d<15 || s-d>120){paMotivo="MODELO INVALIDO";paRechazadas++;return;}
 paSis=s;paDia=d;paValida=true;paFecha=millis();paMotivo="EST. PERSONAL";
 if(fase==2 && !pausaEstadisticas)paAceptadas++;
}
void actualizarExploratorio() {
 if(!fase || !condicionesExploratorias()) {
  if(exConsecutivas || exInit)reiniciarExploratorio();return;
 }
 uint32_t us=micros(),ahora=millis();
 if(ahora-exUltOx>=1000){exUltOx=ahora;calcularOx();}
 adquirir();
 if(ahora-exUltPA>=5000){exUltPA=ahora;calcularRasgoPA();registrarVentana13();}
 exMaxUs=max(exMaxUs,(uint32_t)(micros()-us));
}
void informeExploratorio() {
 Serial.printf("EXP10: SpO2=%.1f estado=%s corr=%.3f PI_RMS=%.3f%% PA_S=%.1f PA_D=%.1f estadoPA=%s W50_ms=%.2f CV=%.3f pulsos=%u calibraciones=%u modelo=%d\n",
 oxVisible()?oxValor:NAN,oxVisible()?"EXPLORATORIA":oxMotivo,oxCorr,oxPI,
 paVisible()?paSis:NAN,paVisible()?paDia:NAN,paMotivo,rasgoValido?w50:NAN,w50CV,paPulsos,calN,modeloPA);adquirir();
}
void resumenExploratorio() {
 Serial.printf("RESUMEN_EXP10: SpO2_n=%lu media=%.2f min=%.1f max=%.1f rechazadas=%lu PA_n=%lu PA_rechazadas=%lu tiempo_aux_max_us=%lu RAW=%u omitidas=%lu\n",
 (unsigned long)oxAceptadas,oxAceptadas?oxSuma/oxAceptadas:NAN,oxAceptadas?oxMin:NAN,oxAceptadas?oxMax:NAN,
 (unsigned long)oxRechazadas,(unsigned long)paAceptadas,(unsigned long)paRechazadas,(unsigned long)exMaxUs,rawN,(unsigned long)rawOmitidas);
 adquirir();Serial.println("SpO2/PA son exploratorias; n cuenta ventanas solapadas, NO observaciones independientes.");adquirir();
}
// Minimos de identificabilidad de una recta; NO son criterios clinicos de exactitud.
void ajustarPA() {
 modeloPA=false;paValida=false;
 if(calN<8){Serial.println("PA: hacen falta al menos 8 pares no solapados C; no hay modelo aun.");return;}
 double mx=0,ms=0,md=0;float xmin=cal10[0].x,xmax=xmin,smin=cal10[0].s,smax=smin,dmin=cal10[0].d,dmax=dmin;
 for(int i=0;i<calN;i++){mx+=cal10[i].x;ms+=cal10[i].s;md+=cal10[i].d;xmin=min(xmin,cal10[i].x);xmax=max(xmax,cal10[i].x);smin=min(smin,cal10[i].s);smax=max(smax,cal10[i].s);dmin=min(dmin,cal10[i].d);dmax=max(dmax,cal10[i].d);}
 mx/=calN;ms/=calN;md/=calN;double xx=0,xs=0,xd=0,ss=0,dd=0;
 for(int i=0;i<calN;i++){double x=cal10[i].x-mx,s=cal10[i].s-ms,d=cal10[i].d-md;xx+=x*x;xs+=x*s;xd+=x*d;ss+=s*s;dd+=d*d;}
 if(xmax-xmin<30 || smax-smin<10 || dmax-dmin<5 || xx<=0 || ss<=0 || dd<=0){Serial.println("PA: variacion insuficiente; mantener SIN CALIBRAR. No provocar cambios de presion para ajustar.");return;}
 r2Sis=xs*xs/(xx*ss);r2Dia=xd*xd/(xx*dd);
 if(r2Sis<0.50f || r2Dia<0.50f){Serial.printf("PA: W50 no explica las referencias; R2 entrenamiento=%.3f/%.3f. SIN MODELO.\n",r2Sis,r2Dia);return;}
 bSis=xs/xx;bDia=xd/xx;aSis=ms-bSis*mx;aDia=md-bDia*mx;
 calXmin=xmin;calXmax=xmax;modeloPA=true;paMotivo="ESPERANDO";
 Serial.printf("MODELO_PA_CONGELADO: S=%.8f%+.8f*W50_ms D=%.8f%+.8f*W50_ms dominio=[%.2f,%.2f] R2_ENTRENAMIENTO=%.3f/%.3f\n",aSis,bSis,aDia,bDia,calXmin,calXmax,r2Sis,r2Dia);
 Serial.println("Ahora G para prueba independiente SIN C. El ajuste no demuestra exactitud. X antes de otra persona/dedo/montaje.");
}
void comandoExploratorio(const char *linea) {
 float s,d;char extra;
 if(sscanf(linea,"%*c %f %f %c",&s,&d,&extra)!=2 || !isfinite(s) || !isfinite(d) || s<60 || s>240 || d<30 || d>150 || s-d<15 || s-d>120){Serial.println("Formato: C 120 80 + Enter. Son referencias REALES del tensiometro, no valores deseados.");guiaAviso="Formato: C 120 60";return;}
 if(modeloPA){Serial.println("Modelo congelado: C no modifica la validacion. X borra modelo y pares.");return;}
 if(guia!=4){Serial.println("Usa K para calibracion guiada; ingresa C solo cuando LCD lo indique.");return;}
 if(!capturaUtil){
 Serial.printf("REFERENCIA_NO_UTIL: SIS=%.1f DIA=%.1f motivo=%s. NO se agrega al modelo.\n",s,d,motivoCaptura);
 guia=7;fase=0;pruebaTerminada=true;guiaAviso=motivoCaptura;ultimoLCD=0;return;
 }
 if(calN>=24){Serial.println("24 referencias: F ajusta o X borra.");return;}
 // Instantanea al pulsar T: escribir despacio NO cambia la ventana asociada.
 cal10[calN++]={guiaW50,s,d,ensayo,guiaRasgoFecha};ultimaCalFecha=millis();huboCal=true;
 Serial.printf("CAL_PA: n=%u ensayo=%lu fin_ventana_ms=%lu W50=%.3f SIS=%.1f DIA=%.1f\n",calN,(unsigned long)ensayo,(unsigned long)guiaRasgoFecha,guiaW50,s,d);
 Serial.println("GUARDADO. Podes retirar dedo. K otra referencia; F ajustar; E exportar. Reset pierde referencias.");
 fase=0;pruebaTerminada=true;guia=5;guiaAviso="Podes retirar el dedo";

}
void exportarCrudo() {
 if(!enviandoCrudo)return;
 if(!crudos10){Serial.println("CRUDO100 no disponible: buffer PSRAM no asignado.");enviandoCrudo=false;return;}
 if(rawEnvio>=rawN){Serial.println("FIN_CRUDO100");enviandoCrudo=false;return;}
 const Crudo10 &r=crudos10[rawEnvio];char buf[70];
 int n=snprintf(buf,sizeof(buf),"%u,%lu,%lu,%lu\n",rawEnvio,(unsigned long)r.ms,(unsigned long)r.ir,(unsigned long)r.rojo);
 if(n>0 && Serial.availableForWrite()>=n){Serial.write((uint8_t*)buf,n);rawEnvio++;}
}
// G/D siguen siendo inmediatos. Solo C utiliza una linea terminada en Enter.
bool serialExploratorio() {
 static char linea[48];static uint8_t pos=0;static bool capturando=false,exceso=false;
 if(!Serial.available())return false;
 char c=Serial.peek();
 if(!capturando && (c=='K'||c=='k')) {
  Serial.read();
  if(fase || enviandoCrudo || enviandoTraza)Serial.println("Termina prueba/exportacion o Q cancela referencia.");
  else if(modeloPA)Serial.println("Modelo congelado. G valida; X borra para recalibrar.");
  else if(calN>=24)Serial.println("Limite 24 referencias. F ajusta o X borra.");
  else if(!listo)Serial.println("ERROR de sensores: reinicia y copia registro.");
  else {pedirK=true;guia=1;guiaAviso="";capturaUtil=false;Serial.println("PASO 1/4: Apoya el dedo y espera. La LCD indicara cuando enviar M.");}
  return true;
 }
 if(!capturando && (c=='Q'||c=='q')){Serial.read();if(guia)cancelarGuia();return true;}
 if(!capturando && (c=='M'||c=='m')){
  Serial.read();
  if(guia==3){Serial.println("M ya recibido. Sigue midiendo; al terminar envia T.");return true;}
  if(guia!=2){Serial.println("M fuera de paso: segui la indicacion de LCD.");return true;}
  limpiarVentanas13();guia=3;guiaInicio=millis();guiaPerdidas=perdidas;guiaAviso="";ultimoLCD=0;
  Serial.println("M ACEPTADA. PASO 3/4: Inicia tensiometro AHORA. Cuando termine envia T.");return true;
 }
 if(!capturando && (c=='T'||c=='t')){
  Serial.read();
  if(guia==4){Serial.println("T ya recibida. Escribe C sistolica diastolica + Enter.");return true;}
  if(guia!=3){Serial.println("T fuera de paso: segui la indicacion de LCD.");return true;}
  finalizarManguito();return true;
 }

 if(capturando){Serial.read();if(c=='\n' || c=='\r'){linea[pos]=0;if(!exceso)comandoExploratorio(linea);else Serial.println("Comando C demasiado largo.");pos=0;capturando=false;exceso=false;}else if(pos<sizeof(linea)-1)linea[pos++]=c;else exceso=true;return true;}
 if(c=='C' || c=='c'){Serial.read();linea[0]='C';pos=1;capturando=true;return true;}
 if(c=='R' || c=='r'){Serial.read();if(!crudos10)Serial.println("R no disponible: buffer PSRAM no asignado.");else if(!fase && !enviandoTraza && !enviandoCrudo && rawN){rawEnvio=0;enviandoCrudo=true;Serial.println("INICIO_CRUDO100: indice,ms_llegada,IR,ROJO");}return true;}
 if(c=='F' || c=='f'){Serial.read();if(!fase && !enviandoTraza && !enviandoCrudo){ajustarPA();guia=6;guiaAviso=modeloPA?"*Experimental sin validar":"Ver motivo en Serie";}else Serial.println("F: espera FIN y fin de exportacion.");return true;}
 if(c=='X' || c=='x'){Serial.read();if(!fase && !enviandoTraza && !enviandoCrudo){guia=0;modeloPA=paValida=false;calN=0;huboCal=false;paMotivo="SIN CALIBRAR";Serial.println("Calibracion PA borrada. Sin coeficientes de otra persona.");}return true;}
 if(c=='E' || c=='e'){Serial.read();if(!fase && !enviandoTraza && !enviandoCrudo){Serial.println("CAL: ensayo,fin_ventana_ms,W50_ms,SIS_ref,DIA_ref");for(int i=0;i<calN;i++)Serial.printf("%lu,%lu,%.3f,%.1f,%.1f\n",(unsigned long)cal10[i].ensayo,(unsigned long)cal10[i].ms,cal10[i].x,cal10[i].s,cal10[i].d);}return true;}
 return false;
}

// V15: estimacion de PA en PC con pesos publicados; nunca sustituye BPM/SpO2.
const uint16_t BP15_N=700;
uint32_t bpRing[BP15_N],bpTimes[BP15_N],bpTx[BP15_N];
uint16_t bpPos=0,bpN=0,bpTxPos=0;uint8_t bpTxState=0;
uint32_t bpSeq=0,bpPending=0,bpCapture=0,bpResultCapture=0,bpSent=0,bpWarm=0,bpHello=0,bpLast=0;
uint32_t bpBleSeq=0,bpBleLast=0; // B19: secuencia/cadencia del transporte binario Android.
bool bpPC=false,bpValid=false;float bpS=NAN,bpD=NAN;
void resetPA15(){bpN=bpPos=0;bpWarm=0;bpValid=false;bpPending=0;bpTxState=0;}
bool visiblePA15(){return fase && bpValid && condicionesExploratorias() && millis()-bpResultCapture<15000;}
void samplePA15(){
 if(!fase || !condicionesExploratorias() || ir<8000 || ir>=260000 || rojo>=260000){resetPA15();return;}
 uint32_t t=millis();if(!bpWarm){bpWarm=t;return;}if(t-bpWarm<2000)return;
 bpRing[bpPos]=ir;bpTimes[bpPos]=t;bpPos=(bpPos+1)%BP15_N;if(bpN<BP15_N)bpN++;
}
void receivePA15(){
 static char line[96];static uint8_t n=0;static bool overflow=false;
 for(int k=0;k<24 && Serial.available();k++){
  char c=Serial.read();
  if(c=='\r'||c=='\n'){
   line[n]=0;
   if(!overflow && n){
    if(!strcmp(line,"H")){bpPC=true;bpHello=millis();}
    else if(!strcmp(line,"Q")){fase=0;resetPA15();}
    else if(!strcmp(line,"G")){if(!fase)iniciarContinuo14();}
    else if(line[0]=='P'){
     unsigned long seq;float a,b;char extra;
     if(sscanf(line,"P %lu %f %f %c",&seq,&a,&b,&extra)==3 && bpPending && seq==bpPending && millis()-bpCapture<15000){
      // Descartar resultados imposibles; no recortar a numeros plausibles.
      bpValid=isfinite(a)&&isfinite(b)&&a>=60&&a<=240&&b>=30&&b<=150&&a>b&&condicionesExploratorias();
      if(bpValid)bpResultCapture=bpCapture;bpS=bpValid?a:NAN;bpD=bpValid?b:NAN;bpPending=0;bpLast=millis();
      Serial.printf("PA15_RESULT: seq=%lu SBP=%.2f DBP=%.2f aceptada=%d origen=LSTM_publicada_NO_validada\n",seq,a,b,bpValid);
     }
    }else if(line[0]=='N'){unsigned long seq;if(sscanf(line,"N %lu",&seq)==1 && bpPending && seq==bpPending){bpPending=0;bpValid=false;bpLast=millis();Serial.println("PA15: PC rechazo ventana.");}}
   }
   n=0;overflow=false;
  } else if(n<sizeof(line)-1)line[n++]=c;else overflow=true;
 }
}
void tickPA15(){
 uint32_t t=millis();
 if(bpPC && t-bpHello>30000)bpPC=false;
 if(!bpPC || !fase)return;
 if(bpPending && t-bpCapture>15000){bpPending=0;bpTxState=0;bpValid=false;}
 if(!bpPending && !bpTxState && bpN==BP15_N && t-bpLast>=3000){
  uint32_t span=bpTimes[(bpPos+BP15_N-1)%BP15_N]-bpTimes[bpPos];
  if(span<6500 || span>7500){resetPA15();return;}
  for(int i=0;i<BP15_N;i++)bpTx[i]=bpRing[(bpPos+i)%BP15_N];
  bpPending=++bpSeq;bpCapture=t;bpTxState=1;bpTxPos=0;
 }
 if(!bpTxState)return;
 char out[64];int len=0;
 if(bpTxState==1)len=snprintf(out,sizeof(out),"BP15,B,%lu,700,100\n",(unsigned long)bpPending);
 else if(bpTxState==2)len=snprintf(out,sizeof(out),"BP15,D,%lu,%u,%lu,%lu\n",(unsigned long)bpPending,bpTxPos,(unsigned long)bpTx[bpTxPos],(unsigned long)bpTx[bpTxPos+1]);
 else len=snprintf(out,sizeof(out),"BP15,E,%lu\n",(unsigned long)bpPending);
 if(len>0 && Serial.availableForWrite()>=len){
  Serial.write((uint8_t*)out,len);
  if(bpTxState==1)bpTxState=2;
  else if(bpTxState==2){bpTxPos+=2;if(bpTxPos>=BP15_N)bpTxState=3;}
  else {bpTxState=0;bpSent=t;}
 }
}

void adquirir() { static unsigned long consulta=0;
 if(!listo || !maxOK)return;
 unsigned long t=millis(),pausa=t-consulta;
 if(pausa<2)return;
 if(fase && !pausaEstadisticas)maxPausaMs=max(maxPausaMs,(uint32_t)pausa);
 consulta=t;
 uint16_t recibidas=sensor.check(),disponibles=sensor.available();
 if(recibidas>disponibles) {
   if(fase && !pausaEstadisticas){perdidas+=recibidas-disponibles;reiniciosDatos++;}
   resetPA15();reiniciarDetector(); reiniciarExploratorio(); reiniciarPRV17(false);prv17ReiniciosVistos=reiniciosDatos; // Hueco: invalidar tambien ventanas exploratorias/PRV.
 }
 while(sensor.available()) {
   ir=sensor.getFIFOIR();rojo=sensor.getFIFORed();sensor.nextSample();
   datosVistos=true;ultimoDato=millis();alarmaDatos=false;
   unsigned long anterior=ultimoLatidoRegistrado;procesarPPG();muestraPRV17();observarMuestra(anterior);muestraExploratoria();samplePA15();
   if(fase && !pausaEstadisticas){nPPG++;sumaIR+=ir;minIR=min(minIR,ir);maxIR=max(maxIR,ir);if(contactoCrudo)conContacto++;if(!ir || !rojo)ceros++;}
 }
 if(!datosFrescos() && !alarmaDatos){alarmaDatos=true;if(fase && !pausaEstadisticas)sinDatosEventos++;}
 if(!datosFrescos() && contacto){if(fase && !pausaEstadisticas)reiniciosDatos++;reiniciarDetector();reiniciarExploratorio();reiniciarPRV17(false);prv17ReiniciosVistos=reiniciosDatos;}
} unsigned long restantes() {
  unsigned long p=(fase==1 ? (contactoDesde ? millis()-contactoDesde : 0) : millis()-inicio),d=duraciones[fase];
  return p>=d ? 0 : (d-p+999)/1000;
} void resumen() {
  Registro copiaTotal=total;float t=(millis()-inicio)/1000.0f;adquirir();
  Serial.printf("\n=== RESUMEN %s (%.2f s) ===\n",fases[fase],t);adquirir();
  Serial.printf("PPG: muestras=%lu Fs=%.1f Hz descartadas_buffer=%lu ceros=%lu\n",
    (unsigned long)nPPG,nPPG/t,(unsigned long)perdidas,(unsigned long)ceros);adquirir();
  Serial.printf("Pausa_max_servicio=%lu ms Reinicios_datos=%lu Primer_BPM_desde_contacto=%lu ms\n",(unsigned long)maxPausaMs,(unsigned long)reiniciosDatos,primerBPMms);adquirir();
  if(nPPG)Serial.printf("IR min=%lu promedio=%.1f max=%lu Contacto=%.1f%%\n",
    (unsigned long)minIR,(double)sumaIR/nPPG,(unsigned long)maxIR,100.0f*conContacto/nPPG);adquirir();
  if(minFiltro<1e9)Serial.printf("Butterworth min=%.2f max=%.2f\n",minFiltro,maxFiltro);adquirir();
  Serial.printf("Intervalos aceptados=%lu Estimaciones BPM=%lu\n",(unsigned long)latidos,(unsigned long)nBPM);adquirir();
  if(nBPM)Serial.printf("BPM estimado: min=%.1f promedio=%.1f max=%.1f\n",minBPM,sumaBPM/nBPM,maxBPM);
  else Serial.println("BPM: sin estimacion suficiente.");adquirir();
  Serial.printf("IMU validas=%lu Fs=%.1f Hz errores=%lu limite=%lu\n",
    (unsigned long)validas,validas/t,(unsigned long)errores,(unsigned long)saturadas);adquirir();
  if(validas)Serial.printf("Dinamica media=%.4f g Reposo=%.1f%% Leve=%.1f%% Moderado=%.1f%%\n",
    sumaD/validas,100.0f*conteos[0]/validas,100.0f*conteos[1]/validas,100.0f*conteos[2]/validas);adquirir();
 imprimirRegistro(copiaTotal);informarEstado8();adquirir();
 if(fase==2 && n9) {
  Serial.printf("RESUMEN_V9: visible=%.1f%% reciente_menor5s=%.1f%% retenido_10s_o_mas=%.1f%% edad_max_ms=%lu primer_visible_fase_ms=%ld\n",
   100.0f*visible9/n9,100.0f*reciente9/n9,100.0f*antiguo9/n9,(unsigned long)maxEdad9,primero9?(long)(primero9-1):-1L);adquirir();
 }
 Serial.printf("Traza: filas=%u omitidas=%lu\n",nTraza,(unsigned long)trazaOmitida);adquirir();
 Serial.printf("Acumulado ensayo: recuperaciones_MAX=%lu rearmes_detector=%lu rechazos(rango/cercano/cambio)=%lu/%lu/%lu\n",
 (unsigned long)recuperacionesBus,(unsigned long)rearmes,(unsigned long)rechazoRango,(unsigned long)rechazoCercano,(unsigned long)rechazoCambio);adquirir();
 Serial.printf("Sin_datos_eventos=%lu\n",(unsigned long)sinDatosEventos);adquirir();
}
// ===== ETAPA 1 UI v6 FINAL: 172 x 320 portrait =====
// Distribucion: 40% FC (0..127), 40% movimiento (128..255), 20% conectividad/bateria (256..319).
// La UI NO modifica filtro, detector, umbrales, promedios ni reglas V9.
// Para no perturbar la adquisicion a 100 Hz, el estado principal se revisa a 5 Hz y
// la animacion solo redibuja cuando cambia de fase; entre primitivas se atiende el FIFO.
static inline uint16_t uiGris(){return lcd.color565(185,185,195);}
static inline uint16_t uiLinea(){return lcd.color565(62,64,72);}
static inline uint16_t uiRojo(){return lcd.color565(255,18,62);}
static inline uint16_t uiRosaClaro(){return lcd.color565(255,105,140);}
static inline uint16_t uiMagenta(){return lcd.color565(247,0,105);}
static inline uint16_t uiAzulBT(){return lcd.color565(0,145,255);}
static inline uint16_t uiNaranja(){return lcd.color565(255,112,0);}
static inline uint16_t uiAmarillo(){return lcd.color565(255,205,0);}
static inline uint16_t uiVerde(){return lcd.color565(0,205,55);}

const uint32_t UI_ESTADO_MS=200;          // 5 Hz: suficiente para estados, reduce carga grafica.
const uint32_t UI_ANIM_TICK_MS=40;        // solo comprueba tiempos; no redibuja cada 40 ms.
const uint32_t UI_PULSO_PERIODO_MS=850;   // animacion decorativa de presencia, NO latido medido.
const uint32_t UI_PULSO_GRANDE_MS=125;
const uint32_t UI_PUNTOS_MS=380;
unsigned long ultimoAnimUI=0;

void textoCentroCajaUI(const char *txt,int x,int y,int w,int h,const GFXfont *fuente,uint16_t color){
 lcd.setTextSize(1);                       // evita heredar setTextSize(2) del arranque
 lcd.setFont(fuente);lcd.setTextColor(color);
 int16_t x1,y1;uint16_t tw,th;lcd.getTextBounds(txt,0,0,&x1,&y1,&tw,&th);
 int cx=x+(w-(int)tw)/2-x1;int cy=y+(h-(int)th)/2-y1;
 lcd.setCursor(cx,cy);lcd.print(txt);lcd.setFont(NULL);adquirir();
}

void lineaGruesaUI(int x0,int y0,int x1,int y1,uint8_t grosor,uint16_t color){
 int dx=x1-x0,dy=y1-y0;float L=sqrtf((float)dx*dx+(float)dy*dy);
 if(L<1){lcd.fillCircle(x0,y0,max(1,(int)grosor/2),color);return;}
 float nx=-dy/L,ny=dx/L;int r=max(1,(int)grosor/2);
 for(int k=-r;k<=r;k++)lcd.drawLine(x0+(int)lroundf(nx*k),y0+(int)lroundf(ny*k),x1+(int)lroundf(nx*k),y1+(int)lroundf(ny*k),color);
 lcd.fillCircle(x0,y0,r,color);lcd.fillCircle(x1,y1,r,color);
}

// Corazon v6: silueta continua calculada con la ecuacion implicita clasica
// (x^2+y^2-1)^3 - x^2*y^3 <= 0. A diferencia de circulos + triangulo,
// el lateral es una curva continua: no aparecen los dos vertices/picos que se veian antes.
// Escala X/Y practicamente 1:1: el corazon normal ocupa aprox. 71 x 70 px.
//
// Para evitar el parpadeo NO se borra el rectangulo completo del corazon. Se precalculan
// hasta dos tramos horizontales por fila (necesarios para conservar la hendidura superior)
// para el tamano normal y el tamano grande. Durante la animacion solo se pintan/borran
// los pequenos tramos que cambian entre ambos estados. No hay ningun frame negro.
// Estas tablas ocupan menos de 2 kB; no son framebuffer ni sprite.
const int HEART_X0=3,HEART_X1=81;
const int HEART_Y0=8,HEART_Y1=112,HEART_H=HEART_Y1-HEART_Y0+1;
// V5.1: usamos una matriz simple [l1,r1,l2,r2] en lugar de un struct.
// Esto evita un problema del generador automatico de prototipos del Arduino IDE,
// que intentaba declarar funciones con HeartRowUI antes de conocer el tipo.
int16_t heartRows[2][HEART_H][4];
bool heartMaskReady=false;
int8_t corazonEstadoDibujadoUI=-1;       // -1 desconocido, 0 normal, 1 grande

bool puntoDentroCorazonUI(float xn,float yn){
 float x2=xn*xn,y2=yn*yn;
 float q=x2+y2-1.0f;
 return (q*q*q - x2*yn*y2)<=0.0f;
}

void prepararCorazonUI(){
 if(heartMaskReady)return;
 const float sx[2]={31.5f,33.5f};
 const float sy[2]={31.0f,33.0f};
 const float cx=42.0f,cy=59.0f;
 for(int m=0;m<2;m++){
  for(int iy=0;iy<HEART_H;iy++){
   int16_t row[4]={-1,-1,-1,-1};
   int run=0,inicio=-1;
   const int py=HEART_Y0+iy;
   const float yn=-(py-cy)/sy[m];
   for(int px=HEART_X0;px<=HEART_X1+1;px++){
    bool dentro=false;
    if(px<=HEART_X1){
     float xn=(px-cx)/sx[m];
     dentro=puntoDentroCorazonUI(xn,yn);
    }
    if(dentro && inicio<0)inicio=px;
    if(!dentro && inicio>=0){
     int fin=px-1;
     if(run==0){row[0]=inicio;row[1]=fin;}
     else if(run==1){row[2]=inicio;row[3]=fin;}
     run++;inicio=-1;
    }
   }
   for(int k=0;k<4;k++)heartRows[m][iy][k]=row[k];
  }
 }
 heartMaskReady=true;
}

bool pixelEnFilaCorazonUI(const int16_t r[4],int x){
 return (r[0]>=0 && x>=r[0] && x<=r[1]) || (r[2]>=0 && x>=r[2] && x<=r[3]);
}

void dibujarFilaCorazonCompletaUI(const int16_t r[4],int y,uint16_t color){
 if(r[0]>=0)lcd.drawFastHLine(r[0],y,r[1]-r[0]+1,color);
 if(r[2]>=0)lcd.drawFastHLine(r[2],y,r[3]-r[2]+1,color);
}

void dibujarReflejoCorazonUI(){
 // El reflejo permanece fijo durante el pulso: asi nunca hay un flash negro dentro del corazon.
 lcd.fillCircle(27,30,6,uiRosaClaro());
 lcd.fillCircle(24,27,2,lcd.color565(255,180,195));adquirir();
}

void dibujarCorazonEstadoUI(bool grande){
 prepararCorazonUI();
 const int nuevo=grande?1:0;
 const uint16_t rojo=uiRojo();
 if(corazonEstadoDibujadoUI<0){
  for(int iy=0;iy<HEART_H;iy++){
   dibujarFilaCorazonCompletaUI(heartRows[nuevo][iy],HEART_Y0+iy,rojo);
   if((iy&15)==15)adquirir();
  }
  dibujarReflejoCorazonUI();
  corazonEstadoDibujadoUI=nuevo;return;
 }
 if(corazonEstadoDibujadoUI==nuevo)return;
 const int viejo=corazonEstadoDibujadoUI;
 // Redibujado diferencial por corridas: solo cambia el borde entre normal/grande.
 for(int iy=0;iy<HEART_H;iy++){
  const int16_t *ro=heartRows[viejo][iy];
  const int16_t *rn=heartRows[nuevo][iy];
  const int py=HEART_Y0+iy;
  int x=HEART_X0;
  while(x<=HEART_X1){
   bool oldIn=pixelEnFilaCorazonUI(ro,x);
   bool newIn=pixelEnFilaCorazonUI(rn,x);
   if(oldIn==newIn){x++;continue;}
   const bool pintar=newIn;
   int x0=x;
   do{
    x++;
    if(x>HEART_X1)break;
    oldIn=pixelEnFilaCorazonUI(ro,x);
    newIn=pixelEnFilaCorazonUI(rn,x);
   }while(oldIn!=newIn && newIn==pintar);
   lcd.drawFastHLine(x0,py,x-x0,pintar?rojo:ST77XX_BLACK);
  }
  if((iy&15)==15)adquirir();
 }
 dibujarReflejoCorazonUI();
 corazonEstadoDibujadoUI=nuevo;
}

void mujerBaseUI(int cx,int cabezaY,uint16_t c){
 lcd.fillCircle(cx,cabezaY,7,c);
 lcd.fillCircle(cx+9,cabezaY+1,4,c);
 lcd.fillTriangle(cx+10,cabezaY+1,cx+16,cabezaY+6,cx+11,cabezaY+7,c);
}

void dibujarMujerUI(uint8_t estado){
 lcd.fillRect(5,137,64,108,ST77XX_BLACK);adquirir();
 const uint16_t c=uiMagenta();
 if(estado==0){
  mujerBaseUI(34,158,c);
  lcd.fillTriangle(26,172,42,172,46,198,c);
  lcd.fillTriangle(26,172,46,198,22,198,c);
  lineaGruesaUI(27,175,20,194,4,c); lineaGruesaUI(41,175,48,194,4,c);
  lineaGruesaUI(29,198,29,229,5,c); lineaGruesaUI(40,198,40,229,5,c);
 }else if(estado==1){
  mujerBaseUI(33,157,c);
  lcd.fillTriangle(25,171,41,170,46,196,c);
  lcd.fillTriangle(25,171,46,196,21,196,c);
  lineaGruesaUI(25,176,15,187,4,c); lineaGruesaUI(40,175,51,184,4,c);
  lineaGruesaUI(29,195,21,229,5,c); lineaGruesaUI(39,195,53,220,5,c);
 }else{
  mujerBaseUI(35,156,c);
  lcd.fillTriangle(27,170,43,172,45,195,c);
  lcd.fillTriangle(27,170,45,195,22,193,c);
  lineaGruesaUI(27,176,15,181,4,c); lineaGruesaUI(42,176,52,184,4,c);
  lineaGruesaUI(29,193,19,210,5,c); lineaGruesaUI(19,210,31,229,5,c);
  lineaGruesaUI(40,194,50,207,5,c); lineaGruesaUI(50,207,61,201,5,c);
 }
 adquirir();
}

void dibujarBluetoothUI(bool activo){
 lcd.fillRect(4,260,78,58,ST77XX_BLACK);adquirir();
 const int cx=18,cy=288,r=8;
 lcd.fillCircle(cx,cy,r,uiAzulBT());
 // Monograma Bluetooth reducido y centrado dentro del circulo. Se desplaza 1 px hacia arriba.
 const int bx=cx-1,by=cy-1;
 lcd.drawLine(bx,by-5,bx,by+5,ST77XX_WHITE);       // asta central
 lcd.drawLine(bx,by-5,bx+3,by-2,ST77XX_WHITE);
 lcd.drawLine(bx+3,by-2,bx,by,ST77XX_WHITE);
 lcd.drawLine(bx,by,bx+3,by+2,ST77XX_WHITE);
 lcd.drawLine(bx+3,by+2,bx,by+5,ST77XX_WHITE);
 lcd.drawLine(bx-3,by-3,bx+3,by+2,ST77XX_WHITE);
 lcd.drawLine(bx-3,by+3,bx+3,by-2,ST77XX_WHITE);
 textoCentroCajaUI(activo?"ON":"OFF",31,274,48,28,&FreeSansBold9pt7b,ST77XX_WHITE);
}

uint16_t colorBateriaUI(uint8_t porcentaje){
 // Intervalos visuales solicitados:
 // 0..25 % rojo, 26..50 % naranja, 51..75 % amarillo, 76..100 % verde.
 if(porcentaje<=25)return uiRojo();
 if(porcentaje<=50)return uiNaranja();
 if(porcentaje<=75)return uiAmarillo();
 return uiVerde();
}

void dibujarBateriaUI(uint8_t porcentaje){
 lcd.fillRect(86,260,84,58,ST77XX_BLACK);adquirir();
 const uint16_t c=colorBateriaUI(porcentaje);
 char p[7];snprintf(p,sizeof(p),"%u%%",porcentaje);
 textoCentroCajaUI(p,87,274,50,28,&FreeSansBold9pt7b,ST77XX_WHITE);
 const int x=153,y=275,w=13,h=29;
 lcd.fillRoundRect(x+4,y-3,5,4,1,c);
 lcd.drawRoundRect(x,y,w,h,2,c);
 lcd.fillRect(x+2,y+3,w-4,h-5,ST77XX_BLACK);
 const int sx=x+3,sw=w-6,sh=4,gap=1;
 const int top=y+4;
 // 0 %=0 barras; 1..25=1; 26..50=2; 51..75=3; 76..100=4.
 int activos=(porcentaje==0)?0:(porcentaje+24)/25;
 activos=constrain(activos,0,4);
 for(int i=0;i<4;i++){
  int sy=top+(3-i)*(sh+gap);
  lcd.fillRect(sx,sy,sw,sh,i<activos?c:ST77XX_BLACK);
 }
 adquirir();
}

void dibujarBaseUI(){
 lcd.fillScreen(ST77XX_BLACK);adquirir();
 lcd.drawFastHLine(8,127,156,uiLinea());
 lcd.drawFastHLine(8,128,156,uiLinea());
 lcd.drawFastHLine(8,254,156,uiLinea());
 lcd.drawFastHLine(8,255,156,uiLinea());
 lcd.drawFastVLine(83,270,36,uiLinea());
 corazonEstadoDibujadoUI=-1;
 dibujarCorazonEstadoUI(false);
 textoCentroCajaUI("Movimiento",72,154,96,18,&FreeSans9pt7b,uiGris());
 interfazBaseDibujada=true;
}

uint8_t estimarPorcentajeBateriaUI(float v){
 // Aproximacion NO lineal para una celda Li-ion 18650 1S de 3,7 V nominal / 4,20 V cargada.
 // La capacidad (2600 mAh) afecta la autonomia, no esta curva tension->estado de carga.
 // Los puntos representan una curva generica de reposo/carga ligera: es deliberadamente
 // aproximada y suficiente para un indicador de wearable, no para metrologia de bateria.
 if(!isfinite(v) || v<2.50f || v>4.45f)return 0;
 if(v>=4.20f)return 100;
 if(v<=3.30f)return 0;

 static const float voltios[] = {
  3.30f,3.50f,3.61f,3.66f,3.69f,3.71f,3.73f,3.75f,3.77f,3.79f,
  3.82f,3.85f,3.87f,3.91f,3.95f,3.98f,4.02f,4.08f,4.11f,4.15f,4.20f
 };
 static const uint8_t soc[] = {
   0, 5,10,15,20,25,30,35,40,45,
  50,55,60,65,70,75,80,85,90,95,100
 };
 const int N=sizeof(voltios)/sizeof(voltios[0]);
 for(int i=1;i<N;i++){
  if(v<=voltios[i]){
   float f=(v-voltios[i-1])/(voltios[i]-voltios[i-1]);
   float p=soc[i-1]+f*(soc[i]-soc[i-1]);
   return (uint8_t)constrain((int)lroundf(p),0,100);
  }
 }
 return 100;
}

void actualizarBateriaUI(){
 unsigned long t=millis();
 if(ultimaBateriaUI && t-ultimaBateriaUI<BAT_UPDATE_MS)return;
 ultimaBateriaUI=t;

 // Ocho lecturas consecutivas reducen ruido del ADC; esta rutina corre solo cada 10 s.
 uint32_t suma=0;
 for(uint8_t i=0;i<8;i++)suma+=analogReadMilliVolts(BAT_ADC_PIN);
 // Divisor de placa 200k/100k => Vbat ~= 3 * Vadc. mV -> V: x0,003.
 float v=(suma/8.0f)*0.003f;
 if(isfinite(v) && v>2.0f && v<4.8f)
  bateriaVoltiosUI=isnan(bateriaVoltiosUI)?v:0.82f*bateriaVoltiosUI+0.18f*v;
 else bateriaVoltiosUI=NAN;

 uint8_t nuevo=estimarPorcentajeBateriaUI(bateriaVoltiosUI);
 static int previo=-1;
 // Se redibuja solo si cambia al menos 1 punto porcentual.
 if((int)nuevo!=previo){
  bateriaEstadoUI=nuevo;
  dibujarBateriaUI(nuevo);
  previo=nuevo;
 }
}

void actualizarBluetoothUI(bool activo){
 static int previo=-1;bluetoothActivoUI=activo;
 if(previo!=(int)activo){dibujarBluetoothUI(activo);previo=activo;}
}

void actualizarMovimientoUI(){
 static int previo=-1;int actual=lecturaOK?min(2,(int)nivel):0;
 if(actual==previo)return;
 dibujarMujerUI(actual);
 lcd.fillRect(70,174,101,66,ST77XX_BLACK);adquirir();
 const char *txt=actual==0?"Reposo":actual==1?"Leve":"Moderado";
 textoCentroCajaUI(txt,72,185,96,30,&FreeSansBold9pt7b,ST77XX_WHITE);
 previo=actual;
}

void actualizarPuntosMidiendoUI(bool activo){
 static int pasoPrevio=-1;
 if(!activo){pasoPrevio=-1;return;}
 int paso=(millis()/UI_PUNTOS_MS)%3 + 1;
 if(paso==pasoPrevio)return;
 char puntos[4]={0};for(int i=0;i<paso;i++)puntos[i]='.';
 // Solo se limpia la pequeña caja de puntos. "Midiendo" no se redibuja continuamente.
 lcd.fillRect(92,72,70,22,ST77XX_BLACK);adquirir();
 textoCentroCajaUI(puntos,92,72,70,18,&FreeSansBold9pt7b,uiGris());
 pasoPrevio=paso;
}

bool contactoFisicoUI(){
 // Misma evidencia optica que informa Serie: muestra reciente + IR por encima del umbral.
 // Se consulta el IR mas reciente directamente para que una reinicializacion del detector
 // nunca pueda dejar la LCD congelada en "Sin contacto" si el MAX ya esta leyendo el dedo.
 return datosVistos && (millis()-ultimoDato<500) && (ir>=8000);
}

void actualizarContactoUI(){actualizarFrecuenciaCardiacaUI();}

void actualizarFrecuenciaCardiacaUI(){
 // No llamar vigilarEstado8() desde la UI: el loop conserva exactamente la vigilancia original.
 const bool contactoUI=contactoFisicoUI();
 const bool bpmValidoUI=contactoUI && mostrarBPM8();
 // 0=sin contacto, 1=contacto/midiendo, 2=BPM disponible.
 int modo=!contactoUI?0:!bpmValidoUI?1:2;
 int b=bpmValidoUI?(int)lroundf(bpm8.ultimo):-1;
 static int previoBpm=-999,previoModo=-1;
 static uint32_t confirmarRedibujoUI=0;
 uint32_t ahora=millis();
 bool cambioEstado=(b!=previoBpm || modo!=previoModo);
 // Cada cambio real se dibuja inmediatamente y se confirma UNA sola vez 650 ms despues.
 // Asi, si una transaccion SPI aislada no llego a verse, la LCD se autocorrige sin
 // refrescos periodicos ni parpadeos continuos.
 if(cambioEstado)confirmarRedibujoUI=ahora+650;
 bool confirmacion=confirmarRedibujoUI && (int32_t)(ahora-confirmarRedibujoUI)>=0;
 bool cambio=cambioEstado || confirmacion;
 if(cambio){
  lcd.fillRect(81,10,91,110,ST77XX_BLACK);adquirir();
  if(modo==0){
   textoCentroCajaUI("Sin",83,39,87,22,&FreeSansBold12pt7b,ST77XX_WHITE);
   textoCentroCajaUI("contacto",83,64,87,20,&FreeSansBold9pt7b,ST77XX_WHITE);
  }else if(modo==1){
   textoCentroCajaUI("Midiendo",83,47,87,22,&FreeSansBold9pt7b,ST77XX_WHITE);
   // Los puntos se dibujan abajo por actualizarPuntosMidiendoUI().
  }else{
   // La UI ya no colorea de amarillo ni muestra RET./RETENIDO.
   // El valor mostrado es el ultimo BPM validado por V9, siempre en blanco.
   char n[5];snprintf(n,sizeof(n),"%d",b);
   const GFXfont *fuenteBPM=(b>=100)?&FreeSansBold18pt7b:&FreeSansBold24pt7b;
   textoCentroCajaUI(n,82,24,88,50,fuenteBPM,ST77XX_WHITE);
   textoCentroCajaUI("LPM",86,80,82,20,&FreeSansBold9pt7b,uiGris());
  }
  previoBpm=b;previoModo=modo;
  if(confirmacion)confirmarRedibujoUI=0;
 }
 actualizarPuntosMidiendoUI(modo==1);
}

void animarCorazonUI(){
 // v4: la animacion depende SOLO de contacto optico reciente. No usa BPM, V9, filtro,
 // ultimoLatidoAnim ni ninguna aceptacion cardiaca. Es una animacion de "sensor en contacto".
 static bool contactoPrev=false;
 static bool grande=false;
 static unsigned long finGrande=0,proximoPulso=0;
 unsigned long ahora=millis();
 bool contactoUI=contactoFisicoUI();

 if(!contactoUI){
  if(contactoPrev || grande)dibujarCorazonEstadoUI(false);
  contactoPrev=false;grande=false;finGrande=0;proximoPulso=0;
  return;
 }
 if(!contactoPrev){
  // Primera respuesta visual inmediata al apoyar el dedo.
  dibujarCorazonEstadoUI(true);grande=true;
  finGrande=ahora+UI_PULSO_GRANDE_MS;
  proximoPulso=ahora+UI_PULSO_PERIODO_MS;
  contactoPrev=true;return;
 }
 if(grande && (long)(ahora-finGrande)>=0){
  dibujarCorazonEstadoUI(false);grande=false;return;
 }
 if(!grande && (long)(ahora-proximoPulso)>=0){
  dibujarCorazonEstadoUI(true);grande=true;
  finGrande=ahora+UI_PULSO_GRANDE_MS;
  proximoPulso=ahora+UI_PULSO_PERIODO_MS;
 }
}

void informarMemoriaUI(){
 diagnosticoInicio18.printf("MEM_UI_V8: heap_total=%lu heap_libre=%lu psram_total=%lu psram_libre=%lu. UI sin framebuffer/sprite; corazon con tablas de scanline.\n",
  (unsigned long)ESP.getHeapSize(),(unsigned long)ESP.getFreeHeap(),
  (unsigned long)ESP.getPsramSize(),(unsigned long)ESP.getFreePsram());
}

// ===== B18 BLE: transport only. B17 exclusively owns sensors and physiology. =====
// Arduino-ESP32 3.3.12 / bundled NimBLE. No Notify, formatting or BLE wait in acquisition.
class B18BLE {
  struct Snapshot {
    uint32_t captured=0, sampleTime=0, bpmDate=0, bpmSeq=0, oxDate=0, batDate=0;
    float bpmValue=NAN, wave=NAN, oxygen=NAN, batteryV=NAN;
    uint8_t activity=3, quality=0, state=0, battery=0, oxReason=1;
    bool ready=false, fresh=false, contact=false, bpmSource=false, bpmVisible=false;
    bool oxVisibleNow=false, prvQuality=false;
    PRV17Estado prv;
  };
  struct Link {
    bool connected=false, subscribed=false, paSubscribed=false;
    uint16_t mtu=23,handle=0xFFFF;
    uint32_t epoch=0;
  };
  struct Command { uint32_t epoch; char text[32]; };
  struct Frame { char data[193]; uint16_t length=0, offset=0; uint32_t epoch=0; };
  portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
  Link link;
  QueueHandle_t snapshots=nullptr, commands=nullptr;
  TaskHandle_t worker=nullptr;
  BLEServer *server=nullptr;
  BLECharacteristic *tx=nullptr;
  BLECharacteristic *paTx=nullptr;
  Snapshot latest;
  Frame frame;
  bool running=false, haveSnapshot=false, v2=false, resync=false;
  bool needHello=false, needState=false, needOx=false, needPrv=false, needDiag=false;
  bool lastOxValid=false,lastPrvValid=false;
  uint8_t streamHz=20;
  uint32_t activeEpoch=0, bootHigh=0, bootLow=0, lastPublish=0;
  uint32_t bSeq=0,sSeq=0,oSeq=0,rSeq=0,dSeq=0;
  uint32_t lastB=0,lastS=0,lastO=0,lastR=0,lastD=0,lastFragment=0,backoffUntil=0;
  uint32_t txOK=0,txErrors=0,streamSkipped=0,snapshotSkipped=0,commandDrops=0;
  uint32_t maxNotifyUs=0,formatErrors=0,asyncErrors=0,seenAsyncErrors=0;
  uint32_t publishSeq=0,seenPublishSeq=0;
  // B19 PA transport is deliberately independent from the NUS frame state.
  bool paBusy=false,paQueued=false;
  uint32_t paQueuedSeq=0,paQueuedSpan=0;
  uint32_t paAsyncErrors=0,seenPaAsyncErrors=0,paTxOK=0,paTxErrors=0,paDropped=0;
  uint32_t paActiveSeq=0,paActiveSpan=0,paCrc=0,paLastFragment=0,paBackoffUntil=0;
  uint16_t paStartIndex=0;
  uint8_t paPhase=0; // 0 idle, 1 BEGIN, 2 DATA, 3 END.
  struct Mail { Snapshot value; uint32_t sequence; };

  Link readLink() {
    portENTER_CRITICAL(&mux); Link copy=link; portEXIT_CRITICAL(&mux); return copy;
  }
  void changeConnection(bool connected,uint16_t handle=0xFFFF) {
    portENTER_CRITICAL(&mux);
    link.connected=connected;link.subscribed=false;link.paSubscribed=false;link.mtu=23;link.handle=handle;++link.epoch;
    portEXIT_CRITICAL(&mux);
  }
  void changeSubscription(bool subscribed) {
    portENTER_CRITICAL(&mux);
    if(link.subscribed!=subscribed){link.subscribed=subscribed;++link.epoch;}
    portEXIT_CRITICAL(&mux);
  }
  void changePASubscription(bool subscribed) {
    portENTER_CRITICAL(&mux);
    link.paSubscribed=subscribed;
    portEXIT_CRITICAL(&mux);
  }
  bool paIsBusy() {
    portENTER_CRITICAL(&mux); bool busy=paBusy; portEXIT_CRITICAL(&mux); return busy;
  }
  bool queuePAWindow(uint32_t seq,uint32_t span) {
    bool accepted=false;
    portENTER_CRITICAL(&mux);
    if(link.connected && link.subscribed && link.paSubscribed && !paBusy){
      paBusy=true;paQueued=true;paQueuedSeq=seq;paQueuedSpan=span;accepted=true;
    }
    portEXIT_CRITICAL(&mux);
    return accepted;
  }
  void releasePAWindow(bool dropped) {
    portENTER_CRITICAL(&mux);
    paBusy=false;paQueued=false;if(dropped)++paDropped;
    portEXIT_CRITICAL(&mux);
  }
  void countCommandDrop() {
    portENTER_CRITICAL(&mux);++commandDrops;portEXIT_CRITICAL(&mux);
  }
  class ServerCallbacks : public BLEServerCallbacks {
    B18BLE &owner;
   public:
    explicit ServerCallbacks(B18BLE &o):owner(o){}
    void onConnect(BLEServer *,ble_gap_conn_desc *desc) override {
      owner.changeConnection(true,desc->conn_handle);
    }
    void onDisconnect(BLEServer *,ble_gap_conn_desc *) override { owner.changeConnection(false); }
    void onMtuChanged(BLEServer *,ble_gap_conn_desc *desc,uint16_t mtu) override {
      portENTER_CRITICAL(&owner.mux);
      if(desc->conn_handle==owner.link.handle)owner.link.mtu=constrain((int)mtu,23,517);
      portEXIT_CRITICAL(&owner.mux);
    }
  } serverCallbacks{*this};
  class RXCallbacks : public BLECharacteristicCallbacks {
    B18BLE &owner;
   public:
    explicit RXCallbacks(B18BLE &o):owner(o){}
    void onWrite(BLECharacteristic *c) override {
      Link l=owner.readLink();
      if(!l.connected || !l.subscribed || !owner.commands)return;
      // The core returns Arduino String. Limit before copying; no peripheral calls.
      String value=c->getValue(); Command cmd{};cmd.epoch=l.epoch;
      size_t n=value.length();
      if(n==0 || n>31){strcpy(cmd.text,"!FORMAT");}
      else {
        bool valid=true;
        for(size_t i=0;i<n;i++){
          uint8_t ch=(uint8_t)value[i];
          if(ch==0 || (ch<32 && ch!='\r' && ch!='\n') || ch>126)valid=false;
          cmd.text[i]=(char)ch;
        }
        cmd.text[n]=0;
        while(n && (cmd.text[n-1]=='\r'||cmd.text[n-1]=='\n'))cmd.text[--n]=0;
        for(size_t i=0;i<n;i++)if(cmd.text[i]=='\r'||cmd.text[i]=='\n')valid=false;
        if(!n || !valid)strcpy(cmd.text,"!FORMAT");
      }
      if(xQueueSend(owner.commands,&cmd,0)!=pdTRUE)owner.countCommandDrop();
    }
  } rxCallbacks{*this};
  class TXCallbacks : public BLECharacteristicCallbacks {
    B18BLE &owner;
   public:
    explicit TXCallbacks(B18BLE &o):owner(o){}
    void onSubscribe(BLECharacteristic *,ble_gap_conn_desc *,uint16_t value) override {
      owner.changeSubscription((value&1)!=0);
    }
    void onStatus(BLECharacteristic *,Status status,uint32_t) override {
      // Asynchronous NimBLE host notification event. No data/task work here.
      if(status!=SUCCESS_NOTIFY){
        portENTER_CRITICAL(&owner.mux);++owner.asyncErrors;portEXIT_CRITICAL(&owner.mux);
      }
    }
  } txCallbacks{*this};
  class PATXCallbacks : public BLECharacteristicCallbacks {
    B18BLE &owner;
   public:
    explicit PATXCallbacks(B18BLE &o):owner(o){}
    void onSubscribe(BLECharacteristic *,ble_gap_conn_desc *,uint16_t value) override {
      owner.changePASubscription((value&1)!=0);
    }
    void onStatus(BLECharacteristic *,Status status,uint32_t) override {
      if(status!=SUCCESS_NOTIFY){
        portENTER_CRITICAL(&owner.mux);++owner.paAsyncErrors;portEXIT_CRITICAL(&owner.mux);
      }
    }
  } paTxCallbacks{*this};

  static void taskEntry(void *arg){static_cast<B18BLE*>(arg)->task();}
  static const char *number(float value,bool valid,char *out,size_t capacity) {
    if(!valid || !isfinite(value) || fabsf(value)>10000000.0f){strcpy(out,"NA");return out;}
    snprintf(out,capacity,"%.1f",(double)value);return out;
  }
  static const char *age(uint32_t now,uint32_t date,bool valid,char *out,size_t capacity) {
    if(!valid){strcpy(out,"NA");return out;}
    snprintf(out,capacity,"%lu",(unsigned long)(now-date));return out;
  }
  bool live(uint32_t now) const {
    return haveSnapshot && latest.ready && (uint32_t)(now-latest.captured)<250 &&
           latest.fresh && (uint32_t)(now-latest.sampleTime)<500;
  }
  bool oxygenValid(uint32_t now) const {
    return live(now) && latest.oxVisibleNow && now-latest.oxDate<2500 && isfinite(latest.oxygen);
  }
  bool prvValid(uint32_t now) const {
    return live(now) && latest.prv.valido && latest.prvQuality &&
      now-latest.prv.fecha<6000 && isfinite(latest.prv.ppMedio) &&
      isfinite(latest.prv.rmssd) && isfinite(latest.prv.sdnn) && isfinite(latest.prv.pnn50);
  }
  void finishFrame(int length) {
    if(length<=0 || length>192){++formatErrors;frame.length=frame.offset=0;return;}
    frame.length=(uint16_t)length;frame.offset=0;frame.epoch=activeEpoch;
  }
  void response(const char *text) {
    finishFrame(snprintf(frame.data,sizeof(frame.data),v2?"%s\n":"%s",text));
  }
  void hello() {
    // Fixed capability mask: B=1,S=2,O=4,R=8,D=16. PA and old modes are absent.
    finishFrame(snprintf(frame.data,sizeof(frame.data),
      "H:2,%08lX%08lX,%lu,B18,31,100,20,192,180\n",
      (unsigned long)bootHigh,(unsigned long)bootLow,(unsigned long)activeEpoch));
  }
  void makeB(uint32_t now) {
    bool fresh=live(now),visible=fresh && latest.bpmVisible && isfinite(latest.bpmValue);
    uint8_t a=fresh?latest.activity:3,c=fresh?latest.quality:0,e=fresh?latest.state:0;
    bool bat=haveSnapshot && isfinite(latest.batteryV) && latest.batteryV>=2.50f &&
             latest.batteryV<=4.45f && now-latest.batDate<30000;
    if(!v2){
      // Legacy keeps a conservative numeric BPM: retained values are not fresh measurements.
      int b=visible && e==4 ? (int)lroundf(latest.bpmValue):0;
      finishFrame(snprintf(frame.data,sizeof(frame.data),"B:%03d A:%u C:%u P:%03d",
        b,a,c,bat?(int)latest.battery:-1));return;
    }
    char b[12],p[12],d[12];
    if(visible)snprintf(b,sizeof(b),"%03d",(int)lroundf(latest.bpmValue));else strcpy(b,"NA");
    if(bat)snprintf(p,sizeof(p),"%03u",latest.battery);else strcpy(p,"NA");
    age(now,latest.bpmDate,haveSnapshot && latest.bpmSource,d,sizeof(d));
    finishFrame(snprintf(frame.data,sizeof(frame.data),
      "B:%s A:%u C:%u P:%s I:%lu T:%lu V:%u E:%u D:%s N:%lu\n",
      b,a,c,p,(unsigned long)++bSeq,(unsigned long)now,visible?1:0,e,d,(unsigned long)latest.bpmSeq));
  }
  void makeS(uint32_t now) {
    bool valid=live(now) && latest.contact && isfinite(latest.wave) && fabsf(latest.wave)<=999999.0f;
    if(!v2){
      finishFrame(snprintf(frame.data,sizeof(frame.data),"S:%.1f",valid?(double)latest.wave:0.0));return;
    }
    char y[24],t[12];number(latest.wave,valid,y,sizeof(y));
    if(valid)snprintf(t,sizeof(t),"%lu",(unsigned long)latest.sampleTime);else strcpy(t,"NA");
    finishFrame(snprintf(frame.data,sizeof(frame.data),"S:%lu,%s,%s,%u\n",
      (unsigned long)++sSeq,t,y,valid?1:0));
  }
  void makeO(uint32_t now) {
    bool valid=oxygenValid(now);char value[24],d[12];
    number(latest.oxygen,valid,value,sizeof(value));
    age(now,latest.oxDate,haveSnapshot && latest.oxDate!=0,d,sizeof(d));
    uint8_t reason=valid?0:(!live(now)?4:latest.oxReason);
    if(!valid && reason==0)reason=now-latest.oxDate>=2500?5:4;
    finishFrame(snprintf(frame.data,sizeof(frame.data),"O:%lu,%lu,%s,%u,%s,%u\n",
      (unsigned long)++oSeq,(unsigned long)now,value,valid?1:0,d,reason));
    lastOxValid=valid;
  }
  void makeR(uint32_t now) {
    bool valid=prvValid(now);char pp[24],rm[24],sd[24],pn[24],d[12],flag[4];
    number(latest.prv.ppMedio,valid,pp,sizeof(pp));number(latest.prv.rmssd,valid,rm,sizeof(rm));
    number(latest.prv.sdnn,valid,sd,sizeof(sd));number(latest.prv.pnn50,valid,pn,sizeof(pn));
    age(now,latest.prv.fecha,haveSnapshot && latest.prv.fecha!=0,d,sizeof(d));
    if(valid)snprintf(flag,sizeof(flag),"%u",latest.prv.patronEctopico?1:0);else strcpy(flag,"NA");
    finishFrame(snprintf(frame.data,sizeof(frame.data),
      "R:%lu,%lu,%u,%s,%lu,%u,%u,%u,%u,%s,%s,%s,%s,%u,%u,%s\n",
      (unsigned long)++rSeq,(unsigned long)now,valid?1:0,d,(unsigned long)latest.prv.spanMs,
      latest.prv.nNN,latest.prv.nTotal,latest.prv.paresNN,latest.prv.porcentajeLimpio,
      pp,rm,sd,pn,latest.prv.irregulares30s,latest.prv.patronesEctopicos30s,flag));
    lastPrvValid=valid;
  }
  void makeD(uint32_t now) {
    uint32_t dropped;
    portENTER_CRITICAL(&mux);dropped=commandDrops;portEXIT_CRITICAL(&mux);
    finishFrame(snprintf(frame.data,sizeof(frame.data),
      "D:%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
      (unsigned long)++dSeq,(unsigned long)now,
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
      (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
      (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
      (unsigned long)uxTaskGetStackHighWaterMark(nullptr),
      (unsigned long)txOK,(unsigned long)txErrors,(unsigned long)streamSkipped,
      (unsigned long)snapshotSkipped,(unsigned long)dropped,(unsigned long)maxNotifyUs,
      (unsigned long)formatErrors));
  }
  static void put16(uint8_t *p,uint16_t value) {
    p[0]=(uint8_t)(value&0xFF);p[1]=(uint8_t)(value>>8);
  }
  static void put32(uint8_t *p,uint32_t value) {
    p[0]=(uint8_t)(value&0xFF);p[1]=(uint8_t)((value>>8)&0xFF);
    p[2]=(uint8_t)((value>>16)&0xFF);p[3]=(uint8_t)((value>>24)&0xFF);
  }
  static uint32_t crc32PA() {
    uint32_t crc=0xFFFFFFFFu;
    for(uint16_t i=0;i<BP15_N;i++){
      uint32_t value=bpTx[i];
      for(uint8_t byteIndex=0;byteIndex<4;byteIndex++){
        crc^=(uint8_t)(value>>(8*byteIndex));
        for(uint8_t bit=0;bit<8;bit++)crc=(crc>>1)^((crc&1)?0xEDB88320u:0u);
      }
    }
    return crc^0xFFFFFFFFu;
  }
  bool takeQueuedPA() {
    uint32_t seq=0,span=0;bool take=false;
    portENTER_CRITICAL(&mux);
    if(paQueued){seq=paQueuedSeq;span=paQueuedSpan;paQueued=false;take=true;}
    portEXIT_CRITICAL(&mux);
    if(!take)return false;
    paActiveSeq=seq;paActiveSpan=span;paCrc=crc32PA();paStartIndex=0;paPhase=1;
    paLastFragment=0;paBackoffUntil=0;return true;
  }
  void cancelPA(bool dropped=true) {
    paPhase=0;paStartIndex=0;paBackoffUntil=0;releasePAWindow(dropped);
  }
  void sendPA(const Link &current,uint32_t now) {
    if(!paTx || !v2)return;
    if(!paPhase && !takeQueuedPA())return;
    if(now-paLastFragment<10 || (paBackoffUntil && (int32_t)(now-paBackoffUntil)<0))return;
    Link l=readLink();
    if(!l.connected || !l.subscribed || !l.paSubscribed || l.epoch!=current.epoch || l.epoch!=activeEpoch){
      cancelPA(true);return;
    }
    uint16_t payloadLimit=min((int)l.mtu-3,180);
    if(payloadLimit<20){cancelPA(true);return;}
    uint8_t packet[180];uint16_t length=0;
    packet[0]='P';packet[1]='A';packet[2]=1;packet[3]=paPhase;put32(packet+4,paActiveSeq);
    if(paPhase==1){
      put16(packet+8,BP15_N);put16(packet+10,100);put32(packet+12,paActiveSpan);put32(packet+16,paCrc);length=20;
    } else if(paPhase==2){
      uint16_t capacity=(payloadLimit-12)/4;
      if(!capacity){cancelPA(true);return;}
      uint16_t count=min((uint16_t)(BP15_N-paStartIndex),capacity);
      put16(packet+8,paStartIndex);put16(packet+10,count);
      for(uint16_t i=0;i<count;i++)put32(packet+12+4*i,bpTx[paStartIndex+i]);
      length=12+4*count;
    } else {
      put16(packet+8,BP15_N);put16(packet+10,0);put32(packet+12,paCrc);length=16;
    }
    uint32_t us=micros();os_mbuf *buffer=ble_hs_mbuf_from_flat(packet,length);
    int result=buffer?ble_gatts_notify_custom(l.handle,paTx->getHandle(),buffer):BLE_HS_ENOMEM;
    uint32_t elapsed=micros()-us;maxNotifyUs=max(maxNotifyUs,elapsed);paLastFragment=now;
    if(result!=0){++paTxErrors;cancelPA(true);return;}
    ++paTxOK;
    if(paPhase==1)paPhase=2;
    else if(paPhase==2){
      uint16_t count=(length-12)/4;paStartIndex+=count;if(paStartIndex>=BP15_N)paPhase=3;
    } else {
      paPhase=0;paStartIndex=0;releasePAWindow(false);
    }
  }

  void handleCommand(const Command &cmd) {
    if(cmd.epoch!=activeEpoch)return;
    if(!strcmp(cmd.text,"HELLO:2")){
      v2=true;needHello=true;needState=needOx=needPrv=needDiag=true;
      // No ACK: H is the deterministic successful negotiation response.
      return;
    }
    if(!strncmp(cmd.text,"MODE:",5)){response("ERR:MODE:UNSUPPORTED");return;}
    if(!strncmp(cmd.text,"SCREEN:",7)){response("ERR:SCREEN:UNSUP");return;}
    if(!strcmp(cmd.text,"!FORMAT")){response("ERR:FORMAT");return;}
    if(!strcmp(cmd.text,"GET:STATE")){
      if(!v2){response("ERR:HELLO_REQUIRED");return;}
      needState=needOx=needPrv=needDiag=true;response("ACK:GET:STATE");return;
    }
    if(!strcmp(cmd.text,"STREAM:0") || !strcmp(cmd.text,"STREAM:20")){
      if(!v2){response("ERR:HELLO_REQUIRED");return;}
      streamHz=!strcmp(cmd.text,"STREAM:20")?20:0;lastS=millis();
      response(streamHz?"ACK:STREAM:20":"ACK:STREAM:0");return;
    }
    response("ERR:UNKNOWN");
  }
  void resetTransport(uint32_t epoch,uint32_t now) {
    activeEpoch=epoch;v2=false;streamHz=20;frame.length=frame.offset=0;resync=false;
    bSeq=sSeq=oSeq=rSeq=dSeq=0;
    needHello=false;needState=true;needOx=needPrv=needDiag=false;
    lastB=lastS=lastO=lastR=lastD=now;lastFragment=0;backoffUntil=0;
    lastOxValid=lastPrvValid=false;
    paPhase=0;paStartIndex=0;paLastFragment=0;paBackoffUntil=0;releasePAWindow(false);
  }
  bool sendFragment(const Link &current,uint32_t now) {
    if(!frame.length && !resync)return false;
    if(now-lastFragment<5 || (backoffUntil && (int32_t)(now-backoffUntil)<0))return true;
    backoffUntil=0;
    Link l=readLink();
    if(!l.connected || !l.subscribed || l.epoch!=current.epoch || l.epoch!=activeEpoch){
      frame.length=frame.offset=0;resync=false;return true;
    }
    uint16_t count=resync?2:min((int)(frame.length-frame.offset),min((int)l.mtu-3,180));
    const uint8_t *data=resync?(const uint8_t*)"!\n":(const uint8_t*)frame.data+frame.offset;
    uint32_t us=micros();
    // Core NimBLE primitive gives an immediate return code, unlike void notify().
    // mbuf ownership transfers to ble_gatts_notify_custom even on failure.
    os_mbuf *buffer=ble_hs_mbuf_from_flat(data,count);
    int result=buffer?ble_gatts_notify_custom(l.handle,tx->getHandle(),buffer):BLE_HS_ENOMEM;
    uint32_t elapsed=micros()-us;maxNotifyUs=max(maxNotifyUs,elapsed);lastFragment=now;
    if(result==0){
      ++txOK;
      if(resync)resync=false;
      else {frame.offset+=count;if(frame.offset>=frame.length)frame.length=frame.offset=0;}
    }else{
      ++txErrors;if(v2 && frame.length && frame.data[0]=='H')needHello=true;
      frame.length=frame.offset=0;resync=v2;
      backoffUntil=now+100; // Only TX pauses. Main acquisition never waits.
    }
    return true;
  }
  void task() {
    bool advertisingPending=false;
    uint32_t advertisingAt=0,lastCommand=0;
    for(;;){
      uint32_t now=millis();Link l=readLink();
      if(l.epoch!=activeEpoch){
        resetTransport(l.epoch,now);
        if(!l.connected){advertisingPending=true;advertisingAt=now+200;}
      }
      if(advertisingPending && !l.connected && (int32_t)(now-advertisingAt)>=0){
        server->startAdvertising();advertisingPending=false;
      }
      if(l.connected)advertisingPending=false;
      Mail mail;
      if(xQueueReceive(snapshots,&mail,0)==pdTRUE){
        if(seenPublishSeq){uint32_t difference=mail.sequence-seenPublishSeq;if(difference>1)snapshotSkipped+=difference-1;}
        seenPublishSeq=mail.sequence;latest=mail.value;haveSnapshot=true;
      }
      if(!l.connected || !l.subscribed){vTaskDelay(pdMS_TO_TICKS(10));continue;}
      uint32_t failures;
      portENTER_CRITICAL(&mux);failures=asyncErrors;portEXIT_CRITICAL(&mux);
      if(failures!=seenAsyncErrors){
        txErrors+=failures-seenAsyncErrors;seenAsyncErrors=failures;
        if(v2 && frame.length && frame.data[0]=='H')needHello=true;
        frame.length=frame.offset=0;resync=v2;backoffUntil=now+100;
      }
      uint32_t paFailures;
      portENTER_CRITICAL(&mux);paFailures=paAsyncErrors;portEXIT_CRITICAL(&mux);
      if(paFailures!=seenPaAsyncErrors){
        paTxErrors+=paFailures-seenPaAsyncErrors;seenPaAsyncErrors=paFailures;
        if(paPhase)cancelPA(true);
      }
      if(sendFragment(l,now)){vTaskDelay(pdMS_TO_TICKS(5));continue;}
      if(needHello){needHello=false;hello();}
      else {
        bool ov=oxygenValid(now),rv=prvValid(now);
        if(v2 && ov!=lastOxValid)needOx=true;
        if(v2 && rv!=lastPrvValid)needPrv=true;
        // Deadline priority. Commands capped at 10/s; pending H always precedes v2 data.
        if(needState || now-lastB>=500){needState=false;lastB=now;makeB(now);}
        else if(v2 && (needOx || now-lastO>=1000)){needOx=false;lastO=now;makeO(now);}
        else if(v2 && (needPrv || now-lastR>=5000)){needPrv=false;lastR=now;makeR(now);}
        else if(v2 && (needDiag || now-lastD>=10000)){needDiag=false;lastD=now;makeD(now);}
        else if(now-lastCommand>=100 && uxQueueMessagesWaiting(commands)){
          Command cmd;
          if(xQueueReceive(commands,&cmd,0)==pdTRUE){lastCommand=now;handleCommand(cmd);}
        }
        else if(streamHz && now-lastS>=50){
          uint32_t intervals=(now-lastS)/50;
          if(intervals>1)streamSkipped+=intervals-1;
          lastS+=intervals*50;makeS(now);
        }
        else if(v2 && (l.paSubscribed || paPhase || paQueued))sendPA(l,now);
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
 public:
  bool active(){Link l=readLink();return running && l.connected && l.subscribed;}
  void begin() {
    snapshots=xQueueCreate(1,sizeof(Mail));commands=xQueueCreate(8,sizeof(Command));
    if(!snapshots || !commands){Serial.println("B18 BLE: queue allocation failed; B17 continues.");return;}
    bootHigh=esp_random();bootLow=esp_random();
    BLEDevice::init("PPG-Monitor-S3");BLEDevice::setMTU(247);
    server=BLEDevice::createServer();if(!server)return;
    server->setCallbacks(&serverCallbacks);server->advertiseOnDisconnect(false);
    BLEService *service=server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    if(!service)return;
    tx=service->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",BLECharacteristic::PROPERTY_NOTIFY);
    BLECharacteristic *rx=service->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
      BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_WRITE_NR);
    // B19 optional PA characteristic. Old B18 APKs simply ignore it.
    paTx=service->createCharacteristic("6E400004-B5A3-F393-E0A9-E50E24DCCA9E",BLECharacteristic::PROPERTY_NOTIFY);
    if(!tx || !rx || !paTx)return;
    // NimBLE adds CCCD 0x2902 automatically for PROPERTY_NOTIFY.
    tx->setCallbacks(&txCallbacks);rx->setCallbacks(&rxCallbacks);paTx->setCallbacks(&paTxCallbacks);service->start();
    BLEAdvertising *advertising=server->getAdvertising();
    advertising->addServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    advertising->setScanResponse(true);
    // Explicit scan response guarantees the full name fits beside the 128-bit service UUID.
    BLEAdvertisementData scanResponse;scanResponse.setName("PPG-Monitor-S3");
    advertising->setScanResponseData(scanResponse);
    BaseType_t core=0;
#if !CONFIG_FREERTOS_UNICORE
    core=(xPortGetCoreID()==0)?1:0;
#endif
    if(xTaskCreatePinnedToCore(taskEntry,"B19-BLE-TX",7168,this,1,&worker,core)!=pdPASS){
      Serial.println("B18 BLE: task allocation failed; B17 continues.");return;
    }
    running=true;advertising->start();
    Serial.printf("B19 BLE READY: B18 NUS + PA binary UUID ...0004 internal_free=%lu largest=%lu TX_core=%d\n",
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
      (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(int)core);
  }
  void publish() {
    if(!running)return;
    uint32_t now=millis();if(now-lastPublish<10)return;lastPublish=now;
    Mail mail{};Snapshot &s=mail.value;
    s.captured=now;s.ready=listo;s.fresh=datosFrescos();s.sampleTime=ultimoDato;
    s.contact=contactoCrudo;s.bpmValue=bpm8.ultimo;s.bpmSource=bpm8.valido;
    s.bpmVisible=contactoFisicoUI() && mostrarBPM8();s.bpmDate=bpm8.fecha;s.bpmSeq=bpm8.secuencia;
    s.wave=filtrada;s.activity=lecturaOK?min((int)nivel,2):3;
    const char *q=calidad8();s.quality=(!s.fresh || !s.contact)?0:!strcmp(q,"ALTA")?4:!strcmp(q,"MEDIA")?3:2;
    const char *e=estado8();
    s.state=!strcmp(e,"SIN DATOS")?0:!strcmp(e,"SIN CONTACTO")?1:!strcmp(e,"ESTABILIZANDO")?2:
      !strcmp(e,"CALCULANDO")?3:!strcmp(e,"ACTUALIZADO")?4:!strcmp(e,"RETENIDO")?5:6;
    s.battery=bateriaEstadoUI;s.batteryV=bateriaVoltiosUI;s.batDate=ultimaBateriaUI;
    s.oxygen=oxValor;s.oxDate=oxFecha;s.oxVisibleNow=oxVisible();
    s.oxReason=s.oxVisibleNow?0:!condicionesExploratorias()?4:!strcmp(oxMotivo,"ESPERANDO")?1:
      !strcmp(oxMotivo,"CALIDAD")?2:!strcmp(oxMotivo,"FUERA DE RANGO")?3:5;
    s.prv=prv17Estado;s.prvQuality=prv17CalidadInstantanea();
    mail.sequence=++publishSeq;xQueueOverwrite(snapshots,&mail);

    // B19: freeze a coherent BP15 raw-IR window only when Android subscribed.
    // This bounded 2.8 kB copy happens after the original acquisition/UI work.
    // The BLE worker transmits bpTx asynchronously; acquisition keeps using bpRing.
    Link l=readLink();
    if(l.connected && l.subscribed && l.paSubscribed && !bpPC && !bpPending && !bpTxState &&
       bpN==BP15_N && now-bpBleLast>=3000 && !paIsBusy()){
      uint32_t span=bpTimes[(bpPos+BP15_N-1)%BP15_N]-bpTimes[bpPos];
      if(span>=6500 && span<=7500){
        uint16_t first=BP15_N-bpPos;
        memcpy(bpTx,bpRing+bpPos,(size_t)first*sizeof(uint32_t));
        if(bpPos)memcpy(bpTx+first,bpRing,(size_t)bpPos*sizeof(uint32_t));
        uint32_t seq=bpBleSeq+1;
        if(queuePAWindow(seq,span)){bpBleSeq=seq;bpBleLast=now;}
      } else resetPA15();
    }
  }
};
B18BLE ble18;
// ===== End B18 BLE. Original B17 processing below/above remains unchanged. =====

void pantalla() {
 if(!interfazBaseDibujada)dibujarBaseUI();
 if(!listo){
  lcd.fillRect(12,136,148,70,ST77XX_BLACK);adquirir();
  textoCentroCajaUI("ERROR SENSOR",12,151,148,35,&FreeSansBold9pt7b,uiRojo());
  return;
 }
 actualizarContactoUI();
 actualizarMovimientoUI();
 actualizarBluetoothUI(ble18.active()); // B18: connected AND subscribed to Notify.
 actualizarBateriaUI();
}

 void anunciar() {
  if(guia)return;
  Serial.printf("\nFASE %u - %s (%lu s)\n",fase,fases[fase],duraciones[fase]/1000);
  Serial.println(fase==3 ? "Retira el dedo; manten la placa quieta." : "Apoya el dedo sin apretar fuerte; manten todo quieto.");
}
// Recuperacion solo al arrancar: bus externo MAX en GPIO5/6.
// Salidas open-drain: HIGH libera la linea; nunca fuerza 3,3 V contra otro dispositivo.
bool liberarBusMAX() {
 Wire.end();pinMode(5,OUTPUT_OPEN_DRAIN);pinMode(6,OUTPUT_OPEN_DRAIN);
 digitalWrite(5,HIGH);digitalWrite(6,HIGH);delay(2);
 for(int i=0;i<9 && digitalRead(5)==LOW;i++) {
  if(digitalRead(6)==LOW)break;
  digitalWrite(6,LOW);delayMicroseconds(10);
  digitalWrite(6,HIGH);delay(1);
 }
 if(digitalRead(6)==HIGH) { // STOP: SDA sube con SCL alto.
  digitalWrite(6,LOW);digitalWrite(5,LOW);delayMicroseconds(10);
  digitalWrite(6,HIGH);delay(1);digitalWrite(5,HIGH);delay(1);
 }
 bool libre=digitalRead(5)==HIGH && digitalRead(6)==HIGH;
 diagnosticoInicio18.printf("Bus MAX: SDA=%d SCL=%d libre=%d\n",digitalRead(5),digitalRead(6),libre);
 pinMode(5,INPUT);pinMode(6,INPUT);return libre;
}
bool leerRegistroMAX(uint8_t reg,uint8_t &valor) {
 Wire.beginTransmission(0x57);Wire.write(reg);
 if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom((uint8_t)0x57,(uint8_t)1,true)!=1)return false;
 valor=Wire.read();return true;
}
bool iniciarMAXRobusto() {
 // One attempt per call; the caller retries later with a live UI, never 6x3 nested.
 for(uint8_t intento=1;intento<=1;intento++) {
  esperarInicializacionUI(400);
  liberarBusMAX();
  bool bus=Wire.begin(5,6,100000);Wire.setTimeOut(50);Wire.setClock(100000);
  Wire.beginTransmission(0x57);uint8_t ack=Wire.endTransmission();
  uint8_t id=0;bool leido=leerRegistroMAX(0xFF,id);
  diagnosticoInicio18.printf("Arranque MAX %u/1: bus=%d ACK=%u ID_leido=%d ID=0x%02X\n",intento,bus,ack,leido,id);
  if(!bus || ack!=0 || !leido || id!=0x15)continue;
  tickInicializandoUI();
  if(!sensor.begin(Wire,100000))continue;
  Wire.setTimeOut(50);tickInicializandoUI();
  sensor.setup(0x24,1,2,100,411,4096); // Incluye reset del sensor en SparkFun.
  tickInicializandoUI();
  sensor.setPulseAmplitudeRed(0x24);sensor.setPulseAmplitudeIR(0x24);
  Wire.setClock(400000);esperarInicializacionUI(20);
  uint8_t fifoCfg=0,modo=0,config=0;
  bool verificado=leerRegistroMAX(0x08,fifoCfg) && leerRegistroMAX(0x09,modo) && leerRegistroMAX(0x0A,config);
  bool promedioUno=(fifoCfg & 0xE0)==0x00; // SMP_AVE[2:0]=000 -> promedio de 1 muestra.
  diagnosticoInicio18.printf("Configuracion a 400 kHz: lectura=%d FIFO_CFG=0x%02X avg1=%d modo=0x%02X SPO2_CFG=0x%02X\n",
                verificado,fifoCfg,promedioUno,modo,config);
  if(!verificado || !promedioUno || modo!=0x03 || config!=0x27)continue;
  sensor.clearFIFO();diagnosticoInicio18.println("MAX listo: I2C 400 kHz, sampleAverage=1, sampleRate=100 Hz.");return true;
 }
 diagnosticoInicio18.println("MAX no inicio; se reintentara con UI activa. Revisar ACK/ID/SDA/SCL.");return false;
}

// Verificacion exclusiva del arranque en frio. Al cargar firmware el ESP32 se reinicia,
// pero el MAX30102 puede quedar alimentado durante todo el proceso de programacion.
// En algunas condiciones el primer flujo FIFO tarda o queda en un estado residual.
// En vez de esperar a la recuperacion periodica de 3 s del loop, comprobamos aqui que
// aparezcan muestras a 100 Hz y, si no aparecen, reconfiguramos inmediatamente el MAX.
// No cambia filtros, umbrales, contacto ni detector: solo garantiza que el FIFO este corriendo
// antes de abandonar la pantalla "Inicializando".
bool asegurarFlujoMAXInicial(){
 if(!maxOK)return false;
 for(uint8_t ronda=0;ronda<1;ronda++){
  sensor.clearFIFO();
  unsigned long t0=millis();
  while(millis()-t0<700){
   tickInicializandoUI();
   sensor.check();
   if(sensor.available()>0){
    uint16_t n=0;
    while(sensor.available()){sensor.nextSample();n++;}
    sensor.clearFIFO();
    diagnosticoInicio18.printf("MAX flujo inicial OK: ronda=%u muestras=%u\n",(unsigned)(ronda+1),(unsigned)n);
    return true;
   }
   delay(5);
  }
  // Return to the UI; retry on the next startup pass, not recursively here.
 }
 diagnosticoInicio18.println("MAX configurado pero sin flujo FIFO inicial.");
 return false;
}

// Un intento cada 3 s si no llegan muestras durante 2 s. Nunca por retirar el dedo.
void vigilarMAX() {
 unsigned long t=millis();
 if(!fase || (maxOK && datosVistos && t-ultimoDato<2000) || t-ultimoIntentoBus<3000)return;
 ultimoIntentoBus=t;recuperacionesBus++;maxOK=false;
 contactoCrudo=false;datosVistos=false;ultimoBPMVisible=0;instanteBPMVisible=0;reiniciarDetector();
 diagnosticoInicio18.println("RECUPERANDO MAX: interrupcion de datos; BPM no vigente.");
 liberarBusMAX();bool ok=Wire.begin(5,6,100000);Wire.setTimeOut(50);
 uint8_t id=0,fifoCfg=0,modo=0,cfg=0;
 ok=ok && leerRegistroMAX(0xFF,id) && id==0x15 && sensor.begin(Wire,100000);
 if(ok) {
  sensor.setup(0x24,1,2,100,411,4096);sensor.setPulseAmplitudeRed(0x24);sensor.setPulseAmplitudeIR(0x24);
  Wire.setClock(400000);
  ok=leerRegistroMAX(0x08,fifoCfg) && leerRegistroMAX(0x09,modo) && leerRegistroMAX(0x0A,cfg) &&
     ((fifoCfg & 0xE0)==0x00) && modo==3 && cfg==0x27;
  if(ok){sensor.clearFIFO();while(sensor.available())sensor.nextSample();}
 }
 maxOK=ok;reiniciosDatos++;ultimoDato=millis();
 diagnosticoInicio18.println(ok ? "MAX reconfigurado; esperando muestras nuevas." : "MAX sin respuesta; se reintentara.");
}
void enviarTraza() {
 if(!enviandoTraza)return;
 if(!trazas){Serial.println("TRAZA no disponible: buffer PSRAM no asignado.");enviandoTraza=false;return;}
 if(indiceEnvio>=nTraza){Serial.println("FIN_TRAZA");enviandoTraza=false;return;}
 const Traza &r=trazas[indiceEnvio];char linea[96];
 int n=snprintf(linea,sizeof(linea),"%lu,%lu,%.2f,%.2f,%u\n",(unsigned long)r.ms,(unsigned long)r.ir,r.filtrada,r.umbral,r.flags);
 if(n>0 && n<(int)sizeof(linea) && Serial.availableForWrite()>=n){Serial.write((const uint8_t*)linea,n);indiceEnvio++;}
}

// Las cinco referencias aportadas por el usuario tienen PA identica.
// NO se construye una regresion ni se presenta un valor constante como PA medida.
// Referencia documental propia de esta persona, no transferible a otra.
void cargarReferencias14(){
 const float anchos[5]={835.624f,872.561f,696.784f,806.445f,737.336f};
 const uint32_t ensayos[5]={1,2,3,4,6},tiempos[5]={29822,36491,67682,89917,29916};
 calN=5;for(int i=0;i<5;i++)cal10[i]={anchos[i],110,60,ensayos[i],tiempos[i]};
 modeloPA=false;paValida=false;
}
void iniciarContinuo14(){
 if(!listo || enviandoTraza || enviandoCrudo)return;
 resetPA15();guia=0;reiniciarExploratorio();reiniciarDetector();bpm8=EstadoBPM8();
 fase=2;inicio=millis();limpiarFase();inicio=millis();pausaEstadisticas=false;
 datosVistos=false;contactoCrudo=false;alarmaDatos=false;ultimoIntentoBus=millis();
 nTraza=indiceEnvio=0;rawN=rawEnvio=0;trazaOmitida=rawOmitidas=0;
 diagnosticoInicio18.println("INICIO CONTINUO AUTONOMO BPM/SpO2/IMU/LCD. PA por PC es opcional. Estimacion PA experimental NO validada.");
}
void setup(){
 Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
 // Positive timeout: avoid zero-timeout underflow in affected HWCDC revisions.
 Serial.setTxTimeoutMs(1);
#endif
 delay(250);
 WireIMU.begin(48,47,400000);WireIMU.setTimeOut(20);
 analogReadResolution(12);
 iniciarLCD();iniciarAhorroPantalla();dibujarInicializandoUI();
 inicializarBuffersGrandesPSRAM();tickInicializandoUI(true);
 ble18.begin(); // B19: B18 NUS + optional PA Notify; initialize before MAX acquisition starts.
 maxOK=iniciarMAXRobusto();tickInicializandoUI(true);
 imuOK=iniciarQMI8658();tickInicializandoUI(true);
 if(maxOK)maxOK=asegurarFlujoMAXInicial();
 tickInicializandoUI(true);
 listo=maxOK && imuOK;
 cargarReferencias14();
 diagnosticoInicio18.printf("PPG V15 MODELO EN PC: MAX=%s IMU=%s\n",maxOK?"OK":"ERROR",imuOK?"OK":"ERROR");
 diagnosticoInicio18.println("B19 BLE: protocolo B18 H/B/S/O/R/D intacto; PA binaria disponible en UUID ...0004.");
 diagnosticoInicio18.println("PA: LSTM Schrumpf et al. 2021, pesos Zenodo 5590603. No usa 110/60 como salida fija ni recalibra.");
 diagnosticoInicio18.println("BPM/SpO2/PRV/IMU/LCD intactos. PA puede viajar a Android sin bloquear adquisicion; BP15 USB historico queda opcional.");
 informarMemoriaUI();
 if(maxOK)sensor.clearFIFO();
 iniciarContinuo14();
 reiniciarPRV17(true);prv17ReiniciosVistos=reiniciosDatos;
 diagnosticoInicio18.println("B17 PRV exploratoria activa: SDNN/RMSSD/pNN50/CVNN 60s; patron ectopico solo sospecha PPG, no diagnostico.");
 // If a peripheral did not start, keep the animated startup screen and retry.
 if(!listo)return;
 pantallaInicializandoActiva=false;pasoInicializandoUI=0;
 interfazBaseDibujada=false;
 pantalla();
}
void loop(){
 actualizarAhorroPantalla();
 receivePA15();
 if(!listo){
  static uint32_t ultimoReintento=0;
  tickInicializandoUI();
  uint32_t t=millis();
  if((uint32_t)(t-ultimoReintento)>=3000){
   ultimoReintento=t;
   if(!maxOK){maxOK=iniciarMAXRobusto();if(maxOK)maxOK=asegurarFlujoMAXInicial();}
   tickInicializandoUI();
   if(!imuOK)imuOK=iniciarQMI8658(); // Same existing routine/settings; retry only on failure.
   listo=maxOK && imuOK;
   if(listo){
    sensor.clearFIFO();iniciarContinuo14();
    reiniciarPRV17(true);prv17ReiniciosVistos=reiniciosDatos;
    pantallaInicializandoActiva=false;pasoInicializandoUI=0;
    interfazBaseDibujada=false;pantalla();
   }
   ultimoReintento=millis();
  }
  ble18.publish();delay(1);return;
 }
 adquirir();enviarTraza();exportarCrudo();vigilarMAX();vigilarEstado8();
 unsigned long ahora=millis();
 if(ahora-ultimoIMU>=50){ultimoIMU=ahora;lecturaOK=leerQMI8658();if(lecturaOK)procesar();else if(fase)errores++;}
 adquirir();actualizarExploratorio();adquirir();tickPA15();tickPRV17();
 if(fase && ahora-ultimoInforme>=1000){
  ultimoInforme=ahora;
  // Diagnostico por USB opcional: la medicion no depende de que exista monitor serie.
  if(Serial){informarEstado8();adquirir();
   diagnosticoInicio18.printf("V15: IR=%lu contacto=%d SpO2=%.1f PA_IA_S=%.1f PA_IA_D=%.1f edad_ventana_ms=%lu muestras=%lu perdidas=%lu\n",(unsigned long)ir,datosFrescos()&&contactoCrudo,oxVisible()?oxValor:NAN,visiblePA15()?bpS:NAN,visiblePA15()?bpD:NAN,(unsigned long)(millis()-bpResultCapture),(unsigned long)nPPG,(unsigned long)perdidas);
  }
 }
 // UI principal: 5 Hz y solo redibuja si cambia un estado.
 if(millis()-ultimoLCD>=UI_ESTADO_MS){adquirir();ultimoLCD=millis();pantalla();adquirir();}
 // Animacion del corazon separada: el tick es barato; solo hay SPI al cambiar normal<->grande.
 if(ahora-ultimoAnimUI>=UI_ANIM_TICK_MS){ultimoAnimUI=ahora;animarCorazonUI();adquirir();}
 ble18.publish(); // B19: B18 snapshot + PA queue, AFTER original processing/UI; zero queue wait.
}
