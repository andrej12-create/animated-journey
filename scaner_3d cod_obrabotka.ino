

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <GyverOLED.h>
#include <cmath>

// ─────────────────────────────────────────────
//  ПИНЫ
// ─────────────────────────────────────────────
#define PIN_SENSOR       15   // GP2Y0A41SK0F

#define PIN_OLED_SDA      8
#define PIN_OLED_SCL     18

#define PIN_SD_MISO       7
#define PIN_SD_MOSI       6
#define PIN_SD_SCK        5
#define PIN_SD_CS         4

#define PIN_Z_DIR        11
#define PIN_Z_STEP       10
#define PIN_Z_EN          9

#define PIN_T_DIR        14
#define PIN_T_STEP       13
#define PIN_T_EN         12

#define PIN_BOOT          0   // Boot кнопка

// ─────────────────────────────────────────────
//  МЕХАНИКА
// ─────────────────────────────────────────────
#define MICROSTEPS           8      // 1/8 микрошаг
#define MOTOR_STEPS_REV    250      // полных шагов/оборот
#define STEPS_PER_REV     1600      // 200 x 8

#define STEPS_PER_MM      1600      // Т8 шаг 1мм/об -> 1600 шагов/мм

#define POINTS_PER_LAYER   200      // точек на слой 
#define STEPS_PER_POINT      8      // 1600 / 200

// Направления
#define DIR_TABLE_CW      HIGH
#define DIR_TABLE_CCW     LOW
#define DIR_Z_UP          HIGH
#define DIR_Z_DOWN        LOW

// Скорости
#define SPEED_Z_US        1200
#define SPEED_T_US        1500
#define SPEED_SCAN_US     4000
#define HOMING_SPEED_US   1500

// Расстояние от датчика до центра стола
#define TABLE_CENTER_MM   110.0f

// ── Хоминг Z ──────────────────────────────────────────────────────────────
#define HOMING_LOST_COUNT       2
#define HOMING_MAX_MM           100
#define HOMING_LIFT_MM          2
#define HOMING_STEPS_PER_SAMPLE 1

// ── Prescan ────────────────────────────────────────────────────────────────
#define PRESCAN_MAX_MM      250
#define PRESCAN_EMPTY_STOP    3

// ─────────────────────────────────────────────
//  ДАТЧИК GP2Y0A41SK0F
// ─────────────────────────────────────────────
#define SENSOR_SAMPLES   40         // выборок для медианного фильтра
#define SENSOR_A         2076.0f
#define SENSOR_B           11.0f

// ─────────────────────────────────────────────
//  OLED
// ─────────────────────────────────────────────
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;

// ─────────────────────────────────────────────
//  SD КАРТА
// ─────────────────────────────────────────────
bool   sdOk = false;
File   scanFile;
String scanFilename;

// ─────────────────────────────────────────────
//  СОСТОЯНИЕ СИСТЕМЫ
// ─────────────────────────────────────────────
enum ScanState {
  STATE_CHECK,     // проверка компонентов
  STATE_READY,     // готов
  STATE_PRESCAN,   // хоминг + определение высоты
  STATE_SCAN,      // основное сканирование
  STATE_DONE,      // завершено
  STATE_ERROR      // ошибка
};


volatile ScanState currentState    = STATE_CHECK;
volatile long      zStepsCurrent   = 0;
volatile long      tStepsCurrent   = 0;
volatile bool      cancelRequested = false;

int  totalLayers     = 0;
int  currentLayer    = 0;
bool sensorOk        = false;
bool readyNeedsRedraw = true;

// ─────────────────────────────────────────────
//  ПРЕРЫВАНИЯ
// ─────────────────────────────────────────────
volatile bool          bootPressed   = false;
volatile unsigned long bootPressTime = 0;
volatile bool          endstopTriggered = false;

void IRAM_ATTR bootISR() {
  unsigned long now = millis();
  
  // Антидребезг 500 мс
  if (now - bootPressTime < 500) return;
  
  delayMicroseconds(1000);
  if (digitalRead(PIN_BOOT) != LOW) return;
  
  bootPressTime = now;
  bootPressed   = true;
  
  if (currentState == STATE_SCAN || currentState == STATE_PRESCAN) {
    cancelRequested = true;
  }
}

void IRAM_ATTR endstopISR() {
  endstopTriggered = true;
  if (currentState == STATE_SCAN || currentState == STATE_PRESCAN) {
    cancelRequested = true;
  }
}

// ─────────────────────────────────────────────
//  ДАТЧИК — функции
// ─────────────────────────────────────────────

uint16_t readADCRaw() {
  return (uint16_t)analogRead(PIN_SENSOR);
}

// Медианный фильтр из SENSOR_SAMPLES выборок
uint16_t readMedianADC() {
  uint16_t s[SENSOR_SAMPLES];
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    s[i] = readADCRaw();
    delayMicroseconds(200);
  }
  // Сортировка вставкой
  for (int i = 1; i < SENSOR_SAMPLES; i++) {
    uint16_t key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  return s[SENSOR_SAMPLES / 2];
}

// ADC raw -> расстояние мм. Возвращает -1 если вне диапазона.
float adcToDistance(uint16_t raw) {
  if (raw < 300)  return -1.0f;
  if (raw > 3700) return -1.0f;

  float d = 154700.0f / ((float)raw + 20.0f);
  
  if (d < 40.0f || d > 300.0f) return -1.0f;
  return d;
}

float getDistance() {
  return adcToDistance(readMedianADC());
}

bool checkSensor() {
  int lowCnt = 0, highCnt = 0;
  for (int i = 0; i < 10; i++) {
    uint16_t r = readADCRaw();
    if (r < 50)   lowCnt++;
    if (r > 4050) highCnt++;
    delay(10);
  }
  return (lowCnt < 8 && highCnt < 8);
}

// ─────────────────────────────────────────────
//  ДИСПЛЕЙ — прогресс-бар
// ─────────────────────────────────────────────

void drawProgressBar(int row, int percent) {
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;

  int y0   = row * 8;
  int y1   = y0 + 7;
  int fill = (int)((percent / 100.0f) * 123); // 123 пикс внутри рамки
  if (fill > 123) fill = 123;

  // Рамка
  oled.fastLineH(y0,   0, 127);
  oled.fastLineH(y1,   0, 127);
  oled.fastLineV(  0, y0, y1);
  oled.fastLineV(127, y0, y1);

  oled.rect(2, y0 + 1, 124, y1 - 1, 0);

  if (fill > 0) {
    oled.rect(2, y0 + 1, 2 + fill, y1 - 1, 1);
  }

  oled.update();
}

// ─────────────────────────────────────────────
//  МОТОРЫ
// ─────────────────────────────────────────────

inline void stepMotor(uint8_t stepPin, uint32_t delayUs) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayUs);
}

void enableMotors(bool en) {
  digitalWrite(PIN_Z_EN, en ? LOW : HIGH);
  digitalWrite(PIN_T_EN, en ? LOW : HIGH);
}

bool moveZ(long steps, bool goUp, uint32_t speedUs) {
  digitalWrite(PIN_Z_DIR, goUp ? DIR_Z_UP : DIR_Z_DOWN);
  delayMicroseconds(10);
  for (long i = 0; i < steps; i++) {
    if (cancelRequested) return false;
    stepMotor(PIN_Z_STEP, speedUs);
    zStepsCurrent += goUp ? 1 : -1;
  }
  return true;
}

void moveTable(long steps, bool goForward, uint32_t speedUs) {
  digitalWrite(PIN_T_DIR, goForward ? DIR_TABLE_CW : DIR_TABLE_CCW);
  delayMicroseconds(10);
  for (long i = 0; i < steps; i++) {
    if (cancelRequested) return;
    stepMotor(PIN_T_STEP, speedUs);
    tStepsCurrent += goForward ? 1 : -1;
  }
}

void zStepUp() {
  moveZ(STEPS_PER_MM, true, SPEED_Z_US);
}

void returnTableToZero() {
  if (tStepsCurrent == 0) return;
  bool  forward    = (tStepsCurrent < 0);
  long  stepsBack  = abs(tStepsCurrent);
  bool  savedCancel = cancelRequested;
  cancelRequested   = false;
  moveTable(stepsBack, forward, SPEED_T_US);
  cancelRequested   = savedCancel;
  tStepsCurrent     = 0;
}

void returnZToZero() {
  if (zStepsCurrent == 0) return;
  bool  goUp       = (zStepsCurrent < 0);
  long  stepsBack  = abs(zStepsCurrent);
  bool  savedCancel = cancelRequested;
  cancelRequested   = false;
  moveZ(stepsBack, goUp, SPEED_Z_US);
  cancelRequested   = savedCancel;
  zStepsCurrent     = 0;
}

// ─────────────────────────────────────────────
//  ХОМИНГ Z — поиск нулевой точки
// ─────────────────────────────────────────────
bool homeZ() {
  oled.clear();
  oled.setCursor(0, 0); oled.print("Калибровка Z...");
  oled.update();

  endstopTriggered = false;
  long stepsDone   = 0;
  long maxSteps    = (long)HOMING_MAX_MM * STEPS_PER_MM;

  enum HomingPhase { PHASE_SEEK, PHASE_TRACK, PHASE_FOUND } phase = PHASE_SEEK;
  int  lostCount  = 0;
  bool objectSeen = false;

  digitalWrite(PIN_Z_DIR, DIR_Z_DOWN);
  delayMicroseconds(10);

  while (stepsDone < maxSteps) {
    
    if (cancelRequested) {
      cancelRequested = false;
      moveZ(stepsDone, true, SPEED_Z_US);
      zStepsCurrent = 0;
      return false;
    }

    if (phase == PHASE_FOUND) goto homing_done;

    for (int s = 0; s < HOMING_STEPS_PER_SAMPLE; s++) {
      stepMotor(PIN_Z_STEP, HOMING_SPEED_US);
      zStepsCurrent--;
      stepsDone++;
    }

    {
      float d       = getDistance();
      bool  inRange = (d > 70.0f && d < TABLE_CENTER_MM);

      if (stepsDone % (STEPS_PER_MM / 2) == 0) {
        oled.setCursor(0, 1);
        oled.print(phase == PHASE_SEEK ? "Ищу предмет...  "
                                       : "Слежу за низом  ");
        oled.setCursor(0, 2);
        oled.print("Опуск: ");
        oled.print((int)(stepsDone / STEPS_PER_MM));
        oled.print(" мм    ");
        oled.setCursor(0, 3);
        if (inRange) {
          oled.print("Датчик: ");
          oled.print((int)d);
          oled.print(" мм   ");
        } else {
          oled.print("Датчик: ---    ");
        }
        oled.update();
      }

      switch (phase) {
        case PHASE_SEEK:
          if (inRange) { phase = PHASE_TRACK; objectSeen = true; lostCount = 0; }
          break;
        case PHASE_TRACK:
          if (inRange)  { lostCount = 0; }
          else          { if (++lostCount >= HOMING_LOST_COUNT) phase = PHASE_FOUND; }
          break;
        case PHASE_FOUND:
          break;
      }
    }
  }

  oled.clear();
  oled.setCursor(0, 0);
  oled.print(objectSeen ? "Низ не найден!" : "Предмет не найден");
  oled.setCursor(0, 1);
  oled.print(objectSeen ? "Проверь датчик" : "Поставь предмет");
  oled.update();
  moveZ(stepsDone, true, SPEED_Z_US);
  zStepsCurrent = 0;
  delay(2500);
  return false;

homing_done:
  oled.setCursor(0, 4); oled.print("Край найден!    ");
  oled.setCursor(0, 5); oled.print("Установка нуля..");
  oled.update();

  moveZ((long)HOMING_LIFT_MM * STEPS_PER_MM, true, HOMING_SPEED_US);
  zStepsCurrent = 0;

  oled.setCursor(0, 6); oled.print("Ноль Z = OK     ");
  oled.update();
  delay(800);
  return true;
}

// ─────────────────────────────────────────────
//  PRESCAN — определение высоты объекта
// ─────────────────────────────────────────────
int prescan() {
  int lastLayerWithObject = 0;
  int emptyAfterObject    = 0;

  oled.clear();
  oled.setCursor(0, 0); oled.print("Prescan...");
  oled.setCursor(0, 1); oled.print("Опред. высоты");
  oled.update();

  const int  pointsInLayer = 20;
  const long stepsPerPt    = STEPS_PER_REV / pointsInLayer;

  for (int mm = 0; mm < PRESCAN_MAX_MM; mm++) {
    if (cancelRequested) return 0;

    bool objectFound = false;
    tStepsCurrent    = 0;

    for (int p = 0; p < pointsInLayer; p++) {
      if (cancelRequested) return 0;
      moveTable(stepsPerPt, true, SPEED_T_US);
      float d = getDistance();
      if (d > 0 && d < TABLE_CENTER_MM) objectFound = true;
    }
    returnTableToZero();

    if (objectFound) {
      lastLayerWithObject = mm + 1;
      emptyAfterObject    = 0;
    } else if (lastLayerWithObject > 0) {

      emptyAfterObject++;
      if (emptyAfterObject >= PRESCAN_EMPTY_STOP) {
        break;
      }
    }

    oled.setCursor(0, 2);
    oled.print("Слой: ");
    oled.print(mm + 1);
    oled.print("/");
    oled.print(PRESCAN_MAX_MM);
    oled.print("   ");
    drawProgressBar(4, (mm + 1) * 100 / PRESCAN_MAX_MM);

    zStepUp();
    if (cancelRequested) return 0;
  }

  returnZToZero();
  return lastLayerWithObject;
}

// ─────────────────────────────────────────────
//  SD — создать файл
// ─────────────────────────────────────────────
bool createScanFile() {
  for (int i = 1; i <= 999; i++) {
    char name[16];
    snprintf(name, sizeof(name), "/SCAN_%03d.CSV", i);
    if (!SD.exists(name)) {
      scanFile = SD.open(name, FILE_WRITE);
      if (scanFile) {
        scanFilename = String(name);
        scanFile.println("angle_deg,layer,distance_mm");
        scanFile.flush();
        return true;
      }
    }
  }
  return false;
}

// ─────────────────────────────────────────────
//  ОСНОВНОЕ СКАНИРОВАНИЕ
// ─────────────────────────────────────────────
void doScan(int layers) {
  struct ScanPoint { float angle; float distance; };
  static ScanPoint layerBuf[POINTS_PER_LAYER];

  currentLayer    = 0;
  cancelRequested = false;

  if (!createScanFile()) {
    oled.clear();
    oled.setCursor(0, 0); oled.print("Ошибка SD!");
    oled.update();
    delay(3000);
    currentState = STATE_ERROR;
    return;
  }

  long totalPoints = (long)layers * POINTS_PER_LAYER;
  long donePoints  = 0;

  for (int layer = 0; layer < layers; layer++) {
    currentLayer  = layer;
    
    float prevDist = -1.0f;
    for (int pt = 0; pt < POINTS_PER_LAYER; pt++) {
      if (cancelRequested) goto cancel;

      moveTable(STEPS_PER_POINT, true, SPEED_SCAN_US);

      long angleSteps = tStepsCurrent % STEPS_PER_REV;
      if (angleSteps < 0) angleSteps += STEPS_PER_REV;
      layerBuf[pt].angle = (float)angleSteps * 360.0f / STEPS_PER_REV;
      float currentDist = getDistance();
      
      if (prevDist > 0.0f && currentDist > 0.0f && fabs(currentDist - prevDist) > 8.0f) {
        float d2 = getDistance();
        float d3 = getDistance();
        float arr[3] = {currentDist, d2, d3};
        for (int i = 1; i < 3; i++) {
          float key = arr[i];
          int j = i - 1;
          while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
          arr[j + 1] = key;
        }
        currentDist = arr[1];
      }
      layerBuf[pt].distance = currentDist;
      prevDist = currentDist;
      donePoints++;

      if (pt % 10 == 0) {
        int pct = (int)(donePoints * 100L / totalPoints);
        oled.setCursor(0, 0); oled.print("Сканирование     ");
        oled.setCursor(0, 1);
        oled.print("Слой: "); oled.print(layer + 1); oled.print("/"); oled.print(layers); oled.print("   ");
        oled.setCursor(0, 2);
        oled.print("Точка: "); oled.print(pt + 1); oled.print("/200  ");
        oled.setCursor(0, 3);
        oled.print("Прогресс: "); oled.print(pct); oled.print("%   ");
        drawProgressBar(5, pct);
      }
    }
    if (cancelRequested) goto cancel;
    delay(500);
    returnTableToZero();

    for (int pt = 0; pt < POINTS_PER_LAYER; pt++) {
      if (layerBuf[pt].distance > 0) {
        scanFile.print(layerBuf[pt].angle, 2);   scanFile.print(",");
        scanFile.print(layer + 1);                scanFile.print(",");
        scanFile.println(layerBuf[pt].distance, 2);
      }   
      else {
        scanFile.print(layerBuf[pt].angle, 2);     scanFile.print(",");
        scanFile.print(layer + 1);                scanFile.println(",-1");
      }
    }
    
      
    
    scanFile.flush();

    if (cancelRequested) goto cancel;

    if (layer < layers - 1) {
      zStepUp();
    }
    if (cancelRequested) goto cancel;
  }

  scanFile.close();
  returnZToZero();
  returnTableToZero();
  enableMotors(false);

  oled.clear();
  oled.setCursor(0, 0); oled.print("Сканирование");
  oled.setCursor(0, 1); oled.print("завершено!");
  oled.setCursor(0, 2); oled.print("Файл: "); oled.print(scanFilename);
  oled.setCursor(0, 3); oled.print("Слоев: "); oled.print(layers);
  oled.setCursor(0, 4); oled.print("Точек: "); oled.print(layers * POINTS_PER_LAYER);
  oled.setCursor(0, 6); oled.print("Boot = в начало");
  oled.update();
  currentState = STATE_DONE;
  return;

cancel:
  scanFile.flush();
  scanFile.close();

  oled.clear();
  oled.setCursor(0, 0); oled.print("Отмена!");
  oled.setCursor(0, 1); oled.print("Возврат в 0...");
  oled.update();

  returnTableToZero();
  returnZToZero();

  oled.setCursor(0, 2); oled.print("Готово.");
  oled.update();

  enableMotors(false);
  cancelRequested  = false;
  readyNeedsRedraw = true;
  currentState     = STATE_READY;
}

// ─────────────────────────────────────────────
//  ДИАГНОСТИКА КОМПОНЕНТОВ
// ─────────────────────────────────────────────
void checkComponents() {
  oled.clear();
  oled.setCursor(0, 0); oled.print("Проверка...");
  oled.update();

  sensorOk = checkSensor();
  oled.setCursor(0, 1); oled.print("Датчик: "); oled.print(sensorOk ? "OK" : "ОШИБКА");
  oled.update();

  sdOk = SD.begin(PIN_SD_CS);
  oled.setCursor(0, 2); oled.print("SD карта: "); oled.print(sdOk ? "OK" : "ОШИБКА");
  oled.update();

  endstopTriggered = false;
  digitalWrite(PIN_Z_STEP, LOW);
  digitalWrite(PIN_T_STEP, LOW);
  enableMotors(true);
  delay(50);
  enableMotors(false);
  endstopTriggered = false;
  oled.setCursor(0, 3); oled.print("Моторы: OK");
  oled.update();

  delay(1500);

  if (sensorOk && sdOk) {
    readyNeedsRedraw = true;
    currentState     = STATE_READY;
  } else {
    currentState = STATE_ERROR;
  }
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_Z_DIR,  OUTPUT);
  pinMode(PIN_Z_STEP, OUTPUT);
  pinMode(PIN_Z_EN,   OUTPUT);
  pinMode(PIN_T_DIR,  OUTPUT);
  pinMode(PIN_T_STEP, OUTPUT);
  pinMode(PIN_T_EN,   OUTPUT);

  digitalWrite(PIN_Z_STEP, LOW);
  digitalWrite(PIN_T_STEP, LOW);
  digitalWrite(PIN_Z_DIR,  LOW);
  digitalWrite(PIN_T_DIR,  LOW);
  enableMotors(false);

  pinMode(PIN_BOOT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOOT), bootISR, FALLING);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  oled.init();
  oled.clear();
  oled.setScale(1);
  oled.home();
  oled.update();

  currentState = STATE_CHECK;
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  switch (currentState) {
    
    case STATE_CHECK:
      checkComponents();
      bootPressed = false;
      break;
    case STATE_READY:
      {
        if (readyNeedsRedraw) {
          readyNeedsRedraw = false;
          oled.clear();
          oled.setCursor(0, 0); oled.print("=== 3D SCANNER ===");
          if (sensorOk && sdOk) {
            oled.setCursor(0, 1); oled.print("Все системы OK");
            oled.setCursor(0, 3); oled.print("Нажми Boot");
            oled.setCursor(0, 4); oled.print("для сканирования");
          } else {
            oled.setCursor(0, 1); oled.print("Ошибки:");
            int row = 2;
            if (!sensorOk) { oled.setCursor(0, row++); oled.print("- Датчик GP2Y"); }
            if (!sdOk)      { oled.setCursor(0, row++); oled.print("- SD карта"); }
            oled.setCursor(0, row);     oled.print("Исправь и");
            oled.setCursor(0, row + 1); oled.print("перезапусти.");
          }
          oled.update();
        }

        if (bootPressed) {
          bootPressed = false;
          if (sensorOk && sdOk) {
            readyNeedsRedraw = true;
            currentState     = STATE_PRESCAN;
          }
        }
      }
      break;
    
    case STATE_PRESCAN:
      {
        enableMotors(true);
        cancelRequested  = false;
        bootPressed      = false;
        endstopTriggered = false;
        tStepsCurrent    = 0;

        // Шаг 1: ищем ноль Z
        bool homingOk = homeZ();

        if (!homingOk || cancelRequested) {
          cancelRequested  = false;
          endstopTriggered = false;
          bootPressed      = false;
          returnZToZero();
          enableMotors(false);
          currentState = STATE_ERROR;
          break;
        }

        // Шаг 2: определяем высоту объекта
        totalLayers = prescan();

        if (cancelRequested || totalLayers == 0) {
          cancelRequested  = false;
          bootPressed      = false;
          returnZToZero();
          returnTableToZero();
          enableMotors(false);
          readyNeedsRedraw = true;
          currentState     = STATE_READY;
          break;
        }

        // Показываем результат и ждём подтверждения
        // Показываем результат и сразу начинаем сканирование
        oled.clear();
        oled.setCursor(0, 0); oled.print("Высота объекта:");
        oled.setCursor(0, 1);
        oled.print(totalLayers); oled.print(" мм / ");
        oled.print(totalLayers); oled.print(" слоев");
        oled.setCursor(0, 3); oled.print("Сканирую...");
        oled.update();

        delay(1000);

        cancelRequested = false;
        currentState = STATE_SCAN;
      }
      break;

    // Основное сканирование
    case STATE_SCAN:
      doScan(totalLayers);
      bootPressed     = false;
      cancelRequested = false;
      break;

    // Завершено
    case STATE_DONE:
      if (bootPressed) {
        bootPressed  = false;
        currentState = STATE_CHECK;
      }
      break;

    // Ошибка
    case STATE_ERROR:
      {
        static bool errorDrawn = false;
        if (!errorDrawn) {
          errorDrawn = true;
          oled.clear();
          oled.setCursor(0, 0); oled.print("ОШИБКА");
          int row = 1;
          if (!sensorOk) { oled.setCursor(0, row++); oled.print("- Датчик GP2Y"); }
          if (!sdOk)      { oled.setCursor(0, row++); oled.print("- SD карта"); }
          oled.setCursor(0, row + 1); oled.print("Boot = повторить");
          oled.update();
        }
        if (bootPressed) {
          bootPressed  = false;
          errorDrawn   = false;
          currentState = STATE_CHECK;
        }
      }
      break;
  }

  delay(10);
}
