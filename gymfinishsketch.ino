// smart_rep_counter_final_fixed_v12_amplitude_noise_filter.ino
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MPU9250_asukiaaa.h>

// ============================================
// НАСТРОЙКИ
// ============================================
const char* ap_ssid = "SmartRepCounter";
const char* ap_password = "12345678";

// Аппаратные настройки
#define BUTTON_CALIBRATE 26 // Калибровка
#define BUTTON_RESET 25     // Рестарт (сброс репов)
#define BUTTON_START 14     // Тренировка
#define BUTTON_RESTART 27   // Перезагрузка (физическая кнопка)

// === SENSITIVITY SETTINGS ===
const float MOVEMENT_THRESHOLD = 0.20; // Порог для начала *любого* движения (увеличено для устойчивости к шуму)
const float MIN_AMPLITUDE_RATIO = 0.50; // Мин. амплитуда как % от эталона
const float RETURN_THRESHOLD = 0.15; // Порог для возврата к базе
const unsigned long REP_DEBOUNCE_MS = 500; // Мин. время между репами (мс)
const unsigned long PHASE_TIMEOUT_MS = 2000; // Время (мс) на завершение фазы, иначе сброс
const unsigned long NOISE_FILTER_WINDOW_MS = 150; // Время (мс) для фильтрации кратковременного шума/удара
const unsigned long PAUSE_BETWEEN_REPS_THRESHOLD_MS = 15000; // Порог паузы (мс) между репами для вывода сообщения

// Глобальные переменные
MPU9250_asukiaaa mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
WebServer server(80);

int repCount = 0;
int targetReps = 10;
bool isCalibrating = false;
bool isExercising = false;

// Переменные для истории
int history[20] = {0};
int historyCount = 0;
int calibrationReps = 0;
int approachNumber = 0; // Счётчик подходов (0 = не начат, 1 = калибровка, 2+ = тренировки)

// Reference repetition structure
struct ReferenceRep {
  float firstAmplitude; // Амплитуда первого эталонного повторения
  float minAmplitude;   // Минимально допустимая амплитуда
};

ReferenceRep reference;

// Repetition detection variables (только Z-ось)
float baselineAccel = 0; // Базовая Z-оси
float currentMaxAccel = 0; // Макс. Z во время текущего движения (за цикл)
float currentMinAccel = 0; // Мин. Z во время текущего движения (за цикл)
unsigned long lastRepTime = 0; // Время последнего засчитанного репа
unsigned long phaseStartTime = 0; // Время начала текущей фазы (UP/DOWN)
// Переменные для фильтрации шума
unsigned long noiseStartTime = 0; // Время начала потенциального шума
bool inNoiseWindow = false; // Находимся ли мы в окне фильтрации шума
float noiseStartAccel = 0; // Ускорение в момент начала окна шума

// Переменные для отслеживания паузы между репами и времени подхода
unsigned long lastValidRepTime = 0; // Время последнего *действительного* (с хорошим качеством) репа
unsigned long approachStartTime = 0; // Время начала текущего подхода (калибровки или тренировки)
String adviceMessage = ""; // Сообщение для пользователя
bool pauseAdvisoryGiven = false; // Флаг, чтобы не дублировать сообщение о паузе

// ★ УЛУЧШЕННЫЕ ПЕРЕМЕННЫЕ ДЛЯ ЛОГИКИ (двухфазная + шум)
enum MovementState {
  IDLE,           // Ожидание начала движения или возврат из шума
  PHASE_UP,       // Фаза подъёма (движение от базы к пиковой амплитуде)
  PHASE_DOWN      // Фаза спуска (движение от пиковой амплитуды к базе)
};

MovementState currentMovementState = IDLE;

// Переменные для хранения качества последнего репа
float lastRepQualityPercent = 0.0f;     // Процент качества последнего репа

// Переменные для хранения данных завершённого подхода
struct CompletedApproach {
  int number; // Номер подхода
  int reps;   // Количество репов в подходе
  float quality; // Среднее качество повторений в подходе (или качество эталона для калибровки)
  float durationSecs; // Длительность подхода в секундах
  String advice; // Совет для подхода
};

CompletedApproach completedApproaches[10];
int completedApproachesCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== SMART REP COUNTER INIT (Fixed Detection V12 - Amplitude + Noise Filter) ===");
  
  // Инициализация оборудования
  setupHardware();
  
  // Запуск WiFi точки доступа
  setupWiFi();
  
  // Настройка веб-сервера
  setupWebServer();
  
  Serial.println("=== SYSTEM READY ===");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Connect to: " + String(ap_ssid));
  Serial.println("Open browser: http://192.168.4.1");
}

void setupHardware() {
  Serial.println("Initializing hardware...");
  
  // Инициализация I2C
  Wire.begin();
  
  // Инициализация дисплея
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display init failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("REP");
    display.println("COUNTER");
    display.display();
    delay(1000);
  }
  
  // Инициализация кнопок
  pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
  pinMode(BUTTON_RESET, INPUT_PULLUP); // Рестарт (сброс репов)
  pinMode(BUTTON_START, INPUT_PULLUP); // Тренировка
  pinMode(BUTTON_RESTART, INPUT_PULLUP); // Перезагрузка (физическая кнопка)
  
  // Инициализация MPU
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  
  uint8_t sensorId;
  if (mpu.readId(&sensorId) == 0) {
    Serial.print("MPU9250: OK, ID: 0x"); Serial.println(sensorId, HEX);
  } else {
    Serial.println("MPU9250: Not found!");
    while(1) delay(10); // Остановить выполнение
  }
  
  Serial.println("Hardware initialized");
  
  // Установка базовой акселерации
  setBaselineAcceleration();
  resetReference();
}

void setBaselineAcceleration() {
  float sum = 0;
  int validReadings = 0;
  
  Serial.println("Calculating baseline Z-accel...");
  
  for(int i = 0; i < 100; i++) {
    if(mpu.accelUpdate() == 0) {
      float currentAccel = mpu.accelZ();
      sum += currentAccel;
      validReadings++;
    }
    delay(5); // Уменьшено для более быстрой калибровки
  }
  
  if(validReadings > 0) {
    baselineAccel = sum / validReadings;
    Serial.print("Baseline Z-accel: ");
    Serial.println(baselineAccel, 3);
  } else {
    Serial.println("Error reading baseline Z-accel, using default!");
    baselineAccel = 9.81; // Значение по умолчанию, если ошибка
  }
}

void resetReference() {
  reference.firstAmplitude = 0;
  reference.minAmplitude = 0;
  Serial.println("Reference values reset");
  // Сбросить качество последнего репа
  lastRepQualityPercent = 0.0f;
}

void setupWiFi() {
  Serial.println("Starting Access Point...");
  
  WiFi.disconnect(true);
  delay(100);
  
  if (!WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("AP Failed!");
    return;
  }
  
  delay(2000);
  
  Serial.println("AP Started");
  Serial.print("SSID: ");
  Serial.println(ap_ssid);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    String html = R"=====(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Rep Counter</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #6a11cb 0%, #2575fc 100%);
            color: #fff;
            margin: 0;
            padding: 20px;
            min-height: 100vh;
        }
        .container {
            max-width: 900px;
            margin: auto;
            background: rgba(0, 0, 0, 0.3);
            padding: 30px;
            border-radius: 15px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.2);
        }
        h1 {
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
        }
        .stats {
            display: flex;
            justify-content: space-around;
            margin-bottom: 30px;
        }
        .stat-box {
            background: rgba(255, 255, 255, 0.1);
            padding: 20px;
            border-radius: 10px;
            text-align: center;
            width: 30%;
        }
        .stat-value {
            font-size: 2.5em;
            font-weight: bold;
        }
        .controls {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 20px;
            margin-bottom: 30px;
        }
        .target {
            display: flex;
            align-items: center;
            gap: 15px;
            font-size: 1.5em;
        }
        .target button {
            background: #4CAF50;
            color: white;
            border: none;
            width: 40px;
            height: 40px;
            border-radius: 50%;
            font-size: 1.5em;
            cursor: pointer;
        }
        .target button:hover {
            background: #45a049;
        }
        .btn {
            padding: 12px 30px;
            font-size: 1.2em;
            border: none;
            border-radius: 50px;
            cursor: pointer;
            transition: all 0.3s;
        }
        .btn-calibrate {
            background: #9C27B0;
            color: white;
        }
        .btn-workout {
            background: #2196F3;
            color: white;
        }
        .btn-restart {
            background: #FF5722;
            color: white;
        }
        .btn-reboot {
            background: #9C27B0;
            color: white;
        }
        .btn:hover {
            transform: scale(1.05);
        }
        .quality-bar {
            width: 100%;
            height: 30px;
            background: #444;
            border-radius: 15px;
            margin: 10px 0;
            overflow: hidden;
        }
        .quality-fill {
            height: 100%;
            width: 0%;
            transition: width 0.3s, background-color 0.3s;
        }
        .quality-excellent {
            background-color: #4CAF50;
        }
        .quality-good {
            background-color: #FFEB3B;
        }
        .quality-poor {
            background-color: #f44336;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 20px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            overflow: hidden;
        }
        th, td {
            padding: 15px;
            text-align: center;
            border-bottom: 1px solid rgba(255,255,255,0.1);
        }
        th {
            background: rgba(0,0,0,0.2);
        }
        .calibration-row {
            background-color: rgba(0, 255, 0, 0.2);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Smart Rep Counter</h1>
        
        <div class="stats">
            <div class="stat-box">
                <div>Повторения</div>
                <div class="stat-value" id="reps">0</div>
            </div>
            <div class="stat-box">
                <div>Статус</div>
                <div class="stat-value" id="status">READY</div>
            </div>
            <div class="stat-box">
                <div>Цель</div>
                <div class="stat-value" id="target">10</div>
            </div>
        </div>

        <div class="controls">
            <div class="target">
                <button onclick="changeTarget(-1)">-</button>
                <span>Цель: <span id="targetValue">10</span></span>
                <button onclick="changeTarget(1)">+</button>
            </div>
            <div>
                <button class="btn btn-calibrate" onclick="sendCmd('calibrate')">КАЛИБРОВКА</button>
                <button class="btn btn-workout" onclick="sendCmd('workout')">ТРЕНИРОВКА</button>
                <button class="btn btn-restart" onclick="sendCmd('restart')">РЕСТАРТ</button>
                <button class="btn btn-reboot" onclick="sendCmd('reboot')">ПЕРЕЗАГРУЗКА</button>
            </div>
        </div>

        <h3>Качество последнего повторения</h3>
        <div class="quality-bar">
            <div class="quality-fill" id="qualityFill"></div>
        </div>
        <div id="qualityText">—</div>

        <h3>История Подходов</h3>
        <table id="historyTable">
            <thead>
                <tr>
                    <th>№ Подхода</th>
                    <th>Повторения</th>
                    <th>Время (с)</th>
                    <th>Совет</th>
                </tr>
            </thead>
            <tbody id="historyBody">
                <!-- Данные будут добавлены сюда -->
            </tbody>
        </table>
    </div>

    <script>
        let calibrationValue = 0;
        let lastApproachNumber = 0;

        function sendCmd(cmd) {
            fetch('/' + cmd).then(r => r.text());
        }

        function changeTarget(delta) {
            const newTarget = parseInt(document.getElementById('targetValue').textContent) + delta;
            if (newTarget > 0) {
                document.getElementById('targetValue').textContent = newTarget;
                fetch('/setTarget?value=' + newTarget).then(r => r.text());
            }
        }

        function update() {
            fetch('/data').then(r => r.json()).then(d => {
                document.getElementById('reps').textContent = d.reps; // Теперь отображаем реальный счётчик
                document.getElementById('status').textContent = d.status;
                document.getElementById('target').textContent = d.target;
                document.getElementById('targetValue').textContent = d.target;

                // Обновляем качество ПОСЛЕДНЕГО ПОВТОРЕНИЯ
                if(d.lastRepQuality) {
                    const quality = d.lastRepQuality.percent;
                    const percent = Math.min(quality, 150); // Ограничиваем 150%
                    const fill = document.getElementById('qualityFill');
                    fill.style.width = percent + '%';

                    if (percent >= 85) {
                        fill.className = 'quality-fill quality-excellent';
                    } else if (percent >= 70) {
                        fill.className = 'quality-fill quality-good';
                    } else {
                        fill.className = 'quality-fill quality-poor';
                    }

                    document.getElementById('qualityText').textContent = percent.toFixed(1) + '%';
                }

                // Обновляем историю подходов
                if(d.completedApproaches && d.completedApproaches.length > 0) {
                    const tbody = document.getElementById('historyBody');
                    tbody.innerHTML = ''; // Очистить перед обновлением
                    d.completedApproaches.forEach(approach => {
                        const newRow = document.createElement('tr');
                        let percent = "—";
                        let rowClass = "";

                        if (approach.number === 1) { // Калибровка
                            calibrationValue = approach.reps;
                            percent = "—"; // Прочерк для калибровки
                            rowClass = "calibration-row";
                        } else if (calibrationValue > 0) {
                            // Для тренировок показываем среднее качество подхода
                            percent = approach.quality.toFixed(1) + "%";
                        }

                        newRow.innerHTML = `
                            <td>${approach.number}</td>
                            <td>${approach.reps}</td>
                            <td>${approach.durationSecs.toFixed(1)}</td>
                            <td>${approach.advice}</td>
                        `;
                        if (rowClass) newRow.className = rowClass;
                        tbody.appendChild(newRow);
                    });
                }
            });
        }

        setInterval(update, 500); // Обновление каждые 500мс для более плавного UI
        update();
    </script>
</body>
</html>
)=====";

    server.send(200, "text/html", html);
  });

  server.on("/calibrate", HTTP_GET, []() {
    // Начать калибровку только если НЕ в режиме тренировки
    if (isExercising) {
        server.send(200, "text/plain", "Stop workout first!");
        return;
    }
    resetReference(); // Сбросить эталон перед калибровкой
    startCalibration();
    server.send(200, "text/plain", "Calibrating started");
  });

  server.on("/workout", HTTP_GET, []() {
    if(reference.firstAmplitude == 0) {
      server.send(200, "text/plain", "Calibrate first!");
      return;
    }
    startExercise();
    server.send(200, "text/plain", "Workout started");
  });

  server.on("/reset", HTTP_GET, []() {
    resetReps(); // Сбросить только счётчик повторов
    server.send(200, "text/plain", "Reps reset");
  });

  server.on("/restart", HTTP_GET, []() {
    server.send(200, "text/plain", "Rebooting...");
    delay(100);
    ESP.restart();
  });

  server.on("/setTarget", HTTP_GET, []() {
    if(server.hasArg("value")) {
      int newTarget = server.arg("value").toInt();
      if(newTarget > 0) targetReps = newTarget;
    }
    server.send(200, "text/plain", "Target updated");
  });

  server.on("/data", HTTP_GET, []() {
    String status = "READY";
    if(isCalibrating) status = "CALIBRATING";
    else if(isExercising) status = "EXERCISING";

    // Подготовка JSON с данными
    String json = "{";
    json += "\"reps\":" + String(repCount);
    json += ",\"target\":" + String(targetReps);
    json += ",\"status\":\"" + status + "\"";
    // Передаём качество последнего засчитанного повторения
    json += ",\"lastRepQuality\":{\"percent\":" + String(lastRepQualityPercent, 1) + "}";
    json += ",\"completedApproaches\":[";
    for (int i = 0; i < completedApproachesCount; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"number\":" + String(completedApproaches[i].number);
        json += ",\"reps\":" + String(completedApproaches[i].reps);
        json += ",\"quality\":" + String(completedApproaches[i].quality, 1);
        json += ",\"durationSecs\":" + String(completedApproaches[i].durationSecs, 1);
        json += ",\"advice\":\"" + completedApproaches[i].advice + "\"";
        json += "}";
    }
    json += "]";
    json += "}";

    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started on port 80");
}

void loop() {
  // Обрабатываем HTTP запросы
  server.handleClient();
  
  // Проверяем кнопки
  checkButtons();
  
  // Обновляем дисплей
  updateDisplay();
  
  // Логика подсчёта повторений
  if(isCalibrating) {
    calibrationRoutine();
    // Автоматически завершить калибровку после 6 репов
    if(repCount >= 6) {
        finishCalibration();
    }
  } else if(isExercising) {
    exerciseRoutine();
    // Автоматически завершить подход после достижения цели
    if(repCount >= targetReps) {
        finishApproach();
    }
  }
  
  delay(20); // Уменьшено для более быстрого реагирования
}

void checkButtons() {
  static unsigned long lastCheck = 0;
  if(millis() - lastCheck < 250) return; // Увеличенный дебаунс 250мс
  lastCheck = millis();
  
  if(digitalRead(BUTTON_CALIBRATE) == LOW) {
    Serial.println("Physical CALIBRATE button pressed");
    // Начать калибровку только если НЕ в режиме тренировки
    if (isExercising) {
        Serial.println("Can't calibrate during workout!");
        return;
    }
    resetReference(); // Сбросить эталон перед калибровкой
    startCalibration();
  }
  else if(digitalRead(BUTTON_RESET) == LOW) { // Используем как "Reset Reps"
    Serial.println("Physical RESET button pressed");
    resetReps();
  }
  else if(digitalRead(BUTTON_START) == LOW) { // Используем как "Start Workout" (только если эталон есть)
    Serial.println("Physical START button pressed");
    if(reference.firstAmplitude == 0) {
      Serial.println("Calibrate first!");
    } else {
      startExercise();
    }
  }
  else if(digitalRead(BUTTON_RESTART) == LOW) { // Используем как "Reboot"
    Serial.println("Physical RESTART (Reboot) button pressed");
    ESP.restart();
  }
}

void startCalibration() {
  if(isExercising) return; // Не начинать, если идёт тренировка
  
  Serial.println("=== CALIBRATION STARTED ===");
  isCalibrating = true;
  isExercising = false;
  repCount = 0;
  currentMovementState = IDLE; // Сбросить состояние
  inNoiseWindow = false; // Сбросить флаг шума
  lastRepQualityPercent = 0.0f; // Сбросить качество
  approachNumber = 1; // Калибровка - всегда подход #1
  lastValidRepTime = millis(); // Инициализировать время для отслеживания паузы
  approachStartTime = millis(); // Инициализировать время начала подхода
  pauseAdvisoryGiven = false; // Сбросить флаг совета
  adviceMessage = ""; // Сбросить сообщение
  Serial.println("Perform 6 perfect reps for reference.");
}

void startExercise() {
  if(isCalibrating || reference.firstAmplitude == 0) return; // Не начинать без калибровки
  
  Serial.println("=== EXERCISE STARTED ===");
  isExercising = true;
  isCalibrating = false;
  repCount = 0;
  currentMovementState = IDLE; // Сбросить состояние
  inNoiseWindow = false; // Сбросить флаг шума
  lastRepQualityPercent = 0.0f; // Сбросить качество
  // approachNumber уже должен быть 2 или больше после калибровки
  if (approachNumber == 0) approachNumber = 2; // Если вдруг не был установлен
  lastValidRepTime = millis(); // Инициализировать время для отслеживания паузы
  approachStartTime = millis(); // Инициализировать время начала подхода
  pauseAdvisoryGiven = false; // Сбросить флаг совета
  adviceMessage = ""; // Сбросить сообщение
  Serial.print("Exercise started. Target: "); Serial.println(targetReps);
}

void resetReps() {
  Serial.println("=== REPS RESET ===");
  repCount = 0;
  inNoiseWindow = false; // Сбросить флаг шума
  lastRepQualityPercent = 0.0f; // Сбросить качество при ручном рестарте
  currentMovementState = IDLE; // Сбросить состояние
  lastValidRepTime = millis(); // Обновить время последнего репа при сбросе
  pauseAdvisoryGiven = false; // Сбросить флаг совета при сбросе
  adviceMessage = ""; // Сбросить сообщение при сбросе
  // Не сбрасываем эталон, подходы, режимы
}

void resetAll() {
  Serial.println("=== ALL RESET ===");
  repCount = 0;
  currentMovementState = IDLE;
  inNoiseWindow = false; // Сбросить флаг шума
  resetReference();
  approachNumber = 0; // Сбросить счётчик подходов
  lastRepQualityPercent = 0.0f; // Сбросить качество
  isCalibrating = false;
  isExercising = false;
  // Очистить историю подходов
  completedApproachesCount = 0;
  for (int i = 0; i < 10; i++) {
      completedApproaches[i] = {0, 0, 0.0f, 0.0f, ""};
  }
  lastValidRepTime = millis(); // Инициализировать время
  approachStartTime = millis(); // Инициализировать время
  pauseAdvisoryGiven = false; // Сбросить флаг
  adviceMessage = ""; // Сбросить сообщение
}

void finishCalibration() {
  Serial.println("=== CALIBRATION FINISHED ===");
  Serial.print("Reference amplitude: "); Serial.println(reference.firstAmplitude, 3);
  isCalibrating = false;
  
  // Рассчитать среднее качество калибровки (всегда 100%)
  float avgQuality = 100.0f; // Калибровка - эталон
  // Рассчитать длительность
  float durationSecs = (millis() - approachStartTime) / 1000.0f;
  // Сформировать совет (например, для калибровки - "Отличная работа!" или прочерк)
  String advice = "-"; // Прочерк для калибровки
  
  // Добавить калибровку в историю как подход #1
  if (completedApproachesCount < 10) {
      completedApproaches[completedApproachesCount].number = 1; // Калибровка - всегда #1
      completedApproaches[completedApproachesCount].reps = repCount;
      completedApproaches[completedApproachesCount].quality = avgQuality; // Качество калибровки
      completedApproaches[completedApproachesCount].durationSecs = durationSecs;
      completedApproaches[completedApproachesCount].advice = advice;
      completedApproachesCount++;
  }
  
  // Автоматический переход к тренировке
  startExercise();
}

void finishApproach() {
  Serial.println("=== APPROACH FINISHED ===");
  Serial.print("Approach #"); Serial.print(approachNumber); Serial.print(" reps: "); Serial.println(repCount);
  
  // Рассчитать среднее качество подхода (это пример, в реальности нужно накапливать качества репов)
  // Для простоты, используем качество эталона как "качество подхода" или 100%, если подход идеален
  // Более точно: float sumQuality = ...; float avgQuality = sumQuality / repCount;
  // Пока используем 100% как условное "среднее" для подхода, если все репы были хорошие
  // Или передавать сумму и кол-во репов в finishApproach и вычислять здесь
  // Создадим переменную для суммы качества репов в подходе
  static float sumQualityInApproach = 0.0f; // Статическая переменная для хранения суммы в подходе
  // В detectRepetition, когда реп засчитывается в подходе, добавляем его качество к sumQualityInApproach
  // float avgQuality = (repCount > 0) ? (sumQualityInApproach / repCount) : 0.0f;
  // Так как мы не накапливаем sumQualityInApproach в этом коде, используем условное значение.
  // Правильнее будет накапливать сумму в переменной уровня класса/глобальной при засчитывании репа в подходе.
  // Добавим глобальную переменную для суммы качества в текущем подходе (не калибровке)
  static float sumQualityCurrentApproach = 0.0f; // Сумма качества репов в текущем подходе
  static int validRepsInCurrentApproach = 0; // Кол-во действительных репов в текущем подходе
  
  float avgQuality = (validRepsInCurrentApproach > 0) ? (sumQualityCurrentApproach / validRepsInCurrentApproach) : 0.0f;
  
  // Рассчитать длительность
  float durationSecs = (millis() - approachStartTime) / 1000.0f;
  
  // Сформировать совет на основе среднего качества
  String advice = generateAdvice(avgQuality);
  
  // Добавить подход в историю
  if (completedApproachesCount < 10) {
      completedApproaches[completedApproachesCount].number = approachNumber; // Номер подхода
      completedApproaches[completedApproachesCount].reps = repCount;
      completedApproaches[completedApproachesCount].quality = avgQuality; // Среднее качество подхода
      completedApproaches[completedApproachesCount].durationSecs = durationSecs;
      completedApproaches[completedApproachesCount].advice = advice;
      completedApproachesCount++;
  }
  
  isExercising = false;
  // Сбросить сумму качества для следующего подхода
  sumQualityCurrentApproach = 0.0f;
  validRepsInCurrentApproach = 0;
  // approachNumber увеличивается в startExercise для следующего подхода
}

// Функция для генерации совета
String generateAdvice(float avgQuality) {
  if (avgQuality >= 70.0f && avgQuality < 80.0f) {
    return "Хватит халявить!";
  } else if (avgQuality >= 80.0f && avgQuality < 90.0f) {
    return "Хорошо, но можешь лучше!";
  } else if (avgQuality >= 90.0f && avgQuality <= 100.0f) {
    return "Продолжай в том же духе!";
  } else if (avgQuality > 100.0f && avgQuality <= 110.0f) {
    return "Убавь амплитуду, щас зал поднимаешь!";
  } else if (avgQuality < 70.0f) {
    return "Нужно больше усилий!";
  }
  return "Хорошая работа!"; // Резервный вариант
}

void calibrationRoutine() {
  if(mpu.accelUpdate() == 0) {
    float currentAccel = mpu.accelZ();
    detectRepetition(currentAccel, true); // true = калибровка
  }
}

void exerciseRoutine() {
  if(mpu.accelUpdate() == 0) {
    float currentAccel = mpu.accelZ();
    detectRepetition(currentAccel, false); // false = тренировка
  }
}

void detectRepetition(float currentAccel, bool isCalibration) {
  unsigned long currentTime = millis();
  float deviation = abs(currentAccel - baselineAccel);

  // Фильтрация шума: если отклонение *резко* превышает порог, возможно это удар/шум
  // Проверим, не находится ли текущее отклонение в окне фильтрации шума
  if (deviation > MOVEMENT_THRESHOLD) {
      if (!inNoiseWindow) {
          // Началось потенциально значимое движение
          noiseStartTime = currentTime;
          noiseStartAccel = currentAccel;
          inNoiseWindow = true;
          Serial.println("Noise window started due to high deviation.");
      } else {
          // Мы уже в окне шума, проверим, прошло ли достаточно времени
          // Также проверим, действительно ли движение развивается (например, отклонение увеличивается или ускорение меняется значительно)
          float noiseDeviationSinceStart = abs(currentAccel - noiseStartAccel);
          if (currentTime - noiseStartTime > NOISE_FILTER_WINDOW_MS && noiseDeviationSinceStart > MOVEMENT_THRESHOLD * 0.5) { // Добавлена проверка на развитие движения
              // Окно шума истекло, движение развилось, считаем его "настоящим"
              Serial.println("Noise window ended, significant movement detected. State: IDLE -> PHASE_UP.");
              inNoiseWindow = false; // Сброс флага шума
              // Начать движение, если мы в IDLE и прошёл дебаунс
              if (currentMovementState == IDLE && (currentTime - lastRepTime) > REP_DEBOUNCE_MS) {
                  currentMovementState = PHASE_UP;
                  currentMaxAccel = currentAccel;
                  currentMinAccel = currentAccel;
                  phaseStartTime = currentTime;
              }
              // Если мы не в IDLE, но шум закончился, просто сбрасываем флаг
          } else if (currentTime - noiseStartTime > NOISE_FILTER_WINDOW_MS) {
               // Окно шума истекло, но движение не развилось - это был шум
               Serial.println("Noise window ended, no significant movement. Resetting.");
               inNoiseWindow = false; // Сброс флага шума
               // Ничего не делаем, оставляем текущее состояние (IDLE или другое)
               return; // Выйти из функции, чтобы не обрабатывать как движение
          }
          // Если окно шума не истекло, просто выходим, игнорируя это измерение как шум.
          return;
      }
  } else {
      // Отклонение меньше порога, сброс флага шума
      inNoiseWindow = false;
  }

  // Если мы всё ещё в окне шума, не продолжаем обработку состояний
  if (inNoiseWindow) {
      return;
  }

  // Проверка паузы между репами
  if (isExercising && repCount > 0 && !pauseAdvisoryGiven) { // Только в тренировке, после первого репа, если совет не дан
      if (currentTime - lastValidRepTime > PAUSE_BETWEEN_REPS_THRESHOLD_MS) {
          adviceMessage = "Mission passed, увеличь амплитуду, слабак!";
          pauseAdvisoryGiven = true; // Установить флаг, чтобы не дублировать
          Serial.println(adviceMessage);
      }
  }

  switch(currentMovementState) {
    case IDLE: {
      // Проверить начало *любого* движения, превышающего порог
      // и прошедшего дебаунс
      // Условие дебаунс и отклонения уже проверено выше, но перепроверим для ясности
      if (deviation > MOVEMENT_THRESHOLD && (currentTime - lastRepTime) > REP_DEBOUNCE_MS) {
        currentMovementState = PHASE_UP; // Движение началось -> в фазу UP
        // Инициализировать макс/мин при начале движения
        currentMaxAccel = currentAccel;
        currentMinAccel = currentAccel;
        phaseStartTime = currentTime; // Запоминаем время начала фазы UP
        Serial.println("Movement started (Z-axis). State: PHASE_UP. Timeout: " + String(PHASE_TIMEOUT_MS) + "ms");
      }
      break;
    }

    case PHASE_UP: {
      // Обновить макс/мин амплитуду во время фазы подъёма
      if (currentAccel > currentMaxAccel) currentMaxAccel = currentAccel;
      if (currentAccel < currentMinAccel) currentMinAccel = currentAccel;

      // Проверить таймаут: если прошло PHASE_TIMEOUT_MS, а мы всё ещё в PHASE_UP,
      // и не начали спускаться, считаем движение недействительным.
      if (currentTime - phaseStartTime > PHASE_TIMEOUT_MS) {
          Serial.println("Phase UP timeout. Reset to IDLE.");
          currentMovementState = IDLE;
          return; // Выйти из функции, чтобы не проверять другие условия
      }

      // Проверить завершение фазы UP: движение достигло пика и начало возвращаться к базе
      // Это означает, что пик подъёма пройден, и движение начало возвращаться.
      // Условие: отклонение от базы уменьшается по сравнению с предыдущим измерением *и* текущее значение ближе к базе, чем пик.
      // Более простой способ: если `currentAccel` перестаёт быть новым максимумом и начинает удаляться от него.
      // Проверим, если `currentAccel` меньше, чем `currentMaxAccel - MOVEMENT_THRESHOLD * 0.1` (небольшая дельта для устойчивости)
      // и при этом отклонение от baseline уменьшается или стабилизируется.
      float baselineDeviationNow = abs(currentAccel - baselineAccel);
      float baselineDeviationAtPeak = abs(currentMaxAccel - baselineAccel);

      // Условие завершения фазы UP: отклонение от базы уменьшается *и* текущее значение меньше пика
      // ИЛИ если текущее значение *ближе* к baseline, чем пик (baselineDeviationNow < baselineDeviationAtPeak)
      // и при этом `currentAccel` не является новым максимумом.
      if (baselineDeviationNow < baselineDeviationAtPeak * 0.90 && currentAccel < currentMaxAccel - MOVEMENT_THRESHOLD * 0.08) {
          Serial.println("Phase UP completed. Peak detected. MaxAccel: " + String(currentMaxAccel, 3));
          currentMovementState = PHASE_DOWN;
          phaseStartTime = currentTime; // Запоминаем время начала новой фазы DOWN
      }
      // Если отъехал слишком далеко и не возвращается - сброс
      else if (deviation > MOVEMENT_THRESHOLD * 4) { // Увеличено для устойчивости
          currentMovementState = IDLE; // Сброс состояния
          Serial.println("Phase UP lost (too far), reset to IDLE.");
      }
      break;
    }

    case PHASE_DOWN: {
      // Обновить макс/мин амплитуду во время фазы спуска
      // Важно: `currentMaxAccel` и `currentMinAccel` отслеживают *всё* движение с начала фазы UP!
      // Для фазы DOWN нам важен *возврат* к baseline.
      if (currentAccel > currentMaxAccel) currentMaxAccel = currentAccel;
      if (currentAccel < currentMinAccel) currentMinAccel = currentAccel;

      // Проверить таймаут: если прошло PHASE_TIMEOUT_MS, а мы всё ещё в PHASE_DOWN,
      // и не вернулись к базе, считаем движение недействительным.
      if (currentTime - phaseStartTime > PHASE_TIMEOUT_MS) {
          Serial.println("Phase DOWN timeout. Reset to IDLE.");
          currentMovementState = IDLE;
          return; // Выйти из функции, чтобы не проверять другие условия
      }

      float returnDeviation = abs(currentAccel - baselineAccel);
      // Условие завершения фазы DOWN: возврат к базе
      // Это завершает *один* полный повтор.
      if (returnDeviation < RETURN_THRESHOLD * 0.85) { // Умеренно строгий порог для завершения
        // Рассчитать амплитуду всего цикла (от начала фазы UP до возврата в фазе DOWN)
        float movementAmplitude = abs(currentMaxAccel - currentMinAccel);
        bool amplitudeOK = true;

        if (isCalibration && reference.firstAmplitude == 0) {
          // Это первый реп в калибровке - зафиксировать как эталон
          reference.firstAmplitude = movementAmplitude;
          reference.minAmplitude = movementAmplitude * MIN_AMPLITUDE_RATIO;
          amplitudeOK = true;
          Serial.print("First rep amplitude (Reference): "); Serial.println(reference.firstAmplitude, 3);
          Serial.print("Min required amplitude: "); Serial.println(reference.minAmplitude, 3);
        } else if (!isCalibration) {
          // В тренировке проверить амплитуду
          amplitudeOK = (movementAmplitude >= reference.minAmplitude);
        }

        if (amplitudeOK) {
            // Рассчитать качество этого повторения СРАЗУ
            float qualityPercent = (reference.firstAmplitude > 0) ? (movementAmplitude / reference.firstAmplitude) * 100.0f : 0.0f;
            // Ограничить качество 150%
            if (qualityPercent > 150.0f) qualityPercent = 150.0f;
            lastRepQualityPercent = qualityPercent; // Сохранить для UI

            // Проверить, достаточно ли качество (>= 70%)
            if (qualityPercent >= 70.0f) {
                // Засчитать повторение (только после завершения DOWN и при хорошем качестве)
                repCount++;
                lastRepTime = currentTime;
                lastValidRepTime = currentTime; // Обновить время последнего *действительного* репа
                
                // Накопить качество для расчета среднего в подходе (только для тренировки)
                if (!isCalibration) {
                     static float sumQualityInApproach = 0.0f; // Сумма качества репов в текущем подходе
                     static int validRepsInApproach = 0; // Кол-во действительных репов в текущем подходе
                     sumQualityInApproach += qualityPercent;
                     validRepsInApproach++;
                     // Обновить статические переменные в finishApproach через ссылки или глобально - см. finishApproach
                     // Правильнее сделать глобальные переменные для суммы и кол-ва
                     // Создадим глобальные переменные для суммы и кол-ва репов в текущем подходе
                     extern float sumQualityCurrentApproach; // Объявлено глобально
                     extern int validRepsInCurrentApproach; // Объявлено глобально
                     sumQualityCurrentApproach += qualityPercent;
                     validRepsInCurrentApproach++;
                }

                Serial.print("Rep registered. Count: "); Serial.print(repCount);
                Serial.print(", Amplitude: "); Serial.print(movementAmplitude, 3);
                Serial.print(", Quality: "); Serial.print(qualityPercent, 1); Serial.println("%");

                // Сбросить состояние для следующего повторения
                currentMovementState = IDLE;
                
                // Сбросить флаг паузы и сообщение при успешном репе
                if (isExercising) {
                    pauseAdvisoryGiven = false;
                    if (adviceMessage.indexOf("Mission passed") != -1) {
                         adviceMessage = ""; // Очистить только это сообщение
                    }
                }
            } else {
                Serial.print("Rep NOT registered - Quality too low ("); Serial.print(qualityPercent, 1); Serial.print("% < 70%). Amplitude: "); Serial.print(movementAmplitude, 3); Serial.println(". Reset to IDLE.");
                // Сбросить состояние, но не увеличивать счётчик
                currentMovementState = IDLE;
            }
        } else {
            // Амплитуда недостаточна, даже если мы вернулись к базе - сброс
            Serial.print("Rep NOT registered - Amplitude too low ("); Serial.print(movementAmplitude, 3); Serial.print(" < "); Serial.print(reference.minAmplitude, 3); Serial.println("). Reset to IDLE.");
            currentMovementState = IDLE;
        }
      }
      // Если отъехал от базы (от пика DOWN) - сброс
      else if (abs(currentAccel - baselineAccel) > RETURN_THRESHOLD * 3) { // Увеличено для устойчивости
          currentMovementState = IDLE; // Сброс состояния
          Serial.println("Phase DOWN lost (too far from peak), reset to IDLE.");
      }
      break;
    }
  }
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate < 500) return; // Обновление раз в 500мс
  lastUpdate = millis();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  
  display.print("Reps: ");
  display.println(repCount);
  display.print("Target: ");
  display.println(targetReps);
  
  if(isCalibrating) {
    display.print("CALIBRATING");
  } else if(isExercising) {
    display.print("EXERCISING");
  } else {
    display.print("READY");
  }
  
  // Показать качество последнего репа на дисплее
  display.setCursor(0, 20);
  display.print("Qual: ");
  display.print(lastRepQualityPercent, 1);
  display.print("%");

  // Показать время подхода (если активен)
  if (isExercising || isCalibrating) {
      float currentDuration = (millis() - approachStartTime) / 1000.0f;
      display.setCursor(0, 30);
      display.print("Time: ");
      display.print(currentDuration, 1);
      display.print("s");
  }
  
  // Показать совет на дисплее, если есть
  if (adviceMessage.length() > 0) {
      display.setCursor(0, 40);
      // Обрезать текст, если он длинный
      String dispMsg = adviceMessage;
      if (dispMsg.length() > 16) dispMsg = dispMsg.substring(0, 16) + "...";
      display.print(dispMsg);
  }

  display.setCursor(0, 50);
  display.print("Clients: ");
  display.println(WiFi.softAPgetStationNum());
  
  display.display();
}

// Глобальные переменные для накопления качества в подходе
float sumQualityCurrentApproach = 0.0f;
int validRepsInCurrentApproach = 0;