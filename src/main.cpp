#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TMCStepper.h>
#include <Bluepad32.h> 

// --- CONFIGURACIÓN DE PINES (ESP32) ---
#define EN_PIN           13  

#define DIR_PIN_1        12  
#define STEP_PIN_1       14  
#define DRIVER_ADDRESS_1 0b00  

#define DIR_PIN_2        27  
#define STEP_PIN_2       26  
#define DRIVER_ADDRESS_2 0b10  

#define RX2_PIN          16  
#define TX2_PIN          17  
#define R_SENSE          0.11f 

#define PIN_BRILLO_LCD   18  

// --- OBJETOS ---
TMC2209Stepper driver1(&Serial2, R_SENSE, DRIVER_ADDRESS_1);
TMC2209Stepper driver2(&Serial2, R_SENSE, DRIVER_ADDRESS_2);
LiquidCrystal_I2C lcd(0x27, 16, 2); 

ControllerPtr misControles[BP32_MAX_CONTROLLERS];

// --- VARIABLES GLOBALES VOLÁTILES (Entre núcleos) ---
volatile long velocidadObjetivo1 = 0; 
volatile long velocidadObjetivo2 = 0;
volatile bool controlConectado = false;
volatile int brilloActual = 180; 

// Control de estados por comandos seriales y conflictos
volatile bool comandoSerialRecibido = false;
volatile bool conflictoFuentes = false;

// --- PARÁMETROS AJUSTABLES DE RAMPAS ---
const long RETRASO_MINIMO_M1 = 600;   
const long RETRASO_MINIMO_M2 = 1200;  
const long RETRASO_MAXIMO = 2200;    
const long PASO_ACELERACION = 10;   

int corrienteMA = 1900;   

// CONFIGURACIÓN DE MICROPASOS DINÁMICOS
volatile int micropasosM1 = 2;   
volatile int micropasosM2 = 32;  

// Control de flancos para la cruceta D-Pad
bool dpadArriba_previo = false;
bool dpadAbajo_previo = false;
bool dpadDerecha_previo = false;
bool dpadIzquierda_previo = false;

// Control de flancos para los gatillos
bool botonR_presionadoPrevio = false;       
bool botonL_presionadoPrevio = false;       

unsigned long prevLCDMillis = 0;

// Temporizador para mensaje de brillo
unsigned long tiempoUltimoAjusteBrillo = 0;
bool mostrarMensajeBrillo = false;

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    if (misControles[i] == nullptr) {
      misControles[i] = ctl;
      controlConectado = true;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    if (misControles[i] == ctl) {
      misControles[i] = nullptr;
      controlConectado = false;
      conflictoFuentes = false;
      break;
    }
  }
}

// =========================================================================
// TAREA EXCLUSIVA PARA EL NÚCLEO 0: BLUETOOTH Y PANTALLA LCD
// =========================================================================
void TareaBluetoothYLCD(void *pvParameters) {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); 

  while(1) {
    BP32.update(); 
    
    ControllerPtr miControl = misControles[0]; 
    
    if (conflictoFuentes) {
      velocidadObjetivo1 = 0;
      velocidadObjetivo2 = 0;
    } 
    else if (miControl && miControl->isConnected()) {
      int axisX = miControl->axisX();   // Palanca Izquierda (Motor 1)
      int axisY = miControl->axisRY();  // Palanca Derecha (Motor 2)
      uint8_t dpadVal = miControl->dpad();

      // --- CRUCETA (D-PAD): MOTOR 2 (ARRIBA / ABAJO) ---
      bool dpadArriba_actual = (dpadVal & DPAD_UP);
      if (dpadArriba_actual && !dpadArriba_previo) {
        micropasosM2 *= 2;
        if (micropasosM2 > 256) micropasosM2 = 2;
        driver2.microsteps(micropasosM2);
      }
      dpadArriba_previo = dpadArriba_actual;

      bool dpadAbajo_actual = (dpadVal & DPAD_DOWN);
      if (dpadAbajo_actual && !dpadAbajo_previo) {
        micropasosM2 /= 2;
        if (micropasosM2 < 2) micropasosM2 = 256;
        driver2.microsteps(micropasosM2);
      }
      dpadAbajo_previo = dpadAbajo_actual;

      // --- CRUCETA (D-PAD): MOTOR 1 (DERECHA / IZQUIERDA) ---
      bool dpadDerecha_actual = (dpadVal & DPAD_RIGHT);
      if (dpadDerecha_actual && !dpadDerecha_previo) {
        micropasosM1 *= 2;
        if (micropasosM1 > 256) micropasosM1 = 2;
        driver1.microsteps(micropasosM1);
      }
      dpadDerecha_previo = dpadDerecha_actual;

      bool dpadIzquierda_actual = (dpadVal & DPAD_LEFT);
      if (dpadIzquierda_actual && !dpadIzquierda_previo) {
        micropasosM1 /= 2;
        if (micropasosM1 < 2) micropasosM1 = 256;
        driver1.microsteps(micropasosM1);
      }
      dpadIzquierda_previo = dpadIzquierda_actual;

      // --- LEER GATILLO R -> SUBIR BRILLO LCD ---
      bool botonR_actual = miControl->r1(); 
      if (botonR_actual && !botonR_presionadoPrevio) {
        brilloActual += 15; 
        if (brilloActual > 255) brilloActual = 255;
        analogWrite(PIN_BRILLO_LCD, brilloActual);
        tiempoUltimoAjusteBrillo = millis();
        mostrarMensajeBrillo = true;
      }
      botonR_presionadoPrevio = botonR_actual;

      // --- LEER GATILLO L -> BAJAR BRILLO LCD ---
      bool botonL_actual = miControl->l1(); 
      if (botonL_actual && !botonL_presionadoPrevio) {
        brilloActual -= 15; 
        if (brilloActual < 10) brilloActual = 10; 
        analogWrite(PIN_BRILLO_LCD, brilloActual);
        tiempoUltimoAjusteBrillo = millis();
        mostrarMensajeBrillo = true;
      }
      botonL_presionadoPrevio = botonL_actual;

      // --- EVALUACIÓN DE MOVIMIENTO DE JOYSTICKS ---
      bool moviendoX = (abs(axisX) > 40);
      bool moviendoY = (abs(axisY) > 40);

      // Si el usuario toca cualquier joystick, el mando retoma la autoridad total
      if (moviendoX || moviendoY) {
        comandoSerialRecibido = false;
      }

      // --- LEER EJE X (MOTOR 1) ---
      if (axisX > 40) { 
        digitalWrite(DIR_PIN_1, LOW); 
        velocidadObjetivo1 = map(axisX, 41, 512, RETRASO_MAXIMO, RETRASO_MINIMO_M1);
      } else if (axisX < -40) {
        digitalWrite(DIR_PIN_1, HIGH); 
        velocidadObjetivo1 = map(abs(axisX), 41, 511, RETRASO_MAXIMO, RETRASO_MINIMO_M1);
      } else if (!comandoSerialRecibido) {
        velocidadObjetivo1 = 0; // Se detiene correctamente al soltar la palanca
      }

      // --- LEER EJE Y (MOTOR 2) ---
      if (axisY > 40) {
        digitalWrite(DIR_PIN_2, HIGH);
        velocidadObjetivo2 = map(axisY, 41, 512, RETRASO_MAXIMO, RETRASO_MINIMO_M2);
      } else if (axisY < -40) {
        digitalWrite(DIR_PIN_2, LOW);
        velocidadObjetivo2 = map(abs(axisY), 41, 511, RETRASO_MAXIMO, RETRASO_MINIMO_M2);
      } else if (!comandoSerialRecibido) {
        velocidadObjetivo2 = 0; // Se detiene correctamente al soltar la palanca
      }
    } 
    else if (!comandoSerialRecibido) {
      velocidadObjetivo1 = 0;
      velocidadObjetivo2 = 0;
    }

    // --- Temporizador para ocultar mensaje de brillo ---
    if (mostrarMensajeBrillo && (millis() - tiempoUltimoAjusteBrillo >= 2000)) {
      mostrarMensajeBrillo = false;
      lcd.clear();
    }

    // --- ACTUALIZACIÓN DE PANTALLA LCD ---
    if (millis() - prevLCDMillis >= 600) {
      prevLCDMillis = millis();

      if (conflictoFuentes) {
        lcd.setCursor(0, 0);
        lcd.print("!CONFLICTO CONTROL!");
        lcd.setCursor(0, 1);
        lcd.print("PARADA DE EMERG!  ");
      }
      else if (controlConectado || comandoSerialRecibido) {
        if (mostrarMensajeBrillo) {
          lcd.setCursor(0, 0);
          lcd.print("Ajuste de Brillo");
          lcd.setCursor(0, 1);
          lcd.print("Intensidad: ");
          lcd.print(map(brilloActual, 0, 255, 0, 100));
          lcd.print("%   ");
        } 
        else {
          // --- LÍNEA 1 ---
          lcd.setCursor(0, 0);
          lcd.print("M1 1/"); 
          lcd.print(micropasosM1); 
          lcd.print(": "); 
          if (velocidadObjetivo1 == 0) {
            lcd.print("STOP   ");
          } else {
            lcd.print(velocidadObjetivo1);
            lcd.print("us  ");
          }
          
          // --- LÍNEA 2 ---
          lcd.setCursor(0, 1);
          lcd.print("M2 1/"); 
          lcd.print(micropasosM2); 
          lcd.print(": "); 
          if (velocidadObjetivo2 == 0) {
            lcd.print("STOP   ");
          } else {
            lcd.print(velocidadObjetivo2);
            lcd.print("us  ");
          }
        }
      } else {
        lcd.setCursor(0, 0);
        lcd.print("ESPERANDO MANDO ");
        lcd.setCursor(0, 1);
        lcd.print("O COMANDO SERIAL");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// =========================================================================
// FUNCIÓN PARA PROCESAR COMANDOS SERIALES CORTOS
// =========================================================================
void procesarComandosSeriales() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando == "RST") {
      conflictoFuentes = false;
      Serial.println("OK: RST");
      return;
    }

    // Verificar si el joystick se está moviendo activamente
    ControllerPtr miControl = misControles[0];
    bool mandoEstaMoviendose = false;
    
    if (miControl && miControl->isConnected()) {
      int axisX = miControl->axisX();
      int axisY = miControl->axisRY();
      if (abs(axisX) > 40 || abs(axisY) > 40) {
        mandoEstaMoviendose = true;
      }
    }

    // Conflicto solo si el joystick está en movimiento en el instante exacto que llega el comando serial
    if (mandoEstaMoviendose) {
      conflictoFuentes = true;
      velocidadObjetivo1 = 0;
      velocidadObjetivo2 = 0;
      Serial.println("ERR: CONFLICTO");
      return;
    }

    // --- PROCESAMIENTO DE COMANDOS CORTOS ---

    // Motor 1 Micropasos -> M1S:<valor>
    if (comando.startsWith("M1S:")) {
      int val = comando.substring(4).toInt();
      if (val >= 2 && val <= 256) {
        micropasosM1 = val;
        driver1.microsteps(micropasosM1);
        comandoSerialRecibido = true;
        Serial.print("OK M1S:"); Serial.println(micropasosM1);
      }
    } 
    // Motor 2 Micropasos -> M2S:<valor>
    else if (comando.startsWith("M2S:")) {
      int val = comando.substring(4).toInt();
      if (val >= 2 && val <= 256) {
        micropasosM2 = val;
        driver2.microsteps(micropasosM2);
        comandoSerialRecibido = true;
        Serial.print("OK M2S:"); Serial.println(micropasosM2);
      }
    } 
    // Motor 1 Velocidad -> M1V:<velocidad>
    else if (comando.startsWith("M1V:")) {
      int vel = comando.substring(4).toInt();
      if (vel < 0) {
        digitalWrite(DIR_PIN_1, HIGH);
        velocidadObjetivo1 = abs(vel);
      } else if (vel > 0) {
        digitalWrite(DIR_PIN_1, LOW);
        velocidadObjetivo1 = vel;
      } else {
        velocidadObjetivo1 = 0;
      }
      comandoSerialRecibido = true;
      Serial.print("OK M1V:"); Serial.println(velocidadObjetivo1);
    } 
    // Motor 2 Velocidad -> M2V:<velocidad>
    else if (comando.startsWith("M2V:")) {
      int vel = comando.substring(4).toInt();
      if (vel < 0) {
        digitalWrite(DIR_PIN_2, LOW);
        velocidadObjetivo2 = abs(vel);
      } else if (vel > 0) {
        digitalWrite(DIR_PIN_2, HIGH);
        velocidadObjetivo2 = vel;
      } else {
        velocidadObjetivo2 = 0;
      }
      comandoSerialRecibido = true;
      Serial.print("OK M2V:"); Serial.println(velocidadObjetivo2);
    } 
    // Brillo Pantalla LCD -> B:<intensidad>
    else if (comando.startsWith("B:")) {
      int val = comando.substring(2).toInt();
      brilloActual = constrain(val, 0, 255);
      analogWrite(PIN_BRILLO_LCD, brilloActual);
      tiempoUltimoAjusteBrillo = millis();
      mostrarMensajeBrillo = true;
      comandoSerialRecibido = true;
      Serial.print("OK B:"); Serial.println(brilloActual);
    }
  }
}

// =========================================================================
// CONFIGURACIÓN DE ARRANQUE GENERAL (Core 1)
// =========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  
  pinMode(PIN_BRILLO_LCD, OUTPUT);
  analogWrite(PIN_BRILLO_LCD, brilloActual);
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("BOOT SEPARATED");
  
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);   
  
  driver1.begin();             
  driver1.toff(5);             
  driver1.rms_current(corrienteMA);    
  driver1.microsteps(micropasosM1);      
  driver1.pwm_autoscale(true); 

  driver2.begin();             
  driver2.toff(5);             
  driver2.rms_current(corrienteMA);    
  driver2.microsteps(micropasosM2);      
  driver2.pwm_autoscale(true); 
  
  digitalWrite(EN_PIN, LOW);  
  
  xTaskCreatePinnedToCore(TareaBluetoothYLCD, "TareaBT_LCD", 10000, NULL, 1, NULL, 0);
}

// =========================================================================
// LOOP PRINCIPAL (Core 1)
// =========================================================================
void loop() {
  procesarComandosSeriales();

  if (conflictoFuentes) {
    return;
  }

  unsigned long currentMicros = micros();
  
  static long pasoRetrasoActual1 = 0;
  static long pasoRetrasoActual2 = 0;
  
  static unsigned long prevMicros1 = 0;
  static unsigned long prevMicros2 = 0;

  if (velocidadObjetivo1 > 0) {
    if (pasoRetrasoActual1 == 0) pasoRetrasoActual1 = RETRASO_MAXIMO; 
    else if (pasoRetrasoActual1 > velocidadObjetivo1) {
      pasoRetrasoActual1 -= PASO_ACELERACION;
      if (pasoRetrasoActual1 < velocidadObjetivo1) pasoRetrasoActual1 = velocidadObjetivo1;
    } else if (pasoRetrasoActual1 < velocidadObjetivo1) {
      pasoRetrasoActual1 += PASO_ACELERACION;
    }
  } else {
    if (pasoRetrasoActual1 > 0) {
      pasoRetrasoActual1 += (PASO_ACELERACION * 2);
      if (pasoRetrasoActual1 >= RETRASO_MAXIMO) pasoRetrasoActual1 = 0; 
    }
  }

  if (velocidadObjetivo2 > 0) {
    if (pasoRetrasoActual2 == 0) pasoRetrasoActual2 = RETRASO_MAXIMO;
    else if (pasoRetrasoActual2 > velocidadObjetivo2) {
      pasoRetrasoActual2 -= PASO_ACELERACION;
      if (pasoRetrasoActual2 < velocidadObjetivo2) pasoRetrasoActual2 = velocidadObjetivo2;
    } else if (pasoRetrasoActual2 < velocidadObjetivo2) {
      pasoRetrasoActual2 += PASO_ACELERACION;
    }
  } else {
    if (pasoRetrasoActual2 > 0) {
      pasoRetrasoActual2 += (PASO_ACELERACION * 2);
      if (pasoRetrasoActual2 >= RETRASO_MAXIMO) pasoRetrasoActual2 = 0;
    }
  }

  if (pasoRetrasoActual1 > 0 && (currentMicros - prevMicros1 >= pasoRetrasoActual1)) {
    prevMicros1 = currentMicros;
    digitalWrite(STEP_PIN_1, HIGH);
    delayMicroseconds(2); 
    digitalWrite(STEP_PIN_1, LOW);
  }

  if (pasoRetrasoActual2 > 0 && (currentMicros - prevMicros2 >= pasoRetrasoActual2)) {
    prevMicros2 = currentMicros;
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(2); 
    digitalWrite(STEP_PIN_2, LOW);
  }
}