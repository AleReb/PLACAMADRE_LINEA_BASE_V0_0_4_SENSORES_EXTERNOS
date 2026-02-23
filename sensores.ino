
// funciones no bloqueantes en caso de error de sensores i2c
// --- I²C external sensor (TVOC + SO2) ---
// Helper: returns true si el dispositivo I2C responde en 'addr'
bool isDeviceConnected(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}


void printRawValues() {
  Serial.printf("ADC1 NO2  OP1/CH1 RAW: %.0f", adc2No2Op1Ch1Raw);
  Serial.printf(" ADC1 NO2  OP2/CH2 RAW: %.0f\n", adc2No2Op2Ch2Raw);
  Serial.printf("ADC1 CO   OP1/CH3 RAW: %.0f", adc2CoOp1Ch3Raw);
  Serial.printf(" ADC1 CO   OP2/CH4 RAW: %.0f\n", adc2CoOp2Ch4Raw);
  yield();

  Serial.printf("ADC2 OX   OP1/CH1 RAW: %.0f", adc1OxOp1Ch1Raw);
  Serial.printf(" ADC2 OX   OP2/CH2 RAW: %.0f\n", adc1OxOp2Ch2Raw);
  Serial.printf("ADC2 NO   OP1/CH3 RAW: %.0f", adc1NoOp1Ch3Raw);
  Serial.printf(" ADC2 NO   OP2/CH4 RAW: %.0f\n", adc1NoOp2Ch4Raw);
  yield();

  Serial.printf("ADC3 So2   OP1/CH1 RAW: %.0f", adc3So2Op1Ch1Raw);
  Serial.printf(" ADC3 So2   OP2/CH2 RAW: %.0f\n", adc3So2Op2Ch2Raw);
  // Serial.printf("ADC3 XX   OP1/CH3 RAW: %.0f", adc3XXOp1Ch3Raw);
  // Serial.printf(" ADC3 XX   OP2/CH4 RAW: %.0f\n", adc3XXOp2Ch4Raw);
  yield();
}
// modbus rs485
void preTransmission() {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(RE_PIN, HIGH);
}

void postTransmission() {
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
}
void readWindSensor() {
  modbus.begin(1, rs485Serial);  // ID = 1 estación meteorológica

  uint8_t result = modbus.readHoldingRegisters(0x01F4, 16);
  if (result == modbus.ku8MBSuccess) {
    windSpeed = modbus.getResponseBuffer(0) * 0.1;
    windStrength = modbus.getResponseBuffer(1);
    windDirection = modbus.getResponseBuffer(3);
    windHumidity = modbus.getResponseBuffer(4) * 0.1;
    windTemperature = (int16_t)modbus.getResponseBuffer(5) * 0.1;
    windPressure = modbus.getResponseBuffer(9) * 0.1;
    windRainfall = modbus.getResponseBuffer(13) * 0.1;
    windSolarIrradiance = modbus.getResponseBuffer(15);

    Serial.printf("[WIND] Speed: %.2f m/s  Strength: %u  Dir: %u deg\n", windSpeed, windStrength, windDirection);
    Serial.printf("[WIND] Hum: %.1f %%  Temp: %.1f C  Pres: %.1f kPa  Rain: %.1f mm  Solar: %.1f W/m2\n",
                  windHumidity, windTemperature, windPressure, windRainfall, windSolarIrradiance);
    RS485METEO = true;
  } else {
    Serial.printf("Modbus error (wind): 0x%02X\n", result);
    RS485METEO = false;
  }
}

void readUVSensor() {
  modbus.begin(2, rs485Serial);  // ID = 2 sensor UV

  uint8_t result = modbus.readHoldingRegisters(0x0000, 1);
  if (result == modbus.ku8MBSuccess) {
    uvIntensity_mWcm2 = modbus.getResponseBuffer(0) * 0.01f;
    Serial.printf("[UV] Intensity: %.2f mW/cm²\n", uvIntensity_mWcm2);
    RS485UV = true;
  } else {
    Serial.printf("Modbus error (UV intensity): %u\n", result);
    RS485UV = false;
  }

  result = modbus.readHoldingRegisters(0x0001, 1);
  if (result == modbus.ku8MBSuccess) {
    uvIndex = modbus.getResponseBuffer(0);
    Serial.printf("[UV] Index: %u\n", uvIndex);
    RS485UV = true;
  } else {
    Serial.printf("Modbus error (UV index): %u\n", result);
    RS485UV = false;
  }
}
/* ===================== VARIABLES GLOBALES DE ALMACENAMIENTO
 * ===================== */
// PMS5003
float pmsTempEXT = 0, pmsHumEXT = 0, pmsHchoEXT = 0;
uint32_t pm1EXT = 0, pm25EXT = 0, pm10EXT = 0;

// CO2 MH-Z19
double co2PpmEXT = 0, co2RawEXT = 0, co2CustomEXT = 0;

// VOC Winsen
uint16_t tvocEXT = 0;

// -- Manejo de errores Sensores Externos --
int sensorErrorCount = 0;      // Contador de timeouts
int sensorRestartRetries = 0;   // Contador de reintentos de hardware
bool externalErrorState = false; // Estado de error fatal

/*
  ADS1115: [Direccion][Canal]
  Direcciones mapeadas: 0=0x48, 1=0x49, 2=0x4A, 3=0x4B
*/
int16_t ads0_vEXT[4][4] = { { 0 } };  // I2C_0
int16_t ads1_vEXT[4][4] = { { 0 } };  // I2C_1

/* ===================== FUNCIONES DE AYUDA ===================== */
float extraerValor(String cad, String clave) {
  int p = cad.indexOf(clave + ":");
  if (p == -1)
    return 0;
  int inicio = p + clave.length() + 1;
  int fin = cad.indexOf(",", inicio);
  if (fin == -1)
    fin = cad.length();
  return cad.substring(inicio, fin).toFloat();
}

int getAddrIndex(String addrHex) {
  if (addrHex.equalsIgnoreCase("0x48"))
    return 0;
  if (addrHex.equalsIgnoreCase("0x49"))
    return 1;
  if (addrHex.equalsIgnoreCase("0x4A"))
    return 2;
  if (addrHex.equalsIgnoreCase("0x4B"))
    return 3;
  return -1;
}

void procesarLinea(String linea) {
  int pComa = linea.indexOf(',');
  if (pComa == -1)
    return;

  String tipo = linea.substring(0, pComa);
  String payload = linea.substring(pComa + 1);

  if (tipo == "PMS") {
    pm1EXT = (uint32_t)extraerValor(payload, "PM1.0");
    pm25EXT = (uint32_t)extraerValor(payload, "PM2.5");
    pm10EXT = (uint32_t)extraerValor(payload, "PM10");
    pmsTempEXT = extraerValor(payload, "T");
    pmsHumEXT = extraerValor(payload, "RH");
    pmsHchoEXT = extraerValor(payload, "HCHO");
  } else if (tipo == "CO2") {
    co2PpmEXT = extraerValor(payload, "PPM");
    co2RawEXT = extraerValor(payload, "RAW");
  } else if (tipo == "VOC") {
    tvocEXT = (uint16_t)extraerValor(payload, "TVOC");
  } else if (tipo == "ADS") {
    // Formato: ADS,BUS,ADDR,V0:raw,V1:raw...
    int s2 = payload.indexOf(',');
    String bus = payload.substring(0, s2);
    String remaining = payload.substring(s2 + 1);

    int s3 = remaining.indexOf(',');
    String addr = remaining.substring(0, s3);
    String dataAds = remaining.substring(s3 + 1);

    int addrIdx = getAddrIndex(addr);
    if (addrIdx != -1) {
      int16_t(*ptrArr)[4] = (bus == "I2C_0") ? ads0_vEXT : ads1_vEXT;
      ptrArr[addrIdx][0] = (int16_t)extraerValor(dataAds, "V0");
      ptrArr[addrIdx][1] = (int16_t)extraerValor(dataAds, "V1");
      ptrArr[addrIdx][2] = (int16_t)extraerValor(dataAds, "V2");
      ptrArr[addrIdx][3] = (int16_t)extraerValor(dataAds, "V3");
    }
  }
}

/* ===================== MAPEO MANUAL DE SENSORES EXTERNOS ===================== */
// Esta función permite asignar manualmente los canales de los ADS externos a los globales
void mapearDatosExternos() {
  // --- I2C_1 (Bus de Sensores 1) ---
  
  // 0x48: Sensor NO
  adc1NoOp1Ch3Raw = ads1_vEXT[0][0]; // V0 -> Op1
  adc1NoOp2Ch4Raw = ads1_vEXT[0][2]; // V2 -> Op2
  
  // 0x49: Sensor OX
  adc1OxOp1Ch1Raw = ads1_vEXT[1][0]; // V0 -> Op1
  adc1OxOp2Ch2Raw = ads1_vEXT[1][2]; // V2 -> Op2
  
  // 0x4A: Libre para asignación manual
  adc3So2Op1Ch1Raw = ads1_vEXT[2][0]; // V0 -> Op1
  adc3So2Op2Ch2Raw = ads1_vEXT[2][2]; // V2 -> Op2
  // 0x4B: Libre para asignación manual

  // --- I2C_0 (Bus de Sensores 0) ---
  
  // 0x48: Sensor NO2
  adc2No2Op1Ch1Raw = ads0_vEXT[0][0]; // V0 -> Op1
  adc2No2Op2Ch2Raw = ads0_vEXT[0][2]; // V2 -> Op2
  
  // 0x49: Sensor CO
  adc2CoOp1Ch3Raw = ads0_vEXT[1][0]; // V0 -> Op1
  adc2CoOp2Ch4Raw = ads0_vEXT[1][2]; // V2 -> Op2
  
  // 0x4A: Sensor SO2
  //adc3So2Op1Ch1Raw = ads0_vEXT[2][0]; // V0 -> Op1
  //adc3So2Op2Ch2Raw = ads0_vEXT[2][2]; // V2 -> Op2

  // 0x4B: Libre para asignación manual

  Serial.println(">>> Mapeo de señales externas completado.");
}

void solicitarYLeer() {
  if (externalErrorState) {
    Serial.println("\n--- ERROR EXTERNO: Sensores bloqueados por fallos persistentes ---");
    return;
  }

  Serial.println("\n--- Iniciando ESCANEO On-Demand ---");
  EXTSerial.println("SCAN");

  bool finDeTrama = false;
  unsigned long timeout = millis();

  while (!finDeTrama && (millis() - timeout < 5000)) {
    if (EXTSerial.available()) {
      String data = EXTSerial.readStringUntil('\n');
      data.trim();
      if (data == "END") {
        finDeTrama = true;
      } else if (data.length() > 0) {
        procesarLinea(data);
      }
      timeout = millis();
    }
    yield();
  }

  if (finDeTrama) {
    Serial.println(">>> Datos sincronizados.");
    sensorErrorCount = 0; // Reset éxito
    sensorRestartRetries = 0;

    // Ejecutar el mapeo manual de las señales ADC externas
    mapearDatosExternos();
    
    // // 1. PMS
    // Serial.printf("PMS Temp: %.1f C  Hum: %.1f %%  PM1.0: %u ug/m3  PM2.5: %u "
    //               "ug/m3  PM10: %u ug/m3\n",
    //               pmsTempEXT, pmsHumEXT, pm1EXT, pm25EXT, pm10EXT);

    pm1 = pm1EXT;
    pm25 = pm25EXT;
    pm10 = pm10EXT;
    pmsTemp = pmsTempEXT;
    pmsHum = pmsHumEXT;

    Serial.printf("PMS Temp: %.1f C  Hum: %.1f %%  PM1.0: %u ug/m3  PM2.5: %u ug/m3  PM10: %u ug/m3\n",
                  pmsTemp, pmsHum, pm1, pm25, pm10);
    yield();

    PLANTOWER = true;
    Co2 = true;

    // 2. CO2 (Concatenación y cálculo Custom)
    co2CustomEXT = -0.674 * co2RawEXT + 36442;
    co2Ppm = co2PpmEXT;
    co2Raw = co2RawEXT;
    co2Custom = co2CustomEXT;

    String outputCO2 = "CO2_Internal: " + String(co2Ppm, 0) + " ppm  CO2_Raw: " + String(co2Raw, 2) + "  CO2_Custom: " + String(co2CustomEXT, 2) + " ppm";
    Serial.println(outputCO2);

    // 3. VOC (Impresión simple)
    tvocValue = tvocEXT;
    Serial.println("VOC TVOC: " + String(tvocValue) + " ug/m3");
    I2CTVOCYSo2 = true;

    // 4. ADS Resumen //OJO dentro de los adc esta el so2Ppm !!!
    const char *addrs[] = { "0x48", "0x49", "0x4A", "0x4B" };
    for (int b = 0; b < 2; b++) {
      Serial.print(b == 0 ? "ADS I2C_0: " : "ADS I2C_1: ");
      for (int i = 0; i < 4; i++) {
        int16_t *chs = (b == 0) ? ads0_vEXT[i] : ads1_vEXT[i];
        // Solo mostramos si hay algún valor distinto de cero (opcional, aquí
        // mostramos todo)
        Serial.print("[" + String(addrs[i]) + ":");
        for (int c = 0; c < 4; c++)
          Serial.print(String(chs[c]) + (c < 3 ? "," : ""));
        Serial.print("] ");
      }
      Serial.println();
    }
    Serial.println("------------------------------------------");
  } else {
    // === MANEJO DE TIMEOUT ===
    // Si no se recibe la trama completa ("END") dentro del tiempo esperado
    sensorErrorCount++;
    Serial.printf("TIMEOUT al solicitar datos (Error: %d/5)\n", sensorErrorCount);
    
    // Registrar el fallo en la tarjeta SD para auditoría
    logError("EXT_SENSOR", "TIMEOUT", "No se recibio END de los sensores externos");

    // Si alcanzamos 5 fallos consecutivos, intentamos un reinicio de hardware
    if (sensorErrorCount >= 5) {
      sensorErrorCount = 0; // Resetear contador de errores para el siguiente ciclo
      
      if (sensorRestartRetries < 3) {
        // --- CICLO DE REINICIO DE HARDWARE (Power Cycle) ---
        sensorRestartRetries++;
        Serial.printf("Reiniciando hardware de sensores (Intento %d/3)...\n", sensorRestartRetries);
        logError("EXT_SENSOR", "RESTART", "Ejecutando power cycle de los sensores");
        
        // Apagar alimentación (Pin 4)
        digitalWrite(SENSOR_POWER_PIN, LOW);
        delay(2000);
        // Encender alimentación
        digitalWrite(SENSOR_POWER_PIN, HIGH);
        delay(3000); // Esperar a que los sensores externos inicien correctamente
      } else {
        // --- ESTADO DE ERROR CRÍTICO FINAL ---
        // Se agotaron los reintentos físicos; se bloquea el escaneo para no ralentizar el sistema
        Serial.println(">>> ERROR EXTERNO: Se agotaron los reintentos de reinicio fisico.");
        logError("EXT_SENSOR", "CRITICAL", "Estado de error permanente: Fallo critico de hardware");
        externalErrorState = true;
      }
    }
  }
}
