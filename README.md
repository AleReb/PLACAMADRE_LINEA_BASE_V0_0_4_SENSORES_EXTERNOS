# Sistema de Monitoreo Ambiental (Placa Madre Rev. D)

**Autor:** Alejandro Rebolledo (arebolledo@udd.cl | edo@udd.cl)
**Versión:** 0.4 Sensores Externos (V0_0_4)

## Descripción General
Este firmware implementa un sistema integrado para monitorear múltiples parámetros ambientales y climáticos, basado en un ESP32. El sistema se encarga de adquirir datos desde diversos sensores:
- Sensores Analógicos externos e I2C (TVOC, SO2, NO2, CO, OX, NO).
- Sensores vía UART (PMS5003 PLAN TOWER, MH-Z19).
- Sensores RS485 / Modbus (Estación Meteorológica, Radiación UV).
- Interfaz Serie para captura de fotos con cámara externa.

Los datos recolectados se almacenan de manera robusta en una tarjeta SD en formato CSV, junto con mediciones del estado del sistema (batería, señal celular). Posteriormente, toda la información (y fotografías) se transmite a un servidor mediante un módem GSM/LTE (SIM7600) vía HTTP POST.

## Características Principales
- Adquisición de datos concurrente y redundante (SD + Transmisión).
- Integración con módem SIM7600 para conectividad MQTT/HTTP en terreno.
- Watchdog Timer (WDT) para autorecuperación de fallos de comunicación.
- Almacenamiento local mediante interfaz SPI a tarjeta SD.
- Máquina de estados para reintento de envío en caso de fallas de la red eléctrica o celular.

## Licencia
Este proyecto se distribuye bajo la licencia **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.
Para más información, consulta el archivo [LICENSE](LICENSE) incluido en este repositorio.

## Descargo de Responsabilidad / Disclaimer
> [!WARNING]
> Este código y hardware asociados se ofrecen **"tal cual" ("as is")**, sin garantías de ningún tipo, explícitas o implícitas, incluyendo pero no limitado a garantías de comerciabilidad o idoneidad para un propósito particular. El usuario utiliza este software y sus diagramas bajo su **propio riesgo**. El autor (Alejandro Rebolledo) no se hace responsable por ninguna pérdida de datos, daños a equipos o cualquier otro tipo de incidente derivado del uso o modificación de este código.
