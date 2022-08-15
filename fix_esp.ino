#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

unsigned long previousTime = 0;
int interval = 5000;

byte salinityPin = 33;     // Pin analog salinitas
byte temperaturePin = 25;  // Pin digital Suhu
byte phPin = 35;           // Pin analog ph
byte tssTurbidityPin = 34; // Pin analog Turbidity
byte tdsPin = 32;          // Pin analog TDS
byte mqPin = 27;           // Pin analog Amonia

// Variable Ph
float Po = 0;
float PH_step;
int analogValuePh;
double phVoltage;

// kalibrasi Ph
float PH4 = 4.9f;
float PH7 = 3.7f;

// Variable Suhu
OneWire oneWire(temperaturePin);
DallasTemperature temperatureSensor(&oneWire);
float temperatureValue = 0;

// Variable TSS (Turbidity)
int nodeMCUValue = 4096;
float nodeMCUVolt = 5.0f;
float volt = 0;
float ntu = 0;

float round_to_dp(float in_value, int decimal_place)
{
    float multiplier = powf(10.0f, decimal_place);
    in_value = roundf(in_value * multiplier) / multiplier;
    return in_value;
}

// Variable salinitas
float voltage;
float tds, conductivity;

// Variable TDS
#define VREF 5.0          // analog reference voltage(Volt) of the ADC
#define SCOUNT 30         // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 0;

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++)
    {
        for (i = 0; i < iFilterLen - j - 1; i++)
        {
            if (bTab[i] > bTab[i + 1])
            {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

// Variabel Amonia (mq137)
#define RL 10    // nilai RL = 10 pada sensor
#define m -0.417 // hasil perhitungan gradien
#define b 0.425  // hasil perhitungan perpotongan
#define Ro 205   // hasil pengukuran RO

float VRL, RS, ratio;

const int numReadings = 10; // nilai pengambilan sample pembacaan sebesar 5 kali
float readings[numReadings];
int readIndex = 0;
float total = 0;
float average = 0;

// =======================================================

void setup()
{
    temperatureSensor.begin();
    Serial.begin(115200);

    pinMode(salinityPin, INPUT);
    pinMode(temperaturePin, INPUT);
    pinMode(phPin, INPUT);
    pinMode(tssTurbidityPin, INPUT);
    pinMode(tdsPin, INPUT);
    pinMode(mqPin, INPUT);

    for (int thisReading = 0; thisReading < numReadings; thisReading++)
    {
        readings[thisReading] = 0;
    }
}

// =======================================================

void loop()
{
    unsigned long currentTime = millis();

    //========================== Sensor Suhu ==========================
    temperatureSensor.setResolution(10);

    temperatureSensor.requestTemperatures();
    temperatureValue = temperatureSensor.getTempCByIndex(0);

    //========================== Sensor Ph ==========================
    int analogValuePh = analogRead(phPin);

    phVoltage = 5.0f / 4096.0f * analogValuePh;

    PH_step = (PH4 - PH7) / 3;
    Po = 7.00f + ((PH7 - phVoltage) / PH_step); // Po = 7.00 + ((phVoltage7 - phVoltage) / PhStep);

    //========================== Sensor TSS (Turbidity) ==========================
    for (int i = 0; i < 800; i++)
    {
        volt += ((float)analogRead(tssTurbidityPin) / nodeMCUValue) * 5.0f;
    }

    volt = volt / 800.0f;
    volt = round_to_dp(volt, 2);

    if (volt < (nodeMCUVolt / 2))
    {
        ntu = 3000.0f;
    }
    else if (ntu <= 0)
    {
        ntu = 0;
    }
    else
    {
        ntu = -1120.4f * sq(volt) + 5742.3f * volt - 4353.8f;
    }

    //========================== Sensor Salinitas ==========================
    int value = analogRead(salinityPin);
    voltage = value * (5.0f / 4095.0f);

    tds = (voltage + 0.3045f) / 0.122f;
    conductivity = (0.3442f * voltage) - 0.253f;

    if (tds <= 2.5)
    {
        tds = 0;
    }

    if (conductivity < 0)
    {
        conductivity = 0;
    }

    //========================== Sensor TDS ==========================
    temperatureSensor.requestTemperatures();
    temperatureValue = temperatureSensor.getTempCByIndex(0);
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
    {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(tdsPin); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0f; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        float compensationCoefficient = 1.0f + 0.02f * (temperatureValue - 25.0f);       // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVoltage = averageVoltage / compensationCoefficient;            // temperature compensation

        tdsValue = ((133.42f * compensationVoltage * compensationVoltage * compensationVoltage) - (255.86f * compensationVoltage * compensationVoltage) + (857.39f * compensationVoltage)) * 0.5f; // convert voltage value to tds value

        // Serial.print("Temperature:");
        // Serial.print(temperatureValue);
        // Serial.println("ºC");
    }

    //========================= Sensor MQ137 =========================
    VRL = analogRead(mqPin) * (5 / 4096.0);        // konversi analog ke tegangan
    RS = (5.0 / VRL - 1) * 10;                     // rumus untuk RS
    ratio = RS / Ro;                               // rumus mencari ratio
    float ppm = pow(10, ((log10(ratio) - b) / m)); // rumus mencari ppm

    // total = total - readings[readIndex];
    // ndexreadings[readI] = ppm;
    // total = total + readings[readIndex];
    // readIndex = readIndex + 1;

    // if (readIndex >= numReadings) {
    //     readIndex = 0;
    // }
    // average = total / numReadings;

    // =========================================
    // Bagian untuk print ke serial monitor
    // =========================================
    if (currentTime - previousTime > interval)
    {
        Serial.println("=====================");

        // Print hasil pembacaan sensor suhu
        if (temperatureValue > -127)
        {
            Serial.print("Suhu (Celcius): ");
            Serial.println(temperatureValue, 2);
        }

        // Print hasil pembacaan sensor Ph
        Serial.print("PH : ");
        Serial.println(Po, 2);

        // Print hasil pembacaan sensor TSS/Turbidity
        Serial.print("TSS/Turbidity (NTU): ");
        Serial.println(ntu);

        // Print hasil pembacaan sensor Salinitas (Mirip TDS)
        Serial.print("Salinitas: ");
        Serial.println(tds);
        Serial.print("Konduktivitas: ");
        Serial.println(conductivity);

        // Print hasil pembacaan sensor TDS
        Serial.print("TDS (ppm):");
        Serial.println(tdsValue, 0);

        // Print hasil pembacaan sensor MQ137
        Serial.print("Amonia (ppm) = ");
        Serial.println(ppm); // Display calculated Ro

        Serial.println("=====================");
        Serial.println(value);   // Display calculated Ro
        Serial.println(voltage); // Display calculated Ro
        Serial.println("=====================");

        previousTime = currentTime;
    }
}
