#include <Arduino.h>
#include <OneButton.h>
#include <EEPROM.h>
#include "signal_parser.h"
#include "apparent_temperature.h"

const int RECEIVER_PIN = 2;
const int LED_PIN = 3;
const int BUTTON_PIN = 4;

size_t currentSignalIndex = 0;
const size_t MAX_GAPS_COUNT = 128;
unsigned long gapDurations[MAX_GAPS_COUNT];
unsigned long gapStartMicros = 0;

SignalParser parser;

OneButton button(
    BUTTON_PIN,
    true,
    true);

// The number of weather sensors you want to connect to this program.
const size_t NUM_SENSORS = 2;
uint8_t sensorIds[NUM_SENSORS];
size_t currentSensorsCount = 0;

size_t EEPROM_SENSOR_IDS_ADDRESS = 0x0;
const uint8_t EEPROM_EMPTY_VALUE = 0xFF;
size_t EEPROM_MODE_ADDRESS = 0x0F;

SensorData lastDataPerSensor[NUM_SENSORS];
float lastApparentTemperatures[NUM_SENSORS];

enum Mode
{
  OUTSIDE_TEMPERATURE_LOWER,
  INSIDE_TEMPERATURE_LOWER,
  OUTSIDE_HUMIDITY_LOWER,
  INSIDE_HUMIDITY_LOWER,
  // Dummy value that can be used to determine the number of values in this enum.
  ALWAYS_AT_END,
};
Mode currentMode = OUTSIDE_TEMPERATURE_LOWER;

size_t findSensorIndex(uint8_t sensorId);
bool isKnownSensor(uint8_t sensorId);
bool isSearchingForSensors();
void writeCurrentSensorsToStorage();
bool shouldCompareTemperatures();
void updateTemperatureLed();
void addSensor(uint8_t sensorId);
void onReceiverChanged();
void onButtonLongPressed();
void onButtonClicked();

void addSensor(uint8_t sensorId)
{
  sensorIds[currentSensorsCount++] = sensorId;
  writeCurrentSensorsToStorage();

  Serial.print("Added sensor with ID ");
  Serial.print(sensorId, HEX);
  Serial.println(" to known sensors.");
}

bool shouldCompareTemperatures()
{
  return NUM_SENSORS >= 2 && currentSensorsCount >= 2;
}

void updateTemperatureLed()
{
  float insidePerceivedTemp = lastApparentTemperatures[0];
  float outsidePerceivedTemp = lastApparentTemperatures[1];

  uint8_t insideHumidity = lastDataPerSensor[0].humidity;
  uint8_t outsideHumidity = lastDataPerSensor[1].humidity;

  Serial.print("Perceived temperatures: ");
  Serial.print(insidePerceivedTemp);
  Serial.print(" deg Celsius inside / ");
  Serial.print(outsidePerceivedTemp);
  Serial.println(" deg Celsius outside");
  Serial.print("Humidity: ");
  Serial.print(insideHumidity);
  Serial.print(" inside, ");
  Serial.print(outsideHumidity);
  Serial.println(" outside");

  bool turnLedOn;
  switch(currentMode) {
    case OUTSIDE_TEMPERATURE_LOWER:
      turnLedOn = outsidePerceivedTemp < insidePerceivedTemp;
      break;
    case INSIDE_TEMPERATURE_LOWER:
      turnLedOn = insidePerceivedTemp < outsidePerceivedTemp;
      break;
    case OUTSIDE_HUMIDITY_LOWER:
      turnLedOn = outsideHumidity < insideHumidity;
      break;
    case INSIDE_HUMIDITY_LOWER:
      turnLedOn = insideHumidity < outsideHumidity;
      break;
    default:
      turnLedOn = false;
      break;
  }
  digitalWrite(LED_PIN, turnLedOn);
}

// Returns the index of the sensor with the given ID in the array of sensor IDs or NUM_SENSORS if not found.
size_t findSensorIndex(uint8_t sensorId)
{
  for (size_t i = 0; i < currentSensorsCount; i++)
  {
    if (sensorIds[i] == sensorId)
    {
      return i;
    }
  }
  return NUM_SENSORS;
}

bool isKnownSensor(uint8_t sensorId)
{
  return findSensorIndex(sensorId) < NUM_SENSORS;
}

bool isSearchingForSensors()
{
  return currentSensorsCount < NUM_SENSORS;
}

void writeCurrentSensorsToStorage()
{
  for (size_t i = 0; i < NUM_SENSORS; i++)
  {
    EEPROM.write(EEPROM_SENSOR_IDS_ADDRESS + i, sensorIds[i]);
  }
}

/**
 * ISR for storing the duration of gaps after pulses.
 */
void onReceiverChanged()
{
  // Do not overrun the buffer for signals.
  // If loop() does not empty this buffer in time, we lose data. That's okay for now, loop just has to be kept fast enough.
  if (currentSignalIndex >= MAX_GAPS_COUNT)
  {
    return;
  }

  bool isNowLow = digitalRead(RECEIVER_PIN) == LOW;
  // If this is the start of a gap, start measuring time.
  if (isNowLow)
  {
    gapStartMicros = micros();
  }
  // If this is the end of a gap (a start of a pulse), note down the elapsed time and store it.
  else
  {
    gapDurations[currentSignalIndex++] = micros() - gapStartMicros;
  }
}

void onButtonLongPressed()
{
  // Forget about which sensors are connected.
  memset(sensorIds, EEPROM_EMPTY_VALUE, sizeof(sensorIds));
  writeCurrentSensorsToStorage();
  currentSensorsCount = 0;

  Serial.println("Reset known sensors.");
}

void onButtonClicked() {
  currentMode = static_cast<Mode>((currentMode + 1) % ALWAYS_AT_END);
  Serial.print("Mode is now mode ");
  Serial.println(currentMode + 1);

  EEPROM.write(EEPROM_MODE_ADDRESS, currentMode);

  // Turn the LED off so that turning it on inside the loop makes a perceivable difference.
  digitalWrite(LED_PIN, LOW);
  delay(100);
  // Blink LED to show currently selected mode. Blink one more than mode value since that is zero-indexed
  // but first mode should blink at least once.
  for(int i = 0; i < currentMode + 1; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  delay(1000);

  // Update the LED to represent the state in the newly selected mode.
  updateTemperatureLed();
}

void setup()
{
  Serial.begin(115200);
  // Write the empty value into sensor IDs so that writing the array back into EEPROM marks these entries as "empty".
  memset(sensorIds, EEPROM_EMPTY_VALUE, NUM_SENSORS * sizeof(uint8_t));

  currentSensorsCount = 0;
  Serial.println("Restoring sensor IDs from EEPROM: ");
  for (size_t i = 0; i < NUM_SENSORS; i++)
  {
    uint8_t readId = EEPROM.read(EEPROM_SENSOR_IDS_ADDRESS + i);
    // If a special "empty" value was read, assume that this means there was no data (and not a sensor with an ID which randomly matched this value).
    if (readId == EEPROM_EMPTY_VALUE)
    {
      break;
    }
    sensorIds[currentSensorsCount++] = readId;
    Serial.print("- ");
    Serial.println(readId, HEX);
  }
  Serial.print("Finished reading stored sensor IDs. Read ");
  Serial.print(currentSensorsCount);
  Serial.println(" sensor IDs.");

  Serial.println("Restoring last mode...");
  uint8_t readModeIndex = EEPROM.read(EEPROM_MODE_ADDRESS);
  if(readModeIndex == EEPROM_EMPTY_VALUE || readModeIndex >= ALWAYS_AT_END) {
    currentMode = OUTSIDE_TEMPERATURE_LOWER;
    Serial.println("No valid mode found, switching to default mode.");
  }
  else {
    currentMode = static_cast<Mode>(readModeIndex);
    Serial.print("Valid mode found: ");
    Serial.println(currentMode);
  }

  pinMode(RECEIVER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), onReceiverChanged, CHANGE);

  button.setPressMs(1000);
  button.attachLongPressStop(onButtonLongPressed);
  button.attachClick(onButtonClicked);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void onPacketReceived(const SensorData &decoded)
{
  float apparentTemperature = calculateApparentTemperature(decoded.temperatureCelsius, decoded.humidity);

  // If we were searching for sensors and the sensor had a button pressed on it to send this transmission, store the sensor's ID.
  if (isSearchingForSensors() && !isKnownSensor(decoded.id) && decoded.isButtonPress)
  {
    addSensor(decoded.id);
  }

  // If sensor was not added during previous step, it is not one we are interested in.
  if (!isKnownSensor(decoded.id))
  {
    Serial.print("Received data from sensor with unknown ID ");
    Serial.print(decoded.id, HEX);
    Serial.println(", ignoring it.");
    return;
  }

  size_t sensorIndex = findSensorIndex(decoded.id);
  const SensorData &lastDataForThisSensor = lastDataPerSensor[sensorIndex];

  // If new data is same as old data, ignore.
  if (lastDataForThisSensor == decoded)
  {
    Serial.println("Received data which is not different from previous data, ignoring.");
    return;
  }

  // Store new data as last received data.
  lastDataPerSensor[sensorIndex] = decoded;
  lastApparentTemperatures[sensorIndex] = apparentTemperature;

  Serial.print("ID: ");
  Serial.print(decoded.id, HEX);
  Serial.print(", Temp Celsius: ");
  Serial.print(decoded.temperatureCelsius);
  Serial.print(" (feels like ");
  Serial.print(apparentTemperature);
  Serial.print("), Humidity: ");
  Serial.print(decoded.humidity);
  Serial.print(", Channel: ");
  Serial.print(decoded.channel);
  Serial.print(", Is button press: ");
  Serial.println(decoded.isButtonPress ? "true" : "false");

  if (shouldCompareTemperatures())
  {
    updateTemperatureLed();
  }
}

void loop()
{
  button.tick();

  for (size_t i = 0; i < currentSignalIndex; i++)
  {
    unsigned long currentGapDuration = gapDurations[i];
    SensorData *receivedData = parser.consumeGap(currentGapDuration);
    if (receivedData != nullptr)
    {
      onPacketReceived(*receivedData);
      delete receivedData;
    }
  }

  currentSignalIndex = 0;
  memset(gapDurations, 0, sizeof(gapDurations));
}