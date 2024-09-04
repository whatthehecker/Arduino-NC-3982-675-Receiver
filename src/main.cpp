#include <Arduino.h>
#include <OneButton.h>
#include <EEPROM.h>

const int RECEIVER_PIN = 32;
const int BUTTON_PIN = 27;
const int LED_PIN = 26;

size_t currentSignalIndex = 0;
const size_t MAX_GAPS_COUNT = 256;
unsigned long gapDurations[MAX_GAPS_COUNT];
unsigned long gapStartMicros = 0;

const int SYNC_PREFIX_GAP_DURATION = 8000;
const int SYNC_POSTFIX_GAP_DURATION = 16000;
const int TOLERANCE = 750;
const int HIGH_GAP_DURATION = 4000;
const int LOW_GAP_DURATION = 2000;

uint8_t dataBitIndex = 0;
// The number of data bits expected in each payload.
const size_t PAYLOAD_BITS_COUNT = 40;
const size_t PAYLOAD_BYTES_COUNT = PAYLOAD_BITS_COUNT / 8;
uint8_t dataBytes[PAYLOAD_BYTES_COUNT];

enum State
{
  SEARCHING_PREFIX,
  FOUND_PREFIX,
};
State currentState = SEARCHING_PREFIX;

OneButton button = OneButton(
    BUTTON_PIN,
    true,
    true);

// The number of weather sensors you want to connect to this program.
const size_t NUM_SENSORS = 2;
uint8_t sensorIds[NUM_SENSORS];
size_t currentSensorsCount = 0;

size_t EEPROM_SENSOR_IDS_ADDRESS = 0x0;
const uint8_t EEPROM_EMPTY_VALUE = 0xFF;

struct SensorData
{
  uint8_t id;
  uint8_t checksum;
  bool isButtonPress;
  float temperatureCelsius;
  uint8_t channel;
  uint8_t humidity;
};

// Stores the payload of the last transmission per sensor.
// This could also be defined as a nested 2-dimensional array, but for simplicity sake this works too.
uint8_t lastDataPerSensor[PAYLOAD_BYTES_COUNT * NUM_SENSORS];

float lastApparentTemperatures[NUM_SENSORS];

size_t findSensorIndex(uint8_t sensorId);
bool isKnownSensor(uint8_t sensorId);
bool isSearchingForSensors();
void writeCurrentSensorsToStorage();
bool shouldCompareTemperatures();
void updateTemperatureLed();
void addSensor(uint8_t sensorId);
float calculateApparentTemperature(float temperatureCelsius, float humidity);
SensorData decodeData(uint8_t *data);

SensorData decodeData(uint8_t *data)
{
  uint8_t id = data[0];
  uint8_t checksum = data[1] >> 4;
  uint16_t temperatureRaw = (data[2] << 4) | (data[3] >> 4);
  uint8_t channelRaw = data[4] & 0x0F;
  uint8_t isButtonPress = (data[1] >> 3) & 1;

  float temperatureFahrenheit = (temperatureRaw - 900) * 0.1f;
  float temperatureCelsius = (temperatureFahrenheit - 32.0) / 1.8;
  // Humidity is BCD-encoded with 2 decimal places, so multiply the first one by 10 to receive the decimal value.
  uint8_t humidity = (data[3] & 0x0F) * 10 + (data[4] >> 4);

  return {
      .id = id,
      .checksum = checksum,
      .isButtonPress = isButtonPress,
      .temperatureCelsius = temperatureCelsius,
      .channel = channelRaw,
      .humidity = humidity,
  };
}

float calculateApparentTemperature(float temperatureCelsius, float humidity)
{
  if (humidity >= 40 && temperatureCelsius >= 26.7)
  {
    // Formula and coefficients taken from https://en.wikipedia.org/wiki/Heat_index
    // Empty comments added because VSCode likes to line wrap somewhere else without them.
    return -8.784695                                                              //
           + 1.61139411 * temperatureCelsius                                      //
           + 2.338549 * humidity                                                  //
           - 0.14611605 * temperatureCelsius * humidity                           //
           - 1.2308094 * 1e-2 * temperatureCelsius * temperatureCelsius           //
           - 1.6424828 * 1e-2 * ((float)humidity) * humidity                      //
           + 2.211732 * 1e-3 * temperatureCelsius * temperatureCelsius * humidity //
           + 7.2546 * 1e-4 * temperatureCelsius * ((float)humidity) * humidity    //
           - 3.582 * 1e-6 * temperatureCelsius * temperatureCelsius * ((float)humidity) * humidity;
  }

  return temperatureCelsius;
}

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

  digitalWrite(LED_PIN, outsidePerceivedTemp < insidePerceivedTemp);

  Serial.print("Perceived temperatures: ");
  Serial.print(insidePerceivedTemp);
  Serial.print(" deg Celsius inside / ");
  Serial.print(outsidePerceivedTemp);
  Serial.println(" deg Celsius outside");
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
  EEPROM.commit();
}

/**
 * ISR for storing the duration of gaps after pulses.
 */
void IRAM_ATTR onReceiverChanged()
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

  Serial.println("Resetting known sensors.");
}

void setup()
{
  Serial.begin(115200);

  // On ESP32, this is needed since EEPROM is emulated through the flash storage, which in turn needs to know how many bytes this emulated storage should take up.
  EEPROM.begin(NUM_SENSORS * sizeof(uint8_t));
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
  Serial.println(" sensors IDs.");

  pinMode(RECEIVER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), onReceiverChanged, CHANGE);

  button.setPressMs(1000);
  button.attachLongPressStop(onButtonLongPressed);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

bool isBetween(int32_t value, int32_t lower, int32_t upper)
{
  return lower <= value && value <= upper;
}

void handleData(uint8_t *data)
{
  SensorData decoded = decodeData(data);
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
  uint8_t *lastDataForThisSensor = &(lastDataPerSensor[sensorIndex * PAYLOAD_BYTES_COUNT]);

  // If new data is same as old data, ignore.
  if (memcmp(lastDataForThisSensor, data, PAYLOAD_BYTES_COUNT) == 0)
  {
    Serial.println("Received data which is not different from previous data, ignoring.");
    return;
  }

  // Store new data as last received data.
  memcpy(&(lastDataPerSensor[sensorIndex * PAYLOAD_BYTES_COUNT]), data, PAYLOAD_BYTES_COUNT);
  lastApparentTemperatures[sensorIndex] = apparentTemperature;

  char buf[256];
  snprintf(buf, sizeof(buf),
           "ID: %02X, Temp Celsius: %2.2f (feels like %2.2f), Humidity: %u%%, Channel: %d, Is button press: %d",
           decoded.id, decoded.temperatureCelsius, apparentTemperature, decoded.humidity, decoded.channel, decoded.isButtonPress);
  Serial.println(buf);

  if (shouldCompareTemperatures())
  {
    updateTemperatureLed();
  }
}

void handleSignals()
{
  for (size_t i = 0; i < currentSignalIndex; i++)
  {
    unsigned long currentGapDuration = gapDurations[i];

    switch (currentState)
    {
    case SEARCHING_PREFIX:
      if (isBetween(currentGapDuration, SYNC_PREFIX_GAP_DURATION - TOLERANCE, SYNC_PREFIX_GAP_DURATION + TOLERANCE))
      {
        currentState = FOUND_PREFIX;
      }
      break;
    case FOUND_PREFIX:
      if (isBetween(currentGapDuration, SYNC_POSTFIX_GAP_DURATION - TOLERANCE, SYNC_POSTFIX_GAP_DURATION + TOLERANCE))
      {
        // Interpret data if we received the exact number of bits we expect and the last transmission
        // hasn't been too recent (otherwise this will fire multiple times in a row since the sensor sends
        // its data several times in a row).
        if (dataBitIndex == PAYLOAD_BITS_COUNT)
        {
          handleData(dataBytes);
        }

        dataBitIndex = 0;
        memset(dataBytes, false, sizeof(dataBytes));

        currentState = SEARCHING_PREFIX;
        break;
      }

      bool isHigh = isBetween(currentGapDuration, HIGH_GAP_DURATION - TOLERANCE, HIGH_GAP_DURATION + TOLERANCE);
      bool isLow = isBetween(currentGapDuration, LOW_GAP_DURATION - TOLERANCE, LOW_GAP_DURATION + TOLERANCE);

      // Reset received data if either:
      // - Received data is not a valid bit duration
      // - Transmission contains more bits than we expect
      // If either happens, discard data currently in cache and go back to searching for the next transmission.
      if (!isLow && !isHigh || dataBitIndex >= PAYLOAD_BITS_COUNT)
      {
        dataBitIndex = 0;
        memset(dataBytes, false, sizeof(dataBytes));

        currentState = SEARCHING_PREFIX;
        break;
      }

      // Write received bit into the correct byte at the correct index, starting from the MSB.
      dataBytes[dataBitIndex / 8] |= isHigh << (7 - (dataBitIndex % 8));
      dataBitIndex++;
      break;
    }
  }

  // Clear buffer with received gaps so that interrupt routine may fill the buffer again.
  currentSignalIndex = 0;
  memset(gapDurations, 0, sizeof(gapDurations));
}

void loop()
{
  button.tick();
  handleSignals();
}