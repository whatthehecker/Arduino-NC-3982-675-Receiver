#include <Arduino.h>

const int RECEIVER_PIN = 32;

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
bool dataBits[PAYLOAD_BITS_COUNT];

enum State {
  SEARCHING_PREFIX, FOUND_PREFIX
};
State currentState = SEARCHING_PREFIX;

unsigned long lastFullTransmissionMillis = 0;
const unsigned long MIN_DELAY_BETWEEN_TRANSMISSIONS = 20000;

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

  bool isNowHigh = digitalRead(RECEIVER_PIN) == HIGH;

  // If this is the start of a gap, start measuring time.
  if (!isNowHigh)
  {
    gapStartMicros = micros();
  }
  // If this is the end of a gap (a start of a pulse), note down the elapsed time and store it.
  else
  {
    unsigned long timeDiff = micros() - gapStartMicros;
    gapDurations[currentSignalIndex++] = timeDiff;
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(RECEIVER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_PIN), onReceiverChanged, CHANGE);
}

bool isBetween(int32_t value, int32_t lower, int32_t upper)
{
  return lower <= value && value <= upper;
}

void printDecodedData(uint8_t *data)
{
  uint8_t id = data[0];
  uint8_t checksum = data[1] >> 4;
  // TODO: figure out why data[2] is being shifted although it is declared with full 8 bits of data.
  uint16_t temperatureRaw = (data[2] << 4) | (data[3] >> 4);
  uint8_t channelRaw = data[4] & 0x0F;
  uint8_t isButtonPress = (data[1] >> 3) & 1;

  float temperatureFahrenheit = (temperatureRaw - 900) * 0.1f;
  float temperatureCelsius = (temperatureFahrenheit - 32.0) / 1.8;
  // Humidity is BCD-encoded with 2 decimal places, so multiply the first one by 10 to receive the decimal value.
  uint8_t humidity = (data[3] & 0x0F) * 10 + (data[4] >> 4);

  char buf[256];
  snprintf(buf, sizeof(buf), "ID: %02x\nTemp Celsius: %2.2f\nHumidity: %u%%\nChannel: %d\nIs button press: %d", id, temperatureCelsius, humidity, channelRaw, isButtonPress);
  Serial.println(buf);

  return;
}

void loop()
{

  for (size_t i = 0; i < currentSignalIndex; i++)
  {
    unsigned long currentGapDuration = gapDurations[i];

    switch(currentState) {
      case SEARCHING_PREFIX:
        if(isBetween(currentGapDuration, SYNC_PREFIX_GAP_DURATION - TOLERANCE, SYNC_PREFIX_GAP_DURATION + TOLERANCE)) {
          currentState = FOUND_PREFIX;
        }
        break;
      case FOUND_PREFIX:
        if(isBetween(currentGapDuration, SYNC_POSTFIX_GAP_DURATION - TOLERANCE, SYNC_POSTFIX_GAP_DURATION + TOLERANCE)) {
          char buf[80];
          snprintf(buf, sizeof(buf), "Read %d bytes of data.", dataBitIndex);
          Serial.println(buf);

          // Interpret data if we received the exact number of bits we expect and the last transmission
          // hasn't been too recent (otherwise this will fire multiple times in a row since the sensor sends
          // its data several times in a row).
          if (dataBitIndex == PAYLOAD_BITS_COUNT && millis() - lastFullTransmissionMillis >= MIN_DELAY_BETWEEN_TRANSMISSIONS)
          {
            Serial.println("Data as hex: ");
            size_t bytesCount = PAYLOAD_BITS_COUNT / 8;
            uint8_t dataBytes[bytesCount];

            for (size_t byteIndex = 0; byteIndex < bytesCount; byteIndex++)
            {
              dataBytes[byteIndex] = 0;
              for (size_t bitIndex = 0; bitIndex < 8; bitIndex++)
              {
                dataBytes[byteIndex] |= (dataBits[bitIndex + byteIndex * 8] << (7 - bitIndex));
              }

              Serial.print(dataBytes[byteIndex], HEX);
              Serial.print(" ");
            }

            Serial.println();

            printDecodedData(dataBytes);
            lastFullTransmissionMillis = millis();
          }

          dataBitIndex = 0;
          memset(dataBits, false, sizeof(dataBits));

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
          memset(dataBits, false, sizeof(dataBits));

          currentState = SEARCHING_PREFIX;
          break;
        }

        dataBits[dataBitIndex++] = isHigh;
        break;
    }
  }

  // Clear buffer with received gaps so that interrupt routine may fill the buffer again.
  currentSignalIndex = 0;
  memset(gapDurations, 0, MAX_GAPS_COUNT);
}