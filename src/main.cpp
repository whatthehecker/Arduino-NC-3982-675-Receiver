#include <Arduino.h>

const int RECEIVER_PIN = 32;

const size_t MAX_GAPS_COUNT = 256;

size_t currentSignalIndex = 0;
unsigned long gapDurations[MAX_GAPS_COUNT];

uint32_t currentCounter = 0;

unsigned long gapStartMicros;

const int SYNC_PREFIX_GAP_DURATION = 8000;
const int SYNC_POSTFIX_GAP_DURATION = 16000;
const int TOLERANCE = 750;
const int HIGH_GAP_DURATION = 4000;
const int LOW_GAP_DURATION = 2000;

bool didFindPrefix = false;
uint8_t dataBitsCount = 0;

const size_t PAYLOAD_BITS_COUNT = 40;
bool dataBits[PAYLOAD_BITS_COUNT];

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
  if (currentSignalIndex > 0)
  {
    for (size_t i = 0; i < currentSignalIndex; i++)
    {
      unsigned long currentGapDuration = gapDurations[i];

      if (!didFindPrefix && isBetween(currentGapDuration, SYNC_PREFIX_GAP_DURATION - TOLERANCE, SYNC_PREFIX_GAP_DURATION + TOLERANCE))
      {
        didFindPrefix = true;
        Serial.println("Found prefix, interpreting next values as data...");
        continue;
      }

      else if (!didFindPrefix)
      {
        continue;
      }

      if (isBetween(currentGapDuration, SYNC_POSTFIX_GAP_DURATION - TOLERANCE, SYNC_POSTFIX_GAP_DURATION + TOLERANCE))
      {
        Serial.println("Found postfix, stopping interpreting data.");
        char buf[80];
        snprintf(buf, sizeof(buf), "Read %d bytes of data.", dataBitsCount);
        Serial.println(buf);

        if (dataBitsCount == PAYLOAD_BITS_COUNT)
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
        }

        didFindPrefix = false;
        dataBitsCount = 0;
        memset(dataBits, false, sizeof(dataBits));
        continue;
      }

      bool isHigh = isBetween(currentGapDuration, HIGH_GAP_DURATION - TOLERANCE, HIGH_GAP_DURATION + TOLERANCE);
      bool isLow = isBetween(currentGapDuration, LOW_GAP_DURATION - TOLERANCE, LOW_GAP_DURATION + TOLERANCE);

      if (!isLow && !isHigh)
      {
        char buf[80];
        snprintf(buf, sizeof(buf), "Found invalid gap with length %ld, going back to prefix search.", gapDurations[i]);
        Serial.println(buf);

        didFindPrefix = false;
        dataBitsCount = 0;
        memset(dataBits, false, sizeof(dataBits));
        continue;
      }
      else if (dataBitsCount >= PAYLOAD_BITS_COUNT)
      {
        Serial.println("Received too many data bits, ignoring the rest.");

        didFindPrefix = false;
        dataBitsCount = 0;
        memset(dataBits, false, sizeof(dataBits));
        continue;
      }
      else
      {
        dataBits[dataBitsCount++] = isHigh;
      }
    }

    currentSignalIndex = 0;
    memset(gapDurations, 0, MAX_GAPS_COUNT);
  }
}