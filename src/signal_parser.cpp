#include "signal_parser.h"
#include <cstring>

SensorData *SignalParser::consumeGap(unsigned long gapDuration)
{
    switch (currentState)
    {
    case SEARCHING_PREFIX:
        if (isBetween(gapDuration, SYNC_PREFIX_GAP_DURATION - TOLERANCE, SYNC_PREFIX_GAP_DURATION + TOLERANCE))
        {
            currentState = FOUND_PREFIX;
        }
        break;
    case FOUND_PREFIX:
        if (isBetween(gapDuration, SYNC_POSTFIX_GAP_DURATION - TOLERANCE, SYNC_POSTFIX_GAP_DURATION + TOLERANCE))
        {
            // Interpret data if we received the exact number of bits we expect and the last transmission
            // hasn't been too recent (otherwise this will fire multiple times in a row since the sensor sends
            // its data several times in a row).
            if (dataBitIndex == PAYLOAD_BITS_COUNT)
            {
                return new SensorData(decodeData(dataBytes));
            }

            dataBitIndex = 0;
            memset(dataBytes, false, sizeof(dataBytes));

            currentState = SEARCHING_PREFIX;
            break;
        }

        bool isHigh = isBetween(gapDuration, HIGH_GAP_DURATION - TOLERANCE, HIGH_GAP_DURATION + TOLERANCE);
        bool isLow = isBetween(gapDuration, LOW_GAP_DURATION - TOLERANCE, LOW_GAP_DURATION + TOLERANCE);

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

    return nullptr;
}

SensorData SignalParser::decodeData(uint8_t *data)
{
    uint8_t id = data[0];
    uint16_t temperatureRaw = (data[2] << 4) | (data[3] >> 4);
    uint8_t channelRaw = data[4] & 0x0F;
    uint8_t isButtonPress = (data[1] >> 3) & 1;

    float temperatureFahrenheit = (temperatureRaw - 900) * 0.1f;
    float temperatureCelsius = (temperatureFahrenheit - 32.0) / 1.8;
    // Humidity is BCD-encoded with 2 decimal places, so multiply the first one by 10 to receive the decimal value.
    uint8_t humidity = (data[3] & 0x0F) * 10 + (data[4] >> 4);

    return SensorData(id, isButtonPress, temperatureCelsius, channelRaw, humidity);
}

bool SignalParser::isBetween(int32_t value, int32_t lower, int32_t upper)
{
    return lower <= value && value <= upper;
}

bool SensorData::operator==(const SensorData &other) const
{
    return id == other.id && isButtonPress == other.isButtonPress && temperatureCelsius == other.temperatureCelsius && channel == other.channel && humidity == other.humidity;
}
