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
                return new SensorData(SensorData::decode(dataBytes));
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

bool SignalParser::isBetween(int32_t value, int32_t lower, int32_t upper)
{
    return lower <= value && value <= upper;
}
