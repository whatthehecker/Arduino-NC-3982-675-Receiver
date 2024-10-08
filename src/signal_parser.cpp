#include "signal_parser.h"
#include <string.h>

SensorData *SignalParser::consumeGap(unsigned long gapDuration)
{
    switch (currentState)
    {
    case SEARCHING_PREFIX:
        if (isAround(gapDuration, SYNC_PREFIX_GAP_DURATION, TOLERANCE))
        {
            currentState = FOUND_PREFIX;
        }
        break;
    case FOUND_PREFIX:
        if (isAround(gapDuration, SYNC_POSTFIX_GAP_DURATION, TOLERANCE))
        {
            if (dataBitIndex == PAYLOAD_BITS_COUNT)
            {
                return new SensorData(SensorData::decode(dataBytes));
            }

            dataBitIndex = 0;
            memset(dataBytes, false, sizeof(dataBytes));

            currentState = SEARCHING_PREFIX;
            break;
        }

        bool isHigh = isAround(gapDuration, HIGH_GAP_DURATION, TOLERANCE);
        bool isLow = isAround(gapDuration, LOW_GAP_DURATION, TOLERANCE);

        // Reset received data if either:
        // - Received data is not a valid bit duration
        // - Transmission contains more bits than we expect
        // If either happens, discard data currently in cache and go back to searching for the next transmission.
        if ((!isLow && !isHigh) || dataBitIndex >= PAYLOAD_BITS_COUNT)
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

bool SignalParser::isAround(int32_t value, int32_t target, int32_t tolerance)
{
    return (target - tolerance) <= value && value <= (target + tolerance);
}
