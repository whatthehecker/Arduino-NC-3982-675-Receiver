#ifndef SIGNAL_PARSER_H
#define SIGNAL_PARSER_H
#include <stdint.h>
#include <stddef.h>

class SensorData
{
public:
    uint8_t id;
    bool isButtonPress;
    float temperatureCelsius;
    uint8_t channel;
    uint8_t humidity;

    bool operator==(const SensorData &other) const;

    SensorData() = default;

    SensorData(uint8_t id, bool isButtonPress,
               float temperatureCelsius, uint8_t channel, uint8_t humidity) : id(id), isButtonPress(isButtonPress), temperatureCelsius(temperatureCelsius), channel(channel), humidity(humidity) {}
};

enum State
{
    SEARCHING_PREFIX,
    FOUND_PREFIX,
};

class SignalParser
{
public:
    // The number of data bits expected in each payload.
    static const size_t PAYLOAD_BITS_COUNT = 40;
    static const size_t PAYLOAD_BYTES_COUNT = PAYLOAD_BITS_COUNT / 8;

    /*
     * Advances the internal state machine by handling to the given gap.
     *
     * If this gap marks the end of a valid data packet, the data of that packet it returned.
     */
    SensorData *consumeGap(unsigned long gapDuration);

private:
    static const int SYNC_PREFIX_GAP_DURATION = 8000;
    static const int SYNC_POSTFIX_GAP_DURATION = 16000;
    static const int TOLERANCE = 750;
    static const int HIGH_GAP_DURATION = 4000;
    static const int LOW_GAP_DURATION = 2000;

    static SensorData decodeData(uint8_t *data);
    static bool isBetween(int32_t value, int32_t lower, int32_t upper);

    State currentState = SEARCHING_PREFIX;
    uint8_t dataBitIndex = 0;
    uint8_t dataBytes[PAYLOAD_BYTES_COUNT];
};

#endif