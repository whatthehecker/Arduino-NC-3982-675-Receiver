#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include <stdint.h>

class SensorData
{
public:
    uint8_t id;
    bool isButtonPress;
    float temperatureCelsius;
    uint8_t channel;
    uint8_t humidity;

    static SensorData decode(uint8_t *data);

    bool operator==(const SensorData &other) const;

    SensorData() = default;

    SensorData(uint8_t id, bool isButtonPress,
               float temperatureCelsius, uint8_t channel, uint8_t humidity) : id(id), isButtonPress(isButtonPress), temperatureCelsius(temperatureCelsius), channel(channel), humidity(humidity) {}
};
#endif