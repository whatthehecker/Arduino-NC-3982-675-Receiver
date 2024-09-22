#include "sensor_data.h"

bool SensorData::operator==(const SensorData &other) const
{
    return id == other.id && isButtonPress == other.isButtonPress && temperatureCelsius == other.temperatureCelsius && channel == other.channel && humidity == other.humidity;
}

SensorData SensorData::decode(const uint8_t *data)
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