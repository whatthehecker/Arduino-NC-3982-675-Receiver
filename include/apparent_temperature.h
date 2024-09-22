#ifndef APPARENT_TEMPERATURE_H
#define APPARENT_TEMPERATURE_H

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
#endif