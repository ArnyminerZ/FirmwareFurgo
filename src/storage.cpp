#include <Arduino.h>
#include <Preferences.h>

#include "debug.h"

#define PREFERENCES_NAMESPACE "furgo"

Preferences preferences;

void initialize_storage()
{
    if (preferences.begin(PREFERENCES_NAMESPACE, false))
    {
        infof("Preferences initialized");
    }
    else
    {
        error("Failed to initialize preferences");
    }
}

void set_uint16(const char *key, uint16_t &value, uint16_t defaultValue = 0)
{
    debugf("Setting uint16 value for key '%s'", key);
    if (preferences.putUInt(key, value) == 0)
    {
        errorf("Failed to set value for key '%s'", key);
    }
    else
    {
        debugf("Value for key '%s' set to %u", key, value);
    }
}

uint16_t get_uint16(const char *key, uint16_t defaultValue = 0)
{
    if (preferences.isKey(key))
    {
        uint16_t value = preferences.getUInt(key, defaultValue);
        debugf("Retrieved uint16 value for key '%s': %u", key, value);
        return value;
    }
    else
    {
        debugf("Key '%s' not found, returning default value: %u", key, defaultValue);
        return defaultValue;
    }
}
