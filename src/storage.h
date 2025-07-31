#ifndef STORAGE__h
#define STORAGE__h

#define PREF_CONTROL_BITS "control_bits"

void initialize_storage();

void set_uint16(const char *key, uint16_t &value, uint16_t defaultValue = 0);
uint16_t get_uint16(const char *key, uint16_t defaultValue = 0);

#endif