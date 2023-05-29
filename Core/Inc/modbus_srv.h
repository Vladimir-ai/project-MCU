#ifndef _MODBUS_SRV_H
#define _MODBUS_SRV_H

#include <stdint.h>

#define MIN_ADDR          0U
#define MAX_ADDR          3U

#define DEV_ADDR          1U

typedef struct register_table_s
{
  uint16_t registers[MAX_ADDR + 1];
  uint8_t ready;
} register_table_t;

extern register_table_t g_registers;

void modbus_srv_init(void);

#endif /* _MODBUS_SRV_H */