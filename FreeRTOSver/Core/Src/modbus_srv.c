#include "modbus_srv.h"
#include "modbus_driver.h"
#include "usbd_cdc_if.h"

modbus_ctx_t g_modbus_ctx;
register_table_t g_registers;

static modbus_status_t read_input_registers_req_user(const uint16_t starting_addr,
                                                     const uint16_t reg_num,
                                                     uint8_t *rsp_buf,
                                                     uint8_t *rsp_buf_size)
{
  modbus_status_t ret;
  uint8_t bufsize = *rsp_buf_size;

  if (starting_addr + reg_num - 1 > MAX_ADDR)
  {
    ret = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }
  else if (!g_registers.ready)
  {
    ret = MODBUS_EXCEPTION_DROP;
  }
  else
  {
    ret = MODBUS_STATUS_OK;

    modbus_generate_read_input_registers_rsp(&g_registers.registers[starting_addr],
                                             reg_num,
                                             rsp_buf,
                                             rsp_buf_size);
  }

  return ret;
}


void modbus_srv_init(void)
{
  memset(&g_registers, 0, sizeof(g_registers));
  modbus_init(&g_modbus_ctx, DEV_ADDR, NULL, NULL, NULL, read_input_registers_req_user, NULL, NULL, CDC_Transmit_FS);
}
