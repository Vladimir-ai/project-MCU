#include "modbus_driver.h"
#include <stddef.h>

#define CONCAT_TWO_BYTES(hi,lo)  ((uint16_t)(((hi) & 0xFF) << 8) | ((lo) & 0xFF))

extern modbus_ctx_t g_modbus_ctx;

/* Copypaste from the Internet */
static uint16_t MODBUS_CRC16_v1( const uint8_t *buf, const uint16_t len )
{
  uint16_t crc = 0xFFFF;
  uint16_t i = 0;
  uint8_t bit = 0;

  for( i = 0; i < len; i++ )
  {
    crc ^= buf[i];

    for( bit = 0; bit < 8; bit++ )
    {
      if( crc & 0x0001 )
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}


static modbus_status_t modbus_validate_crc(const uint8_t * const buf_start, const uint8_t buf_len)
{
  // uint16_t crc_expected_value = CONCAT_TWO_BYTES(buf_start[buf_len - 2], buf_start[buf_len - 1]);
  uint16_t crc_expected_value = buf_start[buf_len - 2] | buf_start[buf_len - 1] << 8;
  uint16_t crc_actual_value;
  modbus_status_t ret = MODBUS_STATUS_OK;
  crc_actual_value = MODBUS_CRC16_v1(buf_start, buf_len - 2);

  /* By spec, we should drop pkt in case if it is invalid. */
  if (crc_actual_value != crc_expected_value)
  {
    ret = MODBUS_EXCEPTION_DROP;
  }

  return ret;
}

/* Any action starts by check that function supported.
   Let's put it here to be in one place. */
static modbus_status_t modbus_check_if_func_supported(const uint8_t func_code)
{
  modbus_status_t ret;

  switch(func_code)
  {
    case MODBUS_FUNCTION_CODE_READ_COILS:
      ret = g_modbus_ctx.read_coils_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUT:
      ret = g_modbus_ctx.read_discrete_inputs_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS:
      ret = g_modbus_ctx.read_holding_registers_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTER:
      ret = g_modbus_ctx.read_input_registers_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_COIL:
      ret = g_modbus_ctx.write_coils_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_REGISTER:
      ret = g_modbus_ctx.write_registers_req == NULL ?
            MODBUS_EXCEPTION_ILLEGAL_FUNCTION : MODBUS_STATUS_OK;
      break;

    default:
      ret = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
      break;
  }

  return ret;
}

static modbus_status_t modbus_parse_read_function_and_call_cb(const uint8_t * const buf,
                                                              const uint8_t buf_len,
                                                              uint8_t *rsp_buf,
                                                              const modbus_read_call_t read_callback,
                                                              uint8_t *rsp_buf_size)
{
  modbus_status_t ret = MODBUS_STATUS_OK;
  uint16_t starting_address;
  uint16_t outputs_num;

  /* CRC was excluded earlier, after its check. */
  if (buf_len == 4)
  {
    starting_address = CONCAT_TWO_BYTES(buf[0], buf[1]);
    outputs_num = CONCAT_TWO_BYTES(buf[2], buf[3]);

    ret = read_callback(starting_address, outputs_num, rsp_buf, rsp_buf_size);
  }
  else
  {
    ret = MODBUS_EXCEPTION_DROP;
  }

  return ret;
}

static modbus_status_t modbus_parse_write_function_and_call_cb(const uint8_t * const buf,
                                                               const uint8_t buf_len,
                                                               uint8_t *rsp_buf,
                                                               const modbus_write_call_t write_callback,
                                                               uint8_t *rsp_buf_size)
{
  modbus_status_t ret = MODBUS_STATUS_OK;
  uint16_t output_address;
  uint16_t output_val;

  /* CRC was excluded earlier, after its check. */
  if (buf_len == 4)
  {
    output_address = CONCAT_TWO_BYTES(buf[0], buf[1]);
    output_val = CONCAT_TWO_BYTES(buf[3], buf[4]);

    ret = write_callback(output_address, output_val, rsp_buf, rsp_buf_size);
  }
  else
  {
    ret = MODBUS_EXCEPTION_DROP;
  }

  return ret;
}


static void modbus_send_exception(const uint8_t function_code,
                                  const modbus_status_t exception_code)
{
  uint8_t buf[MODBUS_MSG_LEN_EXCEPTION_RSP];
  uint16_t crc;

  buf[0] = g_modbus_ctx.address;
  buf[1] = function_code + MODBUS_EXCEPTION_OFFSET;
  buf[2] = exception_code;

  crc = MODBUS_CRC16_v1(buf, 3);

  buf[3] = crc & 0xFF;
  buf[4] = (crc >> 8) & 0xFF;

  g_modbus_ctx.tx_cb(buf, MODBUS_MSG_LEN_EXCEPTION_RSP);
}


static modbus_status_t modbus_build_and_tx_response(const uint8_t func_code,
                                                    const uint8_t * const buf,
                                                    const uint8_t payload_len)
{
  modbus_status_t ret;
  uint8_t rsp_buf[MODBUS_MAX_PKT_LEN];
  uint8_t *payload_ptr = &rsp_buf[2];
  uint8_t rsp_buf_size = MODBUS_MAX_PKT_LEN - 2 - 2; /* hdr, code, crc anything else isn't supported. */
  uint16_t crc;

  rsp_buf[0] = g_modbus_ctx.address;
  rsp_buf[1] = func_code;

  /* All this shit don't support subcodes, it'll be parsed separately just in case. */
  switch (func_code)
  {
    case MODBUS_FUNCTION_CODE_READ_COILS:
      ret = modbus_parse_read_function_and_call_cb(buf, payload_len, payload_ptr, g_modbus_ctx.read_coils_req, &rsp_buf_size);
      break;

    case MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUT:
      ret = modbus_parse_read_function_and_call_cb(buf, payload_len, payload_ptr, g_modbus_ctx.read_discrete_inputs_req, &rsp_buf_size);
      break;

    case MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS:
      ret = modbus_parse_read_function_and_call_cb(buf, payload_len, payload_ptr, g_modbus_ctx.read_holding_registers_req, &rsp_buf_size);
      break;

    case MODBUS_FUNCTION_CODE_READ_INPUT_REGISTER:
      ret = modbus_parse_read_function_and_call_cb(buf, payload_len, payload_ptr, g_modbus_ctx.read_input_registers_req, &rsp_buf_size);
      break;

    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_COIL:
      ret = modbus_parse_write_function_and_call_cb(buf, payload_len, payload_ptr, g_modbus_ctx.write_coils_req, &rsp_buf_size);
      break;

    case MODBUS_FUNCTION_CODE_WRITE_SINGLE_REGISTER:
      ret = modbus_parse_write_function_and_call_cb(buf, payload_len, rsp_buf, g_modbus_ctx.write_registers_req, &rsp_buf_size);
      break;

    default:
      ret = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
      break;
  }

  if (ret == MODBUS_STATUS_OK)
  {
    rsp_buf_size = MODBUS_MAX_PKT_LEN - rsp_buf_size;
    crc = MODBUS_CRC16_v1(rsp_buf, rsp_buf_size - 2);
    rsp_buf[rsp_buf_size - 2] = crc & 0xFFU;
    rsp_buf[rsp_buf_size - 1] = (crc >> 8) & 0xFFU;

    g_modbus_ctx.tx_cb(rsp_buf, rsp_buf_size);
  }

  return ret;
}

modbus_status_t modbus_serial_rx(uint8_t * buf_start, const uint8_t buf_len)
{
  uint8_t *buf = buf_start;
  uint8_t addr;
  uint8_t function_code;
  uint8_t payload_len = buf_len;
  modbus_status_t ret;

  addr = *buf++;
  (void) addr; // it's unused currently...
  function_code = *buf++;
  payload_len -= 2;

  /* Looks like that we should drop pkts in case of communication error... */
  ret = modbus_validate_crc(buf_start, buf_len);
  payload_len -=2;

  /* Communication error or empty pkt - drop. */
  if (ret != MODBUS_STATUS_OK || payload_len == 0)
  {
    goto finish;
  }

  ret = modbus_check_if_func_supported(function_code);

  if (ret != MODBUS_STATUS_OK)
  {
    goto finish;
  }

  ret = modbus_build_and_tx_response(function_code, buf, payload_len);

finish: /* Need this shit to drop pkt. Nothing else to do with it. */
  if (!MODBUS_CHECK_VAL_IN_RESERVED_VALUES(ret) && payload_len != 0)
  {
    modbus_send_exception(function_code, ret);
  }
  return ret;
}

void modbus_generate_read_input_registers_rsp(const uint16_t * const registers,
                                              const uint8_t reg_amount,
                                              uint8_t *buf,
                                              uint8_t *buf_len)
{
  uint16_t idx;
  uint8_t *buf_offset = buf;
  buf_offset[0] = reg_amount * 2U; // 2 * N from spec
  buf_offset++;
  *buf_len -= 1;

  for (idx = 0; idx < reg_amount; idx++)
  {
    buf_offset[0] = (registers[idx] >> 8) & 0xFF;
    buf_offset[1] = registers[idx] & 0xFF;

    buf_offset += 2;
    *buf_len -= 2;
  }
}


void modbus_init(modbus_ctx_t *modbus_ctx,
                 uint8_t addr,
                 modbus_read_call_t  read_coils_req,                   /*!< 6.1 by spec */
                 modbus_read_call_t  read_discrete_inputs_req,         /*!< 6.2 by spec */
                 modbus_read_call_t  read_holding_registers_req,       /*!< 6.3 by spec */
                 modbus_read_call_t  read_input_registers_req,         /*!< 6.4 by spec */
                 modbus_write_call_t write_coils_req,                  /*!< 6.5 by spec */
                 modbus_write_call_t write_registers_req,              /*!< 6.6 by spec */
                 modbus_serial_tx    tx_cb)
{
  modbus_ctx->address = addr;
  modbus_ctx->read_coils_req = read_coils_req;
  modbus_ctx->read_discrete_inputs_req = read_discrete_inputs_req;
  modbus_ctx->read_holding_registers_req = read_holding_registers_req;
  modbus_ctx->read_input_registers_req = read_input_registers_req;
  modbus_ctx->write_coils_req = write_coils_req;
  modbus_ctx->write_registers_req = write_registers_req;
  modbus_ctx->tx_cb = tx_cb;
}
