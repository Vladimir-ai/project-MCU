#ifndef _MODBUS_DRIVER
#define _MODBUS_DRIVER

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_MAX_PKT_LEN                             256U /* From spec 2.5.1 */
#define MODBUS_MAX_PAYLOAD_LEN                         (MODBUS_MAX_PKT_LEN - 4) /* address, func_code, crc */


/* DATA ACCESS */
/* Physical Discrete Inputs */
#define MODBUS_FUNCTION_CODE_READ_DISCRETE_INPUT       0x02U /* Read Discrete Inputs */

/* Internal Bits Or Physical coils */
#define MODBUS_FUNCTION_CODE_READ_COILS                0x01U /* Read Coils */
#define MODBUS_FUNCTION_CODE_WRITE_SINGLE_COIL         0x05U /* Write Single Coil */
#define MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_COILS      0x0FU /* Write Multiple Coils */

/* Physical Input Registers */
#define MODBUS_FUNCTION_CODE_READ_INPUT_REGISTER       0x04U /* Read Input Register */

/* Internal Registers Or Physical Output Registers */
#define MODBUS_FUNCTION_CODE_READ_HOLDING_REGISTERS    0x03U /* Read Holding Registers */
#define MODBUS_FUNCTION_CODE_WRITE_SINGLE_REGISTER     0x06U /* Write Single Register */
#define MODBUS_FUNCTION_CODE_WRITE_MULTIPLE_REGISTERS  0x10U /* Write Multiple Registers */
#define MODBUS_FUNCTION_CODE_RW_MULTIPLE_REGISTERS     0x17U /* Read/Write Multiple Registers */
#define MODBUS_FUNCTION_CODE_MASK_WRITE_REGISTER       0x16U /* Mask Write Register */
#define MODBUS_FUNCTION_CODE_READ_FIFO_QUEUE           0x18U /* Read FIFO queue */

/* File record access */
#define MODBUS_FUNCTION_CODE_READ_FILE_RECORD          0x14U /* Read File record */
#define MODBUS_FUNCTION_CODE_WRITE_FILE_RECORD         0x15U /* Write File record */

/* Diagnostics */
#define MODBUS_FUNCTION_CODE_READ_EXCEPTION_STATUS     0x07U /* Read Exception status */
#define MODBUS_FUNCTION_CODE_DIAGNOSTIC                0x08U /* Diagnostic */
#define MODBUS_FUNCTION_CODE_GET_COME_EVT_CTR          0x0BU /* Get Com event counter */
#define MODBUS_FUNCTION_CODE_GET_COME_EVT_LOG          0x0CU /* Get Com Event Log */
#define MODBUS_FUNCTION_CODE_REPORT_SERVER_ID          0x11U /* Report Server ID */
#define MODBUS_FUNCTION_CODE_READ_DEVICE_ID            0x2BU /* Read device Identification */

/* Other */
#define MODBUS_FUNCTION_CODE_READ_DEVICE_ID            0x2BU /* Encapsulated Interface Transport */


#define MODBUS_EXCEPTION_OFFSET                        0x80U /* 7 in spec */
/* Exception codes */

#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION                             0x01U
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS                         0x02U
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE                           0x03U
#define MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE                        0x04U
#define MODBUS_EXCEPTION_ACKNOWLEDGE                                  0x05U
#define MODBUS_EXCEPTION_SERVER_DEVICE_BUSY                           0x06U
#define MODBUS_EXCEPTION_MEMORY_PARITY_ERROR                          0x08U
#define MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAILABLE                     0x0AU
#define MODBUS_EXCEPTION_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND      0x0BU

/* Custom exceptions */
#define MODBUS_STATUS_OK                                              0x00U /* Not by spec, use 0 as OK status. */
#define MODBUS_EXCEPTION_DROP                                         0xFFU /* No such code in spec. */

#define MODBUS_MSG_LEN_EXCEPTION_RSP                                  5 /* addr (1), code (1), error (1), crc (2) */

/* These values will never cause a real exception! */
#define MODBUS_CHECK_VAL_IN_RESERVED_VALUES(ret)  ((ret) == (MODBUS_STATUS_OK) || (ret) == (MODBUS_EXCEPTION_DROP))

typedef uint8_t modbus_status_t;

typedef modbus_status_t (*modbus_read_call_t)(const uint16_t starting_addr, const uint16_t reg_num, uint8_t *rsp_buf, uint8_t *rsp_buf_size);
typedef modbus_status_t (*modbus_write_call_t)(const uint16_t addr, const uint16_t value, uint8_t *rsp_buf, uint8_t *rsp_buf_size);

typedef void (*modbus_serial_tx)(const uint8_t const *buf, const uint16_t len);

typedef struct modbus_ctx_s
{
  uint8_t address;
  modbus_read_call_t  read_coils_req;                   /*!< 6.1 by spec */
  modbus_read_call_t  read_discrete_inputs_req;         /*!< 6.2 by spec */
  modbus_read_call_t  read_holding_registers_req;       /*!< 6.3 by spec */
  modbus_read_call_t  read_input_registers_req;         /*!< 6.4 by spec */
  modbus_write_call_t write_coils_req;                  /*!< 6.5 by spec */
  modbus_write_call_t write_registers_req;              /*!< 6.6 by spec */

  modbus_serial_tx    tx_cb;                            /*!< NOT DEFINED in spec, our own callback to call uart/usb/etc. */
} modbus_ctx_t;


void modbus_init(modbus_ctx_t *modbus_ctx,
                 uint8_t addr,
                 modbus_read_call_t  read_coils_req,                   /*!< 6.1 by spec */
                 modbus_read_call_t  read_discrete_inputs_req,         /*!< 6.2 by spec */
                 modbus_read_call_t  read_holding_registers_req,       /*!< 6.3 by spec */
                 modbus_read_call_t  read_input_registers_req,         /*!< 6.4 by spec */
                 modbus_write_call_t write_coils_req,                  /*!< 6.5 by spec */
                 modbus_write_call_t write_registers_req,              /*!< 6.6 by spec */
                 modbus_serial_tx    tx_cb);

void modbus_generate_read_input_registers_rsp(const uint16_t const *registers,
                                              const uint8_t reg_amount,
                                              uint8_t *buf,
                                              uint8_t *buf_len);

modbus_status_t modbus_serial_rx(const uint8_t const *buf_start, const uint8_t buf_len);

#endif /* _MODBUS_DRIVER */