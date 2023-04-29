# High-level design

Modbus feature is located in `FreeRTOSver` directory.
Magnetometer is turned off there.

## Threading

Target project should have two threads:

- modbus - reads commands from usb and answers them (check `CDC_Receive_FS` function)
- magnetometer - it is a "logical" thread - should be called by timer (TIM16 in our case) or by inner threshold value (see [magnetometer datasheet](https://www.st.com/resource/en/datasheet/lsm303agr.pdf))

## Modbus

Modbus have been written using structure that contains necessary callbacks.
This task uses only type of callbacks.
Idea is:

1. Parse received pkt.
2. Check that response can be generated.
3. Call user callback
4. User should put his info into arrays.
5. User should call function which generates response.

I've used [this spec](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf).
Currently, it isn't working with PC.