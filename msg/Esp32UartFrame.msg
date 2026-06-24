# Raw fixed-length frame received from an ESP32 over UART.

uint64 timestamp # time since system start (microseconds)
uint8[7] frame  # AA 55 distance_high distance_low reserved 0D 0A

uint8 ORB_QUEUE_LENGTH = 8
