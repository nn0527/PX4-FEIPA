<<<<<<< HEAD
# Raw fixed-length frame received from an ESP32 over UART.
=======
# ESP32 UART raw frame (7 bytes: AA 55 d_h d_l sensor_id 0D 0A)
# sensor_id: 0 = body -Z suction/contact sensor (legacy default), 1 = body +X/front sensor
uint64 timestamp
>>>>>>> c49fcf5256 (add wallperch PX4 code)

uint64 timestamp # time since system start (microseconds)
uint8[7] frame  # AA 55 distance_high distance_low reserved 0D 0A

uint8 ORB_QUEUE_LENGTH = 8
