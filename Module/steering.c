#include "steering.h"

#define UART_PTR &huart4
#define MAX_ANGLE 270
#define BUFFER_SIZE  10

uint8_t steering_cmd[BUFFER_SIZE];