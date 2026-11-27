#ifndef BSP_STATUS_H
#define BSP_STATUS_H

typedef enum {
    BSP_OK = 0,
    BSP_INVALID_ARGUMENT,
    BSP_NOT_READY,
    BSP_BUSY,
    BSP_IO_ERROR,
    BSP_OVERFLOW_AMBIGUOUS
} bsp_status_t;

#endif
