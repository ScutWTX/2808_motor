#include <stddef.h>
#include "mb_slave.h"
 
#define __MB_SLAVE_DEV(handle) ((mb_slave_dev_t *)handler)
 
mb_slave_handle mb_slave_init(mb_slave_dev_t *p_dev, uint8_t slave_addr, enum mb_mode mode)
{
    if (!p_dev) 
        return NULL;
    
    p_dev->mode = mode;
    switch (mode) {
        case MB_MODE_RTU:
            p_dev->common.rtu.slave_addr = slave_addr;
            break;
        case MB_MODE_ASCII:
        case MB_MODE_TCP:
            break;
        default:
            break;
    }
    
    return &p_dev->serv;
}
