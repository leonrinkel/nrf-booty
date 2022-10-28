#ifndef DRV_SX1509_H
#define DRV_SX1509_H

#include <stdint.h>

#include "nrf_drv_twi.h"
#include "app_error.h"

#define _REG_InputDisableA    0x01
#define SX1509_REG_PullUpA          0x07
#define SX1509_REG_OpenDrainA       0x0B
#define SX1509_REG_DirA             0x0F
#define SX1509_REG_Clock            0x1E
#define SX1509_REG_Misc             0x1F
#define SX1509_REG_LEDDriverEnableA 0x21

#define SX1509_REG_TOn5   0x3A
#define SX1509_REG_IOn5   0x3B
#define SX1509_REG_Off5   0x3C
#define SX1509_REG_TRise5 0x3D
#define SX1509_REG_TFall5 0x3E

#define SX1509_REG_TOn6   0x3F
#define SX1509_REG_IOn6   0x40
#define SX1509_REG_Off6   0x41
#define SX1509_REG_TRise6 0x42
#define SX1509_REG_TFall6 0x43

#define SX1509_REG_TOn7   0x44
#define SX1509_REG_IOn7   0x45
#define SX1509_REG_Off7   0x46
#define SX1509_REG_TRise7 0x47
#define SX1509_REG_TFall7 0x48

/**
 * @brief Structure for the SX1509 driver configuration.
 */
typedef struct
{
    const nrf_drv_twi_t* p_twi_instance;
    const nrf_drv_twi_config_t* p_twi_config;
    uint8_t address;
} drv_sx1509_conf_t;

ret_code_t drv_sx1509_open(const drv_sx1509_conf_t* p_conf);

void drv_sx1509_close(const drv_sx1509_conf_t* p_conf);

ret_code_t drv_sx1509_reg_get(
    const drv_sx1509_conf_t* p_conf,
    uint8_t reg_addr,
    uint8_t* p_value
);

ret_code_t drv_sx1509_reg_set(
    const drv_sx1509_conf_t* p_conf,
    uint8_t reg_addr,
    uint8_t value
);

#endif /* DRV_SX1509_H */
