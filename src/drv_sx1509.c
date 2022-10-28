#include "drv_sx1509.h"

/**
 * @brief Initializes and enables TWI according to supplied configuration.
 *
 * @param[in]   p_conf  Configuration to use.
 */
ret_code_t drv_sx1509_open(const drv_sx1509_conf_t* p_conf)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_init(
        p_conf->p_twi_instance,
        p_conf->p_twi_config,
        NULL,
        NULL
    );
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    nrf_drv_twi_enable(p_conf->p_twi_instance);

    return NRF_SUCCESS;
}

/**
 * @brief Disables and uninitializes TWI.
 *
 * @param[in]   p_conf  Configuration to use.
 */
void drv_sx1509_close(const drv_sx1509_conf_t* p_conf)
{
    nrf_drv_twi_disable(p_conf->p_twi_instance);
    nrf_drv_twi_uninit(p_conf->p_twi_instance);
}

/**
 * @brief Reads register of SX1509 via TWI.
 *
 * @param[in]   p_conf      Configuration to use.
 * @param[in]   reg_addr    Address of register to read.
 * @param[out]  p_value     Where to store read value.
 */
ret_code_t drv_sx1509_reg_get(
    const drv_sx1509_conf_t* p_conf,
    uint8_t reg_addr,
    uint8_t* p_value
)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(
        p_conf->p_twi_instance,
        p_conf->address,
        &reg_addr,
        1,
        true
    );
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

/**
 * @brief Writes register of SX1509 via TWI.
 *
 * @param[in]   p_conf      Configuration to use.
 * @param[in]   reg_addr    Address of register to write.
 * @param[in]   p_value     Pointer of value to write.
 */
ret_code_t drv_sx1509_reg_set(
    const drv_sx1509_conf_t* p_conf,
    uint8_t reg_addr,
    uint8_t value
)
{
    uint8_t tx_buffer[2] = { reg_addr, value };

    return nrf_drv_twi_tx(
        p_conf->p_twi_instance,
        p_conf->address,
        &(tx_buffer[0]),
        2,
        true
    );
}
