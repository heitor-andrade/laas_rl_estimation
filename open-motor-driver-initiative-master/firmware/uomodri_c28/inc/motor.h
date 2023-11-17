#ifndef __MOTOR_H__
#define __MOTOR_H__

/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "f2838x_device.h"
#include "hal.h"
#include "foc.h"
#include "drv8353.h"
#include "uomodri_user_defines.h"

/***********************************************************************
 * DEFINES
 ***********************************************************************/

/***********************************************************************
 * TYPDEF ENUM
 ***********************************************************************/
/**
 * @enum    motor_state_e
 * @brief   Motor states for the Finite State Machine (FSM).
 */
typedef enum
{
    MOTOR_STATE_INIT            = 0x00,
    MOTOR_STATE_ALIGN_UP        = 0x01,
    MOTOR_STATE_ALIGN_FIX       = 0x02,
    MOTOR_STATE_READY           = 0x04,
    MOTOR_STATE_STOP            = 0x08,
    MOTOR_STATE_COMPUTE_RL      = 0x10,
    MOTOR_STATE_ERROR           = 0xFF,
} motor_state_e;

/***********************************************************************
 * CONTROL STRUCTURES
 ***********************************************************************/
typedef struct __error_reg_t__
{
    uint16_t    enc_mismatch    : 1;                /*!< Bits 0   : R/W - Encoder error too high */
    uint16_t    pos_rollover    : 1;                /*!< Bits 1   : R/W - Position roll-over occurred */
    uint16_t    drv_fault       : 1;                /*!< Bits 2   : R/W - Motor DRV nFault error */
    uint16_t    com_timeout     : 1;                /*!< Bits 3   : R/W - Communication timeout error */
} error_reg_t;

typedef union __error_reg_u__
{
    uint16_t    all;
    error_reg_t bit;
} error_reg_u;

/**
 * @union   hal_motor_cfg_u
 * @brief   Force a 32b address pointer to be allocated on all processors (CLA, CM \& CPUx).
 */
typedef union __hal_motor_cfg_u__
{
  const hal_motor_cfg_t* const      ptr;            /*!< Constant pointer on a constant hal_motor_cfg_t structure declaration. */
  const uint32_t                    align;          /*!< Force a 32bits allocation. Address is constant. */
} hal_motor_cfg_u;

typedef union __drv8353_u__
{
    drv8353_t* const                ptr;
    const uint32_t                  align;
} drv8353_u;

typedef union __foc_control_u__
{
    foc_t* const                    ptr;
    const uint32_t                  align;
} foc_u;

typedef union __uint16_u__
{
    volatile uint16_t* const        ptr;
    volatile const uint32_t         align;
} uint16_u;

typedef struct __estimator_rl__
{
    float32_t                       statorResEst;               /*!< Stator resistance estimation. [Ohm] */
    float32_t                       inf_dtc;                    /*!< Minimum duty-cicle utilised to resistance estimation [no measurement unit] */
    float32_t                       inf_vbus;                   /*!< Voltage at source when min_dtc is assigned [V] */
    float32_t                       sup_dtc;                    /*!< Maximum duty-cicle utilised to resistance estimation [no measurement unit] */
    float32_t                       sup_vbus;                   /*!< Voltage at source when max_dtc is assigned [V] */
    bool                            dtc_initialize;            /*!< Indicates whether the duty cycle should be initialized */
    bool                            comp_resistance;            /*!< Indicates whether the resistance should be computed */
    
    float32_t                       statorIndEst;               /*!< Q-axis inductance estimation. [Henry] */
    float32_t                       current_flt;                /*!< Filtered current */
    float32_t                       amplitude;                  /*!< Current amplitude [A] */
    float32_t                       inf_current;                /*!< Minimum current value when sinusoidal voltage is applied [A] */
    float32_t                       sup_current;                /*!< Maximum current value when sinusoidal voltage is applied [A] */
    float32_t                       A0;                         /*!< First amplitude calculated */
    float32_t                       gain_current;               /*!< Gain in current amplitude compared to A0 */
    float32_t                       frequence;                  /*!< Frequency applied to the voltage [Hz] */
    float32_t                       frequence_ressonance;       /*!< First frequency that causes a gain of less than 1/sqrt(2). Frequency used directly in the inductance calculation. [Hz]*/
    uint32_t                        frequence_iteration;        /*!< Indicates the iteration of the code in which the frequency value was changed */
    bool                            initialize_current_flt;     /*!< Indicates whether the current filter should be initialized */
    bool                            first_frequence;            /*!< Indicates whether the current frequency is the first one applied */

} estimator_rl;


typedef struct __MOTOR_STRUCT_t__
{
    const uint8_t                   motor_id;       /*!< Motor identification */
    const hal_motor_cfg_t*  const   p_motorHalCfg;  /*!< Pointer on the HAL structure (PWM, ADC , IT) associated to the motor */
    drv8353_t* const                p_motorDRV;     /*!< Pointer on the DRV structure associated to the motor */
    foc_t* const                    p_motorFOC;     /*!< Pointer on the FOC structure associated to the motor */
    estimator_rl* const             p_motorRLE;     /*!< Pointer on the RL estimator structure associated to the motor */
    uint64_t                        itCnt;          /*!< Event counter incrementing on every IT call */
    uint32_t                        clCycleNb;

    float32_t                       test;
    float32_t                       test1;
    float32_t                       test2;

    motor_state_e                   motor_state;    /*!< Current motor state for the FSM. */
    error_reg_u                     motor_error;    /*!< Error messages */
    // PWM address register for FOC command
    volatile uint16_t* const        p_motorChAReg;
    volatile uint16_t* const        p_motorChBReg;
    volatile uint16_t* const        p_motorChCReg;
    bool_t                          itDone;
} motor_t;

/***********************************************************************
 * FUNCTIONS DECLARATION
 ***********************************************************************/
inline void MOT_runCommand(motor_t*, float32_t, float32_t, float32_t);
inline void MOT_stopCommand(motor_t*);
inline bool_t MOT_runControl(motor_t*);

#endif /* __MOTOR_H__ */
