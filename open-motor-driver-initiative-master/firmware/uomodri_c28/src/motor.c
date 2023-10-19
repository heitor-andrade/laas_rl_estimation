/***********************************************************************
 * INCLUDE FILES
 ***********************************************************************/
#include "motor.h"
#include "encoder.h"

/***********************************************************************
 * FUNCTIONS DEFINITIONS
 ***********************************************************************/
/**
 * @brief       	Command for the 3 phases of the ePWM
 * @param[out]      p_motor Pointer on the associated motor structure
 * @param[in]       cmd_a   Normalized command on PWM channel A (between 0 and 1)
 * @param[in]       cmd_b   Normalized command on PWM channel B (between 0 and 1)
 * @param[in]       cmd_c   Normalized command on PWM channel C (between 0 and 1)
 */
inline void MOT_runCommand(motor_t* p_motor, float32_t cmd_a, float32_t cmd_b, float32_t cmd_c)
{
    *(p_motor->p_motorChAReg) = (uint16_t)((cmd_a) * PWM_TIMEBASE_CNT );
    *(p_motor->p_motorChBReg) = (uint16_t)((cmd_b) * PWM_TIMEBASE_CNT );
    *(p_motor->p_motorChCReg) = (uint16_t)((cmd_c) * PWM_TIMEBASE_CNT );

    return;
}

/**
 * @brief           Force a hard stop on the 3 phases of the ePWM (low side active)
 * @param[out]      p_motor Pointer on the associated motor structure
 */
inline void MOT_stopCommand(motor_t* p_motor)
{
    *(p_motor->p_motorChAReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChBReg) = (uint16_t)PWM_TIMEBASE_CNT;
    *(p_motor->p_motorChCReg) = (uint16_t)PWM_TIMEBASE_CNT;

    return;
}

/**
 * @brief           Control of the motor states
 * @param[inout]    p_motor Pointer on the associated motor structure
 * @param[in]       p_cmd   Pointer on the command structure driving motor
 */
inline bool_t MOT_runControl(motor_t* p_motor)
{
    static float32_t enc_theta[2] = {MOTOR1_THETA_ALIGN_MAX, MOTOR2_THETA_ALIGN_MAX};

    uint8_t         id          = p_motor->motor_id;
    foc_t*          p_foc       = p_motor->p_motorFOC;
//    error_reg_u*    p_err       = &p_motor->motor_error;
    error_reg_u     err         = {.all = 0};
    encoder_t*      p_enc       = &p_foc->motor_enc;
    cmd_t*          p_cmd       = &p_foc->motor_cmd;
    cmd_reg_t       en_bit      = p_cmd->enableReg.bit;
    p_enc->indexOffset          = en_bit.encOffsetEnable;

    // Read the phases current, voltage & Vbus
    FOC_getMeasures(&p_foc->motor_acq);
    // Read encoder position
    ENC_getPosition(p_enc);
    // Estimate speed
    ENC_getSpeed(p_enc);
    // Rollover test & index
    ENC_getTheta(p_enc);

    // Save number of cycles elapsed since control loop startup
    p_motor->clCycleNb          = EPWM_getTimeBaseCounterValue(p_motor->p_motorHalCfg->p_pwmCntCmp[0]->epwmBase);

#if 1
    err.bit.drv_fault           = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
    err.bit.pos_rollover        = p_enc->rollOverError && en_bit.rollOverEnable;
    err.bit.com_timeout         = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
    err.bit.enc_mismatch        = p_enc->indexDetect && p_enc->indexError;
    p_motor->motor_state        = (err.all)                         ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
    p_motor->motor_error.all    = err.all;
//    p_err->bit.drv_fault        = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
//    p_err->bit.pos_rollover     = p_enc->rollOverError && en_bit.rollOverEnable;
//    p_err->bit.com_timeout      = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
//    p_err->bit.enc_mismatch     = p_enc->indexDetect && p_enc->indexError;
//    p_motor->motor_state        = (p_err->all)                      ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
#else
    p_motor->motor_error.bit.drv_fault      = !GPIO_readPin(p_motor->p_motorDRV->p_drvCfgHandler->gpioNumber_FAULT);
    p_motor->motor_error.bit.pos_rollover   = p_enc->rollOverError && en_bit.rollOverEnable;
    p_motor->motor_error.bit.com_timeout    = (p_cmd->cptTimeout > p_cmd->timeoutRef) && p_cmd->timeoutRef;
    p_motor->motor_error.bit.enc_mismatch   = p_enc->indexDetect && p_enc->indexError;
    p_motor->motor_state        = (p_motor->motor_error.all)        ? (MOTOR_STATE_ERROR)       : (p_motor->motor_state);
#endif

    switch(p_motor->motor_state)
    {
    case MOTOR_STATE_INIT:
    default:
        p_motor->itCnt          = 0U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_ALIGN_UP:
        p_motor->itCnt          = 0U;
        p_foc->idRef           += (id == MOTOR_1)                   ? (MOTOR1_CURRENT_ALIGN_INC): (MOTOR2_CURRENT_ALIGN_INC); // Increment for current alignment procedure
        p_foc->iqRef            = 0.0f;
        enc_theta[id]          -= (id == MOTOR_1)                   ? (MOTOR1_THETA_ALIGN_DEC)  : (MOTOR2_THETA_ALIGN_DEC); // Decrement for position alignment procedure
        p_motor->motor_state    = (p_foc->idRef < p_foc->iAlignMax) ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_ALIGN_FIX);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        (p_motor->motor_state  != MOTOR_STATE_ALIGN_UP)             ? (ENC_resetStruct(p_enc))  : (p_enc->thetaElec = enc_theta[id]);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ALIGN_FIX:
        p_motor->itCnt         += 1U; // wait 2s
        p_foc->idRef            = p_foc->iAlignMax; // Maintain alignment current
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ)) ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);

        // p_foc->dtc_u = p_foc->motor_cmd.kpCoeff;
        // p_foc->dtc_v = p_foc->motor_cmd.velRef;
        // p_foc->dtc_w = p_foc->motor_cmd.iqff;

        if (p_foc->motor_acq.ia < 3 & p_foc->motor_acq.ib < 3 & p_foc->motor_acq.ic < 3){

        p_foc->inductance_line = 0.0002;

        /****** COMPUTE CURRENT AUTOMATIC ***/ 
        
        static const float offset = 0.05;
        static int   f = 10;                // Hz
        static const float amplitude = 0.01;        // Amplitude

        static unsigned int time;
        static bool filled_vector = false;

        float dc_tension, dc_current;

        p_foc->dtc_v = 0;
        p_foc->dtc_w = 0;
        p_foc->frequence = f;
        // p_foc->dc_current = dc_current;
        // p_foc->dc_tension = dc_tension;
        time += 1;

        // Compute dc current 
        if (p_foc->current_case == 0){
            p_foc->dtc_u = offset;
            dc_current = p_foc->motor_acq.ia;
            dc_tension = offset * p_foc->motor_acq.vbus;
            p_foc->current = dc_current;
            p_foc->tension = dc_tension;

            if (time % 4000 == 0)
                p_foc->current_case = 1;
        }

        // Get drops of currents by rising the frequence
        else if (p_foc->current_case == 1){
            
            p_foc->dtc_u = offset + amplitude * __sin(f * 2*M_PI * time / 40000);
            
            static const int num_measurements = 80;
            static int index_count = 0;

            static double sum_current = 0.0;
            static float ias[num_measurements];
            float current_measurement = p_foc->motor_acq.ia;
            float first_measurement = ias[index_count]; // Oldest reading
            
            ias[index_count] = current_measurement;
            sum_current += current_measurement;
            sum_current -= first_measurement;


            static double sum_tension = 0.0;
            static float uas[num_measurements];
            float tension_measurement = p_foc->dtc_u * p_foc->motor_acq.vbus;
            first_measurement = uas[index_count]; // Oldest reading
            uas[index_count] = tension_measurement;
            sum_tension += tension_measurement;
            sum_tension -= first_measurement;

            index_count++;
            if (index_count == num_measurements){ 
                index_count = 0;
                filled_vector = true;
                }
            
            p_foc->current = sum_current / num_measurements;
            p_foc->tension = sum_tension / num_measurements;

            // Go to next case if we have enough drop of current
            if( (p_foc->current < p_foc->dc_current * 0.75) & filled_vector){
                p_foc->current_case = 2;
            }

            if (time % 4000 == 0){ // Entry each 100 ms
                if (f < 1000)
                    f *= 10;        // f = {10, 100, 1000}
                else if(f < 4000 )
                    f += 1000;      // f = {1000, 2000, 3000, 4000}
                else{
                    p_foc->current_case = 2;
                }
            }
        }

        // Calculate Inductance
        else if (p_foc->current_case == 2){
            float dc_impedance, ac_impedance, reactance;
            
            dc_impedance = dc_tension / dc_current;

            ac_impedance = p_foc->tension / p_foc->current;
            reactance = __sqrt(ac_impedance*ac_impedance - dc_impedance*dc_impedance);
            // inductance_line = reactance / (2 * M_PI * f);
            p_foc->inductance_line = 0.0002; //inductance_line;
            p_foc->current_case = -1;

        }
        // Finish test
        else{
            p_foc->dtc_u = 0;
            p_foc->dtc_v = 0;
            p_foc->dtc_w = 0;
        }
    
        }
        else{
            p_foc->dtc_u = 0;
            p_foc->dtc_v = 0;
            p_foc->dtc_w = 0;
        }
        // define the drop of current waited

        // calculates mean current
        // calculate resistance



        // // Compute tension for inductance calculations

        // p_foc->dtc_u = offset + amplitude * __sin(f * 2*M_PI * time / 40000);
        // p_foc->dtc_v = 0;
        // p_foc->dtc_w = 0;
        // time += 1;

        // // Compute mean current and mean tensions of the lasts 1000 reading


        if (p_foc->motor_acq.ia > 6  || p_foc->motor_acq.ib > 6 || p_foc->motor_acq.ic > 6){
            p_foc->dtc_u = 0;
            p_foc->dtc_v = 0;
            p_foc->dtc_w = 0;
        }

        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_STOP:
        p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = (!en_bit.motorEnable)             ? (MOTOR_STATE_STOP)        : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);
        break;

    case MOTOR_STATE_ERROR:
        p_motor->itCnt          = p_motor->itCnt;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = 0.0f;
        p_motor->motor_state    = MOTOR_STATE_ERROR;
//        ENC_resetPeriph(p_enc);
//        FOC_resetStruct(p_foc);
//        FOC_runControl(p_foc);
        MOT_stopCommand(p_motor);
        break;
    }

    return(true);
}
