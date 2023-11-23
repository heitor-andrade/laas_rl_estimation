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
    estimator_rl*   p_rle       = p_motor->p_motorRLE;
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
        p_motor->motor_state    = (en_bit.motorEnable)                      ? (MOTOR_STATE_ALIGN_UP)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.motorEnable & COMPUTE_RL_ENABLE)  ? (MOTOR_STATE_COMPUTE_RL)  : (p_motor->motor_state);
        p_motor->motor_state    = (en_bit.systemEnable)                     ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        // FOC_resetStruct(p_foc);
        MOT_stopCommand(p_motor);
        break;

    case MOTOR_STATE_COMPUTE_RL:
        p_motor->motor_state    = (p_rle->statorIndEst == 0)        ? (MOTOR_STATE_COMPUTE_RL)  : (MOTOR_STATE_ALIGN_UP);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        
        if( (p_rle->dtc_initialize) ){      
            // Initialize duty cicle to 0.5                 
            p_foc->dtc_u = CENTER_DTC_RL;
            p_foc->dtc_v = CENTER_DTC_RL;
            p_foc->dtc_w = CENTER_DTC_RL;
            p_motor->itCnt = 0;
            p_rle->dtc_initialize = false;
        }
        else if (p_rle->statorResEst == 0){                     
            // Compute resistance estimation
            if (p_rle->inf_current == 0 || p_rle->sup_current == 0 || p_rle->computing_mean_sup){
                // Get inferior and supperior current and duty cicles 
                float current = p_foc->motor_acq.ia;

                if (current >= INF_CURRENT_R_ESTIMATION & p_rle->inf_dtc == 0){
                    p_rle->computing_mean_inf = true;
                    p_rle->inf_dtc = p_foc->dtc_u;
                    p_rle->inf_vbus = p_foc->motor_acq.vbus;
                    }
                else if (current >= SUP_CURRENT_R_ESTIMATION & p_rle->sup_dtc == 0){
                    p_rle->computing_mean_sup = true;
                    p_rle->sup_dtc = p_foc->dtc_u;
                    p_rle->sup_vbus = p_foc->motor_acq.vbus;
                    }

                if (p_rle->computing_mean_inf){
                    // Compute mean inferior current
                    if (p_rle->counter_current < CURRENT_SAMPLES_NUMBER){
                        p_rle->counter_current++;
                        p_rle->inf_current += current;
                    }else{
                        p_rle->inf_current /= CURRENT_SAMPLES_NUMBER;
                        p_rle->computing_mean_inf = false;
                        p_rle->counter_current = 0;
                    }
                }
                else if(p_rle->computing_mean_sup){
                    // Compute mean superior current
                    if (p_rle->counter_current < CURRENT_SAMPLES_NUMBER){
                        p_rle->counter_current++;
                        p_rle->sup_current += current;
                    }else{
                        p_rle->computing_mean_sup = false;
                        p_rle->counter_current = 0;
                        p_rle->sup_current /= CURRENT_SAMPLES_NUMBER;
                    }
                }
                else{
                    // Increment dutycicle while not calculating means
                    p_motor->itCnt         += 1U;
                    if ((p_motor->itCnt % 100 == 0))
                        p_foc->dtc_u += 0.001;
                }
            }
            else{
                // Compute resistance 
                p_rle->statorResEst = ((p_rle->sup_dtc * p_rle->sup_vbus - p_rle->inf_dtc * p_rle->inf_vbus) ) / (p_rle->sup_current - p_rle->inf_current);
                
                // Reset variables to inductance estimation 
                p_motor->itCnt = 0;
                p_rle->sup_current = 0;
                p_rle->inf_current = 10;
            }
        }
        else if (p_rle->statorIndEst == 0){

            p_motor->itCnt         += 1U;
            if(p_rle->initialize_current_flt & p_motor->itCnt > 1000){
                // Initialize current filtred
                p_rle->current_flt = p_foc->motor_acq.ia;
                p_rle->initialize_current_flt = false;
            }

            // Filter the current
            float current_measured = p_foc->motor_acq.ia;
            float error = current_measured - p_rle->current_flt;
            if ((error > 0.1) || (error < -0.1))
                p_rle->current_flt = p_rle->current_flt + 0.01*(current_measured - p_rle->current_flt);
            else
                p_rle->current_flt = p_rle->current_flt + 1*(current_measured - p_rle->current_flt);

            
            if(p_motor->itCnt  - p_rle->frequence_iteration > 1500){
                // Get inf and sup current for amplitude computation
                if (p_rle->current_flt > p_rle->sup_current)
                    p_rle->sup_current = p_rle->current_flt;
                if (p_rle->current_flt < p_rle->inf_current)
                    p_rle->inf_current = p_rle->current_flt;
            }

            if (( (p_motor->itCnt % 4000) == 0) & (p_rle->frequence < 1200) ){

                float amplitude = (p_rle->sup_current - p_rle->inf_current) / 2;

                if(p_rle->first_frequence){
                    // Get first amplitude_current
                    p_rle->A0 = amplitude;
                    p_rle->first_frequence = false;
                }
                else{
                    
                    if (!p_rle->computing_mean_gain)// Increase frequence
                        p_rle->frequence += 50;

                    
                    p_rle->gain_current = amplitude / p_rle->A0; // Compute current gain
                    if(p_rle->gain_current <= FM_1DIVSQRT2 & !p_rle->computing_mean_gain) // Start computing mean gain
                        p_rle->computing_mean_gain = true;


                    if(p_rle->computing_mean_gain){
                        p_rle->counter_gain++;
                        p_rle->gain_current_mean += p_rle->gain_current;

                        if (p_rle->counter_gain == CURRENT_SAMPLES_NUMBER){
                            // Compute mean current gain
                            p_rle->gain_current_mean /= CURRENT_SAMPLES_NUMBER;

                            // Compute inductance estimation
                            float impedance_ac = p_rle->statorResEst/p_rle->gain_current_mean;      // Impedance gain is the invese of current_gain. Z = U/I                
                            float reactance = __sqrtf(impedance_ac*impedance_ac - p_rle->statorResEst*p_rle->statorResEst);
                            p_rle->statorIndEst = reactance / (2 * M_PI * p_rle->frequence);
                             
                            // Changes values for the motor phase reference
                            p_rle->statorIndEst = p_rle->statorIndEst * 2 / 3;
                            p_rle->statorResEst = p_rle->statorResEst * 2 / 3;

                            // Update current control constants
                            p_foc->piIq.kp = (p_rle->statorIndEst * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
                            p_foc->piIq.ki = (p_rle->statorResEst * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
                            p_foc->piId.kp = (p_rle->statorIndEst * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
                            p_foc->piId.ki = (p_rle->statorResEst * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);

                            p_motor->test  = p_rle->statorResEst;
                            p_motor->test1 = p_rle->statorIndEst;
                        }
                    } 
                }
                // Reset variables
                p_rle->inf_current = 10;
                p_rle->sup_current = 0;
                p_rle->frequence_iteration = p_motor->itCnt;
            }

            // Apply sinusoidal voltage
            p_foc->dtc_u = (CENTER_DTC_RL + OFFSET_L_ESTIMATION) + AMPLITUDE_L_ESTIMATION * __sin(p_rle->frequence * 2 * M_PI * p_motor->itCnt / 40000);
        }

        if ((p_rle->statorIndEst != 0) & (p_rle->statorResEst != 0)){
            // Reset structure after RL estimation
            ENC_resetPeriph(p_enc);
            ENC_resetStruct(p_enc);
            FOC_runControl(p_foc);
        }

        // Debug variables
        p_motor->test = p_rle->statorResEst;
        p_motor->test1 = p_rle->statorIndEst;
        p_motor->test2 = p_rle->frequence;

        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

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
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ))                             ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.motorEnable)                                          ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)                                         ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        break;

    case MOTOR_STATE_READY:
        p_motor->itCnt         += 1U;
        // p_foc->idRef            = p_foc->motor_cmd.iqff;
        p_foc->idRef            = 0;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_foc->iqRef            = 0;
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        // // // TIME PLOT
        // // change constants
        // float resistance = p_foc->motor_cmd.velRef;
        // float inductance = p_foc->motor_cmd.kdCoeff;

        // p_motor->test = resistance;
        // p_motor->test1 = inductance;

        // p_foc->piIq.kp = (inductance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piIq.ki = (resistance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piId.kp = (inductance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piId.ki = (resistance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);

        // FOC_runControl(p_foc);
        // MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        // // // CONSTANTS TEST AND BODE PLOT
        // // change constants
        // float resistance = p_foc->motor_cmd.velRef;
        // float inductance = p_foc->motor_cmd.kdCoeff;

        // p_foc->piIq.kp = (inductance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piIq.ki = (resistance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piId.kp = (inductance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);
        // p_foc->piId.ki = (resistance * 2.0f * M_PI * MOTOR1_CURRENT_CUTOFF_FREQ);

        // // set id reference
        // float frequence = p_foc->motor_cmd.kpCoeff;
        // p_foc->idRef = __sin(frequence * 2 * M_PI * p_motor->itCnt / 40000);

        // // set variables to send
        // p_motor->test = resistance;
        // p_motor->test1 = frequence;

        // FOC_runControl(p_foc);
        // MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

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
