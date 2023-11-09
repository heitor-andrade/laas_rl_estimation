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
    bool compute_rl = false;

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
        // FOC_resetStruct(p_foc);
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
        p_motor->motor_state    = (p_motor->itCnt < (2 * PWM_FREQ))                             ? (MOTOR_STATE_ALIGN_FIX)   : (MOTOR_STATE_READY);
        p_motor->motor_state    = ((p_motor->motor_state == MOTOR_STATE_READY) & compute_rl)    ? (MOTOR_STATE_COMPUTE_RL)  : (p_motor->motor_state);
        p_motor->motor_state    = (en_bit.motorEnable)                                          ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)                                         ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        ENC_resetPeriph(p_enc);
        ENC_resetStruct(p_enc);
        FOC_runControl(p_foc);
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        break;
    case MOTOR_STATE_COMPUTE_RL:
        p_motor->motor_state    = (p_motor->statorIndEst == 0)        ? (MOTOR_STATE_COMPUTE_RL)  : (MOTOR_STATE_READY);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);


        break;

    case MOTOR_STATE_READY:
        // p_motor->itCnt         += 1U;
        p_foc->idRef            = 0.0f;
        p_foc->iqRef            = FOC_runPD(&p_foc->pdPosVel);
        p_motor->motor_state    = (en_bit.motorEnable)              ? (MOTOR_STATE_READY)       : (MOTOR_STATE_STOP);
        p_motor->motor_state    = (en_bit.systemEnable)             ? (p_motor->motor_state)    : (MOTOR_STATE_INIT);
        // FOC_runControl(p_foc);
        // MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);


        // statorResEst variables
        const float MIN_CURRENT = 4;
        const float MAX_CURRENT = 5;
        const float DTC_VARIATION = 1/100;

        // statorIndEst variables
        const float OFFSET = 0.06;
        const float AMPLITUDE = 0.01;
        static bool frequence_changed = false;


        if( !(p_motor->dtc_initialized) & p_motor->comp_resistance){
            
            p_foc->dtc_u = 0;
            
            p_motor->itCnt         += 1U;
            if(p_motor->itCnt  % 20000)
                p_motor->dtc_initialized = true;
        }
        else if (p_motor->comp_resistance){            // Compute p_foc->motor_cfg.Rs
            if (p_motor->min_dtc == 0 || p_motor->max_dtc == 0){          // Compute p_motor->min_dtc and p_motor->max_dtc
                float current = p_foc->motor_acq.ia;

                if (current >= MIN_CURRENT & p_motor->min_dtc == 0){
                    p_motor->min_dtc = p_foc->dtc_u;
                    p_motor->min_current = current;
                    }
                if (current >= MAX_CURRENT & p_motor->max_dtc == 0){
                    p_motor->max_dtc = p_foc->dtc_u;
                    p_motor->max_current = current;
                    }

                p_motor->itCnt         += 1U;
                if ((p_motor->itCnt % 100 == 0) & p_foc->dtc_u < 0.16)
                    p_foc->dtc_u = p_foc->dtc_u + 0.001;
            }
            else {
                p_foc->motor_cfg.Rs = ((p_motor->max_dtc - p_motor->min_dtc) * p_foc->motor_acq.vbus) / (p_motor->max_current - p_motor->min_current);
                p_motor->comp_resistance = false;
                p_motor->itCnt = 0;
                p_motor->max_current = 0;
                p_motor->min_current = 10;
                p_motor->frequence = 100;
                p_motor->initialize_current_flt = true;
                p_motor->first_frequence = true;
                p_motor->gain_current = 1;
            }
//            p_motor->test = p_foc->dtc_u;
//            p_motor->test1 = p_motor->min_dtc;
//            p_motor->test2 = p_motor->max_dtc;
        }
        else{
            
            // To-do: filtro passa baixa com alpha alto, media com x pontos, fourier e calculo da magnitude 

            float current_measured = p_foc->motor_acq.ia;
            float error, alpha;
            // static bool first_frequence = true;
            // static const float di = 0.01f;
            // static const int counter_tolerance = 4;
            // static float frequence_ressonance = 0;
            // static float gain_current = 1;

            p_motor->itCnt         += 1U;

            // Set frequence e reset variables
            if (( (p_motor->itCnt % 40000) == 0) & (p_motor->frequence < 700) ){

                p_motor->amplitude = (p_motor->max_current - p_motor->min_current) / 2;

                // Compute gain
                if(p_motor->first_frequence){      // Get first amplitude_current
                    p_motor->A0 = p_motor->amplitude;
                    p_motor->first_frequence = false;
                }
                else{                                   // Compute current gain
                    p_motor->gain_current = p_motor->amplitude / p_motor->A0;
                    if(p_motor->gain_current <= FM_1DIVSQRT2){
                        p_motor->frequence_ressonance = p_motor->frequence;
                    }
                }

                p_motor->frequence += 100;
                p_motor->min_current = 10;
                p_motor->max_current = 0;
                p_motor->frequence_iteration = p_motor->itCnt;
            }



            // Filter current
            if(p_motor->initialize_current_flt & p_motor->itCnt > 1000){
                p_motor->current_flt = p_foc->motor_acq.ia;
                p_motor->initialize_current_flt = false;
            }

            error = current_measured - p_motor->current_flt;
            if ((error > 0.1) || (error < -0.1)){
                alpha = 0.1;
            }
            else{
                alpha = 1;
            }

            p_motor->current_flt = p_motor->current_flt + alpha*(current_measured - p_motor->current_flt);

            if(p_motor->itCnt  - p_motor->frequence_iteration > 10000){
                if (p_motor->current_flt > p_motor->max_current)
                    p_motor->max_current = p_motor->current_flt;
                if (p_motor->current_flt < p_motor->min_current)
                    p_motor->min_current = p_motor->current_flt;
            }


            if (p_motor->frequence_ressonance != 0){ // Compute statorIndEst
                float impedance_ac = p_foc->motor_cfg.Rs/p_motor->gain_current;                      // Impedance gain is the invese of current_gain. Z = U/I
                float reactance = __sqrtf(impedance_ac*impedance_ac - p_foc->motor_cfg.Rs*p_foc->motor_cfg.Rs);
                p_motor->statorIndEst = reactance / (2 * M_PI * p_motor->frequence_ressonance);
            }

            p_motor->test = p_motor->statorIndEst;
            p_motor->test1 = p_foc->motor_cfg.Rs;
            p_motor->test2 = p_motor->frequence;


            // // Compute amplitude                
            // if(current_flt > max_current)
            //     counter_max++;
            // if(current_flt < min_current)
            //     counter_min++;
            // if((p_motor->itCnt % 80) == 0){ // happens each period 0.002 s
            //     if (counter_max > counter_tolerance)
            //         {max_current += di;}
            //     else 
            //         {max_current -= di/100;}
            //     if (counter_min > 10)
            //         min_current -= di;
            //     else 
            //         {min_current += di/100;}
                
            //     counter_max = 0;
            //     counter_min = 0;
            // }
            

            // if (p_motor->itCnt % 40000) {

            //     // Compute current gain



            // }
                // Compute amplitude
                // static float p_motor->max_currents[3] = {-100, -100, -100};
                // static float min_currents[3] = {100, 100, 100};

                // static float last_current, A0, frequence_ressonance = 0;
                // static float gain_current = 1;
                // static bool get_next_current_max = false;
                // static bool get_next_current_min = false;
                // static bool first_frequence = true;
                // static float amplitude_current = 0;



                // // Get max and min currents for each frequence
                // if(current > p_motor->max_currents[1]){
                //     p_motor->max_currents[1] = current;
                //     p_motor->max_currents[0] = last_current;
                //     get_next_current_max = true;
                // }
                // else{
                //     if (get_next_current_max){
                //         p_motor->max_currents[2] = current;
                //         get_next_current_max = false;
                //     }
                // }
                // if(current < min_currents[1]){
                //     min_currents[1] = current;
                //     min_currents[0] = last_current;
                //     get_next_current_min = true;
                // }
                // else{
                //     if (get_next_current_min){
                //         min_currents[2] = current;
                //         get_next_current_min = false;
                //     }
                // }
                // last_current = current;




                // if (frequence_changed){ 
                //     frequence_changed = false;

                //     // Compute current amplitude
                //     // float mean_max, mean_min;
                //     // mean_max = (p_motor->max_currents[0] + p_motor->max_currents[1] + p_motor->max_currents[2])/3;
                //     // mean_min = (min_currents[0] + min_currents[1] + min_currents[2])/3;
                //     // amplitude_current = (mean_max - mean_min) / 2;
                //     amplitude_current = (max_current - min_current) / 2;
                //     max_current = -100;
                //     min_current = 100;

                //     int i;
                //     for(i = 0; i < 3; i++){
                //         p_motor->max_currents[i] = -100;
                //         min_currents[i] = 100;
                //     }


                // }

            p_foc->dtc_u = OFFSET + AMPLITUDE * __sin(p_motor->frequence * 2 * M_PI * p_motor->itCnt / 40000);
        }

        // if (p_motor->statorIndEst != 0){
        //     ENC_resetPeriph(p_enc);
        //     ENC_resetStruct(p_enc);
        //     FOC_runControl(p_foc);
        //     p_foc->dtc_u = 0;
        // }

        p_foc->dtc_v = 0;
        p_foc->dtc_w = 0;
        MOT_runCommand(p_motor, p_foc->dtc_u, p_foc->dtc_v, p_foc->dtc_w);

        // Variable persistance test
        // if (p_foc->test == 0){
        //     p_foc->test   = 1;
        // }
        // else if (p_foc->test == 1){
        //     volatile float temp_current = 0.1f;
        //     p_foc->current = 0.1f;
        //     // p_foc->current = temp_current;
        //     p_foc->test =2;
        // }
        // else if (p_foc->test == 2){
        // }
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
