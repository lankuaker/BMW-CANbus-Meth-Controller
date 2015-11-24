/********************************************************************/
/*    Methanol Injection Service Implementation                     */
/*    MethService.cpp                                               */
/*                                                                  */
/*    Author:         Jacob Moroni (jakemoroni@gmail.com)           */
/*                                                                  */
/*    Description:    This file implements the methanol injection   */
/*                    service. This is one of many possible         */
/*                    services that can be incorporated into the    */
/*                    main controller.                              */
/*                                                                  */
/*    Revision:       Oct 12, 2015 (First)                          */
/*                                                                  */
/*    Copyright (C) 2015 Jacob Moroni.  All right reserved.         */
/*                                                                  */
/*    This library is free software; you can redistribute it and/or */
/*    modify it under the terms of the GNU Lesser General Public    */
/*    License as published by the Free Software Foundation; either  */
/*    version 2.1 of the License, or (at your option) any later     */
/*    version.                                                      */
/*    This library is distributed in the hope that it will be       */
/*    useful, but WITHOUT ANY WARRANTY; without even the implied    */
/*    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/*    PURPOSE. See the GNU Lesser General Public License for more   */
/*    details.                                                      */
/*    You should have received a copy of the GNU Lesser General     */
/*    Public License along with this library; if not, write to the  */
/*    Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,  */
/*    Boston, MA 02110-1301 USA                                     */
/********************************************************************/

/********************************************************************/
/*                            INCLUDES                              */
/********************************************************************/

/* Implementation header. */
#include "MethService.h"

/* TimedAction library. This handles the deferred tasks. */
#include "TimedAction.h"

/* Include the MCP2515 API. We send CAN messages directly from */
/* this service. */
#include "MCPCANbusUDS.h"

/********************************************************************/
/*                        EXTERNAL LINKAGES                         */
/********************************************************************/

/* Link to the main CAN class that's been defined in the ControllerMain */
/* file. */
extern MCP_CAN CAN;

/********************************************************************/
/*                    STATIC FUNCTION PROTOTYPES                    */
/********************************************************************/

/* FSM state transition functions. */
static MethServiceState_E _fsmNOOP(void * arg);
static MethServiceState_E _fsmReInit(void * arg);
static MethServiceState_E _fsmFailure(void * arg);
static MethServiceState_E _fsmDisable(void * arg);
static MethServiceState_E _fsmEnterInjecting(void * arg);
static MethServiceState_E _fsmLowTank(void * arg);
static MethServiceState_E _fsmReady(void * arg);
static MethServiceState_E _fsmDisabledWarn(void * arg);
static MethServiceState_E _fsmRXMsg(void * arg);

/* Helper functions. */
static void _stopInjecting(void);
static void _tripFailSafe(void);
static void _untripFailSafe(void);
static double _unscaledKilogramsPerHrToCCPerMin(uint16_t kgPerHour);
static double _linearInterpolation(double x0, double x1, double y0, double y1, double x);
static void _injectPWMEnable(double dutyCycle);
static void _fuelLoadRequestSend(void);
static void _runMethServiceFSM(MethServiceEvent_E event, void * arg);

/********************************************************************/
/*                     STATIC GLOBAL VARIABLES                      */
/********************************************************************/

/* This variable keeps track of this service's state. */
/* Initialization doesn't matter. */
static MethServiceState_E _state = METH_SERVICE_STATE_FAILURE;

/* This keeps track of the number of unanswered fuel load requests. */
/* This is used to transition in to the failure state if we miss */
/* too many samples. */
/* Initialization doesn't matter. */
static uint32_t _fuelLoadReqPendingCount = 0;

/* This "task" handles sending the requests for the fuel load. */
static TimedAction _fuelLoadRequestTimer = TimedAction(METH_SERVICE_FUEL_LOAD_REQUEST_PERIOD_MS, 
                                                       _fuelLoadRequestSend);

/* Table of FSM state transitions. */
static MethServiceFSMTransition_T _transitions[] = 
{
    /* If we are in ANY state and we get the following events... */
    {METH_SERVICE_STATE_ANY, METH_SERVICE_EVENT_SAMPLE_TIMEOUT, _fsmFailure},
    {METH_SERVICE_STATE_ANY, METH_SERVICE_EVENT_EXT_FAILURE, _fsmFailure},
    {METH_SERVICE_STATE_ANY, METH_SERVICE_EVENT_POWERUP, _fsmFailure},
    {METH_SERVICE_STATE_ANY, METH_SERVICE_EVENT_RESTART, _fsmReInit},

    /* If we are in the FAILURE state... */
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmNOOP},
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmNOOP},
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmNOOP},
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_FAILURE, METH_SERVICE_EVENT_MSG_RX, _fsmNOOP},

    /* If we are in the READY_ARMED state... */
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmDisable},
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmNOOP},
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmEnterInjecting},
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmNOOP},
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmLowTank},
    {METH_SERVICE_STATE_READY_ARMED, METH_SERVICE_EVENT_MSG_RX, _fsmNOOP},
    
    /* If we are in the INJECTING state... */
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmDisable},
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmNOOP},
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmNOOP},
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmReady},
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmLowTank},
    {METH_SERVICE_STATE_INJECTING, METH_SERVICE_EVENT_MSG_RX, _fsmRXMsg},

    /* If we are in the DISABLED state... */
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmReady},
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmDisabledWarn},
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED, METH_SERVICE_EVENT_MSG_RX, _fsmNOOP},

    /* If we are in the DISABLED_WARN state... */
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmReady},
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmDisable},
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_DISABLED_WARN, METH_SERVICE_EVENT_MSG_RX, _fsmNOOP},

    /* If we are in the LOW_TANK state... */
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_DISABLE_TRUE, _fsmDisable},
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_DISABLE_FALSE, _fsmNOOP},
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, _fsmNOOP},
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, _fsmNOOP},
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_TANK_EMPTY_TRUE, _fsmNOOP},
    {METH_SERVICE_STATE_LOW_TANK, METH_SERVICE_EVENT_MSG_RX, _fsmNOOP},

    /* This entry must be at the bottom to catch any FSM program errors. */
    /* Every event should be acted upon. */
    {METH_SERVICE_STATE_ANY, METH_SERVICE_EVENT_ANY, _fsmFailure}
};
#define METH_SERVICE_NUM_FSM_TRANSITIONS (sizeof(_transitions)/sizeof(*_transitions))

/********************************************************************/
/*                   STATIC FUNCTION DEFINITIONS                    */
/********************************************************************/

/* This function turns off the meth pump and solenoid. */
static void _stopInjecting(void)
{
    /* Turn off the solenoid. */
    analogWrite(METH_SERVICE_PWM_OUTPUT_PIN, 0);

    /* Turn off pump. */
    digitalWrite(METH_SERVICE_PUMP_OUTPUT_PIN, LOW);
}

/* This function activates the failsafe relay. */
static void _tripFailSafe(void)
{
    digitalWrite(METH_SERVICE_FAILSAFE_OUTPUT_PIN, HIGH);
}

/* This function de-activates the failsafe relay. */
static void _untripFailSafe(void)
{
    digitalWrite(METH_SERVICE_FAILSAFE_OUTPUT_PIN, LOW);
}

/* This function converts the raw kilograms per hour value from the ECU */
/* to CC/Min. */
/* It takes in to account the specific gravity of gasoline which is around .730. */
static double _unscaledKilogramsPerHrToCCPerMin(uint16_t kgPerHour)
{
    return ((((double)kgPerHour * UDS_CAN_FUEL_LOAD_KG_TO_L_MULT_SCALAR) * (double)1000.0)) / (double)60.0;
}

/* This function performs a linear interpolation. */
static double _linearInterpolation(double x0, double x1, double y0, double y1, double x)
{
    if(x1 != x0)
    {
        double a = (y1 - y0) / (x1 - x0);
        double b = -a * x0 + y0;
        return (a * x + b);
    }
    else
    {
        return y0;
    }
}

/* This function turns on the pump and sets the solenoid for the */
/* specified PWM duty cycle. */
static void _injectPWMEnable(double dutyCycle)
{
    /* Turn on pump. */
    digitalWrite(METH_SERVICE_PUMP_OUTPUT_PIN, HIGH);

    /* Set PWM output. */
    analogWrite(METH_SERVICE_PWM_OUTPUT_PIN, (uint8_t)(dutyCycle * 0xFF / 100.0));
}

/* This function transmits a UDS request for the current fuel load. */
/* It gets called periodically by the timer task. */
static void _fuelLoadRequestSend(void)
{
    if(_fuelLoadReqPendingCount >= METH_SERVICE_FUEL_LOAD_SAMPLE_ERROR_LIMIT)
    {
        /* We have sent X requests already and still have not received */
        /* a response. We must shut down all methanol spraying and handle */
        /* the failure. */
        _runMethServiceFSM(METH_SERVICE_EVENT_SAMPLE_TIMEOUT, NULL);
    }
    else
    {
        if(CAN.sendMsgBuf(UDS_CAN_TRANSMIT_ID, 
                       0, 
                       MCP_CAN_MAX_MSG_LEN, 
                       (unsigned char *)UDS_CAN_FUEL_LOAD_REQUEST_MSG_PTR) != CAN_OK)
        {
            /* There was no room in the transmit FIFO. This should never happen. */
            _runMethServiceFSM(METH_SERVICE_EVENT_SAMPLE_TIMEOUT, NULL);
            return;
        }
        _fuelLoadReqPendingCount++;
    }
    return;
}

/* ------- FSM state transition functions. ------- */

/* This function does nothing. It is used for events */
/* that we don't care about when we're in the current state. */
static MethServiceState_E _fsmNOOP(void * arg)
{
    return _state;
}

/* This function initializes the system. It can */
/* be called from ANY state and thus must initialize every */
/* variable. */
static MethServiceState_E _fsmReInit(void * arg)
{
    /* Stop injecting. We don't know which state we are coming from. */
    _stopInjecting();
    /* Reset the failsafe. */
    _untripFailSafe();
    /* Start all of the timers. */
    _fuelLoadRequestTimer.start();
    _fuelLoadReqPendingCount = 0;
    return METH_SERVICE_STATE_READY_ARMED;
}

/* This function is called when one of the failure-inducing events */
/* occurs. This can be called from ANY state. */
static MethServiceState_E _fsmFailure(void * arg)
{
    /* Stop injecting. We don't know which state we are coming from. */
    _stopInjecting();
    /* Trip the failsafe. */
    _tripFailSafe();
    /* Stop all of the timers. */
    _fuelLoadRequestTimer.stop();
    return METH_SERVICE_STATE_FAILURE;
}

/* This function gets called when we are to enter the DISABLED state. */
static MethServiceState_E _fsmDisable(void * arg)
{
    /* Stop injecting. We don't know which state we are coming from. */
    _stopInjecting();
    /* Trip the failsafe. */
    _tripFailSafe();
    return METH_SERVICE_STATE_DISABLED;
}

/* This function is called only from the READY_ARMED state. */
/* This allows the system to accept injection events. */
static MethServiceState_E _fsmEnterInjecting(void * arg)
{
    return METH_SERVICE_STATE_INJECTING;
}

/* This function gets called when we are to enter the LOW_TANK state. */
static MethServiceState_E _fsmLowTank(void * arg)
{
    /* Stop injecting. We don't know which state we are coming from. */
    _stopInjecting();
    /* Trip the failsafe. */
    _tripFailSafe();
    return METH_SERVICE_STATE_LOW_TANK;
}

/* This function gets called when we are to re-enter the READY_ARMED state. */
static MethServiceState_E _fsmReady(void * arg)
{
    /* Stop injecting. We don't know which state we are coming from. */
    _stopInjecting();
    /* Reset the failsafe. */
    _untripFailSafe();
    return METH_SERVICE_STATE_READY_ARMED;
}

/* This function gets called when we go from the */
/* DISABLED state to the DISABLED_WARN state. */
static MethServiceState_E _fsmDisabledWarn(void * arg)
{
    return METH_SERVICE_STATE_DISABLED_WARN;
}

/* This function actually handles the progressive meth spray. */
static MethServiceState_E _fsmRXMsg(void * arg)
{
    uint32_t i;
    double desiredMethFlowCC = 0.0;
    double desiredMethPWM = 0.0;
    /* TODO - Check for NULL ptr. */
    uint8_t * msg = (uint8_t *)arg;
    double ccPerMin = 
        _unscaledKilogramsPerHrToCCPerMin((((uint16_t)msg[UDS_CAN_RX_RESP_DATA_BYTE_0_IDX]) << 8) | 
                                          msg[UDS_CAN_RX_RESP_DATA_BYTE_1_IDX]);

    /* First, find the desired meth flow percentage based on the current */
    /* fuel delivery rate. */
    /* Find points it falls between... */
    for(i = 0; i < METH_SERVICE_TABLE_SIZE; i++)
    {
        if(ccPerMin < METH_SERVICE_FUEL_DELIVERY_RATE_ARRAY[i])
        {
            break;
        }
    }
    if(i == METH_SERVICE_TABLE_SIZE)
    {
        /* It's greater than or equal to the highest X value. */
        /* Don't interpolate; use last value. */
        desiredMethFlowCC = 
            (METH_SERVICE_DESIRED_METH_FLOW_FUEL_DELIVERY_PERCENTAGE[METH_SERVICE_TABLE_SIZE - 1] * ccPerMin) / 100.0;
    }
    else
    {
        desiredMethFlowCC = (_linearInterpolation(METH_SERVICE_FUEL_DELIVERY_RATE_ARRAY[i - 1], 
                                                  METH_SERVICE_FUEL_DELIVERY_RATE_ARRAY[i], 
                                                  METH_SERVICE_DESIRED_METH_FLOW_FUEL_DELIVERY_PERCENTAGE[i - 1], 
                                                  METH_SERVICE_DESIRED_METH_FLOW_FUEL_DELIVERY_PERCENTAGE[i], 
                                                  ccPerMin) * ccPerMin) / 100.0;
    }

    /* Now, find the required PWM output for this desired meth flow CC. */
    /* Find points it falls between. */
    for(i = 0; i < METH_SERVICE_TABLE_SIZE; i++)
    {
        if(desiredMethFlowCC < METH_SERVICE_DESIRED_METH_FLOW_CC_MIN_ARRAY[i])
        {
            break;
        }
    }
    if(i == METH_SERVICE_TABLE_SIZE)
    {
        /* It's greater than or equal to the highest X value. */
        /* Don't interpolate; use last value. */
        desiredMethPWM = METH_SERVICE_DESIRED_METH_FLOW_PWM_ARRAY[METH_SERVICE_TABLE_SIZE - 1];
    }
    else
    {
        desiredMethPWM = _linearInterpolation(METH_SERVICE_DESIRED_METH_FLOW_CC_MIN_ARRAY[i - 1], 
                                              METH_SERVICE_DESIRED_METH_FLOW_CC_MIN_ARRAY[i], 
                                              METH_SERVICE_DESIRED_METH_FLOW_PWM_ARRAY[i - 1], 
                                              METH_SERVICE_DESIRED_METH_FLOW_PWM_ARRAY[i], 
                                              desiredMethFlowCC);
    }

    /* Set outputs. */
    _injectPWMEnable(desiredMethPWM);

    /* Debug statements. */
#if defined(METH_SERVICE_SERIAL_DEBUG)
    Serial.print("DUTY: ");
    Serial.println(desiredMethPWM, 2);
#endif

    return _state;
}

/* This function executes the state machine. */
static void _runMethServiceFSM(MethServiceEvent_E event, void * arg)
{
    uint32_t i;
    for(i = 0; i < METH_SERVICE_NUM_FSM_TRANSITIONS; i++)
    {
        if((_state == _transitions[i].state) || (METH_SERVICE_STATE_ANY == _transitions[i].state))
        {
            if((event == _transitions[i].event) || (METH_SERVICE_EVENT_ANY == _transitions[i].event))
            {
                _state = (_transitions[i].func)(arg);
                /* Only one transition can exist per event. */
                break;
            }
        }
    }
}

/********************************************************************/
/*                   GLOBAL FUNCTION DEFINITIONS                    */
/********************************************************************/

void MethServiceInitEarly(void)
{
    /* Configure all pin directions and initial states. */
    /* Initial state should be low anyway... */
    TCCR1B = (TCCR1B & 0xF8) | 0x05;                          /* Set PWM frequency to ~30 Hz. */
    pinMode(METH_SERVICE_PWM_OUTPUT_PIN, OUTPUT);
    analogWrite(METH_SERVICE_PWM_OUTPUT_PIN, 0);
    pinMode(METH_SERVICE_PUMP_OUTPUT_PIN, OUTPUT);
    digitalWrite(METH_SERVICE_PUMP_OUTPUT_PIN, LOW);
    pinMode(METH_SERVICE_FAILSAFE_OUTPUT_PIN, OUTPUT);
    digitalWrite(METH_SERVICE_FAILSAFE_OUTPUT_PIN, HIGH);     /* Failsafe init ON. */
    pinMode(METH_SERVICE_TANK_LEVEL_INPUT_PIN, INPUT_PULLUP);
    pinMode(METH_SERVICE_DISABLE_INPUT_PIN, INPUT_PULLUP);

    /* Wait 20 milliseconds for input pins to settle. */
    /* This prevents a false jump to the DISABLED or LOW_TANK state */
    /* when we check the input pins in the exec loop the first time. */
    delay(20);

    /* Push POWERUP event to FSM. */
    _runMethServiceFSM(METH_SERVICE_EVENT_POWERUP, NULL);
}

void MethServiceInitLate(void)
{
    /* Push RESTART event to FSM. */
    _runMethServiceFSM(METH_SERVICE_EVENT_RESTART, NULL);
}

void MethServiceExec(void)
{
    /* Check the disable input pin. Push event depending on state. */
    /* NOTE: The pin is active LOW. */
    if(digitalRead(METH_SERVICE_DISABLE_INPUT_PIN) == HIGH)
    {
        _runMethServiceFSM(METH_SERVICE_EVENT_DISABLE_FALSE, NULL);
    }
    else
    {
        _runMethServiceFSM(METH_SERVICE_EVENT_DISABLE_TRUE, NULL);
    }

    /* Check tank level sensor. */
    if(digitalRead(METH_SERVICE_TANK_LEVEL_INPUT_PIN) == HIGH)
    {
        /* All good. */
    }
    else
    {
        /* Pin has been grounded. Tank must be empty. */
        /* TODO - Check to see if the sensor is NC or NO. */
        /* This will work if sensor is not connected... */
        _runMethServiceFSM(METH_SERVICE_EVENT_TANK_EMPTY_TRUE, NULL);
    }

    /* Tick all timers... */
    _fuelLoadRequestTimer.check();
}

void MethServiceRXFuelLoadMsg(uint8_t * msg)
{
    double ccPerMin = 
        _unscaledKilogramsPerHrToCCPerMin((((uint16_t)msg[UDS_CAN_RX_RESP_DATA_BYTE_0_IDX]) << 8) | 
                                          msg[UDS_CAN_RX_RESP_DATA_BYTE_1_IDX]);

    /* Qualify the message based on thresholds. */
    /* This may cause a state change, so we do it before pushing the message. */
    if(ccPerMin < METH_SERVICE_FUEL_LOAD_LOWER_BOUND)
    {
        _runMethServiceFSM(METH_SERVICE_EVENT_FUEL_LOAD_BELOW_LOWER, NULL);
    }
    if(ccPerMin > METH_SERVICE_FUEL_LOAD_UPPER_BOUND)
    {
        _runMethServiceFSM(METH_SERVICE_EVENT_FUEL_LOAD_ABOVE_UPPER, NULL);
    }

    /* Process received message if the state permits us. */
    _runMethServiceFSM(METH_SERVICE_EVENT_MSG_RX, msg);

    /* Clear pending request counter. */
    _fuelLoadReqPendingCount = 0;

    /* Debug statements. */
#if defined(METH_SERVICE_SERIAL_DEBUG)
    Serial.print("FUEL: ");
    Serial.println(ccPerMin, 2);
#endif
}

void MethServiceAbort(void)
{
    /* Process received abort message. */
    /* We must disable the spraying of methanol, stop all request timers, */
    /* and transition to a failure state so that further messages won't re-arm the spray. */
    _runMethServiceFSM(METH_SERVICE_EVENT_EXT_FAILURE, NULL);
}

MethServiceState_E MethServiceGetState(void)
{
    return _state;
}
