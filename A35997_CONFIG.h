#ifndef __A35997_CONFIG_H
#define __A35997_CONFIG_H  


#define _USE_GUI_TO_SET_POWER                                            // Comment this line out for the production system
#define _ENABLE_GUI_RESET                                                // Comment this line out to disable the serial link fault reset
//#define _OPEN_LOOP_MODE                                                  // Comment this line out to disable open loop mode


//#define POWER_PID_P_COMPONENT                              .2
//#define POWER_PID_I_COMPONENT                              .5
//#define POWER_PID_D_COMPONENT                              .01

#define POWER_PID_P_COMPONENT                              .01
#define POWER_PID_I_COMPONENT                              .05
#define POWER_PID_D_COMPONENT                              .00


#define MINIMUM_POWER_TARGET                               100           // 1 Watt - Below this level the amplifier will be disabled
#define MAX_POWER_TARGET                                   53500         // 535 Watts
#define FOLDBACK_POWER_PROGRAM                             25000         // 250 Watts

#define FRONT_PANEL_LED_NUMBER_OF_FLASHES_AT_STARTUP       3






// -------------------- Configuration for Fault Levels ------------------------ //

#define FAULT_OVER_POWER_MULT                              36045  // 1.100006 // The target power is multiplied by this to generate the over power level
#define FAULT_MINIMUM_OVER_POWER                           500    // 5.00 Watts // This is absolute minimum for the over power setting relative to the power setting
#define FAULT_TIME_OVER_POWER                              10     // Time that the unit must be overpower (in 10ms units) to generate an over power fault

#define FAULT_UNDER_POWER_MULT                             58982  // .899994
#define FAULT_MINIMUM_UNDER_POWER                          500    // 5.00 Watts
#define FAULT_TIME_UNDER_POWER                             50     // Time that the unit must be overpower (in 10ms units) to generate an under power fault


#define RF_AMPLIFIER_TEMPERATURE_MIN                       2330           // 233 K, -40 C
#define RF_AMPLIFIER_TEMPERATURE_MAX                       3480           // 348 K, 75 C

#define RF_DETECTOR_TEMPERATURE_MIN                        2330           // 233 K, -40 C
#define RF_DETECTOR_TEMPERATURE_MAX                        3580           // 358 K, 85 C


#define FORWARD_DETECTOR_MAX_POWER                         32500          // 325 Watts
#define FORWARD_OVER_POWER_TRIP_TIME_10MS_UNITS            10             // 100ms
#define REVERSE_DETECTOR_MAX_POWER                         32500          // 325 Watts
#define REVERSE_OVER_POWER_TRIP_TIME_10MS_UNITS            100            // 1000ms

#define FORWARD_DETECTOR_MISMATCH_SCALE                    16384          // .25 Multiplier
#define FORWARD_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT        1000           // 10 Watts
#define FORWARD_DETECTOR_MISMATCH_TIME                     10             // 100mS

#define REVERSE_DETECTOR_MISMATCH_SCALE                    16384          // .25 Multiplier
#define REVERSE_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT        1000           // 10 Watts
#define REVERSE_DETECTOR_MISMATCH_TIME                     100            // 1000mS




// -------------------- Over Reflected Power -------------------- //

#define MAX_STEADY_STATE_REFLECTED_POWER                   25000   // 250 Watts
#define MAX_FOLDBACK_REFLECTED_POWER                       12500   // 125 Watts
#define MAX_OVER_REFLECTED_TIME                            1500    // 15 Seconds
#define MAX_OVER_REFLECTED_TIME_HYSTERESIS                 200     // 2 Seconds






// ----------------- DETECTOR CONFIGURATION NEEDS TO GO INTO EEPROM ---------------------//
#define DETECTOR_A1A3_SCALE_FACTOR                         41796
#define DETECTOR_A1A3_INTERCEPT                            -653

#define DETECTOR_A1A4_SCALE_FACTOR                         41457
#define DETECTOR_A1A4_INTERCEPT                            -777

#define DETECTOR_A1A5_SCALE_FACTOR                         41457
#define DETECTOR_A1A5_INTERCEPT                            -170

#define DETECTOR_A1A6_SCALE_FACTOR                         42489
#define DETECTOR_A1A6_INTERCEPT                            -66







#endif
