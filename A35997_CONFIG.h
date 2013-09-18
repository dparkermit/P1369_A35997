#ifndef __A35997_CONFIG_H
#define __A35997_CONFIG_H  


#define _DO_THERMAL_COMP

//#define _OPEN_LOOP_MODE                                       // When defined, unit operates in open loop mode

#define _DEBUG_MODE                                           // This line enables debugging code 
                                                              // Enables GUI setting the target power
                                                              // Enables GUI reset command
                                                              // Routes analog debug data to TP36
                                                              // Enables other debug features 




#define MINIMUM_POWER_TARGET                               100           // 1 Watt - Below this level the amplifier will be disabled
#define MAX_POWER_TARGET                                   53500         // 535 Watts
#define FOLDBACK_POWER_PROGRAM                             25000         // 250 Watts

#define FRONT_PANEL_LED_NUMBER_OF_FLASHES_AT_STARTUP       3


// ------------  START PID CONFIGURATION --------------- //
#define _POWER_BASED_PID_MODE                                 // Enables multiple PID values based on power level


// This are the values if _POWER_BASED_PID_MODE is not defined
#define POWER_PID_P_COMPONENT                              .0
#define POWER_PID_I_COMPONENT                              .1
#define POWER_PID_D_COMPONENT                              .00



// This are the values if _POWER_BASED_PID_MODE is defined
#define PID_P_10_WATT                                      Q15(.0)
#define PID_I_10_WATT                                      Q15(.999)
#define PID_D_10_WATT                                      Q15(.0)

#define PID_P_50_WATT                                      Q15(.0)
#define PID_I_50_WATT                                      Q15(.4)
#define PID_D_50_WATT                                      Q15(.2)

#define PID_P_100_WATT                                     Q15(.0)
#define PID_I_100_WATT                                     Q15(.2)
#define PID_D_100_WATT                                     Q15(.15)

#define PID_P_250_WATT                                     Q15(.0)
#define PID_I_250_WATT                                     Q15(.09)
#define PID_D_250_WATT                                     Q15(.02)

#define PID_P_500_WATT                                     Q15(.0)
#define PID_I_500_WATT                                     Q15(.06)
#define PID_D_500_WATT                                     Q15(.00)


// ------------  END PID CONFIGURATION --------------- //






// -------------------- Configuration for Fault Levels ------------------------ //

#define FAULT_OVER_POWER_MULT                              32768  // 1.0000 // The target power is multiplied by this to generate the over power level
#define FAULT_MINIMUM_OVER_POWER                           5000    // 50.00 Watts // This is absolute minimum for the over power setting relative to the power setting
#define FAULT_TIME_OVER_POWER                              10     // Time that the unit must be overpower (in 10ms units) to generate an over power fault

#define FAULT_UNDER_POWER_MULT                             58982  // .899994
#define FAULT_MINIMUM_UNDER_POWER                          500    // 5.00 Watts
#define FAULT_TIME_UNDER_POWER                             50     // Time that the unit must be overpower (in 10ms units) to generate an under power fault


#define RF_AMPLIFIER_TEMPERATURE_MIN                       2330           // 233 K, -40 C
#define RF_AMPLIFIER_TEMPERATURE_MAX                       3480           // 348 K, 75 C

#define RF_DETECTOR_TEMPERATURE_MIN                        2330           // 233 K, -40 C
#define RF_DETECTOR_TEMPERATURE_MAX                        3580           // 358 K, 85 C


#define FORWARD_DETECTOR_MAX_POWER                         32500          // 325 Watts
#define FORWARD_OVER_POWER_TRIP_TIME_10MS_UNITS            50             // 500ms
#define REVERSE_DETECTOR_MAX_POWER                         32500          // 325 Watts
#define REVERSE_OVER_POWER_TRIP_TIME_10MS_UNITS            50             //  500ms

#define FORWARD_DETECTOR_MISMATCH_SCALE                    16384          // .25 Multiplier
#define FORWARD_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT        1000           // 10 Watts
#define FORWARD_DETECTOR_MISMATCH_TIME                     10             // 100mS

#define REVERSE_DETECTOR_MISMATCH_SCALE                    16384          // .25 Multiplier
#define REVERSE_DETECTOR_MISMATCH_MINIMUM_FOR_FAULT        1000           // 10 Watts
#define REVERSE_DETECTOR_MISMATCH_TIME                     100            // 1000mS




// -------------------- Over Reflected Power -------------------- //

#define MAX_STEADY_STATE_REFLECTED_POWER                   25000   // 250 Watts
#define MAX_FOLDBACK_REFLECTED_POWER                       12500   // 125 Watts
#define MAX_OVER_REFLECTED_TIME                            6000    // 15 Seconds (10ms Units)
#define MAX_OVER_REFLECTED_TIME_HYSTERESIS                 200     // 2 Seconds (10ms Units)





#endif
