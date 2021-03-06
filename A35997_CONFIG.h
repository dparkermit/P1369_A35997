#ifndef __A35997_CONFIG_H
#define __A35997_CONFIG_H  


#define _DO_TIME_COMP

#define _DO_THERMAL_COMP

//#define _OPEN_LOOP_MODE                                       // When defined, unit operates in open loop mode

//#define _DEBUG_MODE                                           // This line enables debugging code 
                                                              // Enables GUI setting the target power
                                                              // Enables GUI reset command
                                                              // Routes analog debug data to TP36
                                                              // Enables other debug features 




#define MINIMUM_POWER_TARGET                               500           // 5 Watt - Below this level the amplifier will be disabled
#define MINIMUM_POWER_TARGET_HYSTERESIS                    200           // Hysteresis on Minimum power.
                                                                         // Once the program level exceeds 5 watts it must drop below 2 watts before it turns off
#define MAX_POWER_TARGET                                   53500         // 535 Watts

#define FRONT_PANEL_LED_NUMBER_OF_FLASHES_AT_STARTUP       3


// ------------  START PID CONFIGURATION --------------- //
#define _POWER_BASED_PID_MODE                                 // Enables multiple PID values based on power level


// This are the values if _POWER_BASED_PID_MODE is not defined
#define POWER_PID_P_COMPONENT                              .0
#define POWER_PID_I_COMPONENT                              .05
#define POWER_PID_D_COMPONENT                              .00



// This are the values if _POWER_BASED_PID_MODE is defined
#define PID_P_0_WATT                                       Q15(0)
#define PID_I_0_WATT                                       Q15(.99)
#define PID_D_0_WATT                                       Q15(.99)

#define PID_P_50_WATT                                      Q15(.35)
#define PID_I_50_WATT                                      Q15(.52)
#define PID_D_50_WATT                                      Q15(.20)

#define PID_P_100_WATT                                     Q15(.23)
#define PID_I_100_WATT                                     Q15(.29)
#define PID_D_100_WATT                                     Q15(.15)



#define PID_P_150_WATT                                     Q15(.165)
#define PID_I_150_WATT                                     Q15(.205)
#define PID_D_150_WATT                                     Q15(.10)

#define PID_P_200_WATT                                     Q15(.135)
#define PID_I_200_WATT                                     Q15(.17)
#define PID_D_200_WATT                                     Q15(.05)



#define PID_P_250_WATT                                     Q15(.105)
#define PID_I_250_WATT                                     Q15(.135)
#define PID_D_250_WATT                                     Q15(.02)



#define PID_P_300_WATT                                     Q15(.095)
#define PID_I_300_WATT                                     Q15(.115)
#define PID_D_300_WATT                                     Q15(.015)

#define PID_P_350_WATT                                     Q15(.085)
#define PID_I_350_WATT                                     Q15(.105)
#define PID_D_350_WATT                                     Q15(.0095)

#define PID_P_400_WATT                                     Q15(.07)
#define PID_I_400_WATT                                     Q15(.095)
#define PID_D_400_WATT                                     Q15(.0065)

#define PID_P_450_WATT                                     Q15(.05)
#define PID_I_450_WATT                                     Q15(.085)
#define PID_D_450_WATT                                     Q15(.004)






#define PID_P_500_WATT                                     Q15(.025)
#define PID_I_500_WATT                                     Q15(.075)
#define PID_D_500_WATT                                     Q15(.001)


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
#define FOLDBACK_POWER_PROGRAM                             20000   // 200 Watts
#define MAX_STEADY_STATE_REFLECTED_POWER                   20000   // 200 Watts
#define MAX_FOLDBACK_REFLECTED_POWER                       10000   // 100 Watts
#define MAX_OVER_REFLECTED_TIME                            4       // 40mS (10ms Units)
#define MAX_OVER_REFLECTED_TIME_HYSTERESIS                 500     // 5 Sec (10ms Units)



#endif
