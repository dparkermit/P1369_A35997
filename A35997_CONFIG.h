#ifndef __A35997_CONFIG_H
#define __A35997_CONFIG_H  

#define FAULT_OVER_POWER_MULT                              36045  // 1.100006

#define POWER_PID_P_COMPONENT                               .2
#define POWER_PID_I_COMPONENT                               .5
#define POWER_PID_D_COMPONENT                               .01

#define RF_AMPLIFIER_TEMPERATURE_MIN                 2330           // 233 K, -40 C
#define RF_AMPLIFIER_TEMPERATURE_MAX                 3480           // 348 K, 75 C

#define RF_DETECTOR_TEMPERATURE_MIN                  2330           // 233 K, -40 C
#define RF_DETECTOR_TEMPERATURE_MAX                  3580           // 358 K, 85 C


#define MINIMUM_POWER_TARGET            100           // 1 Watt
#define FOLDBACK_POWER_PROGRAM          25000         // 250 Watts


#define FORWARD_DETECTOR_MAX_POWER                   30000
#define FORWARD_OVER_POWER_TRIP_TIME_10MS_UNITS      100
#define REVERSE_DETECTOR_MAX_POWER                   30000
#define REVERSE_OVER_POWER_TRIP_TIME_10MS_UNITS      100



#define FRONT_PANEL_LED_NUMBER_OF_FLASHES_AT_STARTUP                 3


#define MAX_STEADY_STATE_REFLECTED_POWER             25000   // 250 Watts
#define MAX_FOLDBACK_REFLECTED_POWER                 12500   // 125 Watts
#define MAX_OVER_REFLECTED_TIME                      1500    // 15 Seconds
#define MAX_OVER_REFLECTED_TIME_HYSTERESIS           200     // 2 Seconds


#endif
