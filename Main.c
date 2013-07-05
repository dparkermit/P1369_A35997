#include <p30F6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include "LTC2656.h"
#include "A35997_PINS.h"
#include "A35997.h"



/*
DPARKER - AT SOME POINT WE WILL NEED TO READ CONFIGURATION DATA OUT OF EEPROM

void ReadAllEEpromToRAM(void);

_prog_addressT EE_address_ps_magnet_config_in_EEPROM;
unsigned int _EEDATA(32) ps_magnet_config_in_EEPROM[] = PS_MAGNET_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_magnet_config_ram_copy[16];

_prog_addressT EE_address_ps_filament_config_in_EEPROM;
unsigned int _EEDATA(32) ps_filament_config_in_EEPROM[] = PS_FILAMENT_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_filament_config_ram_copy[16];

_prog_addressT EE_address_ps_thyr_cathode_htr_config_in_EEPROM;
unsigned int _EEDATA(32) ps_thyr_cathode_htr_config_in_EEPROM[] = PS_THYR_CATH_HTR_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_thyr_cathode_htr_config_ram_copy[16];

_prog_addressT EE_address_ps_thyr_reservoir_htr_config_in_EEPROM;
unsigned int _EEDATA(32) ps_thyr_reservoir_htr_config_in_EEPROM[] = PS_THYR_RESER_HTR_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_thyr_reservoir_htr_config_ram_copy[16];


_prog_addressT EE_address_ps_hv_lambda_mode_A_config_in_EEPROM;
unsigned int _EEDATA(32) ps_hv_lambda_mode_A_config_in_EEPROM[] = PS_HV_LAMBDA_MODE_A_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_hv_lambda_mode_A_config_ram_copy[16];

_prog_addressT EE_address_ps_hv_lambda_mode_B_config_in_EEPROM;
unsigned int _EEDATA(32) ps_hv_lambda_mode_B_config_in_EEPROM[] = PS_HV_LAMBDA_MODE_B_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_hv_lambda_mode_B_config_ram_copy[16];


_prog_addressT EE_address_ps_magnetron_mode_A_config_in_EEPROM;
unsigned int _EEDATA(32) ps_magnetron_mode_A_config_in_EEPROM[] = PS_MAGNETRON_MODE_A_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_magnetron_mode_A_config_ram_copy[16];

_prog_addressT EE_address_ps_magnetron_mode_B_config_in_EEPROM;
unsigned int _EEDATA(32) ps_magnetron_mode_B_config_in_EEPROM[] = PS_MAGNETRON_MODE_B_DEFAULT_CONFIG;  // Create 16 word structure in EEPROM and load default configuration values
signed int ps_magnetron_mode_B_config_ram_copy[16];

_prog_addressT EE_address_pulse_counter_repository_in_EEPROM;
unsigned int _EEDATA(32) pulse_counter_repository_in_EEPROM[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};  // Create 16 word structure in EEPROM and load zeros
signed int pulse_counter_repository_ram_copy[16];


_prog_addressT EE_address_control_loop_cal_data_in_EEPROM;
unsigned int _EEDATA(32) control_loop_cal_data_in_EEPROM[] = CONTROL_LOOP_CAL_DATA_DEFAULT_CONFIG; // Create 16 word structure in EEPROM and load default configuration values
signed int control_loop_cal_data_ram_copy[16];

*/

/* Configuration Bit Settings */
//_FOSC(ECIO & CSW_FSCM_OFF); // External Oscillator with no PLL and Startup with User Selected Oscillator Source

_FOSC(FRC_PLL16 & CSW_FSCM_OFF); // Internal Oscillator is 7.3738MHz with 16x PLL and 4x Clocks per Instruction cycle
                                 // This yields an Instruction Cycle Frequency of 29.4952 MHz
                                 // This yields an Instruction Cycle Time of 33.904nS


// Watchdog Timeout is 2 Millisconds with no pre scallers
//_FWDT(WDT_ON & WDTPSA_1 & WDTPSB_2);  // Watchdog Timer is enabled, 4ms TIMEOUT
//_FWDT(WDT_ON & WDTPSA_1 & WDTPSB_16);  // Watchdog Timer is enabled, 32ms TIMEOUT
_FWDT(WDT_OFF & WDTPSA_1 & WDTPSB_16);  // Watchdog Timer is disnabled, 32ms TIMEOUT


//_FBORPOR(PWRT_64 & BORV_27 & PBOR_ON & MCLR_EN); // Brown out and Power on Timer settings
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN); // Brown out and Power on Timer settings

// No Boot Segment
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



int main(void) {
  // ReadAllEEpromToRAM();  // Ream all configuration from EEPROM into RAM
  


  while (1) {
    if (PIN_RF_ENABLE == ILL_PIN_RF_ENABLE_ENABLED) {
      // The RF Output should be on
    }
  }
}


// PID Interrupt, this should occur at 100us Intervals


// Internal ADC Interrupt
/*
  This should occur once every 8 ADC readings (if sample rate is 80 Khz then once every 100us)
*/


// Serial Interrupt
/*
  If a new byte has been recieved, move 
 */





/*
void ReadAllEEpromToRAM(void) {
  _init_prog_address(EE_address_ps_magnet_config_in_EEPROM, ps_magnet_config_in_EEPROM);
  _memcpy_p2d16(ps_magnet_config_ram_copy, EE_address_ps_magnet_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_filament_config_in_EEPROM, ps_filament_config_in_EEPROM);
  _memcpy_p2d16(ps_filament_config_ram_copy, EE_address_ps_filament_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_thyr_cathode_htr_config_in_EEPROM, ps_thyr_cathode_htr_config_in_EEPROM);
  _memcpy_p2d16(ps_thyr_cathode_htr_config_ram_copy, EE_address_ps_thyr_cathode_htr_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_thyr_reservoir_htr_config_in_EEPROM, ps_thyr_reservoir_htr_config_in_EEPROM);
  _memcpy_p2d16(ps_thyr_reservoir_htr_config_ram_copy, EE_address_ps_thyr_reservoir_htr_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_hv_lambda_mode_A_config_in_EEPROM, ps_hv_lambda_mode_A_config_in_EEPROM);
  _memcpy_p2d16(ps_hv_lambda_mode_A_config_ram_copy, EE_address_ps_hv_lambda_mode_A_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_hv_lambda_mode_B_config_in_EEPROM, ps_hv_lambda_mode_B_config_in_EEPROM);
  _memcpy_p2d16(ps_hv_lambda_mode_B_config_ram_copy, EE_address_ps_hv_lambda_mode_B_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_magnetron_mode_A_config_in_EEPROM, ps_magnetron_mode_A_config_in_EEPROM);
  _memcpy_p2d16(ps_magnetron_mode_A_config_ram_copy, EE_address_ps_magnetron_mode_A_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_ps_magnetron_mode_B_config_in_EEPROM, ps_magnetron_mode_B_config_in_EEPROM);
  _memcpy_p2d16(ps_magnetron_mode_B_config_ram_copy, EE_address_ps_magnetron_mode_B_config_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_pulse_counter_repository_in_EEPROM, pulse_counter_repository_in_EEPROM);
  _memcpy_p2d16(pulse_counter_repository_ram_copy, EE_address_pulse_counter_repository_in_EEPROM, _EE_ROW);
  ClrWdt();

  _init_prog_address(EE_address_control_loop_cal_data_in_EEPROM, control_loop_cal_data_in_EEPROM);
  _memcpy_p2d16(control_loop_cal_data_ram_copy, EE_address_control_loop_cal_data_in_EEPROM, _EE_ROW);
  ClrWdt();
}
*/
