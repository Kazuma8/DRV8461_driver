/*  DRV8461_Registers.h - copyright Vulintus, Inc., 2023

    Register addresses and settings for the Texas Instruments DRV8461 stepper
    driver.

    UPDATE LOG:
      2023-11-04 - Drew Sloan - Library first created.

*/
#include <stdint.h>

#ifndef DRV8461_REGISTERS_H
#define DRV8461_REGISTERS_H


// REGISTER ADDRESSES ************************************************************************************************// 
enum DRV8461_REG_ADDR : uint8_t {
  DRV8461_REG_FAULT  = 0x00,            // Fault status register.
  DRV8461_REG_DIAG1  = 0x01,            // DIAG status 1.
  DRV8461_REG_DIAG2  = 0x02,            // DIAG status 2.
  DRV8461_REG_DIAG3  = 0x03,            // DIAG status 3.
  DRV8461_REG_CTRL1  = 0x04,            // Control register 1.
  DRV8461_REG_CTRL2  = 0x05,            // Control register 2.
  DRV8461_REG_CTRL3  = 0x06,            // Control register 3.
  DRV8461_REG_CTRL4  = 0x07,            // Control register 4.
  DRV8461_REG_CTRL5  = 0x08,            // Control register 5.
  DRV8461_REG_CTRL6  = 0x09,            // Control register 6.
  DRV8461_REG_CTRL7  = 0x0A,            // Control register 7.
  DRV8461_REG_CTRL8  = 0x0B,            // Control register 8.
  DRV8461_REG_CTRL9  = 0x0C,            // Control register 9.
  DRV8461_REG_CTRL10 = 0x0D,            // Control Register 10.
  DRV8461_REG_CTRL11 = 0x0E,            // Control Register 11.
  DRV8461_REG_CTRL12 = 0x0F,            // Control Register 12.
  DRV8461_REG_CTRL13 = 0x10,            // Control Register 13.
  DRV8461_REG_CTRL14 = 0x3C,            // Control Register 14.
};


// FAULT REGISTER SETINGS ********************************************************************************************// 
enum DRV8461_FAULT_Reg_Val : uint8_t {
  DRV8461_FAULT_FAULT = 0x80,          // Fault bit is high when nFAULT output is low.
  DRV8461_FAULT_SPI_ERR = 0x40,        // Indicates SPI protocol errors.
  DRV8461_FAULT_UVLO = 0x20,           // Indicates undervoltage lockout fault condition.
  DRV8461_FAULT_CPUV = 0x10,           // Indicates charge pump undervoltage fault condition.
  DRV8461_FAULT_OCP = 0x08,            // Indicates overcurrent fault condition.
  DRV8461_FAULT_STL = 0x04,            // Indicates motor stall condition.
  DRV8461_FAULT_TF = 0x02,             // Indicates overtemperature warning/shutdown condition.
  DRV8461_FAULT_OL = 0x01,             // Indicates open-load condition.
};


// DIAG STATUS 1 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_DIAG1_Reg_Val : uint8_t {
  DRV8461_DIAG1_OCP_LS2_B = 0x80,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in BOUT.
  DRV8461_DIAG1_OCP_HS2_B = 0x40,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in BOUT.
  DRV8461_DIAG1_OCP_LS1_B = 0x20,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in BOUT.
  DRV8461_DIAG1_OCP_HS1_B = 0x10,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in BOUT.
  DRV8461_DIAG1_OCP_LS2_A = 0x08,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in AOUT.
  DRV8461_DIAG1_OCP_HS2_A = 0x04,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in AOUT.
  DRV8461_DIAG1_OCP_LS1_A = 0x02,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in AOUT.
  DRV8461_DIAG1_OCP_HS1_A = 0x01,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in AOUT.
};


// DIAG STATUS 2 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_DIAG2_Reg_Val : uint8_t {
  DRV8461_DIAG2_STSL = 0x80,           // Indicates operating in standstill power saving mode
  DRV8461_DIAG2_OTW = 0x40,            // Indicates overtemperature warning.
  DRV8461_DIAG2_OTS = 0x20,            // Indicates overtemperature shutdown.
  DRV8461_DIAG2_STL_LRN_OK = 0x10,     // Indicates stall detection learning is successful.
  DRV8461_DIAG2_STALL = 0x08,          // Indicates motor stall condition.
  DRV8461_DIAG2_OL_B = 0x02,           // Indicates open-load detection on BOUT.
  DRV8461_DIAG2_OL_A = 0x01,           // Indicates open-load detection on AOUT
};


// DIAG STATUS 3 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_DIAG3_Reg_Val : uint8_t {
  DRV8461_DIAG3_NHOME = 0x40,          // Indicates indexer is at a position other than home.
  DRV8461_DIAG3_CNT_OFLW = 0x20,       // Indicates ATQ_CNT is more than ATQ_UL.
  DRV8461_DIAG3_CNT_UFLW = 0x10,       // Indicates ATQ_CNT is less than ATQ_LL.
  DRV8461_DIAG3_NPOR = 0x02,           // Low indicates a prior VCC UVLO event, High indicates that NPOR has been cleared by CLR_FLT or nSLEEP after a VCC UVLO.
};


// CONTROL 1 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL1_Reg_Val : uint8_t {
  DRV8461_CTRL1_EN_OUT  = 0x80,        // Write '0' to disable all outputs.
  DRV8461_CTRL1_SR      = 0x40,        // 0 = Output Rise/Fall time = 140ns, 1 = 70ns.
  DRV8461_CTRL1_IDX_RST = 0x20,        // Resets the indexer to 45˚ electrical angle, but keeps contents of registers.
  DRV8461_CTRL1_TOFF    = 0x18,        // ??? (default is 01b = 19 us).
  DRV8461_CTRL2_DECAY   = 0x07,        // ??? (default is 111b = Smart tune Ripple Control).
};


// Specific values for TOFF
enum DRV8461_PWM_TOFF : uint8_t {
  DRV8461_TOFF_9_5US = 0b00,           // 9.5 μs.
  DRV8461_TOFF_19US  = 0b01,           // 19 us.
  DRV8461_TOFF_27US  = 0b10,           // 27 us.
  DRV8461_TOFF_35US  = 0b11,           // 35 us.
};


enum DRV8461_Decay_Mode : uint8_t {
  DRV8461_DECAY_SLOW_SLOW     = 0b000,  // Increasing SLOW, decreasing SLOW.
  DRV8461_DECAY_SLOW_MIX30    = 0b001,  // Increasing SLOW, decreasing MIXED 30%.
  DRV8461_DECAY_SLOW_MIX60    = 0b010,  // Increasing SLOW, decreasing MIXED 60%.
  DRV8461_DECAY_SLOW_FAST     = 0b011,  // Increasing SLOW, decreasing FAST.
  DRV8461_DECAY_MIX30_MIX30   = 0b100,  // Increasing MIXED 30%, decreasing MIXED 30%.
  DRV8461_DECAY_MIX60_MIX60   = 0b101,  // Increasing MIXED 60%, decreasing MIXED 60%.
  DRV8461_DECAY_DYNAMIC       = 0b110,  // Smart tune Dynamic Decay.
  DRV8461_DECAY_SMART_RIPPLE  = 0b111,  // Smart tune Ripple Control (default).
};


// CONTROL 2 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL2_Reg_Val : uint8_t {
  DRV8461_CTRL2_DIR = 0x80,            // Direction input (when SPI_DIR = 1).
  DRV8461_CTRL2_STEP = 0x40,           // Step input (when SPI_SETP = 1), logic '1' advances one step (self-clearing).
  DRV8461_CTRL2_SPI_DIR = 0x20,        // Direction is controlled by SPI when high.
  DRV8461_CTRL2_SPI_STEP = 0x10,       // Stepping is controlled by SPI when high.
  DRV8461_CTRL2_MICROSTEP_MODE = 0x0F, // Micro-stepping mode (default is 0110b = 1/16 step).
};

enum DRV8461_Micostep_Mode : uint8_t
{
  DRV8461_MICROSTEP_1_100 = 0b0000,    // Full step (2-phase excitation) with 100% current.
  DRV8461_MICROSTEP_1_71  = 0b0001,    // Full step (2-phase excitation) with 71% current.
  DRV8461_MICROSTEP_2_NC  = 0b0010,    // Non-circular 1/2 step.
  DRV8461_MICROSTEP_2     = 0b0011,    // 1/2 step.
  DRV8461_MICROSTEP_4     = 0b0100,    // 1/4 step.
  DRV8461_MICROSTEP_8     = 0b0101,    // 1/8 step.
  DRV8461_MICROSTEP_16    = 0b0110,    // 1/16 step.
  DRV8461_MICROSTEP_32    = 0b0111,    // 1/32 step.
  DRV8461_MICROSTEP_64    = 0b1000,    // 1/64 step.
  DRV8461_MICROSTEP_128   = 0b1001,    // 1/128 step.
  DRV8461_MICROSTEP_256   = 0b1010,    // 1/256 step.
};


// CONTROL 3 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL3_Reg_Val : uint8_t {
  DRV8461_CTRL3_CLR_FLT = 0x80,        // Clear all latched fault bits (self-resetting) by writing 1.
  DRV8461_CTRL3_LOCK = 0x70,           // Lock registers and ignore further register writes except for CLR_FLT.
  DRV8461_CTRL3_TOCP = 0x08,           // Set overcurrent protection deglitch time (0 = 1.2 us, 1 = 2.2us) default 1.
  DRV8461_CTRL3_OCP_MODE = 0x04,       // Overcurrent conditon fault latching (0 = latching, 1 = automatic retrying) default 0.
  DRV8461_CTRL3_OTSD_MODE = 0x02,      // Overtemperature conditon fault latching (0 = latching, 1 = automatic recovery) default 0.
  DRV8461_CTRL3_TW_REP = 0x01,         // Overtemperature warning reporting on nFAULT (0 = not reported, 1 = reported) default 0.
};


// CONTROL 4 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL4_Reg_Val : uint8_t {
  DRV8461_CTRL4_TBLANK_TIME = 0xC0,    // Controls current sense blanking time (default 1.5us).
  DRV8461_CTRL4_STL_LRN = 0x20,        // Automatic learning of stall detection threshold (set to 1, resets to 0).
  DRV8461_CTRL4_EN_STL = 0x10,         // Stall detection (1 = enabled) default 0.
  DRV8461_CTRL4_STL_REP = 0x08,        // Stall detection report on nFAULT (1 = enabled) default 1.
  DRV8461_CTRL4_FRQ_CHG = 0x40,        // STEP input is filtered as per STEP_FRQ_TOL bits (0 = enabled) default 0.
  DRV8461_CTRL4_STEP_FRQ_TOL = 0x03,   // Programs the filter setting for STEP input. Default 2%.
};


enum DRV8461_Step_Frequency : uint8_t {
  DRV8461_STEP_FRQ_FLTR_1 = 0b00,      // 1% filtering
  DRV8461_STEP_FRQ_FLTR_2 = 0b01,      // 2% filtering
  DRV8461_STEP_FRQ_FLTR_4 = 0b10,      // 4% filtering
  DRV8461_STEP_FRQ_FLTR_6 = 0b11,      // 6% filtering
};


// CONTROL 5 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL5_Reg_Val : uint8_t {
  DRV8461_CTRL5_STALL_TH = 0xFF,       // Lower 8-btis of stall threshold (default is 00000011b / 3).
};


// CONTROL 6 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL6_Reg_Val : uint8_t {
  DRV8461_CTRL6_RC_RIPPLE = 0xC0,      // Ripple setting (00b = 1%, 01b = 2%, 10b = 4%, 11b = 6%).
  DRV8461_CTRL6_EN_SSC    = 0x20,      // Enable spread-spectrum (on by default).
  DRV8461_CTRL6_TRQ_SCALE = 0x10,      // Apply 8x torque scaling (off by default).
  DRV8461_CTRL6_STL_REP   = 0x0F,      // Upper 4-bits of stall threshold.
};


enum DRV8461_RC_Ripple : uint8_t {
  DRV8461_RIPPLE_1 = 0b00,    // 1% ripple (default).
  DRV8461_RIPPLE_2 = 0b01,    // 2% ripple (default).
  DRV8461_RIPPLE_4 = 0b10,    // 4% ripple (default).
  DRV8461_RIPPLE_6 = 0b11,    // 6% ripple (default).
};


// CONTROL 7 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL7_Reg_Val : uint8_t {
  DRV8461_CTRL7_TRQ_COUNT = 0xFF,      // Lower 8-bits of TRQ_COUNT.
};


// CONTROL 8 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL8_Reg_Val : uint8_t {
  DRV8461_CTRL8_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};


// CONTROL 9 REGISTER SETINGS ************************************************************************************// 
enum DRV8461_CTRL9_Reg_Val : uint8_t {
  DRV8461_CTRL9_EN_OL     = 0x80,      // Enable open load detection.
  DRV8461_CTRL9_OL_MODE   = 0x40,      // Open-load fault clearing mode.
  DRV8461_CTRL9_OL_T      = 0x30,      // Controls the open load fault detection time. (Default = 60ms, max)
  DRV8461_CTRL9_STEP_EDGE = 0x08,      // Active edge for STEP input (0 = only rising, 1 = both rising and falling) default 0.
  DRV8461_CTRL9_RES_AUTO  = 0x06,      // Controls microstepping resolution in auto mode. (Default = 1/256)
  DRV8461_CTRL9_EN_AUTO   = 0x01,      // Automatic microstepping (0 = disabled, 1 = enabled) default 0.
};


enum DRV8461_Open_Load_Detection_Time : uint8_t {
  DRV8461_OLT_30  = 0b00,              // 30ms
  DRV8461_OLT_60  = 0b01,              // 60ms
  DRV8461_OLT_120 = 0b10,              // 120ms
};


enum DRV8461_Auto_Microstep : uint8_t {
  DRV8461_MICRO_RES_256 = 0b00,        // 1/256
  DRV8461_MICRO_RES_128 = 0b01,        // 1/128
  DRV8461_MICRO_RES_64  = 0b10,        // 1/64
  DRV8461_MICRO_RES_32  = 0b11,        // 1/32
};


// CONTROL 10 REGISTER SETINGS ***********************************************************************************// 


enum DRV8461_CTRL10_Reg_Val : uint8_t {
  DRV8461_CTRL10_ISTSL = 0xFF,         // Determines the holding current
};


enum DRV8461_Holding_Current : uint8_t {
  DRV8461_ISTSL_256 = 0b11111111,      // X/256 * 100
  DRV8461_ISTSL_255
  DRV8461_ISTSL_254
  DRV8461_ISTSL_253
  DRV8461_ISTSL_25
  DRV8461_ISTSL_25
  DRV8461_ISTSL_25
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  DRV8461_ISTSL_24
  
};


// CONTROL 11 REGISTER SETINGS ***********************************************************************************// 


enum DRV8461_CTRL11_Reg_Val : uint8_t {
  DRV8461_CTRL11_TRQ_SCALE = 0x0F,     // Upper 4-bits of TRQ_COUNT.
};


// CONTROL 12 REGISTER SETINGS ***********************************************************************************// 


enum DRV8461_CTRL12_Reg_Val : uint8_t {
  DRV8461_CTRL12_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};


// CONTROL 13 REGISTER SETINGS ***********************************************************************************// 


enum DRV8461_CTRL13_Reg_Val : uint8_t {
  DRV8461_CTRL13_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};


// CONTROL 14 REGISTER SETINGS ***********************************************************************************// 


enum DRV8461_CTRL14_Reg_Val : uint8_t {
  DRV8461_CTRL14_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};


#endif                                    // #ifndef DRV8461_REGISTERS_H
