//! SDRAM chip configuration for W9825G6KH
//!
//! This module provides the configuration for the W9825G6KH SDRAM chip used in
//! the Apollo STM32F429 development board.

use stm32_fmc::SdramChip;
use stm32_fmc::SdramConfiguration;
use stm32_fmc::SdramTiming;

/// Configuration for W9825G6KH SDRAM chip
///
/// Specifications:
/// - Capacity: 32Mbit x 4 banks x 16 bits = 256Mbit (32MB)
/// - Organization: 4 banks, 13-bit row address, 9-bit column address
/// - Speed: 166MHz (6ns)
/// - CAS Latency: 3
pub struct W9825G6KH;

impl SdramChip for W9825G6KH {
    /// Mode register value
    ///
    /// Bits:
    /// - [8:7] = 00: Reserved
    /// - [6:4] = 011: CAS Latency = 3
    /// - [3] = 0: Write burst mode: Programmed burst length
    /// - [2:0] = 000: Burst length = 1 (single location access)
    const MODE_REGISTER: u16 = 0x0230; // Mode register value for CAS latency 3, burst length 1

    /// SDRAM configuration
    const CONFIG: SdramConfiguration = SdramConfiguration {
        // Number of bits of column address
        column_bits: 9,
        // Number of bits of row address
        row_bits: 13,
        // Memory device width
        memory_data_width: 16,
        // Number of the device's internal banks
        internal_banks: 4,
        // SDRAM CAS latency in number of memory clock cycles
        cas_latency: 3,
        // Enables the SDRAM device to be accessed in write mode
        write_protection: false,
        // This bit enable the SDRAM controller to anticipate the next read
        read_burst: true,
        // Delay in system clock cycles on read data path
        read_pipe_delay_cycles: 0,
    };

    /// SDRAM timing
    const TIMING: SdramTiming = SdramTiming {
        // Time between applying a valid clock and any command other than
        // COMMAND INHIBIT or NOP
        startup_delay_ns: 200_000, // 200 μs
        // Maximum SD clock frequency to make timing
        max_sd_clock_hz: 166_000_000, // 166 MHz
        // Period between refresh cycles in nanoseconds
        refresh_period_ns: 64_000_000, // 64 ms
        // Delay between a LOAD MODE register command and an ACTIVATE command
        mode_register_to_active: 2,
        // Delay from releasing self refresh to next command
        exit_self_refresh: 7,
        // Delay between an ACTIVATE and a PRECHARGE command
        active_to_precharge: 4,
        // Auto refresh command duration
        row_cycle: 7,
        // Delay between a PRECHARGE command and another command
        row_precharge: 2,
        // Delay between an ACTIVATE command and READ/WRITE command
        row_to_column: 3,
    };
}
