#![no_main]
#![no_std]

mod key;
mod lcd;
mod led;
mod sdram;

// Print panic message to probe console
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use crate::sdram::SdramDriver;
use stm32f4xx_hal::{
    gpio::alt::fmc as alt_fmc,
    gpio::GpioExt,
    hal::delay::DelayNs,
    pac,
    prelude::*,
    rcc::{Config, RccExt},
};

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    // 1. 获取芯片外设实例
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure system clock - Updated to match C code (HSE=25MHz, SYSCLK=192MHz)
    let clock_cfg = Config::hse(25.MHz())
        .sysclk(180.MHz())
        .hclk(180.MHz()) // AHB总线时钟 = SYSCLK/1
        .pclk1(45.MHz()) // APB1低速外设时钟 = HCLK/4
        .pclk2(90.MHz()); // APB2高速外设时钟 = HCLK/2

    // 2. 配置系统时钟（关键：延时精度依赖时钟频率）
    let mut rcc = dp.RCC.constrain().freeze(clock_cfg);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    info!("system initialized");

    // 配置GPIO端口
    let gpio_b = dp.GPIOB.split(&mut rcc);
    let gpio_c = dp.GPIOC.split(&mut rcc);
    let gpio_d = dp.GPIOD.split(&mut rcc);
    let gpio_e = dp.GPIOE.split(&mut rcc);
    let gpio_f = dp.GPIOF.split(&mut rcc);
    let gpio_g = dp.GPIOG.split(&mut rcc);
    let gpio_h = dp.GPIOH.split(&mut rcc);

    // 配置LED引脚为推挽输出模式并初始化LED模块
    let led0 = gpio_b.pb1.into_push_pull_output();
    let led1 = gpio_b.pb0.into_push_pull_output();
    led::init(led0, led1);

    // 2. 配置GPIO引脚为FMC功能
    // 初始化SDRAM和配置FMC引脚
    let _fmc_pins = (
        // Address pins
        alt_fmc::A0::from(gpio_f.pf0.into_alternate()),
        alt_fmc::A1::from(gpio_f.pf1.into_alternate()),
        alt_fmc::A2::from(gpio_f.pf2.into_alternate()),
        alt_fmc::A3::from(gpio_f.pf3.into_alternate()),
        alt_fmc::A4::from(gpio_f.pf4.into_alternate()),
        alt_fmc::A5::from(gpio_f.pf5.into_alternate()),
        alt_fmc::A6::from(gpio_f.pf12.into_alternate()),
        alt_fmc::A7::from(gpio_f.pf13.into_alternate()),
        alt_fmc::A8::from(gpio_f.pf14.into_alternate()),
        alt_fmc::A9::from(gpio_f.pf15.into_alternate()),
        alt_fmc::A10::from(gpio_g.pg0.into_alternate()),
        alt_fmc::A11::from(gpio_g.pg1.into_alternate()),
        alt_fmc::A12::from(gpio_g.pg2.into_alternate()),
        // Bank address pins
        alt_fmc::Ba0::from(gpio_g.pg4.into_alternate()),
        alt_fmc::Ba1::from(gpio_g.pg5.into_alternate()),
        // Data pins
        alt_fmc::D0::from(gpio_d.pd14.into_alternate()),
        alt_fmc::D1::from(gpio_d.pd15.into_alternate()),
        alt_fmc::D2::from(gpio_d.pd0.into_alternate()),
        alt_fmc::D3::from(gpio_d.pd1.into_alternate()),
        alt_fmc::D4::from(gpio_e.pe7.into_alternate()),
        alt_fmc::D5::from(gpio_e.pe8.into_alternate()),
        alt_fmc::D6::from(gpio_e.pe9.into_alternate()),
        alt_fmc::D7::from(gpio_e.pe10.into_alternate()),
        alt_fmc::D8::from(gpio_e.pe11.into_alternate()),
        alt_fmc::D9::from(gpio_e.pe12.into_alternate()),
        alt_fmc::D10::from(gpio_e.pe13.into_alternate()),
        alt_fmc::D11::from(gpio_e.pe14.into_alternate()),
        alt_fmc::D12::from(gpio_e.pe15.into_alternate()),
        alt_fmc::D13::from(gpio_d.pd8.into_alternate()),
        alt_fmc::D14::from(gpio_d.pd9.into_alternate()),
        alt_fmc::D15::from(gpio_d.pd10.into_alternate()),
        // Control pins
        alt_fmc::Nbl0::from(gpio_e.pe0.into_alternate()),
        alt_fmc::Nbl1::from(gpio_e.pe1.into_alternate()),
        alt_fmc::Sdcke0::from(gpio_h.ph2.into_alternate()),
        alt_fmc::Sdclk::from(gpio_g.pg8.into_alternate()),
        alt_fmc::Sdncas::from(gpio_g.pg15.into_alternate()),
        alt_fmc::Sdne0::from(gpio_h.ph3.into_alternate()),
        alt_fmc::Sdnras::from(gpio_f.pf11.into_alternate()),
        alt_fmc::Sdnwe::from(gpio_c.pc0.into_alternate()),
    );

    // 创建SDRAM驱动实例
    let mut sdram_driver = SdramDriver::new(dp.FMC);

    // 显示初始化前状态
    sdram_driver.debug_config();
    sdram_driver.debug_status();

    info!("开始SDRAM初始化");
    // 初始化SDRAM
    match sdram_driver.init(&mut rcc) {
        Ok(()) => info!("SDRAM初始化成功"),
        Err(e) => panic!("SDRAM初始化失败: {}", e),
    }

    // 显示初始化后状态
    sdram_driver.debug_config();
    sdram_driver.debug_status();

    // 执行基础测试
    // 增加延时确保SDRAM完全稳定
    delay.delay_ms(10);

    // 运行综合测试
    run_comprehensive_sdram_test(&mut sdram_driver);
    
    loop {
        // 检查按键状态并控制LED
        // if key::is_pressed(key::KeyId::Key0) {
        //     led::set_led(led::LedId::Led0, true);
        //     led::set_led(led::LedId::Led1, false);
        //     info!("KEY0 pressed - LED0 on");
        //     key::wait_release(key::KeyId::Key0);
        // } else if key::is_pressed(key::KeyId::Key1) {
        //     led::set_led(led::LedId::Led0, false);
        //     led::set_led(led::LedId::Led1, true);
        //     info!("KEY1 pressed - LED1 on");
        //     key::wait_release(key::KeyId::Key1);
        // } else if key::is_pressed(key::KeyId::Key2) {
        //     led::set_all_leds(&[true, true]);
        //     info!("KEY2 pressed - Both LEDs on");
        //     key::wait_release(key::KeyId::Key2);
        // } else if key::is_pressed(key::KeyId::KeyUp) {
        //     led::set_all_leds(&[false, false]);
        //     info!("KEY_UP pressed - Both LEDs off");
        //     key::wait_release(key::KeyId::KeyUp);
        // }

        delay.delay_ms(100) // 增加延时到100ms，减少CPU占用
    }
}

/// SDRAM读写测试函数
pub fn test_sdram_read_write(driver: &SdramDriver) -> bool {
    let test_addr: u32 = 0x1000;
    let test_size: usize = 256;

    // 创建测试数据
    let mut write_data: [u8; 256] = [0; 256];
    let mut read_data: [u8; 256] = [0; 256];

    // 填充测试数据
    for i in 0..test_size {
        write_data[i] = (i & 0xFF) as u8;
    }

    // 写入数据到SDRAM
    driver.write_buffer(&write_data, test_addr);
    cortex_m::asm::delay(100 * 180); // 增加延时确保写入完成
    // 从SDRAM读取数据
    driver.read_buffer(&mut read_data, test_addr, test_size);

    // 验证数据一致性
    for i in 0..test_size {
        if write_data[i] != read_data[i] {
            defmt::error!(
                "SDRAM test failed at index {}: expected {}, got {}",
                i,
                write_data[i],
                read_data[i]
            );
            return false;
        }
    }

    defmt::info!("SDRAM read/write test passed");
    true
}

/// SDRAM地址线测试函数
pub fn test_sdram_address_lines(driver: &SdramDriver) -> bool {
    let base_addr: u32 = 0x0000;
    let mut test_passed = true;

    // 测试不同的地址模式
    let patterns: [u8; 4] = [0x55, 0xAA, 0xF0, 0x0F];

    for &pattern in &patterns {
        // 在不同地址写入模式数据
        for i in 0..16 {
            let addr = base_addr + (1 << i); // 2^i地址偏移
            let data = [pattern; 4];
            driver.write_buffer(&data, addr);
        }

        // 验证数据
        for i in 0..16 {
            let addr = base_addr + (1 << i);
            let mut read_data = [0u8; 4];
            driver.read_buffer(&mut read_data, addr, 4);

            for j in 0..4 {
                if read_data[j] != pattern {
                    defmt::error!(
                        "Address line test failed at addr 0x{:x}, pattern 0x{:x}",
                        addr,
                        pattern
                    );
                    test_passed = false;
                }
            }
        }
    }

    if test_passed {
        defmt::info!("SDRAM address line test passed");
    }
    test_passed
}

/// SDRAM数据线测试函数
pub fn test_sdram_data_lines(driver: &SdramDriver) -> bool {
    let test_addr: u32 = 0x2000;
    let mut test_passed = true;

    // 测试各个数据位
    let bit_patterns: [u16; 16] = [
        0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080, 0x0100, 0x0200, 0x0400,
        0x0800, 0x1000, 0x2000, 0x4000, 0x8000,
    ];

    for &pattern in &bit_patterns {
        // 写入16位模式数据
        let write_val = pattern.to_le_bytes();
        driver.write_buffer(&write_val, test_addr);

        // 读取并验证
        let mut read_val = [0u8; 2];
        driver.read_buffer(&mut read_val, test_addr, 2);

        let read_pattern = u16::from_le_bytes([read_val[0], read_val[1]]);
        if read_pattern != pattern {
            defmt::error!(
                "Data line test failed: expected 0x{:04x}, got 0x{:04x}",
                pattern,
                read_pattern
            );
            test_passed = false;
        }
    }

    if test_passed {
        defmt::info!("SDRAM data line test passed");
    }
    test_passed
}

/// 综合SDRAM测试函数
pub fn run_comprehensive_sdram_test(driver: &SdramDriver) -> bool {
    defmt::info!("Starting comprehensive SDRAM test...");

    // 基本读写测试
    test_sdram_read_write(driver);

    // 地址线测试
    test_sdram_address_lines(driver);

    // 数据线测试
    test_sdram_data_lines(driver);

    // 大块数据测试
    test_large_block_transfer(driver);
    
    // u8读写测试
    test_u8_read_write(driver);
    
    // u16读写测试
    test_u16_read_write(driver);
    
    // u32读写测试
    test_u32_read_write(driver);

    defmt::info!("All SDRAM tests passed!");
    true
}

/// 大块数据传输测试
fn test_large_block_transfer(driver: &SdramDriver) {
    const BLOCK_SIZE: usize = 4096;
    let test_addr: u32 = 0x4000;

    // 创建测试数据
    let mut write_buffer = [0u8; BLOCK_SIZE];
    let mut read_buffer = [0u8; BLOCK_SIZE];

    // 填充测试数据
    for i in 0..BLOCK_SIZE {
        write_buffer[i] = ((i ^ (i >> 8)) & 0xFF) as u8;
    }

    // 执行大块传输
    driver.write_buffer(&write_buffer, test_addr);
    driver.read_buffer(&mut read_buffer, test_addr, BLOCK_SIZE);

    // 验证数据
    let mut errors = 0;
    for i in 0..BLOCK_SIZE {
        if write_buffer[i] != read_buffer[i] {
            errors += 1;
            if errors < 10 {
                defmt::debug!(
                    "Large block error at offset {}: expected 0x{:02x}, got 0x{:02x}",
                    i,
                    write_buffer[i],
                    read_buffer[i]
                );
            }
        }
    }

    if errors == 0 {
        defmt::info!("Large block transfer test passed ({} bytes)", BLOCK_SIZE);
    } else {
        defmt::warn!("Large block transfer test had {} errors", errors);
    }
}

/// u8读写测试
fn test_u8_read_write(driver: &SdramDriver) {
    defmt::info!("Starting u8 read/write test...");
    
    let test_addr: u32 = 0x5000;
    let test_values: [u8; 4] = [0x12, 0x34, 0x56, 0x78];
    
    // 测试u8写入
    for (i, &value) in test_values.iter().enumerate() {
        let addr = test_addr + i as u32;
        unsafe {
            (0xD0000000 as *mut u8).add(addr as usize).write_volatile(value);  // 修正基地址
        }
    }
    
    // 测试u8读取
    for (i, &expected) in test_values.iter().enumerate() {
        let addr = test_addr + i as u32;
        let read_value = unsafe {
            (0xD0000000 as *const u8).add(addr as usize).read_volatile()  // 修正基地址
        };
        
        if read_value == expected {
            defmt::info!("u8 test passed at address 0x{:x}: 0x{:02x}", addr, expected);
        } else {
            defmt::error!("u8 test failed at address 0x{:x}: expected 0x{:02x}, got 0x{:02x}", 
                         addr, expected, read_value);
        }
    }
    
    // 使用驱动函数测试
    driver.write_buffer(&test_values, test_addr);
    let mut read_values = [0u8; 4];
    driver.read_buffer(&mut read_values, test_addr, 4);
    
    for i in 0..4 {
        if test_values[i] == read_values[i] {
            defmt::info!("u8 driver test passed at index {}: 0x{:02x}", i, test_values[i]);
        } else {
            defmt::error!("u8 driver test failed at index {}: expected 0x{:02x}, got 0x{:02x}", 
                         i, test_values[i], read_values[i]);
        }
    }
}

/// u16读写测试
fn test_u16_read_write(driver: &SdramDriver) {
    defmt::info!("Starting u16 read/write test...");
    
    let test_addr: u32 = 0x6000;
    let test_values: [u16; 4] = [0x1234, 0x5678, 0x9ABC, 0xDEF0];
    
    // 测试u16写入和读取（直接访问）
    for (i, &value) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 2) as u32; // u16占2个字节
        unsafe {
            (0xD0000000 as *mut u16).add((addr/2) as usize).write_volatile(value);  // 修正基地址
        }
    }
    
    for (i, &expected) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 2) as u32;
        let read_value = unsafe {
            (0xD0000000 as *const u16).add((addr/2) as usize).read_volatile()  // 修正基地址
        };
        
        if read_value == expected {
            defmt::info!("u16 test passed at address 0x{:x}: 0x{:04x}", addr, expected);
        } else {
            defmt::error!("u16 test failed at address 0x{:x}: expected 0x{:04x}, got 0x{:04x}", 
                         addr, expected, read_value);
        }
    }
    
    // 使用驱动函数测试
    for (i, &value) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 2) as u32;
        driver.write_u16(value, addr);
    }
    
    for (i, &expected) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 2) as u32;
        let read_value = driver.read_u16(addr);
        
        if read_value == expected {
            defmt::info!("u16 driver test passed at address 0x{:x}: 0x{:04x}", addr, expected);
        } else {
            defmt::error!("u16 driver test failed at address 0x{:x}: expected 0x{:04x}, got 0x{:04x}", 
                         addr, expected, read_value);
        }
    }
}

/// u32读写测试
fn test_u32_read_write(driver: &SdramDriver) {
    defmt::info!("Starting u32 read/write test...");
    
    let test_addr: u32 = 0x7000;
    let test_values: [u32; 4] = [0x12345678, 0x9ABCDEF0, 0x11111111, 0x22222222];
    
    // 测试u32写入和读取（直接访问）
    for (i, &value) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 4) as u32; // u32占4个字节
        unsafe {
            (0xD0000000 as *mut u32).add((addr/4) as usize).write_volatile(value);  // 修正基地址
        }
    }
    
    for (i, &expected) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 4) as u32;
        let read_value = unsafe {
            (0xD0000000 as *const u32).add((addr/4) as usize).read_volatile()  // 修正基地址
        };
        
        if read_value == expected {
            defmt::info!("u32 test passed at address 0x{:x}: 0x{:08x}", addr, expected);
        } else {
            defmt::error!("u32 test failed at address 0x{:x}: expected 0x{:08x}, got 0x{:08x}", 
                         addr, expected, read_value);
        }
    }
    
    // 使用驱动函数测试
    for (i, &value) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 4) as u32;
        driver.write_u32(value, addr);
    }
    
    for (i, &expected) in test_values.iter().enumerate() {
        let addr = test_addr + (i * 4) as u32;
        let read_value = driver.read_u32(addr);
        
        if read_value == expected {
            defmt::info!("u32 driver test passed at address 0x{:x}: 0x{:08x}", addr, expected);
        } else {
            defmt::error!("u32 driver test failed at address 0x{:x}: expected 0x{:08x}, got 0x{:08x}", 
                         addr, expected, read_value);
        }
    }
}
