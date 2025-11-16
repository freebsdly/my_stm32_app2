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
        .pclk1(48.MHz()) // APB1低速外设时钟 = HCLK/4
        .pclk2(96.MHz()); // APB2高速外设时钟 = HCLK/2

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

    // 2. 配置GPIO引脚为FMC功能 (AF12)
    // 初始化SDRAM和配置FMC引脚
    let _fmc_pins = (
        // Address pins
        alt_fmc::A0::from(gpio_f.pf0.into_alternate::<12>()),
        alt_fmc::A1::from(gpio_f.pf1.into_alternate::<12>()),
        alt_fmc::A2::from(gpio_f.pf2.into_alternate::<12>()),
        alt_fmc::A3::from(gpio_f.pf3.into_alternate::<12>()),
        alt_fmc::A4::from(gpio_f.pf4.into_alternate::<12>()),
        alt_fmc::A5::from(gpio_f.pf5.into_alternate::<12>()),
        alt_fmc::A6::from(gpio_f.pf12.into_alternate::<12>()),
        alt_fmc::A7::from(gpio_f.pf13.into_alternate::<12>()),
        alt_fmc::A8::from(gpio_f.pf14.into_alternate::<12>()),
        alt_fmc::A9::from(gpio_f.pf15.into_alternate::<12>()),
        alt_fmc::A10::from(gpio_g.pg0.into_alternate::<12>()),
        alt_fmc::A11::from(gpio_g.pg1.into_alternate::<12>()),
        alt_fmc::A12::from(gpio_g.pg2.into_alternate::<12>()),
        // Bank address pins
        alt_fmc::Ba0::from(gpio_g.pg4.into_alternate::<12>()),
        alt_fmc::Ba1::from(gpio_g.pg5.into_alternate::<12>()),
        // Data pins
        alt_fmc::D0::from(gpio_d.pd14.into_alternate::<12>()),
        alt_fmc::D1::from(gpio_d.pd15.into_alternate::<12>()),
        alt_fmc::D2::from(gpio_d.pd0.into_alternate::<12>()),
        alt_fmc::D3::from(gpio_d.pd1.into_alternate::<12>()),
        alt_fmc::D4::from(gpio_e.pe7.into_alternate::<12>()),
        alt_fmc::D5::from(gpio_e.pe8.into_alternate::<12>()),
        alt_fmc::D6::from(gpio_e.pe9.into_alternate::<12>()),
        alt_fmc::D7::from(gpio_e.pe10.into_alternate::<12>()),
        alt_fmc::D8::from(gpio_e.pe11.into_alternate::<12>()),
        alt_fmc::D9::from(gpio_e.pe12.into_alternate::<12>()),
        alt_fmc::D10::from(gpio_e.pe13.into_alternate::<12>()),
        alt_fmc::D11::from(gpio_e.pe14.into_alternate::<12>()),
        alt_fmc::D12::from(gpio_e.pe15.into_alternate::<12>()),
        alt_fmc::D13::from(gpio_d.pd8.into_alternate::<12>()),
        alt_fmc::D14::from(gpio_d.pd9.into_alternate::<12>()),
        alt_fmc::D15::from(gpio_d.pd10.into_alternate::<12>()),
        // Control pins
        alt_fmc::Nbl0::from(gpio_e.pe0.into_alternate::<12>()),
        alt_fmc::Nbl1::from(gpio_e.pe1.into_alternate::<12>()),
        alt_fmc::Sdcke0::from(gpio_h.ph2.into_alternate::<12>()),
        alt_fmc::Sdclk::from(gpio_g.pg8.into_alternate::<12>()),
        alt_fmc::Sdncas::from(gpio_g.pg15.into_alternate::<12>()),
        alt_fmc::Sdne0::from(gpio_h.ph3.into_alternate::<12>()),
        alt_fmc::Sdnras::from(gpio_f.pf11.into_alternate::<12>()),
        alt_fmc::Sdnwe::from(gpio_c.pc0.into_alternate::<12>()),
    );

    // 创建SDRAM驱动实例
    let mut sdram_driver = SdramDriver::new(dp.FMC);

    // 显示初始化前状态
    sdram_driver.debug_config();

    info!("开始SDRAM初始化");
    // 初始化SDRAM
    match sdram_driver.init(&mut rcc) {
        Ok(()) => info!("SDRAM初始化成功"),
        Err(e) => panic!("SDRAM初始化失败: {}", e),
    }

    // 显示初始化后状态
    sdram_driver.debug_config();

    // 执行基础测试
    // 增加延时确保SDRAM完全稳定
    delay.delay_ms(100); // 增加延时从10ms到100ms确保SDRAM完全稳定

    // 运行综合测试
    test_sdram_full(&mut sdram_driver);

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

/// 更全面的SDRAM测试函数
pub fn test_sdram_full(driver: &SdramDriver) -> bool {
    // 0. 地址必须对齐（关键！！！）
    const ALIGNMENT: u32 = 2; // 16位总线要求2字节对齐
    let test_addr: u32 = 0x1000;
    assert!(test_addr % ALIGNMENT == 0, "测试地址必须是偶数地址");

    // 1. 使用独特数据模式检测覆盖问题
    let test_size: usize = 512; // ≥cache line size
    let mut write_data = [0u8; 512];
    let mut read_data = [0u8; 512];

    // 生成校验序列 (AA 55 AA 55...)
    for i in 0..test_size {
        write_data[i] = if i % 2 == 0 { 0xAA } else { 0x55 };
    }

    // 2. 分段写入（测试自动预充电）
    for chunk in write_data.chunks(64) {
        driver.write_buffer(chunk, test_addr);

        // 精确等待TRP+TRCD（约70ns@96MHz）
        driver.wait_busy(7); // 实现如下方代码块
    }

    // 3. 完整读取验证
    driver.read_buffer(&mut read_data, test_addr, test_size);

    // 4. 边界检查（检测缓冲区溢出）
    if read_data[0] != 0xAA || read_data[test_size - 1] != 0x55 {
        defmt::error!("边界数据错误");
        return false;
    }

    // 5. 完整内容校验（使用SIMD加速）
    for i in (0..test_size).step_by(4) {
        let w = u32::from_ne_bytes([
            write_data[i],
            write_data[i + 1],
            write_data[i + 2],
            write_data[i + 3],
        ]);

        let r = u32::from_ne_bytes([
            read_data[i],
            read_data[i + 1],
            read_data[i + 2],
            read_data[i + 3],
        ]);

        if w != r {
            defmt::error!("不一致在偏移:0x{:x} 预期:{:08x} 读取:{:08x}", i, w, r);
            return false;
        }
    }

    // 6. 压力测试（可选）
    for pattern in &[0x00, 0xFF, 0x55, 0xAA] {
        let fill: [u8; 512] = [*pattern; 512];
        driver.write_buffer(&fill, test_addr);
        driver.wait_busy(10);

        let mut buf = [0u8; 512];
        driver.read_buffer(&mut buf, test_addr, 512);

        if buf.iter().any(|b| *b != *pattern) {
            defmt::error!("全{:02x}图案测试失败", pattern);
            return false;
        }
    }

    defmt::info!("SDRAM通过所有测试");
    true
}