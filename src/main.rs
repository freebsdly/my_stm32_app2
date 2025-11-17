#![no_main]
#![no_std]

mod key;
mod lcd;
mod led;
mod sdram;

// Print panic message to probe console
use cortex_m_rt::entry;
use defmt::{error, info, warn};
use defmt_rtt as _;
use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{Point, Primitive, RgbColor, Size};
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use panic_probe as _;

use crate::sdram::SdramDriver;
use stm32f4xx_hal::ltdc::{BluePins, GreenPins, Layer, LtdcPins, PixelFormat, RedPins};
use stm32f4xx_hal::pac::FMC;
use stm32f4xx_hal::rcc::Enable;
use stm32f4xx_hal::{
    gpio::alt::fmc as alt_fmc,
    gpio::alt::ltdc as alt_ltdc,
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

    // 配置GPIO端口,使能时钟
    let gpio_a = dp.GPIOA.split(&mut rcc);
    let gpio_b = dp.GPIOB.split(&mut rcc);
    let gpio_c = dp.GPIOC.split(&mut rcc);
    let gpio_d = dp.GPIOD.split(&mut rcc);
    let gpio_e = dp.GPIOE.split(&mut rcc);
    let gpio_f = dp.GPIOF.split(&mut rcc);
    let gpio_g = dp.GPIOG.split(&mut rcc);
    let gpio_h = dp.GPIOH.split(&mut rcc);
    let gpio_i = dp.GPIOI.split(&mut rcc);

    // 配置LED引脚为推挽输出模式并初始化LED模块
    let led0 = gpio_b.pb1.into_push_pull_output();
    let led1 = gpio_b.pb0.into_push_pull_output();
    led::init(led0, led1);

    // // 初始化 KEY0 (PH3) - 使用上拉电阻，按键按下时为低电平
    // let key0 = gpio_h.ph3.into_pull_up_input();
    // // 初始化 KEY1 (PH2) - 使用上拉电阻，按键按下时为低电平
    // let key1 = gpio_h.ph2.into_pull_up_input();
    //
    // // 初始化 KEY2 (PC13) - 使用上拉电阻，按键按下时为低电平
    // let key2 = gpio_c.pc13.into_pull_up_input();
    //
    // // 初始化 KEY_UP/WK_UP (PA0) - 使用下拉电阻，按键按下时为高电平
    // let key_up = gpio_a.pa0.into_pull_down_input();
    //
    // // 初始化按键
    // key::init(key0, key1, key2, key_up);

    FMC::enable(&mut rcc);

    // 2. 配置GPIO引脚为FMC功能 (AF12)
    // Address pins
    let _ = (
        alt_fmc::A0::from(gpio_f.pf0),
        alt_fmc::A1::from(gpio_f.pf1),
        alt_fmc::A2::from(gpio_f.pf2),
        alt_fmc::A3::from(gpio_f.pf3),
        alt_fmc::A4::from(gpio_f.pf4),
        alt_fmc::A5::from(gpio_f.pf5),
        alt_fmc::A6::from(gpio_f.pf12),
        alt_fmc::A7::from(gpio_f.pf13),
        alt_fmc::A8::from(gpio_f.pf14),
        alt_fmc::A9::from(gpio_f.pf15),
        alt_fmc::A10::from(gpio_g.pg0),
        alt_fmc::A11::from(gpio_g.pg1),
        alt_fmc::A12::from(gpio_g.pg2),
        // Bank address pins
        alt_fmc::Ba0::from(gpio_g.pg4),
        alt_fmc::Ba1::from(gpio_g.pg5),
        // Data pins
        alt_fmc::D0::from(gpio_d.pd14),
        alt_fmc::D1::from(gpio_d.pd15),
        alt_fmc::D2::from(gpio_d.pd0),
        alt_fmc::D3::from(gpio_d.pd1),
        alt_fmc::D4::from(gpio_e.pe7),
        alt_fmc::D5::from(gpio_e.pe8),
        alt_fmc::D6::from(gpio_e.pe9),
        alt_fmc::D7::from(gpio_e.pe10),
        alt_fmc::D8::from(gpio_e.pe11),
        alt_fmc::D9::from(gpio_e.pe12),
        alt_fmc::D10::from(gpio_e.pe13),
        alt_fmc::D11::from(gpio_e.pe14),
        alt_fmc::D12::from(gpio_e.pe15),
        alt_fmc::D13::from(gpio_d.pd8),
        alt_fmc::D14::from(gpio_d.pd9),
        alt_fmc::D15::from(gpio_d.pd10),
        // Control pins
        alt_fmc::Nbl0::from(gpio_e.pe0),
        alt_fmc::Nbl1::from(gpio_e.pe1),
        alt_fmc::Sdcke0::from(gpio_c.pc3),
        alt_fmc::Sdclk::from(gpio_g.pg8),
        alt_fmc::Sdncas::from(gpio_g.pg15),
        alt_fmc::Sdne0::from(gpio_c.pc2),
        alt_fmc::Sdnras::from(gpio_f.pf11),
        alt_fmc::Sdnwe::from(gpio_c.pc0),
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

    // // Setup LTDC pins for RGB LCD - using correct pins for Apollo STM32F429 board
    // let _ = (
    //     alt_ltdc::R2::from(gpio_h.ph8),
    //     alt_ltdc::R3::from(gpio_h.ph9),
    //     alt_ltdc::R4::from(gpio_h.ph10),
    //     alt_ltdc::R5::from(gpio_h.ph11),
    //     alt_ltdc::R6::from(gpio_h.ph12),
    //     alt_ltdc::R7::from(gpio_g.pg6),
    //     alt_ltdc::G0::from(gpio_e.pe5),
    //     alt_ltdc::G1::from(gpio_e.pe6),
    //     alt_ltdc::G2::from(gpio_h.ph13),
    //     alt_ltdc::G3::from(gpio_h.ph14),
    //     alt_ltdc::G4::from(gpio_h.ph15),
    //     alt_ltdc::G5::from(gpio_i.pi0),
    //     alt_ltdc::G6::from(gpio_i.pi1),
    //     alt_ltdc::G7::from(gpio_i.pi2),
    //     alt_ltdc::B0::from(gpio_e.pe4),
    //     alt_ltdc::B1::from(gpio_g.pg12),
    //     alt_ltdc::B2::from(gpio_g.pg10),
    //     alt_ltdc::B3::from(gpio_g.pg11),
    //     alt_ltdc::B4::from(gpio_i.pi4),
    //     alt_ltdc::B5::from(gpio_i.pi5),
    //     alt_ltdc::B6::from(gpio_i.pi6),
    //     alt_ltdc::B7::from(gpio_i.pi7),
    //     alt_ltdc::Vsync::from(gpio_i.pi9),
    //     alt_ltdc::Hsync::from(gpio_i.pi10),
    //     alt_ltdc::De::from(gpio_f.pf10),
    //     alt_ltdc::Clk::from(gpio_g.pg7),
    // );

    // // LCD backlight enable - 使用PC9作为背光控制引脚
    // let mut backlight = gpio_b.pb5.into_push_pull_output();
    // backlight.set_high();
    //
    // // Setup LTDC pins for RGB LCD - using correct pins for Apollo STM32F429 board
    // let pins = LtdcPins::new(
    //     // Red pins - 根据正点原子阿波罗板子的引脚分配
    //     RedPins::new(
    //         gpio_h.ph2.into_alternate(),  // R0 - PH2 (KEY1)
    //         gpio_h.ph3.into_alternate(),  // R1 - PH3 (KEY0)
    //         gpio_h.ph8.into_alternate(),  // R2 - PH8 (DCMI_HREF)
    //         gpio_h.ph9.into_alternate(),  // R3 - PH9 (LCD_R3)
    //         gpio_h.ph10.into_alternate(), // R4 - PH10 (LCD_R4)
    //         gpio_h.ph11.into_alternate(), // R5 - PH11 (LCD_R5)
    //         gpio_h.ph12.into_alternate(), // R6 - PH12 (LCD_R6)
    //         gpio_g.pg6.into_alternate(),  // R7 - PG6 (LCD_R7)
    //     ),
    //     // Green pins
    //     GreenPins::new(
    //         gpio_e.pe5.into_alternate(),  // G0 - PE5 (SAI1_SCKA)
    //         gpio_e.pe6.into_alternate(),  // G1 - PE6 (SAI1_SDA)
    //         gpio_h.ph13.into_alternate(), // G2 - PH13 (LCD_G2)
    //         gpio_h.ph14.into_alternate(), // G3 - PH14 (LCD_G3)
    //         gpio_h.ph15.into_alternate(), // G4 - PH15 (LCD_G4)
    //         gpio_i.pi0.into_alternate(),  // G5 - PI0 (LCD_G5)
    //         gpio_i.pi1.into_alternate(),  // G6 - PI1 (LCD_G6)
    //         gpio_i.pi2.into_alternate(),  // G7 - PI2 (LCD_G7)
    //     ),
    //     // Blue pins
    //     BluePins::new(
    //         gpio_e.pe4.into_alternate(),  // B0 - PE4 (SAI1_FSA)
    //         gpio_g.pg12.into_alternate(), // B1 - PG12 (NRF_CE)
    //         gpio_g.pg10.into_alternate(), // B2 - PG10 (NRF_CS)
    //         gpio_g.pg11.into_alternate(), // B3 - PG11 (LCD_B3)
    //         gpio_i.pi4.into_alternate(),  // B4 - PI4 (LCD_B4)
    //         gpio_i.pi5.into_alternate(),  // B5 - PI5 (LCD_B5)
    //         gpio_i.pi6.into_alternate(),  // B6 - PI6 (LCD_B6)
    //         gpio_i.pi7.into_alternate(),  // B7 - PI7 (LCD_B7)
    //     ),
    //     // Control pins
    //     gpio_i.pi10.into_alternate(), // HSYNC - PI10 (LCD_HSYNC)
    //     gpio_i.pi9.into_alternate(),  // VSYNC - PI9 (LCD_VSYNC)
    //     gpio_f.pf10.into_alternate(), // DE (Data Enable) - PF10 (LCD_DE)
    //     gpio_g.pg7.into_alternate(),  // CLK - PG7 (LCD_CLK)
    // );
    //
    // let frame_buffer =
    //     unsafe { core::slice::from_raw_parts_mut((sdram::SDRAM_BASE_ADDR) as *mut u16, 800 * 480) };
    //
    // // Create and initialize the LCD
    // let mut display = lcd::LcdDisplay::new(dp.LTDC, dp.DMA2D, pins);
    // display
    //     .controller
    //     .config_layer(Layer::L1, frame_buffer, PixelFormat::RGB565);
    //
    // display.controller.enable_layer(Layer::L1);
    // display.controller.reload();
    //
    // info!("LCD初始化完成");
    //
    // // Draw a background
    // Rectangle::new(Point::new(0, 0), Size::new(800, 480))
    //     .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 0, 0))) // Black background
    //     .draw(&mut display)
    //     .unwrap();
    //
    // // Draw some example graphics
    // // Red rectangle
    // Rectangle::new(Point::new(50, 50), Size::new(200, 150))
    //     .into_styled(PrimitiveStyle::with_fill(Rgb565::new(31, 0, 0)))
    //     .draw(&mut display)
    //     .unwrap();
    //
    // // Green circle
    // Circle::new(Point::new(400, 200), 80)
    //     .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 63, 0)))
    //     .draw(&mut display)
    //     .unwrap();
    //
    // // Blue triangle
    // Triangle::new(
    //     Point::new(600, 100),
    //     Point::new(700, 300),
    //     Point::new(500, 300),
    // )
    // .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 0, 31)))
    // .draw(&mut display)
    // .unwrap();
    //
    // // Hello text
    // let text_style = MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE);
    // Text::new("STM32F429 LTDC LCD Demo", Point::new(300, 400), text_style)
    //     .draw(&mut display)
    //     .unwrap();
    //
    // Text::new("Hello Rust!", Point::new(350, 440), text_style)
    //     .draw(&mut display)
    //     .unwrap();

    // 运行综合测试
    // 实际能用的内存大小为16MB，bank0和bank1
    // run_check_board_test(17 * 1024 * 1024usize);

    test_bank_access();

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
        error!("边界数据错误");
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
            error!("不一致在偏移:0x{:x} 预期:{:08x} 读取:{:08x}", i, w, r);
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
            error!("全{:02x}图案测试失败", pattern);
            return false;
        }
    }

    info!("SDRAM通过所有测试");
    true
}

fn run_check_board_test(size: usize) {
    // 添加基地址检查
    info!("SDRAM 基地址: 0x{:08X}", sdram::SDRAM_BASE_ADDR);

    let buffer = unsafe {
        core::slice::from_raw_parts_mut(
            sdram::SDRAM_BASE_ADDR as *mut u32,
            size / 4, // u32占用4字节
        )
    };

    // 阶段1：写入棋盘格模式
    for (i, cell) in buffer.iter_mut().enumerate() {
        let pattern = if i % 2 == 0 {
            0xAAAAAAAA // 10101010...
        } else {
            0x55555555 // 01010101...
        };
        *cell = pattern;
    }

    // 阶段2：验证读取
    let mut error_count = 0;
    let mut last_error_addr = 0;

    for (addr, cell) in buffer.iter().enumerate() {
        let actual = *cell;
        let expected = if addr % 2 == 0 {
            0xAAAAAAAA
        } else {
            0x55555555
        };

        if actual != expected {
            error_count += 1;
            last_error_addr = sdram::SDRAM_BASE_ADDR as usize + addr * 4;
            // 立即打印第一个错误（串口输出）
            error!(
                "错误 @ 0x{:08X}: 预期={:08X} 实际={:08X}",
                last_error_addr, expected, actual
            );
            // 只显示前几个错误以避免日志过多
            if error_count > 10 {
                break;
            }
        }

        // 进度指示
        if addr % 1_000_000 == 0 {
            info!("进度: {:01}%", addr as f32 / buffer.len() as f32 * 100.0);
        }
    }

    // 测试结果汇总
    info!("======== 测试结果 ========");
    info!("测试容量:   {} MB", size / (1024 * 1024));
    info!("测试单元数: {}", buffer.len());
    info!("错误数量:   {}", error_count);

    if error_count == 0 {
        info!("✅ SDRAM 验证通过!");
    } else {
        warn!("❌ 发现错误!");
        warn!("最后错误地址: 0x{:08X}", last_error_addr);
    }
}

fn test_bank_access() {
    // 每个 bank 8MB（32MB / 4）
    let offsets = [
        0 * 8 * 1024 * 1024, // Bank 0
        1 * 8 * 1024 * 1024, // Bank 1
        2 * 8 * 1024 * 1024, // Bank 2
        3 * 8 * 1024 * 1024, // Bank 3
    ];

    for (i, &off) in offsets.iter().enumerate() {
        unsafe {
            let ptr = (sdram::SDRAM_BASE_ADDR + off) as *mut u32;
            ptr.write_volatile(0x1234_5678 + i as u32);
            cortex_m::asm::delay(1000);
            let val = ptr.read_volatile();
            info!(
                "Bank {}: wrote 0x{:08X}, read 0x{:08X}",
                i,
                0x1234_5678 + i as u32,
                val
            );
        }
    }
}
