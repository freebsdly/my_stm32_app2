#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod key;
mod lcd;
mod led;
mod sdram;

use core::{mem, slice};
// Print panic message to probe console
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle, Triangle},
    text::Text,
};

use static_cell::ConstStaticCell;
use stm32f4xx_hal::{
    fmc::FmcExt,
    gpio::alt::fmc as alt_fmc,
    gpio::GpioExt,
    hal::delay::DelayNs,
    ltdc::{BluePins, GreenPins, Layer, LtdcPins, PixelFormat, RedPins},
    pac,
    prelude::*,
    rcc::{Config, RccExt},
    time::Hertz,
    timer::TimerExt,
};

// Framebuffer size for 800x480 display in RGB565 format (16 bits per pixel)
// Total size: 800 * 480 * 2 = 768,000 bytes
const FRAMEBUFFER_SIZE: usize = 800 * 480 * 2;

// SDRAM base address for STM32F429
const SDRAM_BASE: usize = 0xC0000000;

// We'll place the framebuffer in SDRAM instead of static memory
// Static allocation removed since we'll use SDRAM

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    // 1. 获取芯片外设实例
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure system clock
    let clock_cfg = Config::hse(8.MHz())
        .sysclk(Hertz::MHz(168u32))
        .hclk(Hertz::MHz(168u32));

    // 2. 配置系统时钟（关键：延时精度依赖时钟频率）
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(clock_cfg);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    // 3. 初始化 SysTick 延时（传入系统时钟频率）
    // let mut delay = dp.TIM2.delay_ms(&mut rcc);

    // 配置GPIO端口
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

    // 初始化按键
    // key::init(key0, key1, key2, key_up);

    // LCD backlight enable - 使用PC9作为背光控制引脚
    let mut backlight = gpio_c.pc9.into_push_pull_output();
    backlight.set_high();

    // 初始化SDRAM和配置FMC引脚
    let fmc_pins = (
        alt_fmc::A0::from(gpio_f.pf0.internal_pull_up(true)),
        alt_fmc::A1::from(gpio_f.pf1.internal_pull_up(true)),
        alt_fmc::A2::from(gpio_f.pf2.internal_pull_up(true)),
        alt_fmc::A3::from(gpio_f.pf3.internal_pull_up(true)),
        alt_fmc::A4::from(gpio_f.pf4.internal_pull_up(true)),
        alt_fmc::A5::from(gpio_f.pf5.internal_pull_up(true)),
        alt_fmc::A6::from(gpio_f.pf12.internal_pull_up(true)),
        alt_fmc::A7::from(gpio_f.pf13.internal_pull_up(true)),
        alt_fmc::A8::from(gpio_f.pf14.internal_pull_up(true)),
        alt_fmc::A9::from(gpio_f.pf15.internal_pull_up(true)),
        alt_fmc::A10::from(gpio_g.pg0.internal_pull_up(true)),
        alt_fmc::A11::from(gpio_g.pg1.internal_pull_up(true)),
        alt_fmc::A12::from(gpio_g.pg2.internal_pull_up(true)),
        alt_fmc::Ba0::from(gpio_g.pg4.internal_pull_up(true)),
        alt_fmc::Ba1::from(gpio_g.pg5.internal_pull_up(true)),
        alt_fmc::D0::from(gpio_d.pd14.internal_pull_up(true)),
        alt_fmc::D1::from(gpio_d.pd15.internal_pull_up(true)),
        alt_fmc::D2::from(gpio_d.pd0.internal_pull_up(true)),
        alt_fmc::D3::from(gpio_d.pd1.internal_pull_up(true)),
        alt_fmc::D4::from(gpio_e.pe7.internal_pull_up(true)),
        alt_fmc::D5::from(gpio_e.pe8.internal_pull_up(true)),
        alt_fmc::D6::from(gpio_e.pe9.internal_pull_up(true)),
        alt_fmc::D7::from(gpio_e.pe10.internal_pull_up(true)),
        alt_fmc::D8::from(gpio_e.pe11.internal_pull_up(true)),
        alt_fmc::D9::from(gpio_e.pe12.internal_pull_up(true)),
        alt_fmc::D10::from(gpio_e.pe13.internal_pull_up(true)),
        alt_fmc::D11::from(gpio_e.pe14.internal_pull_up(true)),
        alt_fmc::D12::from(gpio_e.pe15.internal_pull_up(true)),
        alt_fmc::D13::from(gpio_d.pd8.internal_pull_up(true)),
        alt_fmc::D14::from(gpio_d.pd9.internal_pull_up(true)),
        alt_fmc::D15::from(gpio_d.pd10.internal_pull_up(true)),
        // FMC和LTDC共享的引脚，统一配置为AF14(LTDC)
        alt_fmc::D16::from(gpio_h.ph8.into_alternate::<14>()),
        alt_fmc::D17::from(gpio_h.ph9.into_alternate::<14>()),
        alt_fmc::D18::from(gpio_h.ph10.into_alternate::<14>()),
        alt_fmc::D19::from(gpio_h.ph11.into_alternate::<14>()),
        alt_fmc::D20::from(gpio_h.ph12.into_alternate::<14>()),
        alt_fmc::D21::from(gpio_h.ph13.into_alternate::<14>()),
        alt_fmc::D22::from(gpio_h.ph14.into_alternate::<14>()),
        alt_fmc::D23::from(gpio_h.ph15.into_alternate::<14>()),
        alt_fmc::D24::from(gpio_i.pi0.into_alternate::<14>()),
        alt_fmc::D25::from(gpio_i.pi1.into_alternate::<14>()),
        alt_fmc::D26::from(gpio_i.pi2.into_alternate::<14>()),
        // PI3 不用于FMC或LTDC，保留为普通GPIO
        alt_fmc::D27::from(gpio_i.pi3.internal_pull_up(true)),
        alt_fmc::D28::from(gpio_i.pi6.internal_pull_up(true)),
        alt_fmc::D29::from(gpio_i.pi7.internal_pull_up(true)),
        alt_fmc::D30::from(gpio_i.pi9.into_alternate::<14>()),
        alt_fmc::D31::from(gpio_i.pi10.into_alternate::<14>()),
        alt_fmc::Nbl0::from(gpio_e.pe0.internal_pull_up(true)),
        alt_fmc::Nbl1::from(gpio_e.pe1.internal_pull_up(true)),
        // NBL2和NBL3引脚也同时用于LTDC，统一配置为AF14(LTDC)
        alt_fmc::Nbl2::from(gpio_i.pi4.into_alternate::<14>()),
        alt_fmc::Nbl3::from(gpio_i.pi5.into_alternate::<14>()),
        alt_fmc::Sdcke0::from(gpio_h.ph2.internal_pull_up(true)),
        alt_fmc::Sdclk::from(gpio_g.pg8.internal_pull_up(true)),
        alt_fmc::Sdncas::from(gpio_g.pg15.internal_pull_up(true)),
        alt_fmc::Sdne0::from(gpio_h.ph3.internal_pull_up(true)),
        alt_fmc::Sdnras::from(gpio_f.pf11.internal_pull_up(true)),
        alt_fmc::Sdnwe::from(gpio_c.pc0.internal_pull_up(true)),
    );
    let chip = sdram::W9825G6KH;
    let mut sdram = dp.FMC.sdram(fmc_pins, chip, &rcc.clocks);

    // 现在可以使用SDRAM了
    // 将帧缓冲区放在SDRAM中
    let _ram_ptr = sdram.init(&mut delay);
    // 使用SDRAM的起始地址作为帧缓冲区
    let frame_buffer_ptr = SDRAM_BASE as *mut u16;
    // 计算帧缓冲区大小 (800 * 480 * 2 字节)
    let frame_buffer_len = 800 * 480;
    // 安全地从原始指针创建切片
    let frame_buffer =
        unsafe { core::slice::from_raw_parts_mut(frame_buffer_ptr, frame_buffer_len) };

    // Setup LTDC pins for RGB LCD - using correct pins for Apollo STM32F429 board
    let pins = LtdcPins::new(
        // Red pins - 根据正点原子阿波罗板子的引脚分配
        RedPins::new(
            gpio_h.ph8.into_alternate::<14>(),  // R2 - PH8 (DCMI_HREF)
            gpio_h.ph9.into_alternate::<14>(),  // R3 - PH9 (LCD_R3)
            gpio_h.ph10.into_alternate::<14>(), // R4 - PH10 (LCD_R4)
            gpio_h.ph11.into_alternate::<14>(), // R5 - PH11 (LCD_R5)
            gpio_h.ph12.into_alternate::<14>(), // R6 - PH12 (LCD_R6)
            gpio_g.pg6.into_alternate::<14>(),  // R7 - PG6 (LCD_R7)
        ),
        // Green pins
        GreenPins::new(
            gpio_h.ph13.into_alternate::<14>(), // G2 - PH13 (LCD_G2)
            gpio_h.ph14.into_alternate::<14>(), // G3 - PH14 (LCD_G3)
            gpio_h.ph15.into_alternate::<14>(), // G4 - PH15 (LCD_G4)
            gpio_i.pi0.into_alternate::<14>(),  // G5 - PI0 (LCD_G5)
            gpio_i.pi1.into_alternate::<14>(),  // G6 - PI1 (LCD_G6)
            gpio_i.pi2.into_alternate::<14>(),  // G7 - PI2 (LCD_G7)
        ),
        // Blue pins
        BluePins::new(
            gpio_g.pg10.into_alternate::<14>(), // B2 - PG10 (NRF_CS)
            gpio_g.pg11.into_alternate::<14>(), // B3 - PG11 (LCD_B3)
            gpio_i.pi4.into_alternate::<14>(),  // B4 - PI4 (LCD_B4)
            gpio_i.pi5.into_alternate::<14>(),  // B5 - PI5 (LCD_B5)
            gpio_i.pi6.into_alternate::<14>(),  // B6 - PI6 (LCD_B6)
            gpio_i.pi7.into_alternate::<14>(),  // B7 - PI7 (LCD_B7)
        ),
        // Control pins
        gpio_i.pi10.into_alternate::<14>(), // HSYNC - PI10 (LCD_HSYNC)
        gpio_i.pi9.into_alternate::<14>(),  // VSYNC - PI9 (LCD_VSYNC)
        gpio_f.pf10.into_alternate::<14>(), // DE (Data Enable) - PF10 (LCD_DE)
        gpio_g.pg7.into_alternate::<14>(),  // CLK - PG7 (LCD_CLK)
    );

    // Create and initialize the LCD display
    let mut display = lcd::LcdDisplay::new(dp.LTDC, dp.DMA2D, pins);
    display
        .controller
        .config_layer(Layer::L1, frame_buffer, PixelFormat::RGB565);

    display.controller.enable_layer(Layer::L1);
    display.controller.reload();

    // Draw a background
    Rectangle::new(Point::new(0, 0), Size::new(800, 480))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 0, 0))) // Black background
        .draw(&mut display)
        .unwrap();

    // Draw some example graphics
    // Red rectangle
    Rectangle::new(Point::new(50, 50), Size::new(200, 150))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::new(31, 0, 0)))
        .draw(&mut display)
        .unwrap();

    // Green circle
    Circle::new(Point::new(400, 200), 80)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 63, 0)))
        .draw(&mut display)
        .unwrap();

    // Blue triangle
    Triangle::new(
        Point::new(600, 100),
        Point::new(700, 300),
        Point::new(500, 300),
    )
    .into_styled(PrimitiveStyle::with_fill(Rgb565::new(0, 0, 31)))
    .draw(&mut display)
    .unwrap();

    // Hello text
    let text_style = MonoTextStyle::new(&FONT_6X9, Rgb565::WHITE);
    Text::new("STM32F429 LTDC LCD Demo", Point::new(300, 400), text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("Hello Rust!", Point::new(350, 440), text_style)
        .draw(&mut display)
        .unwrap();

    loop {
        // 检查按键状态并控制LED
        if key::is_pressed(key::KeyId::Key0) {
            led::set_led(led::LedId::Led0, true);
            led::set_led(led::LedId::Led1, false);
            info!("KEY0 pressed - LED0 on");
            key::wait_release(key::KeyId::Key0);
        } else if key::is_pressed(key::KeyId::Key1) {
            led::set_led(led::LedId::Led0, false);
            led::set_led(led::LedId::Led1, true);
            info!("KEY1 pressed - LED1 on");
            key::wait_release(key::KeyId::Key1);
        } else if key::is_pressed(key::KeyId::Key2) {
            led::set_all_leds(&[true, true]);
            info!("KEY2 pressed - Both LEDs on");
            key::wait_release(key::KeyId::Key2);
        } else if key::is_pressed(key::KeyId::KeyUp) {
            led::set_all_leds(&[false, false]);
            info!("KEY_UP pressed - Both LEDs off");
            key::wait_release(key::KeyId::KeyUp);
        }

        delay.delay_ms(10)
    }
}
