#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod led;

// Print panic message to probe console
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal::{pac, prelude::*};

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    // 1. 获取芯片外设实例
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    // 2. 配置系统时钟（关键：延时精度依赖时钟频率）
    let mut rcc = dp.RCC.constrain();

    // 3. 初始化 SysTick 延时（传入系统时钟频率）
    let mut delay = dp.TIM2.delay_ms(&mut rcc);

    // 配置GPIO端口B
    let gpio_b = dp.GPIOB.split(&mut rcc);

    // 配置LED引脚为推挽输出模式并初始化LED模块
    let led0 = gpio_b.pb1.into_push_pull_output();
    let led1 = gpio_b.pb0.into_push_pull_output();
    led::init(led0, led1);

    loop {
        // 点亮LED0，熄灭LED1
        led::set_led(led::LedId::Led0, true);
        led::set_led(led::LedId::Led1, false);
        delay.delay(500.millis());
        info!("LED switch");
        
        // 熄灭LED0，点亮LED1
        led::set_led(led::LedId::Led0, false);
        led::set_led(led::LedId::Led1, true);
        delay.delay(500.millis());
    }
}