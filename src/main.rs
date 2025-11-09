#![deny(unsafe_code)]
#![no_main]
#![no_std]

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
    let cp = cortex_m::Peripherals::take().unwrap();

    // 2. 配置系统时钟（关键：延时精度依赖时钟频率）
    let mut rcc = dp.RCC.constrain();

    // 3. 初始化 SysTick 延时（传入系统时钟频率）
    let mut delay = dp.TIM2.delay_ms(&mut rcc);

    let gpio_g = dp.GPIOG.split(&mut rcc);
    let mut led = gpio_g.pg13.into_push_pull_output();

    loop {
        led.set_high();
        info!("LED ON");
        delay.delay(1000.millis());
    }
}
