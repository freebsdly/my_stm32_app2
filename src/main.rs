#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod key;
mod led;

// Print panic message to probe console
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::hal::delay::DelayNs;
use stm32f4xx_hal::pac;
use stm32f4xx_hal::rcc::{Config, RccExt};
use stm32f4xx_hal::time::{Hertz};
use stm32f4xx_hal::timer::TimerExt;

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    // 1. 获取芯片外设实例
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let clock_cfg = Config::default()
        .sysclk(Hertz::MHz(48u32));

    // 2. 配置系统时钟（关键：延时精度依赖时钟频率）
    let mut rcc = dp.RCC.constrain().freeze(clock_cfg);

    // 3. 初始化 SysTick 延时（传入系统时钟频率）
    let mut delay = dp.TIM2.delay_ms(&mut rcc);

    // 配置GPIO端口
    let gpio_b = dp.GPIOB.split(&mut rcc);
    let gpio_h = dp.GPIOH.split(&mut rcc);
    let gpio_c = dp.GPIOC.split(&mut rcc);
    let gpio_a = dp.GPIOA.split(&mut rcc);

    // 配置LED引脚为推挽输出模式并初始化LED模块
    let led0 = gpio_b.pb1.into_push_pull_output();
    let led1 = gpio_b.pb0.into_push_pull_output();
    led::init(led0, led1);

    // 初始化按键
    key::init(gpio_h, gpio_c, gpio_a);

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
