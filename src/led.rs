use defmt::info;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub static LED0: Mutex<CriticalSectionRawMutex, Option<Output>> = Mutex::new(None);
pub static LED1: Mutex<CriticalSectionRawMutex, Option<Output>> = Mutex::new(None);

// 使用枚举标识不同的LED及其实例类型
#[derive(Clone, Copy)]
pub enum LedId {
    Led0,
    Led1,
}

pub async fn init(led0_pin: Peri<'static, impl Pin>, led1_pin: Peri<'static, impl Pin>) {
    let led0 = Output::new(led0_pin, Level::High, Speed::Low);
    let led1 = Output::new(led1_pin, Level::High, Speed::Low);

    LED0.lock().await.replace(led0);
    LED1.lock().await.replace(led1);

    info!("LED initialized")
}

/// 切换指定LED的状态
pub async fn toggle_led(led_id: LedId) {
    match led_id {
        LedId::Led0 => {
            LED0.lock().await.as_mut().map(|led| led.toggle());
        }
        LedId::Led1 => {
            LED1.lock().await.as_mut().map(|led| led.toggle());
        }
    }
}

/// 设置指定LED的状态
/// true 表示点亮，false 表示熄灭
pub async fn set_led(led_id: LedId, state: bool) {
    let setter = |led: &mut Output| {
        if state {
            led.set_high();
        } else {
            led.set_low();
        }
    };
    match led_id {
        LedId::Led0 => {
            LED0.lock().await.as_mut().map(setter);
        }
        LedId::Led1 => {
            LED1.lock().await.as_mut().map(setter);
        }
    }
}
