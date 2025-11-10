//! 按键驱动模块
//!
//! 支持以下按键:
//! - KEY0: PH3 (低电平有效)
//! - KEY1: PH2 (低电平有效)
//! - KEY2: PC13 (低电平有效)
//! - KEY_UP(WK_UP): PA0 (高电平有效)

use crate::led;
use defmt::info;
use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

// 定义静态变量存储按键实例
static KEY0: Mutex<CriticalSectionRawMutex, Option<Input>> = Mutex::new(None);
static KEY1: Mutex<CriticalSectionRawMutex, Option<Input>> = Mutex::new(None);
static KEY2: Mutex<CriticalSectionRawMutex, Option<Input>> = Mutex::new(None);
static KEY_UP: Mutex<CriticalSectionRawMutex, Option<Input>> = Mutex::new(None);
// (当前状态, 上次已确认状态)
static KEY_STATES: Mutex<CriticalSectionRawMutex, [(bool, bool); 4]> =
    Mutex::new([(false, false); 4]);
/// 初始化所有按键
pub async fn init(
    key0_pin: Peri<'static, impl Pin>,
    key1_pin: Peri<'static, impl Pin>,
    key2_pin: Peri<'static, impl Pin>,
    key_up_pin: Peri<'static, impl Pin>,
) {
    let key0 = Input::new(key0_pin, Pull::Up);
    let key1 = Input::new(key1_pin, Pull::Up);
    let key2 = Input::new(key2_pin, Pull::Up);
    let key_up = Input::new(key_up_pin, Pull::Down);

    KEY0.lock().await.replace(key0);
    KEY1.lock().await.replace(key1);
    KEY2.lock().await.replace(key2);
    KEY_UP.lock().await.replace(key_up);
    info!("Key initialized");
}

#[embassy_executor::task]
pub async fn scan_keys() {
    const DEBOUNCE_COUNT: u8 = 3;
    let mut debounce_counters = [0u8; 4];

    loop {
        let raw_states = [
            KEY0.lock().await.as_ref().unwrap().is_low(),
            KEY1.lock().await.as_ref().unwrap().is_low(),
            KEY2.lock().await.as_ref().unwrap().is_low(),
            KEY_UP.lock().await.as_ref().unwrap().is_high(),
        ];

        let mut key_states = KEY_STATES.lock().await;
        for i in 0..4 {
            if raw_states[i] == key_states[i].0 {
                // 状态持续，重置计数器
                debounce_counters[i] = 0;
            } else {
                // 状态变化，增加计数器
                debounce_counters[i] += 1;

                // 如果连续DEBOUNCE_COUNT次都不同，则更新状态
                if debounce_counters[i] >= DEBOUNCE_COUNT {
                    key_states[i].0 = raw_states[i];
                    debounce_counters[i] = 0;

                    // 只在下降沿(按下)时触发事件
                    if key_states[i].0 && !key_states[i].1 {
                        match i {
                            0 => {
                                info!("KEY0 pressed");
                                led::toggle_led(led::LedId::Led0).await;
                            }
                            1 => {
                                info!("KEY1 pressed");
                                led::toggle_led(led::LedId::Led1).await;
                            }
                            2 => info!("KEY2 pressed"),
                            3 => info!("KEY_UP pressed"),
                            _ => unreachable!(),
                        }
                    }
                    key_states[i].1 = key_states[i].0;
                }
            }
        }
        drop(key_states);

        Timer::after_millis(10).await;
    }
}
