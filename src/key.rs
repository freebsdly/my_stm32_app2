//! 按键驱动模块
//!
//! 支持以下按键:
//! - KEY0: PH3 (低电平有效)
//! - KEY1: PH2 (低电平有效)
//! - KEY2: PC13 (低电平有效)
//! - KEY_UP(WK_UP): PA0 (高电平有效)

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::gpio::{Input, PA0, PC13, PH2, PH3};

/// 按键ID枚举
#[derive(Debug, Clone, Copy)]
pub enum KeyId {
    /// KEY0 按键连接到 PH3 (低电平有效)
    Key0,
    /// KEY1 按键连接到 PH2 (低电平有效)
    Key1,
    /// KEY2 按键连接到 PC13 (低电平有效)
    Key2,
    /// KEY_UP 按键连接到 PA0 (高电平有效)
    KeyUp,
}

// 定义静态变量存储按键实例
static KEY0: Mutex<RefCell<Option<PH3<Input>>>> = Mutex::new(RefCell::new(None));
static KEY1: Mutex<RefCell<Option<PH2<Input>>>> = Mutex::new(RefCell::new(None));
static KEY2: Mutex<RefCell<Option<PC13<Input>>>> = Mutex::new(RefCell::new(None));
static KEY_UP: Mutex<RefCell<Option<PA0<Input>>>> = Mutex::new(RefCell::new(None));

/// 初始化所有按键
///
#[allow(unused)]
pub fn init(key0: PH3<Input>, key1: PH2<Input>, key2: PC13<Input>, key_up: PA0<Input>) {
    cortex_m::interrupt::free(|cs| {
        KEY0.borrow(cs).replace(Some(key0));
        KEY1.borrow(cs).replace(Some(key1));
        KEY2.borrow(cs).replace(Some(key2));
        KEY_UP.borrow(cs).replace(Some(key_up));
    });
}

/// 检查指定按键是否被按下
///
/// # 参数
/// * `key_id` - 要检查的按键ID
///
/// # 返回值
/// 如果按键被按下返回true，否则返回false
pub fn is_pressed(key_id: KeyId) -> bool {
    cortex_m::interrupt::free(|cs| {
        match key_id {
            KeyId::Key0 => {
                if let Some(ref key) = KEY0.borrow(cs).borrow().as_ref() {
                    // 按键按下时为低电平
                    key.is_low()
                } else {
                    false
                }
            }
            KeyId::Key1 => {
                if let Some(ref key) = KEY1.borrow(cs).borrow().as_ref() {
                    // 按键按下时为低电平
                    key.is_low()
                } else {
                    false
                }
            }
            KeyId::Key2 => {
                if let Some(ref key) = KEY2.borrow(cs).borrow().as_ref() {
                    // 按键按下时为低电平
                    key.is_low()
                } else {
                    false
                }
            }
            KeyId::KeyUp => {
                if let Some(ref key) = KEY_UP.borrow(cs).borrow().as_ref() {
                    // KEY_UP按键按下时为高电平
                    key.is_high()
                } else {
                    false
                }
            }
        }
    })
}

/// 等待按键释放
///
/// # 参数
/// * `key_id` - 要等待的按键ID
pub fn wait_release(key_id: KeyId) {
    while is_pressed(key_id) {
        // 等待按键释放
        cortex_m::asm::nop();
    }
}
