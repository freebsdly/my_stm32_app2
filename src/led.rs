use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::gpio::{Output, PushPull, PB0, PB1};

// 使用枚举标识不同的LED及其实例类型
#[derive(Clone, Copy)]
pub enum LedId {
    Led0,
    Led1,
}

// 使用Mutex和RefCell包装LEDs以支持安全的内部可变性和跨线程访问
static LED0: Mutex<RefCell<Option<PB1<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static LED1: Mutex<RefCell<Option<PB0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

/// 初始化LED，应在程序开始时调用一次
pub fn init(led0: PB1<Output<PushPull>>, led1: PB0<Output<PushPull>>) {
    cortex_m::interrupt::free(|cs| {
        LED0.borrow(cs).replace(Some(led0));
        LED1.borrow(cs).replace(Some(led1));
    });
}

/// 设置指定LED的状态
/// true 表示点亮，false 表示熄灭
pub fn set_led(led_id: LedId, state: bool) {
    cortex_m::interrupt::free(|cs| match led_id {
        LedId::Led0 => {
            if let Some(ref mut led) = LED0.borrow(cs).borrow_mut().as_mut() {
                if state {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }
        }
        LedId::Led1 => {
            if let Some(ref mut led) = LED1.borrow(cs).borrow_mut().as_mut() {
                if state {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }
        }
    });
}

/// 切换指定LED的状态
#[allow(unused)]
pub fn toggle_led(led_id: LedId) {
    cortex_m::interrupt::free(|cs| match led_id {
        LedId::Led0 => {
            if let Some(ref mut led) = LED0.borrow(cs).borrow_mut().as_mut() {
                led.toggle();
            }
        }
        LedId::Led1 => {
            if let Some(ref mut led) = LED1.borrow(cs).borrow_mut().as_mut() {
                led.toggle();
            }
        }
    });
}

/// 同时设置所有LED的状态
pub fn set_all_leds(states: &[bool]) {
    cortex_m::interrupt::free(|cs| {
        if states.len() > 0 {
            if let Some(ref mut led) = LED0.borrow(cs).borrow_mut().as_mut() {
                if states[0] {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }
        }

        if states.len() > 1 {
            if let Some(ref mut led) = LED1.borrow(cs).borrow_mut().as_mut() {
                if states[1] {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }
        }
    });
}
