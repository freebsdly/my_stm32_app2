use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::gpio::{Output, PushPull, PB0, PB1};

// 定义LED类型别名
pub type Led0 = PB1<Output<PushPull>>;
pub type Led1 = PB0<Output<PushPull>>;

// 使用Mutex和RefCell包装LED以支持安全的内部可变性和跨线程访问
pub static LED0: Mutex<RefCell<Option<Led0>>> = Mutex::new(RefCell::new(None));
pub static LED1: Mutex<RefCell<Option<Led1>>> = Mutex::new(RefCell::new(None));

/// 初始化LED，应在程序开始时调用一次
pub fn init(mut led0: Led0, mut led1: Led1) {
    // 设置初始状态
    led0.set_high(); // 默认关闭
    led1.set_high(); // 默认关闭

    cortex_m::interrupt::free(|cs| {
        LED0.borrow(cs).replace(Some(led0));
        LED1.borrow(cs).replace(Some(led1));
    });
}

/// 设置LED0的状态
pub fn set_led0(state: bool) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = LED0.borrow(cs).borrow_mut().as_mut() {
            if state {
                led.set_low(); // 低电平点亮LED
            } else {
                led.set_high(); // 高电平熄灭LED
            }
        }
    });
}

/// 设置LED1的状态
pub fn set_led1(state: bool) {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = LED1.borrow(cs).borrow_mut().as_mut() {
            if state {
                led.set_low(); // 低电平点亮LED
            } else {
                led.set_high(); // 高电平熄灭LED
            }
        }
    });
}

/// 切换LED0的状态
pub fn toggle_led0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = LED0.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
}

/// 切换LED1的状态
pub fn toggle_led1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut led) = LED1.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
}
