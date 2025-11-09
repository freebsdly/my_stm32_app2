#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod key;
mod led;

// Print panic message to probe console
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use panic_probe as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let dp = embassy_stm32::init(Default::default());

    let led0 = Output::new(dp.PB1, Level::High, Speed::Low);
    let led1 = Output::new(dp.PB0, Level::High, Speed::Low);

    spawner.spawn(blink(led0, led1)).ok();
}

#[embassy_executor::task]
async fn blink(mut led0: Output<'static>, mut led1: Output<'static>) {
    loop {
        Timer::after(Duration::from_millis(500)).await;
        info!("Blinking");
        led0.set_high();
        led1.set_low();
        Timer::after(Duration::from_millis(500)).await;
        led0.set_low();
        led1.set_high();
    }
}
