#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod key;
mod led;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let dp = embassy_stm32::init(Default::default());
    info!("embassy initialized");

    led::init(dp.PB1, dp.PB0).await;
    key::init(dp.PH3, dp.PH2, dp.PC13, dp.PA0).await;

    spawner.spawn(key::scan_keys()).ok();
}
