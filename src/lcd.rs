//! LCD display implementation using LTDC peripheral for STM32F429
//!
//! This module provides support for driving an RGB LCD screen using the LTDC (LCD-TFT Controller)
//! peripheral available on the STM32F429 microcontroller.

use embedded_graphics::{
    pixelcolor::{Rgb565, RgbColor},
    prelude::{DrawTarget, OriginDimensions, Pixel, Size},
};

use stm32f4xx_hal::{
    ltdc::{DisplayConfig, DisplayController, Layer, LtdcPins, PixelFormat, SupportedWord},
    pac::{DMA2D, LTDC},
    prelude::*,
};

/// Display configuration for a 800x480 RGB LCD screen
pub const LCD_CONFIG: DisplayConfig = DisplayConfig {
    active_width: 800,
    active_height: 480,
    h_back_porch: 88,
    h_front_porch: 40,
    h_sync: 128,
    v_back_porch: 32,
    v_front_porch: 10,
    v_sync: 23,
    frame_rate: 60,
    h_sync_pol: false,
    v_sync_pol: false,
    no_data_enable_pol: false,
    pixel_clock_pol: false,
};

/// LCD display driver using LTDC peripheral
pub struct LcdDisplay<T: 'static + SupportedWord> {
    pub controller: DisplayController<T>,
}

impl<T: 'static + SupportedWord> LcdDisplay<T> {
    /// Create a new LCD display instance
    ///
    /// # Arguments
    /// * `ltdc` - LTDC peripheral
    /// * `dma2d` - DMA2D peripheral
    /// * `pins` - LTDC pin configuration
    pub fn new(ltdc: LTDC, dma2d: DMA2D, pins: LtdcPins) -> LcdDisplay<T> {
        let controller = DisplayController::new(
            ltdc,
            dma2d,
            Some(pins),
            PixelFormat::RGB565,
            LCD_CONFIG,
            Some(30.MHz()),
        );

        LcdDisplay { controller }
    }
}

impl DrawTarget for LcdDisplay<u16> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are within bounds
            // Discard any out of bounds pixels without returning an error
            if let (x @ 0..=799, y @ 0..=479) = coord.into() {
                let value: u16 = (color.b() as u16 & 0x1F)
                    | ((color.g() as u16 & 0x3F) << 5)
                    | ((color.r() as u16 & 0x1F) << 11);

                self.controller
                    .draw_pixel(Layer::L1, x as usize, y as usize, value);
            }
        }

        Ok(())
    }
}

impl OriginDimensions for LcdDisplay<u16> {
    fn size(&self) -> Size {
        Size::new(800, 480)
    }
}