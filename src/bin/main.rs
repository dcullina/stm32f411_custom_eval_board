#![deny(unsafe_code)]
#![no_main]
#![no_std]
// #![feature(type_alias_impl_trait)]
#![allow(unused_imports)]

use panic_probe as _;
use defmt_rtt as _;

mod mpu_6050 {
    pub mod interface;
}

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use stm32f4xx_hal::{
        gpio::{gpiob, Edge, Input, Output, PushPull},
        i2c::I2c,
        pac::{syscfg, I2C1, TIM5},
        prelude::*,
        timer::{self, Event},
    };
    use core::str;
    use crate::mpu_6050::interface::*;

    const SYSFREQ: u32 = 100_000_000;
    const MPU_6050_ADDR: u8 = 0x68;

    #[shared]
    struct Shared {
    }

    // Local resources go here
    #[local]
    struct Local {
        led: gpiob::PB13<Output<PushPull>>,
        delay: timer::DelayUs<TIM5>,
        mpu: MPU6050,
        imu: gpiob::PB8<Input>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let mut syscfg = dp.SYSCFG.constrain();
        let clocks = rcc.cfgr.sysclk(SYSFREQ.Hz()).use_hse(24.MHz()).freeze();
        let gpiob = dp.GPIOB.split();

        let mut delay = dp.TIM5.delay_us(&clocks);

        let mut imu = gpiob.pb8.into_pull_up_input();
        imu.make_interrupt_source(&mut syscfg);
        imu.enable_interrupt(&mut dp.EXTI);
        imu.trigger_on_edge(&mut dp.EXTI, Edge::Rising);


        // Initialize i2c connection
        let i2c = I2c::new(dp.I2C1, (gpiob.pb6, gpiob.pb7), 100.kHz(), &clocks);
        let mut mpu: MPU6050 = MPU6050::new(i2c, MPU_6050_ADDR);
        mpu.initialize();
        delay.delay_ms(500);
        mpu.wake();
        
        let mut led = gpiob.pb13.into_push_pull_output();
        led.set_low();
        defmt::info!(" ### Finished Init Task ###");

        (
            Shared {
            },
            Local {
                led,
                delay,
                mpu,
                imu,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [led, delay])]
    fn idle(ctx: idle::Context) -> ! {
        let led = ctx.local.led;
        let delay = ctx.local.delay;
        defmt::info!("Starting main loop");
        loop {
            led.set_high();
            delay.delay_ms(500);
            led.set_low();
            delay.delay_ms(500);
        }
    }

    #[task(binds = EXTI9_5, local = [imu, mpu])]
    fn data_ready(ctx: data_ready::Context) {
        defmt::info!("Interrupt received in hardware");
        let mpu = ctx.local.mpu;
        ctx.local.imu.clear_interrupt_pending_bit();

        mpu.get_accelerometer_data();
        mpu.get_temperature_data();
        mpu.clear_interrupt();

    }

}
