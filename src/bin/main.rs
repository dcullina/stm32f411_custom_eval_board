#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]
#![allow(unsafe_code)]

use core::{ptr::addr_of_mut, slice, str};
use defmt_rtt as _;
use mpu6050::*;
use panic_probe as _;
use stm32f4xx_hal::{
    gpio::alt::otg_fs::{Dm, Dp},
    gpio::{gpioa, gpiob, Alternate, Edge, Input, Output, PushPull},
    i2c::I2c,
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{syscfg, I2C1, TIM5},
    prelude::*,
    timer::{self, Event},
};
use usb_device::{bus::UsbBusAllocator, prelude::*, test_class::SERIAL_NUMBER, UsbError};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    const SYSFREQ: u32 = 100_000_000;

    #[shared]
    struct Shared {
        serial: SerialPort<'static, UsbBus<USB>>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
    }

    #[local]
    struct Local {
        led: gpiob::PB13<Output<PushPull>>,
        delay: timer::DelayUs<TIM5>,
        // mpu: MPU6050,
        mpu: Mpu6050<I2c<I2C1>>,
        imu: gpiob::PB8<Input>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(SYSFREQ.Hz())
            .use_hse(24.MHz())
            .require_pll48clk()
            .freeze();

        let mut syscfg = dp.SYSCFG.constrain();

        let gpiob = dp.GPIOB.split();

        let mut delay = dp.TIM5.delay_us(&clocks);

        let mut imu = gpiob.pb8.into_pull_up_input();
        imu.make_interrupt_source(&mut syscfg);
        imu.enable_interrupt(&mut dp.EXTI);
        imu.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        let gpioa = dp.GPIOA.split();

        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let usb: USB = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: stm32f4xx_hal::gpio::alt::otg_fs::Dm::PA11(gpioa.pa11.into_alternate()),
            pin_dp: stm32f4xx_hal::gpio::alt::otg_fs::Dp::PA12(gpioa.pa12.into_alternate()),
            hclk: clocks.hclk(),
        };
        unsafe {
            USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let mut serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let descriptors = StringDescriptors::new(LangID::EN)
            .manufacturer("fakeLLC")
            .product("test_pcb")
            .serial_number("12345");
        let mut usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x5740, 0x0483),
        )
        .strings(&[descriptors])
        .expect("could not build descriptors")
        .device_class(USB_CLASS_CDC)
        .build();

        // Initialize i2c connection
        let i2c: I2c<I2C1> = I2c::new(dp.I2C1, (gpiob.pb6, gpiob.pb7), 100.kHz(), &clocks);
        let mut mpu: Mpu6050<I2c<I2C1>> = Mpu6050::new(i2c);
        delay.delay_ms(1); // give mpu6050 time to initialize otherwise board will hang
        mpu.init(&mut delay).unwrap();

        let mut led = gpiob.pb13.into_push_pull_output();
        led.set_high();
        delay.delay_ms(1000);
        led.set_low();
        delay.delay_ms(1000);
        led.set_high();
        delay.delay_ms(1000);
        led.set_low();
        delay.delay_ms(1000);
        led.set_high();
        defmt::info!("### Finished Init Task ###");

        (
            Shared { serial, usb_dev },
            Local {
                led,
                delay,
                mpu,
                imu,
            },
            // init::Monotonics(),
        )
    }

    #[idle(local = [led, delay], shared = [usb_dev, serial])]
    fn idle(mut ctx: idle::Context) -> ! {
        // Local
        let led = ctx.local.led;
        let delay = ctx.local.delay;

        // Shared
        let mut usb_dev = ctx.shared.usb_dev;
        let mut serial = ctx.shared.serial;

        // defmt::info!("Starting main loop");

        loop {
            defmt::info!("BEEP");
            led.set_high();
            delay.delay_ms(2000);
            led.set_low();
            delay.delay_ms(2000);

            serial.lock(|serial| match serial.write(&[0x3a, 0x29]) {
                Ok(_) => defmt::info!("Message sent!"),
                Err(UsbError::WouldBlock) => defmt::error!("no data written, buffers full"),
                Err(err) => defmt::error!("error occurred"),
            });

            // let mut buf = [0u8; 64];
            // if let Ok(count) = serial.lock(|serial| serial.read(&mut buf)) {
            //     if count > 0 {
            //         defmt::info!("\r\nUSB count greater than 0, writing \"yosh\"!!!");
            //         serial.lock(|serial| serial.write(b"\r\nyosh\r\n")).ok();
            //     }
            // }
        }
    }

    // #[task(binds = EXTI9_5, local = [imu, mpu], shared = [serial])]
    // fn data_ready(mut ctx: data_ready::Context) {
    // #[task(binds = EXTI9_5, local = [imu, mpu])]
    // fn data_ready(ctx: data_ready::Context) {
    //     let mpu = ctx.local.mpu;

    //     let temp = mpu.get_temp().unwrap();

    //     let accel = mpu.get_ac().unwrap();
    //     // let accel_str: &str = "hello world";

    //     let mut buf = [0u8; 64];
    //     let mut idx = 0;

    //     for &byte in accel_str.as_bytes().iter() {
    //         if idx < buf.len() {
    //             buf[idx] = byte;
    //             idx += 1;
    //         }
    //     }

    //     // ctx.shared.serial.lock(|serial| {
    //     //     let _ = serial.write(&buf[0..idx]);
    //     // });

    //     ctx.local.imu.clear_interrupt_pending_bit();
    //     mpu.clear_interrupt();

    // }
}
