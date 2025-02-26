#![no_main]
#![no_std]
#![allow(dead_code)]
#![allow(static_mut_refs)]
#![allow(unsafe_code)]
#![allow(unused_imports)]
#![feature(type_alias_impl_trait)]

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
use usb_device::{class_prelude::*, prelude::*, UsbError};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        serial: SerialPort<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        led: gpiob::PB13<Output<PushPull>>,
        // mpu: MPU6050,
        delay: timer::DelayUs<TIM5>,
        mpu: Mpu6050<I2c<I2C1>>,
        imu: gpiob::PB8<Input>,
    }

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(24.MHz())
            .sysclk(84.MHz()) // 24 MHz
            .hclk(48.MHz())
            .pclk1(48.MHz()) // 24 MHz
            .pclk2(48.MHz()) // 24 MHz
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

        delay.delay_ms(1); // give mpu6050 time to initialize otherwise board will hang

        let usb: USB = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: clocks.hclk(),
        };
        let usb_bus: &'static _ = ctx
            .local
            .usb_bus
            .insert(unsafe { UsbBus::new(usb, &mut EP_MEMORY) });

        let serial = SerialPort::new(usb_bus);

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .device_class(USB_CLASS_CDC)
            .strings(&[StringDescriptors::default()
                .manufacturer("fakeLLC")
                .product("test_pcb")
                .serial_number("12345")])
            .expect("could not build descriptors")
            .build();

        // Initialize i2c connection
        let i2c: I2c<I2C1> = I2c::new(dp.I2C1, (gpiob.pb6, gpiob.pb7), 100.kHz(), &clocks);
        let mut mpu: Mpu6050<I2c<I2C1>> = Mpu6050::new(i2c);
        delay.delay_ms(1); // give mpu6050 time to initialize otherwise board will hang
        mpu.init(&mut delay).unwrap();

        let led = gpiob.pb13.into_push_pull_output();

        // match usb_dev.self_powered() {
        //     true => defmt::info!("device is self-powered"),
        //     false => defmt::info!("device is not self-powered"),
        // }

        // match usb_dev.state() {
        //     UsbDeviceState::Default => defmt::info!("USB State:\tDefault"),
        //     UsbDeviceState::Suspend => defmt::info!("USB State:\tSuspend"),
        //     UsbDeviceState::Configured => defmt::info!("USB State:\tConfigured"),
        //     UsbDeviceState::Addressed => defmt::info!("USB State:\tAddressed"),
        // }
        defmt::info!("### Finished Init Task ###");

        defmt::info!("sysclk freq: {:?}", clocks.sysclk());

        (
            Shared { serial, usb_dev },
            Local {
                led,
                mpu,
                delay,
                imu,
            },
            // init::Monotonics(),
        )
    }

    #[idle(local = [led, delay], shared = [serial])]
    fn idle(ctx: idle::Context) -> ! {
        // Local
        let led = ctx.local.led;
        let delay = ctx.local.delay;

        // Shared
        let mut serial = ctx.shared.serial;

        // defmt::info!("Starting main loop");

        loop {
            defmt::info!("BEEP");
            led.toggle();
            delay.delay_ms(2000);
            led.toggle();
            delay.delay_ms(2000);

            serial.lock(|serial| {
                let _ = serial.write(b"BEEP\n\r");
            });
        }
    }

    #[task(binds=OTG_FS, shared=[serial, usb_dev])]
    fn usb_fs(ctx: usb_fs::Context) {
        let serial = ctx.shared.serial;
        let usb_dev = ctx.shared.usb_dev;

        (serial, usb_dev).lock(|serial, usb_dev| {
            if usb_dev.poll(&mut [serial]) {
                let mut buf = [0u8; 64];
                if let Ok(count) = serial.read(&mut buf) {
                    defmt::info!("{}", buf);
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    if count > 0 {
                        let _ = serial.write(b"\nReceived data!\n\r");
                        let _ = serial.flush();
                    }

                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => {
                                wr_ptr = &wr_ptr[len..];
                                let _ = serial.flush();
                            }
                            Err(_) => break,
                        }
                    }
                }
            }
        });
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
