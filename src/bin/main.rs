#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]
#![allow(unsafe_code)]

// use rtic_control_testing as _;

use panic_probe as _;
use defmt as _;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
// use mpu6050::*;

// mod mpu_6050 {
//     pub mod interface;
// }

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBus<USB>>> = None;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {

    use defmt::Format;
    // use mpu6050::Mpu6050;
    use stm32f4xx_hal::{
        gpio::{gpiob, Edge, Input, Output, PushPull},
        i2c::I2c,
        otg_fs::{UsbBus, USB},
        pac::{adc1::htr::W, syscfg, I2C1, TIM5},
        prelude::*,
        timer::{self, Event}
    };
    use usb_device::{prelude::*, test_class::SERIAL_NUMBER};
    use usbd_serial::{SerialPort, USB_CLASS_CDC};
    use core::{ptr::addr_of_mut, str, slice};
    use mpu6050::*;
    // use crate::{mpu_6050::interface::*, EP_MEMORY, USB_BUS};
    use crate::{EP_MEMORY, USB_BUS};

    const SYSFREQ: u32 = 100_000_000;

    #[shared]
    struct Shared {
        // serial: SerialPort<'static, UsbBus<USB>>,
        // usb_dev: UsbDevice<'static, UsbBus<USB>>,
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
        let mut syscfg = dp.SYSCFG.constrain();
        let clocks = rcc.cfgr.sysclk(SYSFREQ.Hz()).use_hse(24.MHz()).freeze();
        // let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        let mut delay = dp.TIM5.delay_us(&clocks);

        let mut imu = gpiob.pb8.into_pull_up_input();
        imu.make_interrupt_source(&mut syscfg);
        imu.enable_interrupt(&mut dp.EXTI);
        imu.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        // let usb = USB::new(
        //     (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        //     (gpioa.pa11, gpioa.pa12),
        //     &clocks,
        // );

        // unsafe {
        //     USB_BUS = Some(UsbBus::new(usb, slice::from_raw_parts_mut(
        //         addr_of_mut!(EP_MEMORY) as *mut u32,
        //         EP_MEMORY.len()
        //     )));
        // }

        // let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };
        // let usb_bus = UsbBus::new(usb, unsafe { slice::from_raw_parts_mut(
        //     addr_of_mut!(EP_MEMORY) as *mut u32,
        //     EP_MEMORY.len()
        // )});

        // let mut serial = SerialPort::new(usb_bus);

        // let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //     .device_class(usbd_serial::USB_CLASS_CDC)
        //     .strings(&[StringDescriptors::default()
        //         .manufacturer("Fake Company")
        //         .product("Testing thisss yeaaaa")
        //         .serial_number("TESTSS")])
        //     .unwrap()
        //     .build();


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
        // defmt::info!("### Finished Init Task ###");

        (
            Shared {
                // serial,
                // usb_dev,
            },
            Local {
                led,
                delay,
                mpu,
                imu,
            },
            // init::Monotonics(),
        )
    }

    // #[idle(local = [led, delay], shared = [usb_dev, serial])]
    // fn idle(mut ctx: idle::Context) -> ! {
    #[idle(local = [led, delay])]
    fn idle(ctx: idle::Context) -> ! {
        // Local
        let led = ctx.local.led;
        let delay = ctx.local.delay;

        // defmt::info!("Starting main loop");

        loop {
            led.set_high();
            delay.delay_ms(500);
            led.set_low();
            delay.delay_ms(500);

            // ctx.shared.usb_dev.lock(|usb_dev| {
            //     ctx.shared.serial.lock(|serial| {
            //         usb_dev.poll(&mut [serial]);
            //     });
            // });
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