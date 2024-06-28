use stm32f4xx_hal::{
    i2c::I2c,
    pac::{I2C1, TIM5},
    timer::{self, Instance},
};

// #[derive(Debug)]
// pub enum Mpu6050Error<E> {
//     I2c(E),

//     InvalidChipId(u8),
// }


const MASTER_CLOCK: u8 = 0b0000_1101;
const ACCEL_SENSITIVITY: i16 = 16384;
const GRAVITY_ACCEL: f32 = 9.81;

const SMPRT_DIV: u8 = 0x19;
const CONFIG: u8 = 0x1A;
const INT_PIN_CFG: u8 = 0x37;
const INT_ENABLE: u8 = 0x38;
const INT_STATUS: u8 = 0x3A;
const ACCEL_XOUT: u8 = 0x3B;
const ACCEL_YOUT: u8 = 0x3D;
const ACCEL_ZOUT: u8 = 0x3F;
const TEMP_OUT: u8 = 0x41;
const TEMP_OUT_H: u8 = 0x41;
const TEMP_OUT_L: u8 = 0x42;
const PWR_MGMT_1: u8 = 0x6B;
const PWR_MGMT_2: u8 = 0x6C;
const USER_CTRL: u8 = 0x6A;
const WHO_AM_I: u8 = 0x75;

pub struct MPU6050 {
    pub i2c_interface: I2c<I2C1>,
    mpu_address: u8,
}

impl MPU6050 {
    pub fn new(i2c_interface: I2c<I2C1>, mpu_address: u8) -> Self {
        MPU6050 {
            i2c_interface,
            mpu_address,
        }
    }

    pub fn initialize(&mut self) -> () {
        // self.reset_device();
        self.configure_data_interrupt();
        self.configure_dlpf();
        // self.set_sample_rate();
        self.clear_interrupt();
        // defmt::info!("Interface initialized mpu-6050!");
    }

    // fn reset_device(&mut self) -> () {

    //     self.i2c_interface.write(self.mpu_address, &[PWR_MGMT_1, 0b1000_0000]).unwrap();
    //     defmt::info!("Device Reset!");
    // }

    pub fn wake(&mut self) -> () {

        // set clock sources as x-gyro
        // take out of sleep
        self.i2c_interface
            .write(self.mpu_address, &[PWR_MGMT_1, 0b0010_0001])
            .unwrap();

        // set LP_WAKE_CRTL
        self.i2c_interface
            .write(self.mpu_address, &[PWR_MGMT_2, 0b0100_0000])
            .unwrap();

        // defmt::info!("Device awoken...")
        
    }

    fn configure_dlpf(&mut self) -> () {

        // set lpf to 44 Hz (4.9ms delay)
        self.i2c_interface
            .write(self.mpu_address, &[CONFIG, 0b0000_0011])
            .unwrap();
        // defmt::info!("DLPF configured...")

    }

    fn configure_data_interrupt(&mut self) -> () {
        
        self.i2c_interface
            .write(self.mpu_address, &[INT_PIN_CFG, 0b0010_0000])
            .unwrap();

        // Enable interrupts
        self.i2c_interface
            .write(self.mpu_address, &[INT_ENABLE, 0b0000_0001])
            .unwrap();

        // defmt::info!("Data interrupt configured!");
        
    }

    // fn set_sample_rate(&mut self) -> () {

    //     // set sample rate
    //     self.i2c_interface
    //         .write(self.mpu_address, &[SMPRT_DIV, 0b1100_0111])
    //         .unwrap();

    //     defmt::info!("Sample rate configured: 5 Hz");

    // }

    pub fn clear_interrupt(&mut self) -> () {
        
        let mut int_status_buf: [u8; 1] = [0u8; 1];

        // Check the interrupt status
        self.i2c_interface
            .write_read(self.mpu_address, &[INT_STATUS], &mut int_status_buf)
            .unwrap();
        
    }

    pub fn get_temperature_data(&mut self) -> f32 {

        let mut temp_buffer: [u8; 2] = [0u8; 2];

        let temp: i16;

        self.i2c_interface.write_read(self.mpu_address, &[TEMP_OUT], &mut temp_buffer).unwrap();

        temp = ((temp_buffer[0] as i16) << 8) | temp_buffer[1] as i16;

        let temp_phys: f32 = temp as f32 / 340.0 + 36.53;

        temp_phys

        // defmt::info!("Current Temperature: {:?} degC", temp_phys);
        
    }

    pub fn get_accelerometer_data(&mut self) -> [f32; 3] {

        let mut acc_x_buffer: [u8; 2] = [0u8; 2];
        let mut acc_y_buffer: [u8; 2] = [0u8; 2];
        let mut acc_z_buffer: [u8; 2] = [0u8; 2];

        let acc_x: i16;
        let acc_y: i16;
        let acc_z: i16;

        self.i2c_interface.write_read(self.mpu_address, &[ACCEL_XOUT], &mut acc_x_buffer).unwrap();
        self.i2c_interface.write_read(self.mpu_address, &[ACCEL_YOUT], &mut acc_y_buffer).unwrap();
        self.i2c_interface.write_read(self.mpu_address, &[ACCEL_ZOUT], &mut acc_z_buffer).unwrap();

        acc_x = ((acc_x_buffer[0] as i16) << 8) | acc_x_buffer[1] as i16;
        acc_y = ((acc_y_buffer[0] as i16) << 8) | acc_y_buffer[1] as i16;
        acc_z = ((acc_z_buffer[0] as i16) << 8) | acc_z_buffer[1] as i16;

        let acc_x_phys: f32 = acc_x as f32 * GRAVITY_ACCEL / ACCEL_SENSITIVITY as f32;
        let acc_y_phys: f32 = acc_y as f32 * GRAVITY_ACCEL / ACCEL_SENSITIVITY as f32;
        let acc_z_phys: f32 = acc_z as f32 * GRAVITY_ACCEL / ACCEL_SENSITIVITY as f32;

        [
            acc_x_phys,
            acc_y_phys,
            acc_z_phys,
        ]

        // defmt::info!("--- x: {:?} m/s^2 --- y: {:?} m/s^2 --- z: {:?} m/s^2 ---",
        //     acc_x_phys,
        //     acc_y_phys,
        //     acc_z_phys
        // );

    }
}