#![no_std]
#![no_main]
use core::sync::atomic::AtomicI16;
use core::sync::atomic::AtomicU16;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use core::*;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::i2c::{self};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, can, peripherals};
use {defmt_rtt as _, panic_probe as _};
use {defmt_rtt as _, panic_probe as _};
// use embassy_stm32::gpio::{Input, Output, Pin, Pull, Speed};
use embassy_stm32::{Config, peripherals::*, usart};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<USART2>;
    USART3  =>usart::InterruptHandler<USART3>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
        FDCAN1_IT0=>can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1=>can::IT1InterruptHandler<FDCAN1>;
});

const UART_BAUDRATE: u32 = 9600;
const CMD_TEMP: [u8; 3] = [0x55, 0x04, 0x0E];
const CMD_PRESS: [u8; 3] = [0x55, 0x04, 0x0D];
const RET_START_FLAG: u8 = 0xAA;
const RET_TYPE_TEMP: u8 = 0x0A;
const RET_TYPE_PRESS: u8 = 0x09;
const TEMP_FRAME_LEN: usize = 6;
const PRESS_FRAME_LEN: usize = 8;
const IN_PRESS: u8 = 1;
const OUT_PRESS: u8 = 2;
const DEV_ADDR: u8 = 0x40;
const BUS_VOLTAGE: u8 = 0x02;
const BUS_VOLTAGE_LSB: f32 = 0.004; //LSB=4mV
const MASTER_ID: u32 = 0x1F010115;
const LOW_DATA: [u8; 8] = [0xB6, 0x3E, 0x37, 0x21, 0xC4, 0x25, 0x9C, 0x00]; //B3
const HIGH_DATA: [u8; 8] = [0x00, 0x00, 0xC1, 0x20, 0x17, 0x15, 0xE5, 0x07]; //B4
const KEEP_LIVE: [u8; 1] = [0x01]; //B0
const B1_DATA: [u8; 8] = [0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00];
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x01 != 0 {
                crc = (crc >> 1) ^ 0x8c;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[derive(Debug, Default, Clone, Copy)]
struct SensorData {
    temp_raw: i16,
    press_raw: u32,
    voltage_raw: u16,
    o2_concentration: u32,
}

struct AtomicSensorData {
    atom_temperature: AtomicI16,
    atom_pressure: AtomicU32,
    o2_connect: AtomicU32,
    o2_rx: AtomicU32,
    raw_bus_voltage: AtomicU16,
}

impl AtomicSensorData {
    const fn new() -> Self {
        Self {
            atom_temperature: AtomicI16::new(0),
            atom_pressure: AtomicU32::new(0),
            o2_connect: AtomicU32::new(0),
            o2_rx: AtomicU32::new(0),
            raw_bus_voltage: AtomicU16::new(0),
        }
    }

    fn load_full_data(&self) -> SensorData {
        SensorData {
            temp_raw: self.atom_temperature.load(Ordering::Relaxed),
            press_raw: self.atom_pressure.load(Ordering::Relaxed),
            voltage_raw: self.raw_bus_voltage.load(Ordering::Relaxed),
            o2_concentration: self.o2_connect.load(Ordering::Relaxed),
        }
    }

    fn store_raw(&self, local_data: &SensorData) {
        //store  write
        self.atom_pressure
            .store(local_data.press_raw, Ordering::Relaxed);
        self.atom_temperature
            .store(local_data.temp_raw, Ordering::Relaxed);
        self.raw_bus_voltage
            .store(local_data.voltage_raw, Ordering::Relaxed);
        self.o2_connect
            .store(local_data.o2_concentration, Ordering::Relaxed);
    }

    fn o2_store(&self, rx_o2: u32) {
        self.o2_rx.store(rx_o2, Ordering::Relaxed);
    }

    fn load_raw_press(&self) -> u32 {
        //read
        //let temp_raw = self.atom_temperature.load(Ordering::Relaxed);
        let press_raw = self.atom_pressure.load(Ordering::Relaxed);

        press_raw
    }

    fn load_raw_candata(&self) -> u32 {
        let can_data = self.o2_connect.load(Ordering::Relaxed);
        can_data
    }

    fn load_rec_candata(&self) -> u32 {
        let rec_data = self.o2_rx.load(Ordering::Relaxed);
        rec_data
    }

    fn load_raw_voltage(&self) -> u16 {
        let valid_data = self.raw_bus_voltage.load(Ordering::Relaxed);
        valid_data
    }

    fn load_f32(&self) -> f32 {
        //acquire primitive
        let press_raw = self.load_raw_press();
        press_raw as f32 / 1000.0
    }

    fn voltage_read(&self) -> f32 {
        let valid_data = self.load_raw_voltage();
        let bus_voltage = (valid_data >> 3) as f32 * BUS_VOLTAGE_LSB;
        bus_voltage
    }
}

#[derive(Debug)]
struct SensorManager {
    sensor_type: u8,
    data: SensorData,
    received_data: [u8; 16],
    has_read_temp: bool,
}

impl SensorManager {
    fn new(sensor_type: u8) -> Self {
        Self {
            sensor_type,
            data: SensorData::default(),
            received_data: [0u8; 16],
            has_read_temp: false,
        }
    }

    fn parse_temp_frame(&mut self, frame: &[u8]) {
        self.data.temp_raw = frame[3] as i16 | ((frame[4] as i16) << 8);
    }

    fn parse_press_frame(&mut self, frame: &[u8]) {
        self.data.press_raw = frame[3] as u32
            | (frame[4] as u32) << 8
            | (frame[5] as u32) << 16
            | (frame[6] as u32) << 24;
    }
    fn sync_to_global(&self, global: &AtomicSensorData) {
        global.store_raw(&self.data);
    }
}

fn create_cmd(cmd_base: &[u8; 3]) -> [u8; 4] {
    let crc = crc8(cmd_base); // 计算CRC
    [cmd_base[0], cmd_base[1], cmd_base[2], crc]
}

fn validate_frame(frame: &[u8]) -> Option<&[u8]> {
    match frame.len() {
        TEMP_FRAME_LEN => {
            if frame[0] == RET_START_FLAG
                && frame[1] == TEMP_FRAME_LEN as u8
                && frame[2] == RET_TYPE_TEMP
            {
                let crc = crc8(&frame[0..(TEMP_FRAME_LEN - 1)]);
                if crc == frame[TEMP_FRAME_LEN - 1] {
                    return Some(&frame[0..TEMP_FRAME_LEN]);
                }
            }
            None
        }
        PRESS_FRAME_LEN => {
            if frame[0] == RET_START_FLAG
                && frame[1] == PRESS_FRAME_LEN as u8
                && frame[2] == RET_TYPE_PRESS
            {
                let crc = crc8(&frame[0..(PRESS_FRAME_LEN - 1)]);
                if crc == frame[PRESS_FRAME_LEN - 1] {
                    return Some(&frame[0..PRESS_FRAME_LEN]);
                }
            }
            None
        }
        _ => None,
    }
}

static GLOBAL_INNER_SENSOR: AtomicSensorData = AtomicSensorData::new();
static GLOBAL_OUTER_SENSOR: AtomicSensorData = AtomicSensorData::new();
static O2_CONCENTRATION: AtomicSensorData = AtomicSensorData::new();
static BUS_V: AtomicSensorData = AtomicSensorData::new();
#[embassy_executor::task(pool_size = 2)] //allow two example
async fn sensor_task(
    mut uart: usart::Uart<'static, Async>,
    sensor_type: u8,
    global_data: &'static AtomicSensorData,
) {
    let mut manager = SensorManager::new(sensor_type);
    let mut current_cmd: Option<[u8; 4]> = None;
    let temp_cmd = create_cmd(&CMD_TEMP);
    let press_cmd = create_cmd(&CMD_PRESS);

    loop {
        Timer::after(Duration::from_millis(50)).await;

        if current_cmd != Some(temp_cmd) && !manager.has_read_temp {
            match uart.write(&temp_cmd).await {
                Ok(_) => {
                    current_cmd = Some(temp_cmd);
                }
                Err(e) => {
                    error!("UART Write Error: {:?}", e);
                    current_cmd = None;
                    Timer::after(Duration::from_millis(100)).await;
                    continue;
                }
            }
        } else {
            match uart.write(&press_cmd).await {
                Ok(_) => {}
                Err(e) => {
                    error!("UART Write Error: {:?}", e);
                    current_cmd = None;
                    Timer::after(Duration::from_millis(100)).await;
                    continue;
                }
            }
        }
        let mut recv_buf = manager.received_data;
        let timeout_fut = Timer::after_millis(100);
        let read_fut = uart.read_until_idle(&mut recv_buf);

        let res = select(timeout_fut, read_fut).await;
        let recv_len = match res {
            Either::Second(Ok(len)) => len,
            _ => {
                continue;
            }
        };

        let valid_frame = validate_frame(&recv_buf[0..recv_len]);
        if let Some(frame) = valid_frame {
            if frame.len() == TEMP_FRAME_LEN {
                manager.parse_temp_frame(frame);
                manager.sync_to_global(global_data);
            } else if frame.len() == PRESS_FRAME_LEN {
                manager.parse_press_frame(frame);
                manager.sync_to_global(global_data);
                //   info!("data:{:02X}", recv_buf);
            } else {
                //      warn!("收到无效帧: {:02X}", &recv_buf[0..recv_len]);
            }
        }
    }
}

#[embassy_executor::task]
async fn ina_task(mut i2c: i2c::I2c<'static, Async, i2c::Master>) {
    let mut bus_voltage_buf = [0u8; 2];
    loop {
        match i2c
            .write_read(DEV_ADDR, &[BUS_VOLTAGE], &mut bus_voltage_buf)
            .await
        {
            Ok(_) => {
                let voltage_raw = u16::from_be_bytes(bus_voltage_buf);
                let mut sensor_data = BUS_V.load_full_data();
                sensor_data.voltage_raw = voltage_raw;
                BUS_V.store_raw(&sensor_data);
            }
            Err(e) => {
                error!("INA219 error:{:?}", e);
            }
        }
        Timer::after_millis(50).await;
    }
}

fn can_data_crtl(value: u32) -> [u8; 4] {
    let data = [
        (value & 0xFF) as u8,
        ((value >> 8) & 0xFF) as u8,
        ((value >> 16) & 0xFF) as u8,
        ((value >> 24) & 0xFF) as u8,
    ];
    data
}

fn can_data_rec(data_rx: &[u8]) -> u32 {
    let data = (data_rx[3] as u32) << 24
        | (data_rx[2] as u32) << 16
        | (data_rx[1] as u32) << 8
        | (data_rx[0] as u32);
    data
}

#[embassy_executor::task]
async fn can_task(mut can: CanConfigurator<'static>) {
    can.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo0(),
    );
    can.set_bitrate(500_000);

    let can = can.start(can::OperatingMode::NormalOperationMode); //normal mode
    //let can = can.start(can::OperatingMode::InternalLoopbackMode);
    let (mut tx, mut rx, _properties) = can.split(); //send and receive

    let mut tick = 10;
    const TICK_STEP: u64 = 10;
    loop {
        let o2_value = O2_CONCENTRATION.load_raw_candata();

        let data = can_data_crtl(o2_value);

        let frame_b2 = can::frame::Frame::new_extended(0x1F0101B2, &data).unwrap();
        tx.write(&frame_b2).await;

        if tick % 500 == 0 {
            let frame_b0 = can::frame::Frame::new_extended(0x1F0101B0, &KEEP_LIVE).unwrap();
            tx.write(&frame_b0).await;
        }

        if tick % 1000 == 0 {
            let frame_b3 = can::frame::Frame::new_extended(0x1F0101B3, &LOW_DATA).unwrap();
            tx.write(&frame_b3).await;
            let frame_b4 = can::frame::Frame::new_extended(0x1F0101B4, &HIGH_DATA).unwrap();
            tx.write(&frame_b4).await;
            let frame_b1 = can::frame::Frame::new_extended(0x1F0101B1, &B1_DATA).unwrap();
            tx.write(&frame_b1).await;
        }

        // match rx.read().await {
        //     Ok(received_frame) => {
        //         if let embedded_can::Id::Extended(id) = received_frame.frame.id() {
        //             if id.as_raw() == 0x1F0101B2 {
        //                 let data_rx = received_frame.frame.data();

        //                 O2_CONCENTRATION.o2_store(can_data_rec(&data_rx));

        //                 info!("receive data:{:02X}", data_rx);
        //                 Timer::after_millis(50).await;
        //             }
        //         }
        //     }
        //     Err(e) => {
        //         info!("error:{}", e);
        //     }
        // }
        Timer::after_millis(TICK_STEP).await;
        tick += TICK_STEP;
        if tick >= 1000 {
            tick = 0;
        }
    }
}

#[embassy_executor::task]
async fn diff_task() {
    loop {
        let bus_voltage = BUS_V.voltage_read();
        let inner_press = GLOBAL_INNER_SENSOR.load_f32();
        let outer_press = GLOBAL_OUTER_SENSOR.load_f32();
        let o2_rec = O2_CONCENTRATION.load_rec_candata();
        let o2_data = (o2_rec as f32) / 100.0;

        let press_diff1 = if (inner_press - outer_press).abs() >= 10.0 {
            ((bus_voltage / 0.0492) / (inner_press / 1.67)) * 100.0
        } else {
            ((bus_voltage / 0.0492) / (inner_press)) * 100.0
        };

        // let press_diff1 = ((bus_voltage / 0.0493) / (inner_press)) * 100.0;

        let o2_concentration: u32 = (press_diff1 * 100.0) as u32;
        let sensor_data = SensorData {
            o2_concentration,
            ..Default::default()
        };

        O2_CONCENTRATION.store_raw(&sensor_data);

        info!("修改前内部传感器:气压={}kPa", inner_press);
        // info!("修改后内部传感器:气压={}kPa", inner_press_corrected);
        info!("外部传感器:气压={}kPa", outer_press);
        info!("采样电压为:{}V", bus_voltage);
        info!("氧气浓度={}%", press_diff1);
        //  info!("返回氧气浓度数据为:{}%", o2_data);
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe { cortex_m::interrupt::enable() };
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL80,
            divp: None,
            divq: Some(PllQDiv::DIV8), // 40 Mhz for fdcan.
            divr: Some(PllRDiv::DIV2), // Main system clock at 160 MHz
        });
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(config);
    let mut uart_config = usart::Config::default();
    uart_config.baudrate = UART_BAUDRATE;

    let uart2 = usart::Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        uart_config,
    )
    .expect("");

    let uart3 = usart::Uart::new(
        p.USART3,
        p.PB11,
        p.PB10,
        Irqs,
        p.DMA1_CH5,
        p.DMA1_CH6,
        uart_config,
    )
    .expect("");

    let can = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz::khz(400);

    let i2c = i2c::I2c::new(
        p.I2C1, p.PB8, p.PB7, Irqs, p.DMA1_CH3, p.DMA1_CH4, i2c_config,
    );

    //spawner.must_spawn(sensor_task(uart2));
    //  Timer::after_millis(15).await;
    spawner.must_spawn(sensor_task(uart3, IN_PRESS, &GLOBAL_INNER_SENSOR));
    spawner.must_spawn(sensor_task(uart2, OUT_PRESS, &GLOBAL_OUTER_SENSOR));
    spawner.must_spawn(ina_task(i2c));
    spawner.must_spawn(can_task(can));
    spawner.must_spawn(diff_task());
}
