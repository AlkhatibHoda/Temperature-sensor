#![no_std]
#![no_main]

use core::str::from_utf8;
use core::panic::PanicInfo;
use display::SPIDeviceInterface;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::Spi;
use embassy_rp::{peripherals::*, spi};
use embassy_time::{Duration, Timer};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::PrimitiveStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::geometry::Point;
use embedded_graphics::primitives::Circle;
use heapless::String;
use log::{info, warn};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, IpAddress, IpEndpoint, Stack, StackResources};
use cyw43_pio::PioSpi;
use embassy_rp::pio::{InterruptHandler, Pio};
use static_cell::StaticCell;
use core::fmt::Write as fmtWrite;

// I2C
use embassy_rp::i2c::{I2c, InterruptHandler as I2CInterruptHandler};
use embassy_rp::peripherals::I2C0;
use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_rp::spi::Blocking;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};


// PWM
mod display;

// USB driver
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as USBInterruptHandler};
use embassy_rp::bind_interrupts;

use embassy_time::Delay;
use embedded_graphics::mono_font::iso_8859_16::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::text::renderer::CharacterStyle;
use embedded_graphics::text::Text;
use st7789::{Orientation, ST7789};
use bmp280_rs::{self, I2CAddress, ModeSleep, BMP280};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => USBInterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct IrqsI2C {
    I2C0_IRQ => I2CInterruptHandler<I2C0>;
});

const DISPLAY_FREQ: u32 = 64_000_000; // clock frequency

// The task used by the serial port driver over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

fn increment_time(second: &mut i32, minute: &mut i32, hour: &mut i32, day: &mut u32, month: &mut u32, year: &mut u32) {
    *second += 1;
    if *second >= 60 {
        *second = 0;
        *minute += 1;
        if *minute >= 60 {
            *minute = 0;
            *hour += 1;
            if *hour >= 24 {
                *hour = 0;
                *day += 1;
                if *day > 30 {
                    *day = 1;
                    *month += 1;
                    if *month > 12 {
                        *month = 1;
                        *year += 1;
                    }
                }
            }
        }
    }
}

//============================================================================//
//============================================================================//
const WIFI_NETWORK: &str = "hudawifi";
const WIFI_PASSWORD: &str = "parola123";
//============================================================================//
//============================================================================//

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());
    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

   
    let fw = include_bytes!("../net_drivers/43439A0.bin");
    let clm = include_bytes!("../net_drivers/43439A0_clm.bin");

    
    let pwr = Output::new(peripherals.PIN_23, Level::Low);
    let cs = Output::new(peripherals.PIN_25, Level::High);
    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        peripherals.PIN_24,
        peripherals.PIN_29,
        peripherals.DMA_CH0,
    );

 
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(wifi_task(runner)).unwrap();

 
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());
    
    let seed = 0x0123_4567_89ab_cdef;

    
    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        net_device,
        config,
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));

    
    spawner.spawn(net_task(stack)).unwrap();

    
    loop {
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status {}", err.status);
            }
        }
    }

    
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

   
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    
    let miso = peripherals.PIN_4;
    let mosi = peripherals.PIN_19;
    let clk = peripherals.PIN_18;

    
    let spi_display: Spi<'_, _, Blocking> =
        Spi::new_blocking(peripherals.SPI0, clk, mosi, miso, display_config.clone());
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(spi_display));
    let display_cs = Output::new(peripherals.PIN_17, Level::High);

    
    let display_spi = SpiDeviceWithConfig::new(&spi_bus, display_cs, display_config);
    let rst = peripherals.PIN_0;
    let dc = peripherals.PIN_16;
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::Low);
    let di = SPIDeviceInterface::new(display_spi, dc);

    
    let mut display = ST7789::new(di, rst, 240, 240);
    display.init(&mut Delay).unwrap();
    display.set_orientation(Orientation::Portrait).unwrap();
    display.clear(Rgb565::MAGENTA).unwrap();

    
    let mut style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    style.set_background_color(Some(Rgb565::MAGENTA));

    
    let i2c_config = embassy_rp::i2c::Config::default();
    let mut i2c = I2c::new_async(peripherals.I2C0, peripherals.PIN_21, peripherals.PIN_20, IrqsI2C, i2c_config);

    let bmp280_config = bmp280_rs::Config::indoor_navigation();
    let bmp280_address = I2CAddress::SdoGrounded; 
    let bmp280 = BMP280::<_, ModeSleep>::new(&mut i2c, bmp280_address, bmp280_config);
    
   
    let bmp280 = match bmp280 {
        Ok(sensor) => sensor,
        Err(_e) => {
            return;
        }
    };
    
    
    let mut bmp280 = match bmp280.into_normal_mode(&mut i2c) {
        Ok(sensor) => sensor,
        Err(_e) => {
            return;
        }
    };
   
    let datetime_str = String::<128>::new();
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));
    let endpoint = IpEndpoint::new(IpAddress::v4(82, 202, 222, 252), 80);
    let request = "GET /api/now?tz=Europe/Bucharest&format=dd/MM/yyyy HTTP/1.1\r\nHost: tools.aimylogic.com\r\n\r\n";
    if let Err(e) = socket.connect(endpoint).await {
        warn!("Connect error: {:?}", e);
        return;
    }

    if let Err(e) = socket.write(request.as_bytes()).await {
        warn!("Write error: {:?}", e);
        return;
    }

    let n = match socket.read(&mut buf).await {
        Ok(0) => return,
        Ok(n) => n,
        Err(e) => {
            warn!("Read error: {:?}", e);
            return;
        }
    };

    let mut hour: i32 = 0;
    let mut minute: i32 = 0;
    let mut second: i32 = 0;
    let mut display_strings = String::<128>::new();
    if datetime_str.is_empty() {
      
        let response = from_utf8(&buf[..n]).unwrap();
        info!("{response}");
        if let Some(datetime_str) = response.lines().find(|datetime_str| datetime_str.starts_with("Date:")) {
            let (date_part, time_part) = datetime_str.split_at(23);
            let time_part = time_part.trim();
            let (hour_str, rest) = time_part.split_at(2); 
            hour = hour_str.parse().unwrap();
            minute = rest[1..3].parse().unwrap();
            second = rest[4..6].parse().unwrap();
            hour += 3;//timezone adjustment.
            if hour >= 24 {
                hour -= 24;
            }
 
    
            let date_part = date_part[6..].trim();
            let (_day_str, mut rest) = date_part.split_at(3); 
            rest = rest[2..].trim();
            let (day, mut rest) = rest.split_at(2);
            rest = rest[1..].trim();
            let (month, mut rest) = rest.split_at(3);
            rest = rest[1..].trim();
            let (year, mut _rest) = rest.split_at(4);
            let year = year.trim();
            write!(display_strings, "<3 Date: {day} {month} {year}", day = day, month = month, year = year).unwrap();
        }
    }


    let mut day = 1;
    let mut month = 1;
    let mut year = 2024;


    let heart_style = PrimitiveStyle::with_fill(Rgb565::RED);    
    Circle::new(Point::new(80, 120), 30)
        .into_styled(heart_style)
        .draw(&mut display)
        .unwrap();
    
    Circle::new(Point::new(120, 120), 30)
        .into_styled(heart_style)
        .draw(&mut display)
        .unwrap();



    let mut config_pwm: PwmConfig = Default::default();
    config_pwm.top = 0xFFFF;
    config_pwm.compare_b = 0;

   
    let mut buzzer = Pwm::new_output_b(peripherals.PWM_SLICE0, peripherals.PIN_1, config_pwm.clone());


    loop {
       
        increment_time(&mut second, &mut minute, &mut hour, &mut day, &mut month, &mut year);


        let mut current_time = String::<64>::new();
        write!(current_time, "<3 Time: {hour:02}:{minute:02}:{second:02}", hour = hour, minute = minute, second = second).unwrap();

        let mut text = String::<64>::new();
        write!(text, "{}\n{}", display_strings, current_time).unwrap();

        Text::new(&text, Point::new(1, 70), style)
            .draw(&mut display)
            .unwrap();



    
        let mut text = String::<64>::new();

        match bmp280.read_temperature(&mut i2c) {
            Ok(temperature) => {
                let temperature_celsius = temperature as f32 / 100.0;
                info!("Temperature actual {temperature_celsius}");
                write!(text, "<3 Temperature: {:.2}°C", temperature_celsius).unwrap();
                if temperature > 25 {
                    config_pwm.compare_b = config_pwm.top / 2;
                } else {
                    config_pwm.compare_b = 0;
                }
        
                buzzer.set_config(&config_pwm);
        
            }
            Err(_e) => {
                write!(text, "> Temperature: Error").unwrap();
            }
        }
        buzzer.set_config(&config_pwm);


        match bmp280.read_pressure(&mut i2c) {
            Ok(pressure) => {
                let pressure_pascals = pressure as f32 / 100.0;
                info!("Pressure actual {pressure_pascals}");
                write!(text, "\n<3 Press: {:.2} hPa", pressure_pascals).unwrap();
            }
            Err(_e) => {
                write!(text, "\n> Pressure: Error").unwrap();
            }
        }



        Text::new(&text, Point::new(1, 30), style)
            .draw(&mut display)
            .unwrap();

        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
