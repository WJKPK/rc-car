#![no_std]
#![no_main]

mod motor_control;
mod pwm_split;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    gpio::{any_pin::AnyPin, Io},
    ledc::{LSGlobalClkSource, Ledc},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize, EspWifiInitFor,
};
use portable_atomic::{AtomicBool, Ordering};
use pwm_split::PwmChannel;
use motor_control::{Direction, Angle};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static PEER_FOUND: AtomicBool = AtomicBool::new(false);

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = mk_static!(
        Clocks<'static>,
        ClockControl::max(system.clock_control).freeze()
    );
    let ledc = mk_static!(Ledc<'static>, Ledc::new(peripherals.LEDC, clocks));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();
    defmt::info!("esp-now version {}", esp_now.get_version().unwrap());

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, clocks);
    esp_hal_embassy::init(clocks, timer_group0);

    let (manager, sender, receiver) = esp_now.split();
    let manager = mk_static!(EspNowManager<'static>, manager);
    let sender = mk_static!(
        Mutex::<NoopRawMutex, EspNowSender<'static>>,
        Mutex::<NoopRawMutex, _>::new(sender)
    );

    let splitted = mk_static!(
        pwm_split::SplitedPwm<'static>,
        pwm_split::SplitedPwm::new(ledc)
    );

    let front_power = motor_control::PowerMotorInternals::new(
        [AnyPin::new(io.pins.gpio3), AnyPin::new(io.pins.gpio2)],
        [PwmChannel::Channel0, PwmChannel::Channel1],
    );

    let back_power = motor_control::PowerMotorInternals::new(
        [AnyPin::new(io.pins.gpio8), AnyPin::new(io.pins.gpio10)],
        [PwmChannel::Channel2, PwmChannel::Channel3],
    );

    let serwo =
        motor_control::SerwoMotorInternals::new(AnyPin::new(io.pins.gpio7), PwmChannel::Channel4);
    let driver = mk_static!(
        motor_control::MotorDescriptorBoundle<AnyPin>,
        motor_control::MotorDescriptorBoundle::new(front_power, back_power, serwo)
    );

    spawner.spawn(listener(manager, receiver)).ok();
    spawner.spawn(broadcaster(sender)).ok();
    spawner.spawn(motor_driver(splitted, driver)).ok();

    defmt::info!("Bye!");
}

#[embassy_executor::task]
async fn motor_driver(
    splitted: &'static mut pwm_split::SplitedPwm<'static>,
    motor_controller: &'static mut motor_control::MotorDescriptorBoundle<AnyPin<'static>>,
) {
    let motor_driver = motor_controller.setup_driver(splitted);
    motor_driver.drive_motor(Direction::Forward, 0.9f32).unwrap();
    if let Some(angle) = Angle::new(0) {
        motor_driver.drive_serwo(angle).unwrap();
    }
    loop {}
}

#[embassy_executor::task]
async fn broadcaster(sender: &'static Mutex<NoopRawMutex, EspNowSender<'static>>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    while !PEER_FOUND.load(Ordering::Relaxed) {
        ticker.next().await;

        defmt::info!("Send Broadcast...");
        let mut sender = sender.lock().await;
        let status = sender.send_async(&BROADCAST_ADDRESS, b"Hello.").await;
        println!("Send broadcast status: {:?}", status);
    }
}

#[embassy_executor::task]
async fn listener(manager: &'static EspNowManager<'static>, mut receiver: EspNowReceiver<'static>) {
    loop {
        let r = receiver.receive_async().await;
        defmt::info!("Received {:?}", r.get_data());
        if !manager.peer_exists(&r.info.src_address) {
            manager
                .add_peer(PeerInfo {
                    peer_address: r.info.src_address,
                    lmk: None,
                    channel: None,
                    encrypt: false,
                })
                .unwrap();
            PEER_FOUND.store(true, Ordering::Relaxed);
            defmt::info!("Added peer {:?}", r.info.src_address);
        }
    }
}
