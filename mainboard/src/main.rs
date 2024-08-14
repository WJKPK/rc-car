#![no_std]
#![no_main]

use heapless::Vec;
use embassy_executor::Spawner;
use portable_atomic::{AtomicBool, Ordering};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{any_pin::AnyPin, Io},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
    },

};
use esp_println::println;
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize,
    EspWifiInitFor,
};
mod motor_control;
mod pwm_split;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

static PEER_FOUND: AtomicBool = AtomicBool::new(false);

//type FrontMotorDriver = motor_control::MotorControlDriver<'static, GpioPin<0>>;
//type BackMotorDriver = motor_control::MotorControlDriver<'static, GpioPin<1>>;

#[main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let mut ledc = Ledc::new(peripherals.LEDC, &clocks);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let splitted = pwm_split::SplitedPwm::new(&ledc);
    let mut vec = Vec::<pwm_split::PwmChannelLowSpeed<AnyPin>, 5>::new();
    splitted.create_channels(&mut vec, AnyPin::new(io.pins.gpio8));
    let wifi = peripherals.WIFI;
    let esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();
    defmt::info!("esp-now version {}", esp_now.get_version().unwrap());

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0);

    let (manager, sender, receiver) = esp_now.split();
    let manager = mk_static!(EspNowManager<'static>, manager);
    let sender = mk_static!(
        Mutex::<NoopRawMutex, EspNowSender<'static>>,
        Mutex::<NoopRawMutex, _>::new(sender)
    );

    spawner.spawn(listener(manager, receiver)).ok();
    spawner.spawn(broadcaster(sender)).ok();

//    let mut ticker = Ticker::every(Duration::from_millis(500));
//    loop {
//        ticker.next().await;
//        let peer = match manager.fetch_peer(false) {
//            Ok(peer) => peer,
//            Err(_) => {
//                if let Ok(peer) = manager.fetch_peer(true) {
//                    peer
//                } else {
//                    continue;
//                }
//            }
//        };
//
//        defmt::info!("Send hello to peer {:?}", peer.peer_address);
//        let mut sender = sender.lock().await;
//        let status = sender.send_async(&peer.peer_address, b"Hello Peer.").await;
//        defmt::info!("Send hello status: {:?}", status);
//    }
}

//#[embassy_executor::task]
//async fn motor_driver(front_driver: &'static FrontMotorDriver,
//    back_driver: &'static BackMotorDriver, adc: ADC1) {
//}

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

