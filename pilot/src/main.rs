#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use embassy_sync::pubsub::{Publisher, PubSubChannel};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    gpio::{GpioPin, Io, Output, Level},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize,
    EspWifiInitFor,
};
use comunication::{RcCarControlViaEspReady, Angle, Percent};
mod joystick;
use joystick::Joystick;
use zerocopy::AsBytes;

type PilotJoystick<'a> = Joystick<'a, GpioPin<0>, GpioPin<1>>;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

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
    Output::new(io.pins.gpio8, Level::High);

    let channel = mk_static!(PubSubChannel::<NoopRawMutex, (u16, u16), 4, 1, 1>, PubSubChannel::<NoopRawMutex, (u16, u16), 4, 1, 1>::new());
    let mut sub0 = channel.subscriber().unwrap();
    let pub0 = mk_static!(Publisher<NoopRawMutex, (u16, u16), 4, 1, 1>, channel.publisher().unwrap());
    let joystick = mk_static!(PilotJoystick, Joystick::new(io.pins.gpio0, io.pins.gpio1, peripherals.ADC1));
    spawner.spawn(measurements(pub0, joystick)).ok();
    loop {
        let (pin0_read, pin1_read) = sub0.next_message_pure().await;
        let (angle, power) = PilotJoystick::convert((pin1_read, pin0_read));

        let peer = match manager.fetch_peer(false) {
            Ok(peer) => peer,
            Err(_) => {
                if let Ok(peer) = manager.fetch_peer(true) {
                    peer
                } else {
                    continue;
                }
            }
        };
        let (angle, power) = PilotJoystick::convert((pin1_read, pin0_read));
        let payload: Option<RcCarControlViaEspReady> = RcCarControlViaEspReady::new(angle, power);
        match payload {
            Some(x) => {
                let mut sender = sender.lock().await;
                let _ = sender.send_async(&peer.peer_address, x.as_bytes()).await;
            },
            _ => {
                defmt::error!("Invalid data to peer {:?}", peer.peer_address);
            }
        }
    }
}

#[embassy_executor::task]
async fn measurements(publisher: &'static mut Publisher<'static, NoopRawMutex, (u16, u16), 4, 1, 1>, joystick: &'static mut PilotJoystick<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(40));
    loop {
        ticker.next().await;
        let (pin0_mv, pin1_mv) = joystick.read().unwrap();
        println!("ADC reading = {pin0_mv} : {pin1_mv}");
        publisher.publish_immediate((pin0_mv, pin1_mv));
    }
}

#[embassy_executor::task]
async fn listener(manager: &'static EspNowManager<'static>, mut receiver: EspNowReceiver<'static>) {
    loop {
        let r = receiver.receive_async().await;
        defmt::info!("Received {:?}", r.get_data());
        if r.info.dst_address == BROADCAST_ADDRESS {
            if !manager.peer_exists(&r.info.src_address) {
                manager
                    .add_peer(PeerInfo {
                        peer_address: r.info.src_address,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    })
                    .unwrap();
                defmt::info!("Added peer {:?}", r.info.src_address);
            }
        }
    }
}

