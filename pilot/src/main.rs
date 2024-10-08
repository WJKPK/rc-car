#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use embassy_sync::pubsub::{Publisher, PubSubChannel};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{GpioPin, Io, Output, Level},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize,
    EspWifiInitFor,
};
use common::{
    mk_static,
    communication::{RcCarControlViaEspReady,COMUNICATION_PERIOD_MS},
    rc_filter::RCLowPassFilter
};
mod joystick;
use joystick::Joystick;
use zerocopy::AsBytes;

type PilotJoystick<'a> = Joystick<'a, GpioPin<0>, GpioPin<1>>;

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

    let sampling_frequency_f32: f32 = 1000.0/COMUNICATION_PERIOD_MS as f32;
    let power_filter = RCLowPassFilter::new(sampling_frequency_f32, 500.0);
    let angle_filter = RCLowPassFilter::new(sampling_frequency_f32, 50.0);

    loop {
        let (pin0_read, pin1_read) = sub0.next_message_pure().await;
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
        let (angle, power) = PilotJoystick::convert((pin1_read, pin0_read), power_filter, angle_filter);
        let payload: Option<RcCarControlViaEspReady> = RcCarControlViaEspReady::new(angle, power);
        if let Some(payload) = payload {
            let _ = sender.lock().await.send_async(&peer.peer_address, payload.as_bytes()).await;
        }
    }
}

#[embassy_executor::task]
async fn measurements(publisher: &'static mut Publisher<'static, NoopRawMutex, (u16, u16), 4, 1, 1>, joystick: &'static mut PilotJoystick<'static>) {

    let mut ticker = Ticker::every(Duration::from_millis(COMUNICATION_PERIOD_MS));
    loop {
        ticker.next().await;
        if let Ok((pin0_mv, pin1_mv)) = joystick.read() {
            publisher.publish_immediate((pin0_mv, pin1_mv));
        };
    }
}

#[embassy_executor::task]
async fn listener(manager: &'static EspNowManager<'static>, mut receiver: EspNowReceiver<'static>) {
    loop {
        let r = receiver.receive_async().await;
        if r.info.dst_address == BROADCAST_ADDRESS {
            if !manager.peer_exists(&r.info.src_address) {
                match manager
                    .add_peer(PeerInfo {
                        peer_address: r.info.src_address,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    }) {
                        Ok(_) => defmt::info!("Added peer {:?}", r.info.src_address),
                        Err(_) => defmt::error!("Add peer failed"),
                }
            }
        }
    }
}

