#![no_std]
#![no_main]

mod motor_control;
mod pwm_split;
use defmt::println;
use common::{
    mk_static,
    safe_types::Angle,
    communication::{RcCarControlViaEspReady, COMUNICATION_PERIOD_MS}
};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks},
    gpio::{any_pin::AnyPin, Io, OutputPin, GpioPin},
    ledc::{LSGlobalClkSource, Ledc},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize, EspWifiInitFor,
};
use motor_control::{Direction, MotorError, MotorDriver, MotorCurrentObserver, MotorDescriptorBoundle};
use num_traits::float::FloatCore;
use portable_atomic::{AtomicBool, Ordering};
use pwm_split::PwmChannel;

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

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, clocks);
    esp_hal_embassy::init(clocks, timer_group0);

    let (manager, sender, receiver) = esp_now.split();
    let manager = mk_static!(EspNowManager<'static>, manager);
    let sender = mk_static!(
        Mutex::<NoopRawMutex, EspNowSender<'static>>,
        Mutex::<NoopRawMutex, _>::new(sender)
    );

    let channel = mk_static!(
        PubSubChannel::<NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>,
        PubSubChannel::<NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>::new()
    );
    let sub0 = mk_static!(Subscriber<NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>, channel.subscriber().unwrap());
    let pub0 = mk_static!(Publisher<NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>, channel.publisher().unwrap());

    let splitted = mk_static!(
        pwm_split::SplitedPwm<'static>,
        pwm_split::SplitedPwm::new(ledc).expect("Splitt setup failed")
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
    let current_observer = mk_static!(MotorCurrentObserver<'static, GpioPin<0>, GpioPin<1>>,
        MotorCurrentObserver::new(io.pins.gpio0, io.pins.gpio1, peripherals.ADC1));

    spawner.spawn(listener(pub0, manager, receiver)).ok();
    spawner.spawn(broadcaster(sender)).ok();
    spawner.spawn(motor_driver(sub0, splitted, driver, current_observer)).ok();
    println!("Yo!");
}

fn handle_incoming_control_message<F: Fn() -> Result<(), MotorError>, O: OutputPin>(
    motor_driver: &MotorDriver<O>,
    control: RcCarControlViaEspReady,
    stop_car: F
) {
    motor_driver
        .drive_serwo(control.turn)
        .or_else(|_| stop_car())
        .expect("Motor servo setup failed");

    let power = control.power();
    let direction = match power {
        p if p < 0.0 => Direction::Backward,
        p if p == 0.0 => Direction::Stop,
        _ => Direction::Forward,
    };

    motor_driver
        .drive_motor(direction, power.abs() / 100.0)
        .or_else(|_| stop_car())
        .expect("Power motor setup failed");
}

#[embassy_executor::task]
async fn motor_driver(
    subscriber: &'static mut Subscriber<'static, NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>,
    splitted: &'static mut pwm_split::SplitedPwm<'static>,
    motor_controller: &'static mut MotorDescriptorBoundle<AnyPin<'static>>,
    current_observer: &'static mut MotorCurrentObserver<'static, GpioPin<0>, GpioPin<1>>
) {
    let mut timeout = Ticker::every(Duration::from_millis(COMUNICATION_PERIOD_MS * 3));
    let motor_driver = motor_controller
        .setup_driver(splitted)
        .expect("Motor driver setup failed");
    let stop_car = || -> Result<(), MotorError> {
        motor_driver.drive_motor(Direction::Stop, 0.0f32)?;
        match Angle::new(0) {
            Some(x) => motor_driver.drive_serwo(x),
            None => unreachable!(),
        }
    };
    loop {
        let control = select(subscriber.next_message_pure(), timeout.next()).await;
        match control {
            Either::First(control) => {
                if !current_observer.current_in_range() {
                    stop_car().expect("Motor driver with current outside range cannot stop");
                    continue;
                }
                handle_incoming_control_message(&motor_driver, control, stop_car);
                timeout.reset();
            },
            Either::Second(_) => {
                stop_car().expect("Stopping the car failed");
            }
        }
    }
}

#[embassy_executor::task]
async fn broadcaster(sender: &'static Mutex<NoopRawMutex, EspNowSender<'static>>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    while !PEER_FOUND.load(Ordering::Relaxed) {
        ticker.next().await;

        let mut sender = sender.lock().await;
        let _ = sender.send_async(&BROADCAST_ADDRESS, b"Hello.").await;
    }
}

fn safe_copy_from_slice<T: Copy>(dst: &mut [T], src: &[T]) -> usize {
    let copy_len = core::cmp::min(dst.len(), src.len());
    dst[..copy_len].copy_from_slice(&src[..copy_len]);
    copy_len
}

#[embassy_executor::task]
async fn listener(
    publisher: &'static mut Publisher<'static, NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>,
    manager: &'static EspNowManager<'static>,
    mut receiver: EspNowReceiver<'static>,
) {
    const CONTROL_DATA_SIZE: usize = core::mem::size_of::<RcCarControlViaEspReady>();
    let mut array = [0u8; CONTROL_DATA_SIZE];

    loop {
        let r = receiver.receive_async().await;
        defmt::info!("Control data received {:?}", r.get_data());
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

        let bytes_count = safe_copy_from_slice(&mut array, r.get_data());
        if bytes_count < CONTROL_DATA_SIZE {
            defmt::warn!("Received too small message");
            continue;
        }

        let control_data: RcCarControlViaEspReady = unsafe { core::mem::transmute(array) };

        if !control_data.crc_correct() {
            defmt::warn!("CRC8 incorrect!");
            continue;
        }

        publisher.publish_immediate(control_data);
    }
}
