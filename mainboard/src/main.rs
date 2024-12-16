#![no_std]
#![no_main]

mod motor_driver;
mod motor_manager;
mod ble_terminal;
mod communication_manager;
mod cli;
mod pwm_split;
use common::{
    mk_static,
    communication::RcCarControlViaEspReady
};
use embassy_executor::Spawner;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_backtrace as _;
use esp_hal::{
    reset::software_reset,
    gpio::{AnyPin, Io, GpioPin},
    ledc::{LSGlobalClkSource, Ledc},
    prelude::*,
    rng::Rng,
    timer::{timg::TimerGroup, systimer::{SystemTimer, Target}},
};
use esp_wifi::{
    esp_now::{EspNowManager, EspNowSender},
    init, EspWifiInitFor, EspWifiInitialization,
};
use motor_driver::MotorCurrentObserver;
use pwm_split::PwmChannel;
use motor_manager::motor_manager;
use communication_manager::{broadcaster, listener};

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(144 * 1024);
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(systimer.alarm0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = mk_static!(EspWifiInitialization, init(
        EspWifiInitFor::WifiBle,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap());

    let ledc = mk_static!(Ledc<'static>, Ledc::new(peripherals.LEDC));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

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

    let front_power = motor_driver::PowerMotorInternals::new(
        [AnyPin::from(io.pins.gpio3.degrade()), AnyPin::from(io.pins.gpio2.degrade())],
        [PwmChannel::Channel0, PwmChannel::Channel1],
    );

    let back_power = motor_driver::PowerMotorInternals::new(
        [AnyPin::from(io.pins.gpio8.degrade()), AnyPin::from(io.pins.gpio10.degrade())],
        [PwmChannel::Channel2, PwmChannel::Channel3],
    );

    let serwo =
        motor_driver::SerwoMotorInternals::new(AnyPin::from(io.pins.gpio7.degrade()), PwmChannel::Channel4);
    let driver = mk_static!(
        motor_driver::MotorDescriptorBoundle,
        motor_driver::MotorDescriptorBoundle::new(front_power, back_power, serwo)
    );

    let wifi = peripherals.WIFI;
    let esp_now = esp_wifi::esp_now::EspNow::new(init, wifi).unwrap();
    let (manager, sender, receiver) = esp_now.split();

    let manager = mk_static!(EspNowManager<'static>, manager);
    let sender = mk_static!(
        EspNowSender<'static>,
        sender
    );
    let current_observer = mk_static!(MotorCurrentObserver<'static, GpioPin<0>, GpioPin<1>>,
        MotorCurrentObserver::new(io.pins.gpio0, io.pins.gpio1, peripherals.ADC1));

    spawner.must_spawn(listener(pub0, manager, receiver));
    spawner.must_spawn(broadcaster(sender));
    spawner.must_spawn(motor_manager(sub0, splitted, driver, current_observer));
    spawner.must_spawn(ble_terminal::ble_driver(init, peripherals.BT));
    spawner.must_spawn(cli::cli_driver());
}

#[no_mangle]
pub extern "Rust" fn custom_halt() -> ! {
    loop {
        software_reset();
    }
}

