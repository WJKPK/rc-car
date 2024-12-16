use common::{
    safe_types::Angle,
    communication::{RcCarControlViaEspReady, COMUNICATION_PERIOD_MS}
};
use embassy_futures::select::{select, Either};
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, CriticalSectionRawMutex};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::gpio::GpioPin;
use num_traits::float::FloatCore;
use crate::pwm_split::SplitedPwm;
use crate::motor_driver::{MotorError, MotorDriver, MotorDescriptorBoundle, MotorCurrentObserver};
pub use crate::motor_driver::{Direction, MiliAmperes};

#[derive(Debug, ufmt::derive::uDebug, Default, Clone, Copy, PartialEq)]
pub struct MotorsState {
    direction: Direction,
    power: i32,
    current: Option<(MiliAmperes, MiliAmperes)>
}

pub type MotorStatusNotificator = PubSubChannel::<CriticalSectionRawMutex, MotorsState, 4, 1, 1>;
pub type MotorStatusSubscriber<'a> = Subscriber::<'a, CriticalSectionRawMutex, MotorsState, 4, 1, 1>;

static MOTOR_STATUS: MotorStatusNotificator = MotorStatusNotificator::new();

pub fn get_status_subscriber<'a>() -> Option<MotorStatusSubscriber<'a>> {
    if let Ok(subscriber) = MOTOR_STATUS.subscriber() {
        return Some(subscriber);
    }
    None
}

fn handle_incoming_control_message(
    motor_driver: &MotorDriver,
    control: RcCarControlViaEspReady,
    current_observer: &mut MotorCurrentObserver<'static, GpioPin<0>, GpioPin<1>>,
) {
    motor_driver
        .drive_serwo(control.turn)
        .or_else(|_| stop_car(&motor_driver))
        .expect("Motor servo setup failed");

    let power = control.power();
    let direction = match power {
        p if p < 0.0 => Direction::Backward,
        p if p == 0.0 => Direction::Stop,
        _ => Direction::Forward,
    };

    motor_driver
        .drive_motor(direction, power.abs())
        .or_else(|_| stop_car(&motor_driver))
        .expect("Power motor setup failed");

   let mut motor_current: Option<(MiliAmperes, MiliAmperes)> = None;

   if let Ok(results) = current_observer.read() {
       motor_current = Some(results);
   }
   if let Ok(publisher) = MOTOR_STATUS.publisher() {
        let msg = MotorsState {
            direction,
            power: power as i32,
            current: motor_current
        };
        publisher.publish_immediate(msg);
   }

}

fn stop_car(motor_driver: &MotorDriver) -> Result<(), MotorError> {
    motor_driver.drive_motor(Direction::Stop, 0.0f32)?;
    match Angle::new(0) {
        Some(x) => motor_driver.drive_serwo(x),
        None => unreachable!(),
    }
}

#[embassy_executor::task]
pub async fn motor_manager(
    subscriber: &'static mut Subscriber<'static, NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>,
    splitted: &'static mut SplitedPwm<'static>,
    motor_driver: &'static mut MotorDescriptorBoundle,
    current_observer: &'static mut MotorCurrentObserver<'static, GpioPin<0>, GpioPin<1>>
) {
    let mut timeout = Ticker::every(Duration::from_millis(COMUNICATION_PERIOD_MS * 3));
    let motor_driver = motor_driver
        .setup_driver(splitted)
        .expect("Motor driver setup failed");
    loop {
        let control = select(subscriber.next_message_pure(), timeout.next()).await;
        match control {
            Either::First(control) => {
                handle_incoming_control_message(&motor_driver, control, current_observer);
                timeout.reset();
            },
            Either::Second(_) => {
                stop_car(&motor_driver).expect("Stopping the car failed");
            }
        }
    }
}

