use embedded_cli::Command;
use embedded_cli::cli::CliBuilder;
use ufmt::uwrite;
use common::mk_static;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_executor::Spawner;
use embassy_futures::select::{select_array, select, Either};
use crate::motor_manager::{self, MotorStatusSubscriber, MotorsState};
use heapless::String;
use strum::EnumCount;

use crate::ble_terminal::{Writer, CLI_RX_CHANNEL, CLI_TX_CHANNEL};

#[derive(Command, Clone)]
enum Base<'a> {
    /// Collect motor current readings 
    MotorCurrent {
        /// enable/disable flag 
        switch: &'a str,
    },
}

type CliChannel<'a> = PubSubChannel::<NoopRawMutex, CliCallbackType, 4, 1, 1>;
type CliSubscriber<'a> = Subscriber<'a, NoopRawMutex, CliCallbackType, 4, 1, 1>;
type CliPublisher<'a> = Publisher<'a, NoopRawMutex, CliCallbackType, 4, 1, 1>;

const CLI_MAX_VALID_COMMAND_LEN: usize = 2;
#[derive(Debug, Clone, PartialEq, EnumCount)]
enum CliCallbackType {
    MotorCurrentCallback(heapless::String<CLI_MAX_VALID_COMMAND_LEN>)
}

async fn do_work() -> () {
    ()
}

async fn do_work_2() -> () {
    ()
}

#[derive(EnumCount)]
enum CliAction<F> {
    MotorCurrent(F),
}

fn async_wrapper<F>(f: F) -> CliAction<F>
where
    F: core::future::Future<Output = ()>,
{
    CliAction::MotorCurrent(async move {
        f.await;
    })
}

#[embassy_executor::task]
pub async fn cli_callback_performer(command_subscriber: &'static mut CliSubscriber<'static>, mut writer: Writer) {
    let mut current_subscriber = motor_manager::get_status_subscriber().unwrap();
    let cli_action = [async_wrapper(do_work()), async_wrapper(do_work())];
    loop {
        let control = select(command_subscriber.next_message_pure(), current_subscriber.next_message_pure()).await;
        match control {
            Either::First(control_message) => {
                match control_message {
                    CliCallbackType::MotorCurrentCallback(switch) => {
                        match switch.as_str() {
                            "-e" => {
                                let _ = uwrite!(writer, "Motor current monitoring enabled");
                            }
                            "-d" => {
                                let _ = uwrite!(writer, "Motor current monitoring disabled");
                            }
                            _ => {
                                let _ = uwrite!(writer, "Invalid switch. Use -e to enable or -d to disable");
                            }
                        }
                    }
                }
            },
            Either::Second(current_message) => {
            }
        }
    }
}

#[embassy_executor::task]
pub async fn cli_driver() {
    let writer = Writer::new(&CLI_TX_CHANNEL);
    let receiver = CLI_RX_CHANNEL.receiver();

    let (command_buffer, history_buffer) = unsafe {
            static mut COMMAND_BUFFER: [u8; 32] = [0; 32];
            static mut HISTORY_BUFFER: [u8; 32] = [0; 32];
            (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
        };
    let cli = CliBuilder::default()
        .writer(writer.clone())
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build();

    let mut cli = cli.unwrap();
    let channel = mk_static!(
        CliChannel,
        CliChannel::new()
    );
    let sub0 = mk_static!(CliSubscriber, channel.subscriber().unwrap());
    let pub0 = mk_static!(CliPublisher, channel.publisher().unwrap());

    let spawner = Spawner::for_current_executor().await;
    spawner.must_spawn(cli_callback_performer(sub0, writer));

    loop {
        let received_bytes = receiver.receive().await;
        for b in received_bytes {
            let _ = cli.process_byte::<Base, _>(
            b,
            &mut Base::processor(|cli, command| {
                match command {
                    Base::MotorCurrent { switch } => {
                        let _ = pub0.try_publish(CliCallbackType::MotorCurrentCallback(heapless::String::try_from(switch).unwrap_or(String::new())));
                    }
                }
                Ok(())
            }),
            );
        }
    }
}
