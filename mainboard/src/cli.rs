use embedded_cli::Command;
use embedded_cli::cli::CliBuilder;
use ufmt::uwrite;
use common::mk_static;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use crate::motor_manager::{self, MotorsState};
use heapless::String;
use strum::EnumCount;
use core::mem::discriminant;
use crate::ble_terminal::{Writer, CLI_RX_CHANNEL, CLI_TX_CHANNEL, get_logger_writter};

#[derive(Command, Clone)]
enum Diagnostics<'a> {
    /// Collect motor current readings 
    MotorStatus {
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
    MotorStatusCallback(heapless::String<CLI_MAX_VALID_COMMAND_LEN>)
}

struct CliBackgroundHandlerFilter {
    filter: heapless::Vec<CliBackgroundHandler, {CliCallbackType::COUNT}>
}

impl CliBackgroundHandlerFilter {
    fn new() -> Self {
        CliBackgroundHandlerFilter {
            filter: heapless::Vec::new()
        }
    }
    fn filter_action(&self, current_action: CliBackgroundHandler) -> Option<CliBackgroundHandler> {
        if self.filter.iter().any(|filter_item| discriminant(filter_item) == discriminant(&current_action)) {
            Some(current_action)
        } else {
            None
        }
    }
    fn remove_variant(&mut self, variant_to_remove: CliBackgroundHandler) {
        self.filter.retain(|item| discriminant(item) != discriminant(&variant_to_remove));
    }
    fn add_varitant(&mut self, variant_to_add: CliBackgroundHandler) -> Result<(), CliBackgroundHandler> {
        self.filter.push(variant_to_add)
    }
}

#[derive(Debug, Clone, PartialEq)]
enum CliBackgroundHandler {
    MotorStatus(MotorsState)
}

impl CliBackgroundHandler {
    async fn get_next(filter: &CliBackgroundHandlerFilter) -> Option<Self> {
        let mut motor_status_subscriber = motor_manager::get_status_subscriber().unwrap();
        filter.filter_action(CliBackgroundHandler::MotorStatus(motor_status_subscriber.next_message_pure().await))
    }
}

#[embassy_executor::task]
pub async fn cli_callback_performer(command_subscriber: &'static mut CliSubscriber<'static>, mut writer: Writer) {
    let mut logger = get_logger_writter();
    let mut filter = CliBackgroundHandlerFilter::new();
    loop {
        let control = select(command_subscriber.next_message_pure(), CliBackgroundHandler::get_next(&filter)).await;
        match control {
            Either::First(control_message) => {
                match control_message {
                    CliCallbackType::MotorStatusCallback(switch) => {
                        match switch.as_str() {
                            "-e" => {
                                let _ = uwrite!(writer, "Motor current monitoring enabled");
                                let _ = filter.add_varitant(CliBackgroundHandler::MotorStatus(MotorsState::default()));
                            }
                            "-d" => {
                                let _ = uwrite!(writer, "Motor current monitoring disabled");
                                filter.remove_variant(CliBackgroundHandler::MotorStatus(MotorsState::default()));
                            }
                            _ => {
                                let _ = uwrite!(writer, "Invalid switch. Use -e to enable or -d to disable");
                            }
                        }
                    }
                }
            },
            Either::Second(message) => {
                if let Some(message) = message {
                    match message {
                        CliBackgroundHandler::MotorStatus(status) => {let _ = uwrite!(logger, "{:?}", status);}
                    }
                }
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
            let _ = cli.process_byte::<Diagnostics, _>(
            b,
            &mut Diagnostics::processor(|_cli, command| {
                match command {
                    Diagnostics::MotorStatus { switch } => {
                        let _ = pub0.try_publish(CliCallbackType::MotorStatusCallback(heapless::String::try_from(switch).unwrap_or(String::new())));
                    }
                }
                Ok(())
            }),
            );
        }
    }
}
