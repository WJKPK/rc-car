extern crate alloc;
use alloc::vec::Vec;
use alloc::str;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_backtrace as _;
use esp_hal::{
    peripherals::BT,
    time, };
use esp_wifi::{
    ble::controller::asynch::BleConnector, EspWifiInitialization,
};
use bleps::{
    ad_structure::{
        create_advertising_data,
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use core::convert::Infallible;
use embassy_futures::select::{select, Either};
use embassy_sync::channel::{Channel, Sender};

pub type TextOverBleBuffer = Channel::<CriticalSectionRawMutex, Vec<u8>, CLI_CHANNEL_CAPACITY>;
const CLI_CHANNEL_CAPACITY: usize = 12;
pub static CLI_TX_CHANNEL: TextOverBleBuffer = TextOverBleBuffer::new();
pub static LOG_TX_CHANNEL: TextOverBleBuffer = TextOverBleBuffer::new();
pub static CLI_RX_CHANNEL: TextOverBleBuffer = TextOverBleBuffer::new();

pub fn get_logger_writter() -> Writer {
    Writer::new(&LOG_TX_CHANNEL)
}

#[macro_export]
macro_rules! make_notification_closure {
    (
        $channel:expr,
        $handle:expr,
        $prefix:expr,
        $timeout_ms:expr
    ) => {
        || {
            async {
                let receive_channel = $channel.receiver();
                let mut collect_buffer: Vec<u8> = Vec::new();
                loop {
                    match select(
                        receive_channel.receive(),
                        embassy_time::Timer::after(embassy_time::Duration::from_millis($timeout_ms)),
                    )
                    .await
                    { Either::First(mut message) => {
                            collect_buffer.append(&mut message);
                        }
                        Either::Second(_) => {
                            if collect_buffer.is_empty() {
                                return None;
                            }
                            defmt::info!(
                                "{}: {}",
                                $prefix,
                                core::str::from_utf8(collect_buffer.as_slice()).unwrap()
                            );
                            return Some(NotificationData::new($handle, collect_buffer.as_slice()));
                        }
                    }
                }
            }
        }
    };
}

#[derive(Clone)]
pub struct Writer {
    sender: Sender<'static, CriticalSectionRawMutex, Vec<u8>, CLI_CHANNEL_CAPACITY>
}

impl Writer {
    pub fn new(buffer: &'static TextOverBleBuffer) -> Self {
        Writer{sender: buffer.sender()}
    }
}

impl embedded_io::ErrorType for Writer {
    type Error = Infallible;
}

impl embedded_io::Write for Writer {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
       let line = Vec::from(buf);
       let _ = self.sender.try_send(line);
       Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl ufmt::uWrite for Writer {
    type Error = Infallible;

    fn write_str(&mut self, buf: &str) -> Result<(), Self::Error> {
       let line = Vec::from(buf);
       let _ = self.sender.try_send(line);
       Ok(())
    }
}

#[embassy_executor::task]
pub async fn ble_driver(init: &'static EspWifiInitialization, mut bluetooth: BT) {
    let now = || time::now().duration_since_epoch().to_millis();
    let connector = BleConnector::new(init, &mut bluetooth);
    let mut ble = Ble::new(connector, now);
    loop {
        let _ = ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                AdStructure::CompleteLocalName(esp_hal::chip!()),
            ])
            .expect("Failed to create advertising data")
        ).await;
        let _ = ble.cmd_set_le_advertise_enable(true).await;
        defmt::info!("started advertising");
    
        let mut rec_serial = |_offset: usize, data: &[u8]| {
            let sender = CLI_RX_CHANNEL.sender();
            let line = Vec::from(data);
            let _ = sender.try_send(line);
        };
    
        let mut sending_serial = |_offset: usize, data: &mut [u8]| {
            let receiver = CLI_TX_CHANNEL.receiver();
            if let Ok(line) = receiver.try_receive() {
                data[..].copy_from_slice(line.as_slice());
                return line.len();
            }
            0
        };

        let mut sending_log_serial = |_offset: usize, data: &mut [u8]| {
            let receiver = LOG_TX_CHANNEL.receiver();
            if let Ok(line) = receiver.try_receive() {
                data[..].copy_from_slice(line.as_slice());
                return line.len();
            }
            0
        };
   
        gatt!([service {
            uuid: "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
            characteristics: [
                    characteristic {
                    name: "cli_rx_line",
                    uuid: "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
                    write: rec_serial,
                },
                characteristic {
                    name: "cli_tx_line",
                    uuid: "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
                    notify: true,
                    read: sending_serial,
                },
                characteristic {
                    name: "log_tx_line",
                    uuid: "6e400004-b5a3-f393-e0a9-e50e24dcca9e",
                    notify: true,
                    read: sending_log_serial,
                },
            ],
        },]);
    
        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);
        let cli_notifier = make_notification_closure!(
            CLI_TX_CHANNEL,
            cli_tx_line_handle,
            "CLI",
            25
        );
        
        let log_notifier = make_notification_closure!(
            LOG_TX_CHANNEL,
            log_tx_line_handle,
            "LOG",
            25
        );
        let mut notifier = || async {
            loop {
               if let Some(result) = cli_notifier().await {
                   return result;
               };
               if let Some(result) = log_notifier().await {
                   return result;
               };
            }
        };
        srv.run(&mut notifier).await.expect("Failed to run notifier!");
    }
}

