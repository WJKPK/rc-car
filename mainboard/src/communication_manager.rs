use common::communication::RcCarControlViaEspReady;
use embassy_sync::pubsub::Publisher;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_wifi::esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS};
use portable_atomic::{AtomicBool, Ordering};

static PEER_FOUND: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
pub async fn broadcaster(sender: &'static mut EspNowSender<'static>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    while !PEER_FOUND.load(Ordering::Relaxed) {
        ticker.next().await;

        let _ = sender.send_async(&BROADCAST_ADDRESS, b"Hello.").await;
    }
}

#[embassy_executor::task]
pub async fn listener(
    publisher: &'static mut Publisher<'static, NoopRawMutex, RcCarControlViaEspReady, 4, 1, 1>,
    manager: &'static EspNowManager<'static>,
    mut receiver: EspNowReceiver<'static>,
) {
    loop {
        let r = receiver.receive_async().await;
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

        let control_data: Option<&RcCarControlViaEspReady> = RcCarControlViaEspReady::from_bytes(r.get_data());
        if let Some(data) = control_data {
            publisher.publish_immediate(*data);
        } else {
            defmt::warn!("Received invalid message!");
        }
    }
}

