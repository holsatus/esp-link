#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex as Mut, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Io, Level, Output},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{config::Config, UartRx, UartTx},
    Async,
};
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize, EspWifiInitFor,
};

#[collapse_debuginfo(yes)]
macro_rules! println {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "println")]
            ::esp_println::println!($s, $($x)*);
            #[cfg(not(feature = "println"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! make_static {
    ($t:ty,$val:expr) => {{
        use static_cell::StaticCell as SC;
        static STATIC_CELL: SC<$t> = SC::new();
        #[deny(unused_attributes)]
        STATIC_CELL.init(($val))
    }};
}

// Insert the MAC addresses of the two devices to pair
static PAIR: [[u8; 6]; 2] = [
    [154, 244, 171, 102, 66, 24],
    [10, 58, 242, 183, 104, 76]
];

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    #[cfg(feature = "println")]
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let esp_now = esp_wifi::esp_now::EspNow::new(&init, peripherals.WIFI).unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    // This may be wrong if we are not on esp32
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let (manager, snd, rcv) = esp_now.split();
    let manager = make_static!(EspNowManager<'static>, manager);

    let mut addr = [0u8; 6];
    esp_wifi::wifi::get_ap_mac(&mut addr);

    println!("MY: ADDR: {:?}", addr);

    let mut peer = if PAIR[0] == addr { PAIR[1] } else { PAIR[0] };
    peer[0] -= 2; // Tell me WHY!?!

    if !manager.peer_exists(&peer) && peer != addr {
        let peer_info = PeerInfo {
            peer_address: peer,
            lmk: None,
            channel: None,
            encrypt: false,
        };
        manager.add_peer(peer_info).unwrap();
    }

    // manager.set_pmk(&PMK).unwrap();

    let config = Config::default().rx_timeout(Some(16));

    #[cfg(not(feature = "usb-serial"))]
    let uart = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART1,
        config,
        io.pins.gpio14,
        io.pins.gpio15,
    )
    .unwrap();

    #[cfg(feature = "usb-serial")]
    let uart = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART0,
        config,
        io.pins.gpio3,
        io.pins.gpio1,
    )
    .unwrap();

    let (tx, rx) = uart.split();

    
    let timeout = make_static!(Signal<Mut, bool>, Signal::new());
    
    spawner.must_spawn(esp_tx(snd, tx));
    spawner.must_spawn(esp_rx(peer, rcv, rx, timeout));
    
    // Define LED used to indicate that we are paired
    let mut led = Output::new(io.pins.gpio2, Level::Low);
    loop {
        if timeout.wait().await {
            for _ in 0..2 {
                led.set_high();
                Timer::after_millis(50).await;
                led.set_low();
                Timer::after_millis(50).await;
            }
            Timer::after_millis(500).await;
        } else {
            led.set_high();
        }
    }
}

#[cfg(not(feature = "usb-serial"))]
type UARTX = esp_hal::peripherals::UART1;

#[cfg(feature = "usb-serial")]
type UARTX = esp_hal::peripherals::UART0;

const HEARTBEAT: &[u8] = b"bridge-heartbeat";
const INTERVAL: Duration = Duration::from_hz(4);

#[embassy_executor::task]
async fn esp_tx(mut sender: EspNowSender<'static>, mut uart_rx: UartRx<'static, UARTX, Async>) {
    let mut buffer = [0u8; 1024 * 4];

    // Drain the UART RX FIFO completely
    while uart_rx.drain_fifo(&mut buffer) > 0 {}

    loop {
        match select(
            uart_rx.read_async(&mut buffer),
            Timer::after(INTERVAL)
        ).await {

            // Successfully received a message from UART, send it via esp-now
            Either::First(Ok(n)) => {
                if let Err(e) = sender.send_async(&BROADCAST_ADDRESS, &buffer[..n]).await {
                    println!("Error sending message via esp-now: {:?}", e);
                } else {
                    println!("Sent message of {} bytes via esp-now", n);
                }
            }

            // Error receiving UART message, skip this iteration
            Either::First(Err(e)) => println!("Error receiving Uart message: {:?}", e),

            // No message received within INTERVAL, send a heartbeat
            Either::Second(()) => {
                if let Err(e) = sender.send_async(&BROADCAST_ADDRESS, HEARTBEAT).await {
                    println!("Error sending heartbeat via esp-now: {:?}", e);
                } else {
                    println!("Broadcasted heartbeat via esp-now");
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn esp_rx(
    peer: [u8; 6],
    mut receiver: EspNowReceiver<'static>,
    mut uart_tx: UartTx<'static, UARTX, Async>,
    timeout: &'static Signal<Mut, bool>,
) {

    // Drain the ESPNOW RX FIFO completely
    while receiver.receive().is_some() {}

    loop {
        match select(
            receiver.receive_async(),
            Timer::after(INTERVAL * 4)
        ).await {

            // Successfully received a message from esp-now
            Either::First(r) => {

                // Reset the RX timeout signal
                timeout.signal(false);

                // Check if the message is from the expected peer
                if r.info.src_address != peer {
                    println!("Received from unknown peer: {:?}", r.info.src_address);
                    continue
                }

                // .. and it is not a heartbeat message
                if r.get_data() == HEARTBEAT {
                    println!("Received heartbeat from {:?}", r.info.src_address);
                    continue
                }

                // Send the message to UART
                if let Err(e) = uart_tx.write_async(r.get_data()).await {
                    println!("Error writing to UART: {:?}", e);
                } else {
                    println!("Received message: {:?}", r.get_data());
                }
                if let Err(e) = uart_tx.flush_tx() {
                    println!("Error flushing UART Tx: {:?}", e);
                }
            }

            // Signal that we experienced an RX timeout
            Either::Second(()) => timeout.signal(true),
        }
    }
}
