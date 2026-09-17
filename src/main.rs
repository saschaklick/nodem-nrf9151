#![no_std]
#![no_main]

extern crate alloc;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_nrf::buffered_uarte::{self, BufferedUarte};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::twim::{self, Frequency, Twim};
use embassy_nrf::uarte;
use embassy_nrf::{bind_interrupts, peripherals};
use embedded_alloc::LlffHeap as Heap;
use panic_probe as _;
use static_cell::StaticCell;

use ssd1306_embassy::Ssd1306;

mod nodem;
mod oled;
mod uart;

use nodem::NodemState;

bind_interrupts!(struct Irqs {
    // Each SERIALn is just a hardware block that can drive any protocol on
    // any GPIO pin via PSEL - which physical pins end up as I2C vs. UART is
    // decided entirely by what's passed to `Twim::new`/`BufferedUarte::new`
    // below, not by which SERIALn number is used for which.
    SERIAL0 => buffered_uarte::InterruptHandler<peripherals::SERIAL0>;
    SERIAL1 => twim::InterruptHandler<peripherals::SERIAL1>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

const HEAP_SIZE: usize = 32 * 1024;
static HEAP_MEM: StaticCell<[u8; HEAP_SIZE]> = StaticCell::new();

const UART_RX_BUF_LEN: usize = 1024;
const UART_TX_BUF_LEN: usize = 1024;
static UART_RX_BUF: StaticCell<[u8; UART_RX_BUF_LEN]> = StaticCell::new();
static UART_TX_BUF: StaticCell<[u8; UART_TX_BUF_LEN]> = StaticCell::new();

static NODEM_STATE: StaticCell<nodem::NodemMutex> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    embassy_nrf::pac::NVMC.icachecnf().write(|w| w.set_cacheen(true));

    let heap_mem = HEAP_MEM.uninit();
    unsafe { HEAP.init(heap_mem.as_ptr() as usize, HEAP_SIZE) };
    
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(nrf_config);

    let mut led = Output::new(p.P0_00, Level::Low, OutputDrive::Standard);

    info!("set up i2c ");
    let mut config = twim::Config::default();
    config.frequency = Frequency::K400;
    
    let i2c = Twim::new(p.SERIAL1, Irqs, p.P0_11, p.P0_12, config, &mut []);
    let display = Ssd1306::new(i2c);

    info!("set up uart ");
    // Plain `Uarte::read` re-arms EasyDMA for every single byte, and the gap
    // between one byte's ENDRX and the next's STARTRX is enough to overrun
    // an incoming line typed/pasted at any real speed - `BufferedUarte` runs
    // a continuously double-buffered DMA reception (via `TIMER0`/2 PPI
    // channels) into these buffers instead, so nothing gets dropped between
    // reads.
    let rx_buf = UART_RX_BUF.init([0u8; UART_RX_BUF_LEN]);
    let tx_buf = UART_TX_BUF.init([0u8; UART_TX_BUF_LEN]);
    let uart = BufferedUarte::new(
        p.SERIAL0,
        p.TIMER0,
        p.PPI_CH0,
        p.PPI_CH1,
        p.PPI_GROUP0,
        p.P0_28,
        p.P0_29,
        Irqs,
        uarte::Config::default(),
        rx_buf,
        tx_buf,
    );
    let (uart_rx, uart_tx) = uart.split();

    let state = NODEM_STATE.init(nodem::NodemMutex::new(NodemState::new()));

    spawner.spawn(nodem::nodem_task(state, uart_tx).unwrap());
    spawner.spawn(oled::oled_task(display, state).unwrap());
    spawner.spawn(uart::uart_task(uart_rx, state).unwrap());

    loop {        
        led.set_high();
        Timer::after_millis(500).await;        
        led.set_low();
        Timer::after_millis(500).await;
        info!("loop");
    }
}
