#![no_std]
#![no_main]

extern crate panic_semihosting;

pub extern crate stm32f4xx_hal as hal;

use cortex_m;
use cortex_m_rt::{entry, exception};
use smoltcp;
use stm32f4_smoltcp::{EthernetDevice, RDes, RDesRing, TDes, TDesRing, ETH_BUF_SIZE};

use self::hal::gpio::Speed::VeryHigh;
use self::hal::prelude::*;
use self::hal::stm32::Interrupt;
use self::hal::*;

mod network;

const ETH_NUM_TD: usize = 4;
const ETH_NUM_RD: usize = 4;

static mut TICKS: u32 = 0;

static mut TDESRING: TDesRing = TDesRing {
    td: &mut [TDes::new(); ETH_NUM_TD],
    tbuf: &mut [[0; ETH_BUF_SIZE / 4]; ETH_NUM_RD],
    tdidx: 0,
};

static mut RDESRING: RDesRing = RDesRing {
    rd: &mut [RDes::new(); ETH_NUM_RD],
    rbuf: &mut [[0; ETH_BUF_SIZE / 4]; ETH_NUM_RD],
    rdidx: 0,
};

const PTP_MASTER: bool = false;

fn rcc_init(p: &mut stm32::Peripherals) {
    let rcc = &p.RCC;
    let syscfg = &p.SYSCFG;

    // Set SYSCFG early to RMII mode
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());
    syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

    // Set up peripheral clocks
    rcc.ahb1enr.modify(|_, w| {
        w.crcen()
            .enabled()
            .ethmacrxen()
            .enabled()
            .ethmactxen()
            .enabled()
            .ethmacen()
            .enabled()
    });
}

/// Set up the systick to provide a 1ms timebase
fn systick_init(syst: &mut stm32::SYST) {
    syst.set_reload((100_000_000 / 8) / 1000);
    syst.clear_current();
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
    syst.enable_interrupt();
    syst.enable_counter();
}

#[entry]
fn main() -> ! {
    let mut p = stm32::CorePeripherals::take().unwrap();
    let mut dp = stm32::Peripherals::take().unwrap();

    rcc_init(&mut dp);

    let rcc = dp.RCC;

    rcc.cfgr.modify(|_, w| w.mco2().hse());

    // Create ethernet driver
    let mac_addr = if PTP_MASTER {
        smoltcp::wire::EthernetAddress::from_bytes(&[0x02, 0x00, 0x00, 0x00, 0x00, 0x10])
    } else {
        smoltcp::wire::EthernetAddress::from_bytes(&[0x02, 0x00, 0x00, 0x00, 0x00, 0x11])
    };
    let mut ethdev = unsafe {
        EthernetDevice::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_DMA,
            dp.ETHERNET_PTP,
            &mut RDESRING,
            &mut TDESRING,
            0,
        )
    };

    // Constrain clock registers
    let rcc = rcc.constrain();

    // Configure clock to 100 MHz and freeze it
    let _ = rcc.cfgr.use_hse(26.mhz())
                    .sysclk(100.mhz())
                    .freeze();

    // Status LED
    let gpioe = dp.GPIOE.split();
    let mut led = gpioe.pe7.into_push_pull_output();
    led.set_low();
    // Traffic LED
    let mut traffic_led = gpioe.pe9.into_push_pull_output();
    traffic_led.set_low();

    // Configure ethernet related GPIO:
    // GPIOA 1, 2, 7
    // GPIOB 11, 12, 13
    // GPIOC 1, 4, 5
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let _a1 = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
    let _a2 = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
    let _a7 = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
    let _b11 = gpiob.pb11.into_alternate_af11().set_speed(VeryHigh);
    let _b12 = gpiob.pb12.into_alternate_af11().set_speed(VeryHigh);
    let _b13 = gpiob.pb13.into_alternate_af11().set_speed(VeryHigh);
    let _c1 = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
    let _c4 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
    let _c5 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);

    // Configure PTP PPS output
    let _pps = gpiob.pb5.into_alternate_af11().set_speed(VeryHigh);

    // MCO2
    let _mco2 = gpioc.pc9.into_alternate_af0().set_speed(VeryHigh);

    // Initialise ethernet
    ethdev.init(mac_addr.clone());

    // Enable ethernet interrupts
    ethdev.enable_rx_interrupt();
    ethdev.enable_tx_interrupt();

    // Wait until we have a link
    ethdev.block_until_link();

    // Change LED to high
    led.set_high();

    // Create network interface
    let ip_addr = if PTP_MASTER {
        smoltcp::wire::Ipv4Address::from_bytes(&[192, 168, 2, 20])
    } else {
        smoltcp::wire::Ipv4Address::from_bytes(&[192, 168, 2, 21])
    };
    let ip_cidr = smoltcp::wire::Ipv4Cidr::new(ip_addr, 24);
    let cidr = smoltcp::wire::IpCidr::Ipv4(ip_cidr);
    network::init(ethdev, mac_addr, cidr);

    // Start systicks at 1ms to update timestamp used for networking
    systick_init(&mut p.SYST);

    // Enable Ethernet peripheral interrupts
    let mut nvic = p.NVIC;
    nvic.enable(Interrupt::ETH);
    unsafe { nvic.set_priority(Interrupt::ETH, 1) };
    hal::stm32::NVIC::unpend(Interrupt::ETH);

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn ETH() {
    hal::stm32::NVIC::unpend(Interrupt::ETH);
    let dma = unsafe { &(*hal::stm32::ETHERNET_DMA::ptr()) };
    let dmasr = dma.dmasr.read();

    // Handle packet reception interrupt
    if dmasr.rs().bit_is_set() {
        network::poll(unsafe { TICKS });
        dma.dmasr.write(|w| w.rs().set_bit());
    }

    // Handle packet transmission interrupt
    if dmasr.ts().bit_is_set() {

        let ptr = hal::stm32::GPIOE::ptr();
        match unsafe { (*ptr).odr.read().odr9() } {
            hal::stm32::gpioi::odr::ODR15R::HIGH => {
                unsafe { (*ptr).bsrr.write(|w| w.br9().set_bit()) };
            },
            hal::stm32::gpioi::odr::ODR15R::LOW => {
                unsafe { (*ptr).bsrr.write(|w| w.bs9().set_bit()) };
            },
        }

        match unsafe { TDESRING.last_interrupt() } {
            Some(tdes) => network::tx_complete(tdes),
            None => (),
        };
        dma.dmasr.write(|w| w.ts().set_bit());
    }
}

#[exception]
fn SysTick() {
    static mut LAST_SYNC: u32 = 0;

    unsafe { TICKS += 1 };

    if PTP_MASTER && unsafe { TICKS } - *LAST_SYNC >= 1000 {
        network::send_ptp_sync();
        *LAST_SYNC = unsafe { TICKS };
    }

    network::poll(unsafe { TICKS });
}
