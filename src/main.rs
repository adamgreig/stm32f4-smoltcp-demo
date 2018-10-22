#![no_std]
#![no_main]

extern crate panic_abort;

use cortex_m_rt::{entry, exception};
use cortex_m;
use stm32f4::stm32f407;
use smoltcp;
use stm32f4_smoltcp::{EthernetDevice, TDesRing, TDes, RDesRing, RDes, ETH_BUF_SIZE};

mod network;

const ETH_NUM_TD: usize = 4;
const ETH_NUM_RD: usize = 4;

static mut TDESRING: TDesRing = TDesRing {
    td: &mut [TDes::new(); ETH_NUM_TD],
    tbuf: &mut [[0; ETH_BUF_SIZE/4]; ETH_NUM_RD],
    tdidx: 0,
};

static mut RDESRING: RDesRing = RDesRing {
    rd: &mut [RDes::new(); ETH_NUM_RD],
    rbuf: &mut [[0; ETH_BUF_SIZE/4]; ETH_NUM_RD],
    rdidx: 0,
};

fn rcc_init(p: &mut stm32f407::Peripherals) {
    let rcc = &p.RCC;
    let flash = &p.FLASH;
    let syscfg = &p.SYSCFG;

    // Ensure HSI is on and stable
    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsion().bit_is_clear() {}

    // Set system clock to HSI
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}

    // Clear registers to reset value
    rcc.cr.write(|w| w.hsion().set_bit());
    rcc.cfgr.write(|w| unsafe { w.bits(0) });

    // Configure PLL: 16MHz /8 *168 /2, source HSI
    rcc.pllcfgr.write(|w| unsafe {
        w.pllq().bits(4)
         .pllsrc().hsi()
         .pllp().div2()
         .plln().bits(168)
         .pllm().bits(8)
    });
    // Activate PLL
    rcc.cr.modify(|_, w| w.pllon().set_bit());

    // Set other clock domains: PPRE2 to /2, PPRE1 to /4, HPRE to /1
    rcc.cfgr.modify(|_, w|
        w.ppre2().div2()
         .ppre1().div4()
         .hpre().div1());

    // Flash setup: I$ and D$ enabled, prefetch enabled, 5 wait states (OK for 3.3V at 168MHz)
    flash.acr.write(|w| unsafe {
        w.icen().set_bit()
         .dcen().set_bit()
         .prften().set_bit()
         .latency().bits(5)
    });

    // Swap system clock to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // Set SYSCFG early to RMII mode
    rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());
    syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

    // Set up peripheral clocks
    rcc.ahb1enr.modify(|_, w|
        w.gpioaen().enabled()
         .gpioben().enabled()
         .gpiocen().enabled()
         .gpioden().enabled()
         .gpioeen().enabled()
         .gpiogen().enabled()
         .crcen().enabled()
         .ethmacrxen().enabled()
         .ethmactxen().enabled()
         .ethmacen().enabled()
    );
}

fn gpio_init(p: &mut stm32f407::Peripherals) {
    let gpioa = &p.GPIOA;
    let gpiob = &p.GPIOB;
    let gpioc = &p.GPIOC;
    let gpioe = &p.GPIOE;

    // Status LED
    gpioe.moder.modify(|_, w| w.moder7().output());
    gpioe.odr.modify(|_, w| w.odr7().clear_bit());

    // Configure ethernet related GPIO:
    // GPIOA 1, 2, 7
    // GPIOC 1, 4, 5
    // GPIOG 11, 13, 14
    // All set to AF11 and very high speed.
    gpioa.moder.modify(|_, w|
        w.moder1().alternate()
         .moder2().alternate()
         .moder7().alternate());
    gpiob.moder.modify(|_, w|
         w.moder11().alternate()
          .moder12().alternate()
          .moder13().alternate());
    gpioc.moder.modify(|_, w|
        w.moder1().alternate()
         .moder4().alternate()
         .moder5().alternate());
    gpioa.ospeedr.modify(|_, w|
        w.ospeedr1().very_high_speed()
         .ospeedr2().very_high_speed()
         .ospeedr7().very_high_speed());
    gpiob.ospeedr.modify(|_, w|
        w.ospeedr11().very_high_speed()
         .ospeedr12().very_high_speed()
         .ospeedr13().very_high_speed());
    gpioc.ospeedr.modify(|_, w|
        w.ospeedr1().very_high_speed()
         .ospeedr4().very_high_speed()
         .ospeedr5().very_high_speed());
    gpioa.afrl.modify(|_, w|
        w.afrl1().af11()
         .afrl2().af11()
         .afrl7().af11());
    gpiob.afrh.modify(|_, w|
        w.afrh11().af11()
         .afrh12().af11()
         .afrh13().af11());
    gpioc.afrl.modify(|_, w|
        w.afrl1().af11()
         .afrl4().af11()
         .afrl5().af11());
}

/// Set up the systick to provide a 1ms timebase
fn systick_init(syst: &mut stm32f407::SYST) {
    syst.set_reload((168_000_000 / 8) / 1000);
    syst.clear_current();
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::External);
    syst.enable_interrupt();
    syst.enable_counter();
}

#[entry]
fn main() -> ! {
    let mut p = stm32f407::CorePeripherals::take().unwrap();
    let mut dp = stm32f407::Peripherals::take().unwrap();

    rcc_init(&mut dp);
    gpio_init(&mut dp);

    // Create ethernet driver
    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(
        &[0x02, 0x00, 0x00, 0x00, 0x00, 0x10]);
    let mut ethdev = unsafe {
        EthernetDevice::new(dp.ETHERNET_MAC, dp.ETHERNET_DMA, &mut RDESRING, &mut TDESRING, 0)
    };
    ethdev.init(&mut dp.RCC, mac_addr.clone());
    ethdev.block_until_link();

    // Create network interface
    let ip_addr = smoltcp::wire::Ipv4Address::from_bytes(
        &[192, 168, 2, 101]);
    let ip_cidr = smoltcp::wire::Ipv4Cidr::new(ip_addr, 24);
    let cidr = smoltcp::wire::IpCidr::Ipv4(ip_cidr);
    network::init(ethdev, mac_addr, cidr);

    // Start systicks at 1ms to poll smoltcp
    systick_init(&mut p.SYST);

    loop {
        cortex_m::asm::wfi();
    }
}

#[exception]
fn SysTick() {
    static mut TICKS: u32 = 0;
    *TICKS = *TICKS + 1;
    network::poll(*TICKS as i64);
}
