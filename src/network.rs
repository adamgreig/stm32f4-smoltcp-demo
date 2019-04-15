use core::convert::TryInto;
use core::ops;

use cortex_m;
use smoltcp::iface::{EthernetInterface, EthernetInterfaceBuilder, Neighbor, NeighborCache};
use smoltcp::socket::{SocketHandle, SocketSet, SocketSetItem};
use smoltcp::socket::{UdpPacketMetadata, UdpSocket, UdpSocketBuffer};
use smoltcp::{
    self,
    time::Instant,
    wire::{EthernetAddress, IpAddress, Ipv4Address, IpCidr, IpEndpoint},
};
use stm32f4_smoltcp::{EthernetDevice, TDes};

const PTP_EVENT_PORT: u16 = 319u16;
const PTP_GENERAL_PORT: u16 = 320u16;

#[derive(Copy, Clone, Debug)]
struct PTPTimestamp {
    seconds: i32,
    nanos: i32,
}

impl PTPTimestamp {
    pub fn from_packet(data: &[u8]) -> Option<PTPTimestamp> {
        if data.len() >= 12 {
            Some(PTPTimestamp {
                seconds: i32::from_le_bytes(data[4..8].try_into().ok()?),
                nanos: i32::from_le_bytes(data[8..12].try_into().ok()?),
            })
        } else {
            None
        }
    }

    pub fn from_u64(ts: u64) -> PTPTimestamp {
        PTPTimestamp {
            seconds: (ts >> 32) as i32,
            nanos: ts as i32,
        }
    }

    pub fn rationalise(&mut self) {
        while self.nanos > 1_000_000_000 {
            self.seconds += 1;
            self.nanos -= 1_000_000_000;
        }
        while self.nanos < -1_000_000_000 {
            self.seconds -= 1;
            self.nanos += 1_000_000_000;
        }
    }

    pub const fn zero() -> PTPTimestamp {
        PTPTimestamp { seconds: 0, nanos: 0 }
    }

    pub fn halve(&mut self) {
        if self.seconds & 1 == 1 {
            self.nanos += 1_000_000_000;
        }
        self.seconds /= 2;
        self.nanos /= 2;
    }

    pub fn to_f32(&self) -> f32 {
        self.seconds as f32 + (self.nanos as f32)/1e9
    }
}

impl ops::Add<PTPTimestamp> for PTPTimestamp {
    type Output = PTPTimestamp;

    fn add(self, rhs: PTPTimestamp) -> PTPTimestamp {
        let mut rv = PTPTimestamp::zero();
        rv.seconds = self.seconds + rhs.seconds;
        rv.nanos = self.nanos + rhs.nanos;
        rv.rationalise();
        rv
    }
}

impl ops::Add<&PTPTimestamp> for &PTPTimestamp {
    type Output = PTPTimestamp;
    fn add(self, rhs: &PTPTimestamp) -> PTPTimestamp {
        (*self).add(*rhs)
    }
}

impl ops::Add<&PTPTimestamp> for PTPTimestamp {
    type Output = PTPTimestamp;
    fn add(self, rhs: &PTPTimestamp) -> PTPTimestamp {
        self.add(*rhs)
    }
}

impl ops::Add<PTPTimestamp> for &PTPTimestamp {
    type Output = PTPTimestamp;
    fn add(self, rhs: PTPTimestamp) -> PTPTimestamp {
        (*self).add(rhs)
    }
}

impl ops::Sub<PTPTimestamp> for PTPTimestamp {
    type Output = PTPTimestamp;

    fn sub(self, rhs: PTPTimestamp) -> PTPTimestamp {
        let mut rv = PTPTimestamp::zero();
        rv.seconds = self.seconds - rhs.seconds;
        rv.nanos = self.nanos - rhs.nanos;
        rv.rationalise();
        rv
    }
}

impl ops::Sub<&PTPTimestamp> for &PTPTimestamp {
    type Output = PTPTimestamp;
    fn sub(self, rhs: &PTPTimestamp) -> PTPTimestamp {
        (*self).sub(*rhs)
    }
}

impl ops::Sub<&PTPTimestamp> for PTPTimestamp {
    type Output = PTPTimestamp;
    fn sub(self, rhs: &PTPTimestamp) -> PTPTimestamp {
        self.sub(*rhs)
    }
}

impl ops::Sub<PTPTimestamp> for &PTPTimestamp {
    type Output = PTPTimestamp;
    fn sub(self, rhs: PTPTimestamp) -> PTPTimestamp {
        (*self).sub(rhs)
    }
}

impl ops::Neg for PTPTimestamp {
    type Output = PTPTimestamp;

    fn neg(self) -> PTPTimestamp {
        let mut rv = self.clone();
        rv.seconds = -rv.seconds;
        rv.nanos = -rv.nanos;
        rv.rationalise();
        rv
    }
}

impl ops::Neg for &PTPTimestamp {
    type Output = PTPTimestamp;
    fn neg(self) -> PTPTimestamp {
        (*self).neg()
    }
}

#[derive(Copy, Clone, Debug)]
struct PTPTimestamps {
    t1: Option<PTPTimestamp>,
    t2: Option<PTPTimestamp>,
    t3: Option<PTPTimestamp>,
    t4: Option<PTPTimestamp>,
    last_t1: Option<PTPTimestamp>,
    last_t2: Option<PTPTimestamp>,
    addend: u32,
}

impl PTPTimestamps {
    pub const fn new() -> PTPTimestamps {
        PTPTimestamps {
            t1: None,
            t2: None,
            t3: None,
            t4: None,
            last_t1: None,
            last_t2: None,
            addend: 1<<31,
        }
    }

    /// Compute the time offset when all four timestamps are available
    pub fn offset(&self) -> Option<PTPTimestamp> {
        match (&self.t1, &self.t2, &self.t3, &self.t4) {
            (Some(t1), Some(t2), Some(t3), Some(t4)) => {
                let mut off = -(t2 - t1 - t4 + t3);
                off.halve();
                Some(off)
            },
            _ => None
        }
    }

    pub fn update(&mut self, ptp: &hal::stm32::ethernet_ptp::RegisterBlock) {
        match self.offset() {
            Some(off) => {
                self.coarse_update(off, ptp);
                if off.seconds == 0 {
                    self.fine_update(off, ptp);
                }
            },
            None => (),
        }
        self.last_t1 = self.t1;
        self.last_t2 = self.t2;
        self.t1 = None;
        self.t2 = None;
        self.t3 = None;
        self.t4 = None;
    }

    fn fine_update(&mut self, off: PTPTimestamp, ptp: &hal::stm32::ethernet_ptp::RegisterBlock) {
        match (&self.t1, &self.t2, &self.last_t1, &self.last_t2) {
            (Some(t1), Some(t2), Some(last_t1), Some(last_t2)) => {
                let dm = t1 - last_t1;
                let _dc = t2 - last_t2;
                let error = off.to_f32() / dm.to_f32();
                if error > -0.01 && error < 0.01 {
                    self.addend = (self.addend as f32 * (1.0 + error/5.0)) as u32;
                    self.write_addend(ptp);
                }
            },
            _ => ()
        }
    }

    fn write_addend(&self, ptp: &hal::stm32::ethernet_ptp::RegisterBlock) {
        unsafe {
            ptp.ptptsar.write(|w| w.tsa().bits(self.addend));
            ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
            while ptp.ptptscr.read().ttsaru().bit_is_set() {}
        }
    }

    /// Perform coarse update of given offset
    fn coarse_update(&self, offset: PTPTimestamp, ptp: &hal::stm32::ethernet_ptp::RegisterBlock) {
        if offset.seconds < 0 || (offset.seconds == 0 && offset.nanos < 0) {
            unsafe {
                ptp.ptptshur.write(|w| w.tsus().bits(-offset.seconds as u32));
                ptp.ptptslur.write(|w| w.tsupns().set_bit().tsuss().bits(-offset.nanos as u32));
            }
        } else {
            unsafe {
                ptp.ptptshur.write(|w| w.tsus().bits(offset.seconds as u32));
                ptp.ptptslur.write(|w| w.tsupns().clear_bit().tsuss().bits(offset.nanos as u32));
            }
        }
        ptp.ptptscr.modify(|_, w| w.tsstu().set_bit());
        while ptp.ptptscr.read().tsstu().bit_is_set() {}
        self.write_addend(ptp);
    }

    /// Write all timestamps and computed offset into a u8 array for debug
    pub fn to_bytes(&self) -> [u8; 60] {
        let mut buf = [0u8; 60];
        if let Some(t1) = self.t1 {
            buf[0..4].copy_from_slice(&t1.seconds.to_le_bytes()[..]);
            buf[4..8].copy_from_slice(&t1.nanos.to_le_bytes()[..]);
        }
        if let Some(t2) = self.t2 {
            buf[8..12].copy_from_slice(&t2.seconds.to_le_bytes()[..]);
            buf[12..16].copy_from_slice(&t2.nanos.to_le_bytes()[..]);
        }
        if let Some(t3) = self.t3 {
            buf[16..20].copy_from_slice(&t3.seconds.to_le_bytes()[..]);
            buf[20..24].copy_from_slice(&t3.nanos.to_le_bytes()[..]);
        }
        if let Some(t4) = self.t4 {
            buf[24..28].copy_from_slice(&t4.seconds.to_le_bytes()[..]);
            buf[28..32].copy_from_slice(&t4.nanos.to_le_bytes()[..]);
        }
        if let Some(to) = self.offset() {
            buf[32..36].copy_from_slice(&to.seconds.to_le_bytes()[..]);
            buf[36..40].copy_from_slice(&to.nanos.to_le_bytes()[..]);
        }
        if let Some(last_t1) = self.last_t1 {
            buf[40..44].copy_from_slice(&last_t1.seconds.to_le_bytes()[..]);
            buf[44..48].copy_from_slice(&last_t1.nanos.to_le_bytes()[..]);
        }
        if let Some(last_t2) = self.last_t2 {
            buf[48..52].copy_from_slice(&last_t2.seconds.to_le_bytes()[..]);
            buf[52..56].copy_from_slice(&last_t2.nanos.to_le_bytes()[..]);
        }
        buf[56..60].copy_from_slice(&self.addend.to_le_bytes()[..]);
        buf
    }
}

struct NetworkBuffers {
    udp_tx_buf1: [u8; 1536],
    udp_rx_buf1: [u8; 1536],
    udp_tx_meta1: [UdpPacketMetadata; 16],
    udp_rx_meta1: [UdpPacketMetadata; 16],
    udp_tx_buf2: [u8; 1536],
    udp_rx_buf2: [u8; 1536],
    udp_tx_meta2: [UdpPacketMetadata; 16],
    udp_rx_meta2: [UdpPacketMetadata; 16],
}

static mut NETWORK_BUFFERS: NetworkBuffers = NetworkBuffers {
    udp_tx_buf1: [0u8; 1536],
    udp_rx_buf1: [0u8; 1536],
    udp_tx_meta1: [UdpPacketMetadata::EMPTY; 16],
    udp_rx_meta1: [UdpPacketMetadata::EMPTY; 16],
    udp_tx_buf2: [0u8; 1536],
    udp_rx_buf2: [0u8; 1536],
    udp_tx_meta2: [UdpPacketMetadata::EMPTY; 16],
    udp_rx_meta2: [UdpPacketMetadata::EMPTY; 16],
};

pub struct Network<'a> {
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 16],
    ip_addr: Option<[IpCidr; 1]>,
    eth_iface: Option<EthernetInterface<'a, 'a, 'a, EthernetDevice>>,
    sockets_storage: [Option<SocketSetItem<'a, 'a>>; 2],
    sockets: Option<SocketSet<'a, 'a, 'a>>,
    udp_handle_event: Option<SocketHandle>,
    udp_handle_general: Option<SocketHandle>,
    ptp_timestamps: PTPTimestamps,
    initialised: bool,
}
static mut NETWORK: Network = Network {
    neighbor_cache_storage: [None; 16],
    ip_addr: None,
    eth_iface: None,
    sockets_storage: [None, None],
    sockets: None,
    udp_handle_event: None,
    udp_handle_general: None,
    ptp_timestamps: PTPTimestamps::new(),
    initialised: false,
};

#[derive(Debug)]
#[repr(u8)]
enum MsgType {
    Sync = 0,
    FollowUp = 8,
    DelayReq = 1,
    DelayResp = 9,
}

impl MsgType {
    pub fn from_u8(v: u8) -> Option<MsgType> {
        match v {
            v if v == MsgType::Sync as u8 => Some(MsgType::Sync),
            v if v == MsgType::FollowUp as u8 => Some(MsgType::FollowUp),
            v if v == MsgType::DelayReq as u8 => Some(MsgType::DelayReq),
            v if v == MsgType::DelayResp as u8 => Some(MsgType::DelayResp),
            _ => None,
        }
    }
}

/// Initialise the static NETWORK.
///
/// Sets up the required EthernetInterface and sockets.
///
/// Do not call more than once or this function will panic.
pub fn init<'a>(eth_dev: EthernetDevice, mac_addr: EthernetAddress, ip_addr: IpCidr) {
    // Unsafe required for access to NETWORK.
    // NETWORK.initialised guards against calling twice.
    unsafe {
        cortex_m::interrupt::free(|_| {
            if NETWORK.initialised {
                panic!("NETWORK already initialised");
            }
            NETWORK.initialised = true;
        });

        let neighbor_cache = NeighborCache::new(&mut NETWORK.neighbor_cache_storage.as_mut()[..]);

        NETWORK.ip_addr = Some([ip_addr]);
        NETWORK.eth_iface = Some(
            EthernetInterfaceBuilder::new(eth_dev)
                .ethernet_addr(mac_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut NETWORK.ip_addr.as_mut().unwrap()[..])
                .finalize(),
        );

        NETWORK.sockets = Some(SocketSet::new(&mut NETWORK.sockets_storage.as_mut()[..]));
        let udp_rx_buf1 = UdpSocketBuffer::new(&mut NETWORK_BUFFERS.udp_rx_meta1.as_mut()[..],
                                               &mut NETWORK_BUFFERS.udp_rx_buf1.as_mut()[..]);
        let udp_tx_buf1 = UdpSocketBuffer::new(&mut NETWORK_BUFFERS.udp_tx_meta1.as_mut()[..],
                                               &mut NETWORK_BUFFERS.udp_tx_buf1.as_mut()[..]);
        let udp_rx_buf2 = UdpSocketBuffer::new(&mut NETWORK_BUFFERS.udp_rx_meta2.as_mut()[..],
                                               &mut NETWORK_BUFFERS.udp_rx_buf2.as_mut()[..]);
        let udp_tx_buf2 = UdpSocketBuffer::new(&mut NETWORK_BUFFERS.udp_tx_meta2.as_mut()[..],
                                               &mut NETWORK_BUFFERS.udp_tx_buf2.as_mut()[..]);
        let udp_socket1 = UdpSocket::new(udp_rx_buf1, udp_tx_buf1);
        let udp_socket2 = UdpSocket::new(udp_rx_buf2, udp_tx_buf2);
        NETWORK.udp_handle_event = Some(NETWORK.sockets.as_mut().unwrap().add(udp_socket1));
        NETWORK.udp_handle_general = Some(NETWORK.sockets.as_mut().unwrap().add(udp_socket2));
    }
}

/// Poll network stack.
///
/// Arrange for this function to be called frequently.
pub fn poll(time_ms: u32) {
    // Unsafe required to access static mut NETWORK.
    // Since the entire poll is run in an interrupt-free context no
    // other access to NETWORK can occur.
    cortex_m::interrupt::free(|_| unsafe {
        // Bail out early if NETWORK is not initialised.
        if !NETWORK.initialised {
            return;
        }

        // Poll smoltcp
        let timestamp = Instant::from_millis(i64::from(time_ms));
        let sockets = NETWORK.sockets.as_mut().unwrap();
        match NETWORK.eth_iface.as_mut().unwrap().poll(sockets, timestamp) {
            Ok(_) | Err(smoltcp::Error::Exhausted) => (),
            Err(_) => (),
        }

        // Handle receiving PTP event messages
        {
            let mut socket = sockets.get::<UdpSocket>(NETWORK.udp_handle_event.unwrap());
            if !socket.is_open() {
                socket.bind(PTP_EVENT_PORT).unwrap();
            }
            match socket.recv() {
                Ok((data, endpoint)) => {
                    match MsgType::from_u8(data[0]) {
                        Some(MsgType::Sync) => {
                            // Received SYNC Packet: store reception time as t2
                            NETWORK.ptp_timestamps.t2 = PTPTimestamp::from_packet(data);
                        },
                        Some(MsgType::DelayReq) => {
                            // Received DELAY_REQ packet: respond with reception time
                            let mut txdata = [0u8; 12];
                            txdata[0] = MsgType::DelayResp as u8;
                            txdata[4..12].copy_from_slice(&data[4..12]);
                            let mut ep = endpoint;
                            ep.port = PTP_GENERAL_PORT;
                            socket.send_slice(&txdata[..], ep).unwrap();
                        },
                        _ => (),
                    }
                }
                Err(_) => ()
            };
        }

        // Handle receiving PTP general messages
        {
            let mut socket = sockets.get::<UdpSocket>(NETWORK.udp_handle_general.unwrap());
            if !socket.is_open() {
                socket.bind(PTP_GENERAL_PORT).unwrap();
            }
            match socket.recv() {
                Ok((data, endpoint)) => {
                    match MsgType::from_u8(data[0]) {
                        Some(MsgType::FollowUp) => {
                            // Received FOLLOW_UP packet

                            // Store contained time as t1
                            NETWORK.ptp_timestamps.t1 = PTPTimestamp::from_packet(data);

                            // Send a DELAY_REQ in response (we'll store the transmit time as t3)
                            let mut txdata = [0u8; 12];
                            txdata[0] = MsgType::DelayReq as u8;
                            let mut ep = endpoint;
                            ep.port = PTP_EVENT_PORT;
                            socket.send_slice(&txdata[..], ep).unwrap();
                        },
                        Some(MsgType::DelayResp) => {
                            // Received DELAY_RESP packet

                            // Store contained time as t4
                            NETWORK.ptp_timestamps.t4 = PTPTimestamp::from_packet(data);

                            // Send debug information
                            let buf = NETWORK.ptp_timestamps.to_bytes();
                            let ep = IpEndpoint::new(
                                IpAddress::Ipv4(Ipv4Address::new(192, 168, 2, 2)), 10000);
                            socket.send_slice(&buf[..], ep).unwrap();

                            // Process {t1, t2, t3, t4} to update our local clock
                            let ptp = &(*hal::stm32::ETHERNET_PTP::ptr());
                            NETWORK.ptp_timestamps.update(ptp);
                        },
                        _ => (),
                    }
                }
                Err(_) => ()
            };
        }
    });
}

/// Call on transmission complete interrupt.
/// Specify &TDes with the corresponding transmitted descriptor.
pub fn tx_complete(tdes: &TDes) {
    // Unsafe required to access static mut NETWORK.
    // Since the entire poll is run in an interrupt-free context no
    // other access to NETWORK can occur.
    cortex_m::interrupt::free(|_| unsafe {
        // Bail out early if NETWORK is not initialised.
        if !NETWORK.initialised {
            return;
        }
        let sockets = NETWORK.sockets.as_mut().unwrap();
        let ts = match tdes.get_timestamp() {
            Some(ts) => ts,
            None => return,
        };

        if tdes.is_ptp_sync() {
            // Transmit a FOLLOW_UP message with transmit timestamp
            let mut socket = sockets.get::<UdpSocket>(NETWORK.udp_handle_general.unwrap());
            let mut txdata = [0u8; 12];
            txdata[ 0] = MsgType::FollowUp as u8;
            txdata[ 4] = (ts >> 32) as u8;
            txdata[ 5] = (ts >> 40) as u8;
            txdata[ 6] = (ts >> 48) as u8;
            txdata[ 7] = (ts >> 56) as u8;
            txdata[ 8] = (ts >>  0) as u8;
            txdata[ 9] = (ts >>  8) as u8;
            txdata[10] = (ts >> 16) as u8;
            txdata[11] = (ts >> 24) as u8;
            let ep = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::new(255, 255, 255, 255)), PTP_GENERAL_PORT);
            socket.send_slice(&txdata[..], ep).ok();
        } else if tdes.is_ptp_delay_req() {
            // Store transmit timestamp as t3
            NETWORK.ptp_timestamps.t3 = Some(PTPTimestamp::from_u64(ts));
        }

    });
}

pub fn send_ptp_sync() {
    // Unsafe required to access static mut NETWORK.
    // Since the entire poll is run in an interrupt-free context no
    // other access to NETWORK can occur.
    cortex_m::interrupt::free(|_| unsafe {
        // Bail out early if NETWORK is not initialised.
        if !NETWORK.initialised {
            return;
        }

        let sockets = NETWORK.sockets.as_mut().unwrap();
        {
            let mut socket = sockets.get::<UdpSocket>(NETWORK.udp_handle_event.unwrap());
            let mut txdata = [0u8; 12];
            txdata[0] = MsgType::Sync as u8;
            let ep = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::new(255, 255, 255, 255)), PTP_EVENT_PORT);
            socket.send_slice(&txdata[..], ep).unwrap();
        }
    });
}
