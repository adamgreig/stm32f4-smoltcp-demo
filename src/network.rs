use cortex_m;
use stm32f4_smoltcp::EthernetDevice;
use smoltcp::{self, time::Instant, wire::{EthernetAddress, IpAddress, IpCidr}};
use smoltcp::iface::{Neighbor, NeighborCache, EthernetInterface, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, SocketSetItem, SocketHandle, TcpSocket, TcpSocketBuffer};

struct NetworkBuffers {
    tcp_tx_buf: [u8; 1536],
    tcp_rx_buf: [u8; 1536],
}

static mut NETWORK_BUFFERS: NetworkBuffers = NetworkBuffers {
    tcp_tx_buf: [0u8; 1536],
    tcp_rx_buf: [0u8; 1536],
};

pub struct Network<'a> {
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 16],
    ip_addr: Option<[IpCidr; 1]>,
    eth_iface: Option<EthernetInterface<'a, 'a, 'a, EthernetDevice>>,
    sockets_storage: [Option<SocketSetItem<'a, 'a>>; 1],
    sockets: Option<SocketSet<'a, 'a, 'a>>,
    tcp_handle: Option<SocketHandle>,
    initialised: bool,
}
static mut NETWORK: Network = Network {
    neighbor_cache_storage: [None; 16],
    ip_addr: None,
    eth_iface: None,
    sockets_storage: [None],
    sockets: None,
    tcp_handle: None,
    initialised: false,
};

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
        NETWORK.eth_iface = Some(EthernetInterfaceBuilder::new(eth_dev)
                                .ethernet_addr(mac_addr)
                                .neighbor_cache(neighbor_cache)
                                .ip_addrs(&mut NETWORK.ip_addr.as_mut().unwrap()[..])
                                .finalize());

        NETWORK.sockets = Some(SocketSet::new(&mut NETWORK.sockets_storage.as_mut()[..]));
        let tcp_rx_buf = TcpSocketBuffer::new(&mut NETWORK_BUFFERS.tcp_rx_buf.as_mut()[..]);
        let tcp_tx_buf = TcpSocketBuffer::new(&mut NETWORK_BUFFERS.tcp_tx_buf.as_mut()[..]);
        let tcp_socket = TcpSocket::new(tcp_rx_buf, tcp_tx_buf);
        NETWORK.tcp_handle = Some(NETWORK.sockets.as_mut().unwrap().add(tcp_socket));
    }
}

/// Poll network stack.
///
/// Arrange for this function to be called frequently.
pub fn poll(time_ms: i64) {
    // Unsafe required to access static mut NETWORK.
    // Since the entire poll is run in an interrupt-free context no
    // other access to NETWORK can occur.
    cortex_m::interrupt::free(|_| unsafe {
        // Bail out early if NETWORK is not initialised.
        if !NETWORK.initialised {
            return;
        }

        let sockets = NETWORK.sockets.as_mut().unwrap();

        // Poll smoltcp
        let timestamp = Instant::from_millis(time_ms);
        match NETWORK.eth_iface.as_mut().unwrap().poll(sockets, timestamp) {
            Ok(_) | Err(smoltcp::Error::Exhausted) => (),
            Err(_) => (),
        }
    });
}
