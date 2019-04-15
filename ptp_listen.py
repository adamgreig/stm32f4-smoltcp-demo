import socket
import struct


def format_packet(data):
    (t1s, t1n, t2s, t2n, t3s, t3n, t4s, t4n,
     tos, ton, last_t1s, last_t1n, last_t2s, last_t2n,
     addend) = struct.unpack("<iiiiiiiiiiiiiiI", data)
    t1 = t1s + t1n/1e9
    t2 = t2s + t2n/1e9
    t3 = t3s + t3n/1e9
    t4 = t4s + t4n/1e9
    to = tos + ton/1e9
    # our_to = 0.5 * -(t2 - t1 - t4 + t3)
    last_t1 = last_t1s + last_t1n/1e9
    last_t2 = last_t2s + last_t2n/1e9
    addend_ppm = ((addend - (1 << 31)) / (1 << 31)) * 1e6
    dm = t1 - last_t1
    # dc = t2 - last_t2
    error = to / dm
    return (f"t1={t1:+.09f} t2={t2:+.09f} t3={t3:+.09f} "
            f"t4={t4:+.09f} tO={to:+.09f} "
            f"last_t1={last_t1:+.09f} last_t2={last_t2:+.09f} "
            f"addend={addend} ({addend_ppm:.02f}ppm) error={error:+.04e}")


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 10000))
    s.setblocking(True)
    while True:
        try:
            data, addr = s.recvfrom(64)
        except socket.error as e:
            continue
        print(addr[0], format_packet(data))


if __name__ == "__main__":
    main()
