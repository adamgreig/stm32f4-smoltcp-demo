import socket
import struct


def format_packet(data):
    t1s, t1n, t2s, t2n, t3s, t3n, t4s, t4n, tos, ton = struct.unpack(
        "<iiiiiiiiii", data)
    t1 = t1s + t1n/1e9
    t2 = t2s + t2n/1e9
    t3 = t3s + t3n/1e9
    t4 = t4s + t4n/1e9
    to = tos + ton/1e9
    return (f"t1={t1:+.09f} t2={t2:+.09f} t3={t3:+.09f} "
            f"t4={t4:+.09f} tO={to:+.09f}")


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
