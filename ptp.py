import time
import errno
import struct
import socket


TYPES = {
    0: "SYNC",
    1: "DELAY_REQ",
    8: "FOLLOW_UP",
    9: "DELAY_RESP",
}

TYPES_INV = {v: k for k, v in TYPES.items()}

T_OFFSET = 0


def local_time():
    return time.monotonic() - T_OFFSET


def type_from_packet(data):
    v = data[0]
    return TYPES.get(v)


def ts_from_packet(data):
    secs, nanos = struct.unpack("<ii", data[4:12])
    return secs + nanos/1e9


def send_delay_req(s_e, addr):
    msg = struct.pack("B", TYPES_INV["DELAY_REQ"]) + b"\x00"*11
    s_e.sendto(msg, (addr[0], 319))
    tx_ts = local_time()
    print("DELAY_REQ", tx_ts)
    return tx_ts


def main():
    global T_OFFSET
    s_e = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s_g = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s_e.bind(("0.0.0.0", 319))
    s_g.bind(("0.0.0.0", 320))
    s_e.setblocking(False)
    s_g.setblocking(False)
    t1 = t2 = t3 = t4 = None
    while True:
        try:
            data, addr = s_e.recvfrom(64)
            rx_ts = local_time()
            if type_from_packet(data) == "SYNC":
                t2 = rx_ts
                print("SYNC", t2)
                t1 = t3 = t4 = None
        except socket.error as e:
            if e.args[0] == errno.EWOULDBLOCK:
                pass
            else:
                break

        try:
            data, addr = s_g.recvfrom(64)
            if t2 is not None and type_from_packet(data) == "FOLLOW_UP":
                t1 = ts_from_packet(data)
                print("FOLLOW_UP", t1)
                t3 = send_delay_req(s_e, addr)
                t4 = None
            elif t3 is not None and type_from_packet(data) == "DELAY_RESP":
                t4 = ts_from_packet(data)
                print("DELAY_RESP", t4)
                print(t1, t2, t3, t4)
                off = 0.5*(t2 - t1 - t4 + t3)
                T_OFFSET += off
                print("Offset", off)
                print("Delay", 0.5*(t2 - t1 - t4 + t3))
                print("T_OFFSET", T_OFFSET)
                print()
                t1 = t2 = t3 = t4 = None
        except socket.error as e:
            if e.args[0] == errno.EWOULDBLOCK:
                pass
            else:
                break


if __name__ == "__main__":
    main()
