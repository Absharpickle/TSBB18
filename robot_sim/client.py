import socket
from time import sleep, time


def communicate(output_msg='hello world\n', server=("127.0.0.1", 9091)):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.01)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128)  # send buffer size
    sock.sendto(output_msg.encode("utf-8"), server)
    try:
        received = str(sock.recv(1024), "utf-8")
    except socket.timeout as e:
        received = None

    return received


if __name__ == '__main__':

    msg = communicate("base=0.0, shoulder=0.0, elbow=0.0, wrist_pitch=0.0")
    base = 0
    t0 = time()
    no_messages = 0
    t_frame = 0.03  # 33 fps
    while True:
        sleep(t_frame)
        base += 5
        if base > 180:
            base = -180
        msg = communicate(f"base={base}, elbow=40, shoulder=10, wrist_pitch=15.0")
        #msg = communicate(" " * 128)  # Note: If nothing is received, try sending long empty message so that the send buffer is flushed.

        if msg is not None:
            print(f"received {msg}")
            no_messages = 0
        else:
            no_messages += 1
            if no_messages > int(1/t_frame):
                print("No messages received. Is the server running?")
                no_messages = 0
