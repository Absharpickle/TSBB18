import socket
from time import sleep, time
import sys


def communicate(output_msg=None, server=("127.0.0.1", 9091), print_received_msg=False):
    """ Send and receive a message to/from the robot server.
    Note: if the other side did not have time to respond during the timeout period,
    call communicate() again a bit later, without any parameters, and check the response.
    """

    timeout = 0.01  # seconds

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256)  # send buffer size
    if output_msg is not None:
        sock.sendto(output_msg.encode("utf-8"), server)

    try:
        received_msg = str(sock.recv(1024), "utf-8")
        if print_received_msg:
            print(received_msg)
    except socket.timeout as e:
        received_msg = None
    except ConnectionResetError as e:
        print(e, file=sys.stderr)
        print("Is the server in Blender running?", file=sys.stderr)
        received_msg = None

    return received_msg


### Server commands ###

def set_shading_mode(mode='RENDERED'):
    """ Blender starts in 'SOLID' mode, but rendered looks much nicer. """
    assert mode in ('WIREFRAME' 'SOLID' 'MATERIAL' 'RENDERED')
    return communicate(f"self.set_shading_mode(mode='{mode}')")


def state():
    """ Receive robot and scene state. Communication from the server is
    slightly flaky, so it's recommended to call this function repeatedly
    until you get the same (non-empty) response twice. """
    msg = communicate(f"self.state()")
    msg = eval(f"dict({msg})") if msg is not None else dict()
    return msg


def move_robot(base=0.0, elbow=0.0, shoulder=0.0, wrist_pitch=0.0):
    """  Move the robot joints. All values are in degrees. """
    return communicate(f"self.move_robot(base={base}, elbow={elbow}, shoulder={shoulder}, wrist_pitch={wrist_pitch})")


def capture():
    """ Capture an image from the camera and save to disk.
     Check the terminal in which Blender was started to get the path.
     Capturing will take a relatively long time, during which the
     robot server will ignore messages. Sent messages will be queued.
      """
    return communicate("self.capture()")


def drop_lego(count=5):
    """ Drop a random set of lego pieces in front of the robot. """
    return communicate(f"self.drop_lego(pieces={count})")


def place_lego(name, size=(1,1), color='red', x=0.0, y=-100.0, angle=45.0):
    """ Place a piece of lego, anywhere on the ground plane. Distances in millimeters """

    valid_sizes = {(1, 1), (2, 1), (2, 2), (3, 1), (3, 2), (4, 1), (4, 2)}
    valid_colors = {'red', 'green', 'blue'}

    assert size in valid_sizes
    assert color in valid_colors
    msg = communicate(f"self.place_lego(name='{name}', size={size}, color='{color}', x={x}, y={y}, angle={angle})")
    return msg


def clear_lego():
    """ Remove all dropped lego """
    return communicate("self.clear_lego()")


def pick_lego():
    """ Pick up lego. Will remove any piece touched by the robot probe """
    return communicate(f"self.pick_lego()")


if __name__ == '__main__':

    set_shading_mode('RENDERED')
    sleep(0.1)
    clear_lego()
    sleep(0.1)
    move_robot()
    sleep(1)
    move_robot(base=0.0, shoulder=40.0, elbow=60.0, wrist_pitch=0.0)
    sleep(2)
    drop_lego(count=50)
    sleep(0.1)
    capture()
    sleep(1)
