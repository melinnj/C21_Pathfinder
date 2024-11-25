import threading
import time
import numpy as np
import control_real_drone as dn

origin = np.array([0, 0])


def get_coords():
    x, y = dn.grab_x_y()
    print("Coordinate: ", x, "\tX")
    print("Coordinate: ", y, "\tY")
    return np.array([x, y])


def mov1():
    stuff = get_coords()
    while not np.array_equal(stuff, origin):
        print("Moving...")
        dn.mov(origin[0], origin[1])
        stuff = get_coords()


def log(timeout=10):
    start_time = time.time()
    while not dn.is_logging():
        print("Waiting for log...")
        time.sleep(0.5)
        if time.time() - start_time > timeout:
            print("Logging timeout.")
            dn.manual_exit()
            return
    time.sleep(0.5)


def main():
    log()

    mov_thread = threading.Thread(target=mov1)
    mov_thread.start()

    time.sleep(5)
    dn.manual_exit()

    mov_thread.join()


if __name__ == "__main__":
    main()