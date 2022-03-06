
import threading
import struct
import posix_ipc as pipc
import os

class QemuGpioClient:
    MAGIC = 0x4523

    def __init__(self):
        self.mq_to_qemu = pipc.MessageQueue("/to_qemu_bcm2835_gpio", read=False, write=True)
        self.mq_from_qemu = pipc.MessageQueue("/from_qemu_bcm2835_gpio", read=True, write=False)
    
    def thread_recv(self):
        while True:
            msg = self.mq_from_qemu.receive()
            mg, pin, state = struct.unpack(">HBB", msg[0])
            val = state >> 4
            funcsel = state & 0xf;
            if mg != QemuGpioClient.MAGIC:
                print("invalid magic number: " + hex(mg))
            print(f"Pin {pin}: {funcsel} {val}")
            if pin == 127:
                os._exit(0)
    
    def thread_send(self):
        while True:
            cmd = input()
            cmd = list(filter(None, map(str.strip, cmd.strip().split())))
            if len(cmd) == 0:
                continue
            if len(cmd) == 1 and cmd[0] == "q":
                self.mq_to_qemu.send(struct.pack(">HBB", QemuGpioClient.MAGIC, 127, 0))
                continue
            elif len(cmd) != 2 or (cmd[0] != "set" and cmd[0] != "reset"):
                print("command: q | set <pin> | reset <pin>")
                continue
            state = 1 if cmd[0] == "set" else 0
            try:
                pin = int(cmd[1])
                if pin < 0 or pin >= 54:
                    print("pin must be in [0, 53]")
                    continue
            except:
                print("invalid pin number: " + repr(cmd[1]))
                continue
            msg = struct.pack(">HBB", QemuGpioClient.MAGIC, pin, state)
            self.mq_to_qemu.send(msg)

    def run(self):
        self.t1 = threading.Thread(target=self.thread_recv)
        self.t1.start()
        self.t2 = threading.Thread(target=self.thread_send)
        self.t2.start()
        self.t1.join()
        self.t2.join()

def main():
    qgc = QemuGpioClient()
    qgc.run()

if __name__ == "__main__":
    main()
