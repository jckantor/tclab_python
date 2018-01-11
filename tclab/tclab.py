from __future__ import print_function, division

import sys
import time
import serial
from serial.tools import list_ports


class TCLab(object):
    def __init__(self, port=None, baud=9600):
        if (sys.platform == 'darwin') and not port:
            port = '/dev/tty.wchusbserial1410'
        else:
            port = self.findPort()
        self._T1 = self._T2 = 0
        print('Opening connection...')
        self.sp = serial.Serial(port=port, baudrate=baud, timeout=2)
        self.sp.flushInput()
        self.sp.flushOutput()
        time.sleep(3)
        self.version = self.read('VER')
        print('TCLab connected via Arduino on port', port)
        print('Version:', self.version)

    def findPort(self):
        found = False
        for port in list(list_ports.comports()):
            # Arduino Uno
            if port[2].startswith('USB VID:PID=16D0:0613'):
                port = port[0]
                found = True
            # Arduino HDuino
            if port[2].startswith('USB VID:PID=1A86:7523'):
                port = port[0]
                found = True
                # Arduino Leonardo
            if port[2].startswith('USB VID:PID=2341:8036'):
                port = port[0]
                found = True
        if (not found):
            print('Arduino COM port not found')
            print('Please ensure that the USB cable is connected')
            print('--- Printing Serial Ports ---')
            for port in list(serial.tools.list_ports.comports()):
                print(port[0] + ' ' + port[1] + ' ' + port[2])
            print('For Windows:')
            print('  Open device manager, select "Ports (COM & LPT)"')
            print('  Look for COM port of Arduino such as COM4')
            print('For MacOS:')
            print('  Open terminal and type: ls /dev/*.')
            print('  Search for /dev/tty.usbmodem* or /dev/tty.usbserial*. The port number is *.')
            print('For Linux')
            print('  Open terminal and type: ls /dev/tty*')
            print('  Search for /dev/ttyUSB* or /dev/ttyACM*. The port number is *.')
            print('')
            port = input('Input port: ')
            # or hard-code it here
            # port = 'COM3' # for Windows
            # port = '/dev/tty.wchusbserial1410' # for MacOS
        return port

    def stop(self):
        return self.read('X')

    @property
    def T1(self):
        self._T1 = float(self.read('T1'))
        return self._T1

    @property
    def T2(self):
        self._T2 = float(self.read('T2'))
        return self._T2

    def LED(self, pwm):
        pwm = max(0.0, min(100.0, pwm)) / 2.0
        self.write('LED', pwm)
        return pwm

    def Q1(self, pwm):
        pwm = max(0.0, min(100.0, pwm))
        self.write('Q1', pwm)
        return pwm

    def Q2(self, pwm):
        pwm = max(0.0, min(100.0, pwm))
        self.write('Q2', pwm)
        return pwm

    def read(self, cmd):
        cmd_str = self.build_cmd_str(cmd, '')
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except Exception:
            return None
        return self.sp.readline().decode('UTF-8').replace("\r\n", "")

    def write(self, cmd, pwm):
        cmd_str = self.build_cmd_str(cmd, (pwm,))
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except:
            return None
        return self.sp.readline().decode('UTF-8').replace("\r\n", "")

    def build_cmd_str(self, cmd, args=None):
        """
        Build a command string that can be sent to the arduino.
    
        Input:
            cmd (str): the command to send to the arduino, must not
                contain a % character
            args (iterable): the arguments to send to the command
        """
        if args:
            args = ' '.join(map(str, args))
        else:
            args = ''
        return "{cmd} {args}\n".format(cmd=cmd, args=args)

    def close(self):
        try:
            self.sp.close()
            print('Arduino disconnected successfully')
        except:
            print('Problems disconnecting from Arduino.')
            print('Please unplug and replug Arduino.')
        return True

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.Q1(0)
        self.Q2(0)
        self.close()


def flash_led():
    with TCLab() as a:
        print('LED On')
        a.LED(100)
        print('LED Off')
        time.sleep(1)
        a.LED(0)
