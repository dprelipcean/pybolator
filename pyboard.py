from datetime import datetime
from random import randint
from time import sleep
import bdb
import threading
import os
import sys

from pyfiglet import Figlet


MAIN_FILENAME = os.environ.get("PYBOLATOR_MAIN", "main.py")


class _Board:

    leds = {}

    def init(self, hardware=None):
        sys.stderr.write("BOARD:init\n")
        self.hardware = hardware
        self.boot()

        return self

    def set_accel(self):
        accel = self.hardware.get("accel")
        if accel is not None:
            self.accel = _Accel()

    def set_LCD(self):
        LCD = self.hardware.get("LCD")
        if LCD:
            self.LCD = _LCD(LCD)

    def set_leds(self):
        leds = self.hardware.get("LEDS")
        if leds is not None:
            _LED._intensity_min = leds["intensity_min"]
            _LED._intensity_max = leds["intensity_max"]
            c = 1
            for color in leds["items"]:
                self.leds[c] = _LED(color)
                c += 1

    def set_reset(self):
        switch = self.hardware.get("reset")
        if switch is not None:
            self.reset = _Switch('reset', hard_reset)

    def set_switch(self):
        switch = self.hardware.get("switch")
        if switch is not None:
            self.switch = _Switch('user')

    def main(self, pyb_script=None):
        sys.stderr.write("BOARD:main\n")
        self.keep_interpreter_running = True
        self.keep_user_script_running = True

        self.user_script_runner = _Runner(glo=globals())
        self.user_script_runner.start()

        self.interpreter = _Interpreter(pyb_script)
        self.interpreter.start()

    def stop_user_script(self):
        sys.stderr.write("BOARD:stop_user_script\n")
        self.keep_user_script_running = False

    def stop_interpreter(self):
        sys.stderr.write("BOARD:stop_interpreter\n")
        self.interpreter.stop()

    def stop(self):
        sys.stderr.write("BOARD:stop\n")
        self.stop_interpreter()
        self.stop_user_script()

    def boot(self):
        sys.stderr.write("BOARD:boot\n")
        self.boot_time = datetime.now()

        self.set_accel()
        self.set_LCD()
        self.set_leds()
        self.set_reset()
        self.set_switch()

        self.runner = _Runner()


class _Interpreter:

    commands = []

    def __init__(self, script=None):
        if script is not None:
            for line in script.split('\n'):
                self.write(line)

        self.thread = threading.Thread(target=self.target)

    def target(self):
        while _board.keep_interpreter_running:
            command = self.read()
            if command:
                self.exec(command)
            self.update()
            sleep(.5)

    def start(self):
        self.thread.start()

    def stop(self):
        _board.keep_interpreter_running = False

    def update(self):
        _board.switch._update()
        _board.reset._update()

    def read(self):
        sys.stderr.write("INT:read\n")
        if len(self.commands):
            command = self.commands.pop()
            return command

    def write(self, command):
        self.commands.insert(0, command)

    def exec(self, command):
        sys.stderr.write("INT:exec, %s \n" % command)

        if command == "sleep":
            sys.stderr.write("INT:sleeping!!!\n")
            sleep(1)
        elif command == "user-switch:press":
            _board.switch._press()
        elif command == "user-switch:release":
            _board.switch._release()
        elif command == "reset-switch:press":
            _board.reset._press()
        elif command == "reset-switch:release":
            _board.reset._release()
        else:
            sys.stderr.write("INT:COMMAND NOT FOUND, %s \n" % command)


class _Runner:

    class CPU(bdb.Bdb):
        # def user_line(self, frame):
        #     # Stop running user code
        #     if not self.board.keep_user_script_running:
        #         raise bdb.BdbQuit
        #     # print('FRAME: ', frame.f_code)
        pass

    def __init__(self, glo=None):
        self.globals = glo
        self.thread = threading.Thread(target=self.target_bdb,
                                       args=(MAIN_FILENAME,))

    def target_bdb(self, main_filename):
        with open(main_filename) as f:
            statement = 'exec(%r)' % f.read()

        statement += '\nimport time' \
                     '\nwhile _board.keep_user_script_running: time.sleep(1)'

        cpu = self.CPU()
        cpu.board = self.globals['_board']
        cpu.run(statement, self.globals)

    def is_alive(self):
        return self.thread.is_alive()

    def start(self):
        self.thread.start()


class _Accel:

    @property
    def x(self):
        sys.stderr.write("ACCEL: x\n")
        return randint(0, 10)

    @property
    def y(self):
        sys.stderr.write("ACCEL: y\n")
        return randint(0, 10)


class _LCD:

    def __init__(self, LCD):
        self._y = LCD['y']
        self._x = LCD['x']
        self.skin_position = None
        self._buffer = None
        self._hidden_buffer = {}

        self.backlight = None
        self.contrast_value = None

    def __call__(self, skin_position):
        self.skin_position = skin_position
        if skin_position == 'Y':
            self._x, self._y = self._y, self._x
        self.fill(0)

        return self

    def command(self, instr_data, buf):
        """Send an arbitrary command to the LCD. Pass 0 for instr_data to send
        an instruction, otherwise pass 1 to send data. buf is a buffer
        with the instructions/data to send.

        """
        sys.stderr.write("LCD:command: %s %s\n" % (instr_data, buf))
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def contrast(self, value):
        sys.stderr.write("LCD:contrast: %s\n" % value)
        self.contrast_value = value

    def get(self, x, y):
        sys.stderr.write("LCD:get: %sx%s\n" % (x, y))
        if self._buffer is None:
            return False
        return self._buffer[(x, y)]

    def light(self, value):
        sys.stderr.write("LCD:light: %s\n" % value)
        self.backlight = bool(value)

    def fill(self, colour):
        sys.stderr.write("LCD:fill: %s\n" % colour)
        for y in range(self._y):
            for x in range(self._x):
                self.pixel(x, y, colour)

    def pixel(self, x, y, colour):
        # sys.stderr.write("LCD:fill: %sx%s %s\n" % (x, y, colour))
        self._hidden_buffer[(x, y)] = colour

    def show(self):
        sys.stderr.write("LCD:show\n")
        self._buffer = self._hidden_buffer.copy()

    def text(self, text, x, y, colour):
        sys.stderr.write("LCD:text %s %sx%s %s\n" % (text, x, y, colour))
        font = 'clr8x8'
        figlet = Figlet(font=font, width=self._x)
        txt = figlet.renderText(text)
        dx = 0
        dy = 0
        color = 0
        for item in txt:
            if item == ' ':
                color = int(not bool(colour))
            elif item == '#':
                color = colour
            elif item == '\n':
                dy += 1
                dx = 0
                continue
            self.pixel(dx + x, dy + y, color)
            dx += 1

    def write(self, text):
        sys.stderr.write("LCD:write: %s\n" % text)
        self.text(text, 0, 0, 0)
        self.show()

    def _print_hidden_buffer(self):
        for y in range(self._y):
            for x in range(self._x):
                sys.stdout.write(str(self._hidden_buffer[(x, y)]))
            sys.stdout.write('\n')


class _LED:

    # Values in percentage
    _intensity_min = 0
    _intensity_max = 100

    def __init__(self, color):
        self._intensity = 0
        self._color = color

    def on(self):
        sys.stderr.write("LED %s: on\n" % self._color)
        self._intensity = self._intensity_max

    def off(self):
        sys.stderr.write("LED %s: off\n" % self._color)
        self._intensity = self._intensity_min

    def toggle(self):
        sys.stderr.write("LED %s: toggle\n" % self._color)
        if self._intensity == self._intensity_min:
            return self.on()
        return self.off()

    def intensity(self, value=None):
        sys.stderr.write("LED %s: intensity %d\n" % (self._color, value))
        if value is None:
            return self._intensity

        self._intensity = value


class _Switch:

    _pressed = False

    def __init__(self, name, callable=None):
        self._name = name
        self._callable = callable

    def __call__(self):
        sys.stderr.write("SWITCH {} call > {}:\n".format(self._name, self._pressed))
        return self._pressed

    def callback(self, callable):
        sys.stderr.write("SWITCH {}: callback {}\n".format(self._name, callable))
        self._callable = callable

    def _press(self):
        sys.stderr.write("SWITCH {}: pressed\n".format(self._name))
        self._pressed = True

    def _release(self):
        sys.stderr.write("SWITCH {}: released\n".format(self._name))
        self._pressed = False

    def _update(self):
        if self._pressed and self._callable is not None:
            self._callable()


#
# pyb method and classes
#

def Accel():
    return _board.accel


def ADC(pin):
    """http://docs.micropython.org/en/latest/library/pyb.ADC.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def ADCAll(resolution):
    """http://docs.micropython.org/en/latest/library/pyb.ADC.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def CAN(bus):
    """http://docs.micropython.org/en/latest/library/pyb.CAN.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def DAC(port, bits=8):
    """http://docs.micropython.org/en/latest/library/pyb.DAC.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def ExtInt(pin, mode, pull, callback):
    """http://docs.micropython.org/en/latest/library/pyb.ExtInt.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


class I2C():
    """http://docs.micropython.org/en/latest/library/pyb.I2C.html"""

    MASTER = 0
    SLAVE = 1

    def __init__(self, bus):
        self._bus = bus

        # Pyboard connections
        if bus == 1:
            self._scl_pin_number = 'X9'
            self._sda_pin_number = 'X10'

            pass
        elif bus == 2:
            self._scl_pin_number = 'Y9'
            self._sda_pin_number = 'Y10'
            pass

        self._scl_pin = _Pin(self._scl_pin_number)
        self._sda_pin = _Pin(self._sda_pin_number)

        # Parameters
        self._mode = None
        self._addr = None
        self._baudrate = None
        self._gencall = None
        self._dma = None

    def deinit(self):
        """Turn off the I2C bus."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def init(self, mode, addr=0x12, baudrate=400000, gencall=False, dma=False):
        """

        Parameters
        ----------
        mode: str
            Must be either I2C.MASTER or I2C.SLAVE
        addr: str
            Only sensible for a slave.
            Optional, defaults to 0x12.
        baudrate: int
            SCL clock rate (only sensible for a master)
            Optional, defaults to 400000.
        gencall: bool
            Whether to support general call mode.
            Optional, defaults to False.
        dma: bool
            Whether to allow the use of DMA for the I2C transfers
            (note that DMA transfers have more precise timing but
            currently do not handle bus errors properly)
        """
        if self._mode in [self.MASTER, self.SLAVE]:
            self._mode = mode
        else:
            raise AttributeError('Must be either MASTER or Slave')

        self._addr = addr
        self._baudrate = baudrate
        self._gencall = gencall
        self._dma = dma

    def is_ready(self, addr):
        """Check if an I2C device responds to the given address.

        Only valid when in master mode.

        Parameters
        ----------
        addr : str
            I2C address (in hexadecimal) to be investigated.

        Returns
        -------
        out: bool
            True, if an I2C device responds to the given address.
            False, otherwise
        """
        # Todo: Other criterion?
        if self._addr == addr:
            return True
        else:
            return False

    def mem_read(self):
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def mem_write(self):
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def recv(self, recv, addr=0x00, timeout=5000):
        """

        Parameters
        ----------
        recv: int, bytearray
            Can be:
                an integer, which is the number of bytes to receive
                a mutable buffer, which will be filled with received bytes
        addr: str
            The address to receive from (only required in master mode)
        timeout: int
             Timeout in milliseconds to wait for the receive.

        Returns
        -------
        out : int, bytearray
            If recv is an integer then a new buffer of the bytes received.
            Otherwise, the same buffer that was passed in to recv.
        """

        if type(recv) is int:
            out = bytes(recv)
        elif type(recv) is bytearray:
            out = recv
        else:
            raise TypeError('Argument should be of type int or bytearray.')

        if self.is_ready(addr):
            raise OSError('Address not active.')

        return out

    def send(self, send, addr=0x00, timeout=5000):
        """

        Parameters
        ----------
        send: int, bytearray
            Data to send (an integer to send, or a buffer object)
        addr: bytearray
            The address to send to (only required in master mode)
        timeout: int
             Timeout in milliseconds to wait for the receive.
        """
        if type(send) not in [int, bytearray]:
            raise TypeError('Argument should be of type int or bytearray.')
        if addr is not self._addr:
            raise Exception("Address is not active.")

    def scan(self):
        """Scan all I2C addresses and return active ones.

        Scans addressed from 0x01 to 0x7f.
        Only valid when in master mode.

        Returns
        -------
        out : list
            List of I2C addresses that respond
        """
        active_addresses = []

        inspected_addresses_list = range(1, 128)
        for integer in inspected_addresses_list:
            address = hex(integer)
            if self.is_ready(address):
                active_addresses.append(address)

        return active_addresses


def LCD(skin_position):
    """http://docs.micropython.org/en/latest/library/pyb.LCD.html"""
    sys.stdout.write("PYB:LCD:%s\n" % skin_position)
    return _board.LCD(skin_position)


def LED(number):
    sys.stdout.write("PYB:LED:%s\n" % number)
    return _board.leds[number]


class _Pin:
    """http://docs.micropython.org/en/latest/library/pyb.Pin.html"""

    AF_OD = 0
    AF_PP = 1
    ANALOG = 2
    IN = 3
    OUT_OD = 4
    OUT_PP = 5
    PULL_DOWN = 6
    PULL_NONE = 7
    PULL_UP = 8

    def __init__(self, id):
        self._id = id

        self._mode = None
        self._pull = None
        self._af = None

        self._pin_value = bool(0)  # This corresponds to pin down.

    def __str__(self):
        return 'Emulated Pin object: {}'.format(self._id)

    def init(self, mode, pull=PULL_NONE, af=-1):
        self._mode = mode
        self._pull = pull
        self._af = af

    def value(self, value=None):
        # sys.stderr.write("Pin value: {}\n".format(value))
        if value is not None:
            self._pin_value = bool(value)
        else:
            return self._pin_value

    def af(self):
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def af_list(self):
        """Returns an array of alternate functions available for this pin."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def gpio(self):
        """Returns the base address of the GPIO block associated with this pin."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def mode(self):
        """Returns the currently configured mode of the pin.

        The integer returned will match one of the allowed constants for
         the mode argument to the init function."""
        return self._mode

    def name(self):
        """Get the pin name."""
        # Todo:
        return self._id

    def names(self):
        """Returns the cpu and board names for this pin."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def pin(self):
        """Get the pin number."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def port(self):
        """Get the pin port."""
        # Todo
        raise NotImplementedError(
            "Contribute on github.com/alej0varas/pybolator")

    def pull(self):
        """Returns the currently configured pull of the pin.

        The integer returned will match one of the allowed constants for
         the pull argument to the init function."""
        return self._pull


def PinAF(bus):
    """http://docs.micropython.org/en/latest/library/pyb.I2C.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


# http://docs.micropython.org/en/latest/library/pyb.Pin.html#class-pinaf-pin-alternate-functions


def RTC():
    """http://docs.micropython.org/en/latest/library/pyb.RTC.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def Servo(id):
    """http://docs.micropython.org/en/latest/library/pyb.Servo.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def SPI(bus, mode, **kwargs):
    """http://docs.micropython.org/en/latest/library/pyb.SPI.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def Timer(id, **kwargs):
    """http://docs.micropython.org/en/latest/library/pyb.Timer.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def UART(bus, baudrate):
    """http://docs.micropython.org/en/latest/library/pyb.UART.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def USB_VCP():
    """http://docs.micropython.org/en/latest/library/pyb.USB_VCP.html"""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def Switch():
    return _board.switch

#
# Time related functions
#

def delay(milliseconds):
    sys.stderr.write("PYB: delay %s\n" % milliseconds)
    sleep(milliseconds / 1000)


def udelay(us):
    sys.stderr.write("PYB: udelay %s\n" % us)
    sleep(us / 1000000)


def millis():
    sys.stderr.write("PYB: millis\n")
    result = micros() / 1000
    return result


def micros():
    sys.stderr.write("PYB: micros\n")
    delta = datetime.now() - _board.boot_time
    result = delta.total_seconds() * 1000000
    return result


def elapsed_millis(start):
    sys.stderr.write("PYB: elapsed_millis\n")
    result = elapsed_micros(start) / 1000
    return result


def elapsed_micros(start):
    sys.stderr.write("PYB: elapsed_micros\n")
    result = micros() - start
    return result


def hard_reset():
    """Resets the pyboard in a manner similar to pushing the external
    RESET button.
    """
    _board.boot()


def bootloader():
    """Activate the bootloader without BOOT\* pins."""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def disable_irq():
    """Disable interrupt requests.
    Returns the previous IRQ state: ``False``/``True`` for
    disabled/enabled IRQs respectively.  This return value can be
    passed to enable_irq to restore the IRQ to its original state.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def enable_irq(state=True):
    """Enable interrupt requests.
    If ``state`` is ``True`` (the default value) then IRQs are
    enabled.  If ``state`` is ``False`` then IRQs are disabled.  The
    most common use of this function is to pass it the value returned
    by ``disable_irq`` to exit a critical section.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def freq(sysclk=None, hclk=None, pclk1=None, pclk2=None):
    """If given no arguments, returns a tuple of clock frequencies:
    (sysclk, hclk, pclk1, pclk2).
    These correspond to:

        - sysclk: frequency of the CPU
        - hclk: frequency of the AHB bus, core memory and DMA
        - pclk1: frequency of the APB1 bus
        - pclk2: frequency of the APB2 bus

    If given any arguments then the function sets the frequency of the
    CPU, and the busses if additional arguments are given.
    Frequencies are given in Hz.  Eg freq(120000000) sets sysclk (the
    CPU frequency) to 120MHz.  Note that not all values are supported
    and the largest supported frequency not greater than the given
    value will be selected.

    Supported sysclk frequencies are (in MHz): 8, 16, 24, 30, 32, 36,
    40, 42, 48, 54, 56, 60, 64, 72, 84, 96, 108, 120, 144, 168.

    The maximum frequency of hclk is 168MHz, of pclk1 is 42MHz, and of
    pclk2 is 84MHz.  Be sure not to set frequencies above these
    values.

    The hclk, pclk1 and pclk2 frequencies are derived from the sysclk
    frequency using a prescaler (divider).  Supported prescalers for
    hclk are: 1, 2, 4, 8, 16, 64, 128, 256, 512.  Supported prescalers
    for pclk1 and pclk2 are: 1, 2, 4, 8.  A prescaler will be chosen
    to best match the requested frequency.

    A sysclk frequency of 8MHz uses the HSE (external crystal)
    directly and 16MHz uses the HSI (internal oscillator) directly.
    The higher frequencies use the HSE to drive the PLL (phase locked
    loop), and then use the output of the PLL.

    Note that if you change the frequency while the USB is enabled
    then the USB may become unreliable.  It is best to change the
    frequency in boot.py, before the USB peripheral is started.  Also
    note that sysclk frequencies below 36MHz do not allow the USB to
    function correctly.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def wfi():
    """Wait for an internal or external interrupt.

    This executes a ``wfi`` instruction which reduces power
    consumption of the MCU until any interrupt occurs (be it internal
    or external), at which point execution continues.  Note that the
    system-tick interrupt occurs once every millisecond (1000Hz) so
    this function will block for at most 1ms.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def stop():
    """Put the pyboard in a "sleeping" state.

    This reduces power consumption to less than 500 uA.  To wake from
    this sleep state requires an external interrupt or a
    real-time-clock event.  Upon waking execution continues where it
    left off.

    See :meth:`rtc.wakeup` to configure a real-time-clock wakeup
    event.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def standby():
    """Put the pyboard into a "deep sleep" state.

    This reduces power consumption to less than 50 uA.  To wake from
    this sleep state requires a real-time-clock event, or an external
    interrupt on X1 (PA0=WKUP) or X18 (PC13=TAMP1).  Upon waking the
    system undergoes a hard reset.

    See :meth:`rtc.wakeup` to configure a real-time-clock wakeup
    event.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def have_cdc():
    """Return True if USB is connected as a serial device, False otherwise.

    This function is deprecated.  Use pyb.USB_VCP().isconnected()
    instead.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def hid(buttons, x, y, z):
    """Takes a 4-tuple (or list) and sends it to the USB host (the PC) to
    signal a HID mouse-motion event.

    This function is deprecated.  Use pyb.USB_HID().send(...) instead.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def info(dump_alloc_table=None):
    """Print out lots of information about the board."""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def main(filename):
    """Set the filename of the main script to run after boot.py is
    finished.  If this function is not called then the default file
    main.py will be executed.

    It only makes sense to call this function from within boot.py.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def mount(device, mountpoint, *args, readonly=False, mkfs=False):
    """Mount a block device and make it available as part of the
    filesystem.  ``device`` must be an object that provides the block
    protocol:

    - ``readblocks(self, blocknum, buf)``
    - ``writeblocks(self, blocknum, buf)`` (optional)
    - ``count(self)``
    - ``sync(self)`` (optional)

    ``readblocks`` and ``writeblocks`` should copy data between
    ``buf`` and the block device, starting from block number
    ``blocknum`` on the device.  ``buf`` will be a bytearray with
    length a multiple of 512.  If ``writeblocks`` is not defined
    then the device is mounted read-only.  The return value of
    these two functions is ignored.

    ``count`` should return the number of blocks available on the
    device.
    ``sync``, if implemented, should sync the data on the device.

    The parameter ``mountpoint`` is the location in the root of the
    filesystem to mount the device.  It must begin with a
    forward-slash.

    If ``readonly`` is ``True``, then the device is mounted read-only,
    otherwise it is mounted read-write.

    If ``mkfs`` is ``True``, then a new filesystem is created if one
    does not already exist.

    To unmount a device, pass ``None`` as the device and the mount
    location as ``mountpoint``.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def repl_uart(uart):
    """Get or set the UART object where the REPL is repeated on."""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def rng():
    """Return a 30-bit hardware generated random number."""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def sync():
    """Sync all file systems."""
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


def unique_id():
    """Returns a string of 12 bytes (96 bits), which is the unique ID of
    the MCU.
    """
    raise NotImplementedError("Contribute on github.com/alej0varas/pybolator")


_board = _Board()
