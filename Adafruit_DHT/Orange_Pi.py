# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from . import common
import time
import OPi.GPIO as GPIO

#GPIO.setboard(GPIO.THREE)
GPIO.setmode(GPIO.BOARD)


class DHTResult:
    'DHT sensor result returned by DHT.read() method'

    ERR_NO_ERROR = 0
    ERR_MISSING_DATA = -3
    ERR_CRC = -2

    error_code = ERR_NO_ERROR
    temperature = -1
    humidity = -1

    def __init__(self, error_code, temperature, humidity):
        self.error_code = error_code
        self.temperature = temperature
        self.humidity = humidity

    def is_valid(self):
        return self.error_code == DHTResult.ERR_NO_ERROR


class DHT:
    'DHT sensor reader class for Orange Pi'

    __pin = 0

    def __init__(self, sensor=22, pin=16):
        self.__pin = pin
        # Sensor should be set to DHT11 or DHT22.
        if sensor in [22, 11]:
            self.__sensor = sensor
        else:
            raise ValueError('invalid sensor dht')

    def read(self, sensor=None, pin=None):
        if sensor is not None:
            self.__sensor = sensor
        if pin is not None:
            self.__pin = pin

        GPIO.setup(self.__pin, GPIO.OUT)

        # send initial high
        self.__send_and_sleep(1, 0.05)

        # pull down to low
        self.__send_and_sleep(0, 0.02)

        # change to input using pull up
        #gpio.setcfg(self.__pin, gpio.INPUT, gpio.PULLUP)
        GPIO.setup(self.__pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # collect data into an array
        data = self.__collect_input()

        # parse lengths of all data pull up periods
        pull_up_lengths = self.__parse_data_pull_up_lengths(data)

        # if bit count mismatch, return error (4 byte data + 1 byte checksum)
        # Fix issue on my Board with AM2301 to ensure at least the data is
        # available
        pull_up_lengths_size = len(pull_up_lengths)
        if (self.__sensor == 22 and pull_up_lengths_size < 40) or (self.__sensor == 11 and pull_up_lengths_size != 40):
            result = DHTResult(DHTResult.ERR_MISSING_DATA, 0, 0)
            return result.error_code, result.humidity, result.temperature

        # calculate bits from lengths of the pull up periods
        bits = self.__calculate_bits(pull_up_lengths)

        # we have the bits, calculate bytes
        the_bytes = self.__bits_to_bytes(bits)

        # calculate checksum and check
        checksum = self.__calculate_checksum(the_bytes)
        if the_bytes[4] != checksum:
            result = DHTResult(DHTResult.ERR_CRC, 0, 0)
            return result.error_code, result.humidity, result.temperature

        if self.__sensor == 22:
            # Compute to ensure negative values are taken into account
            c = (float)(((the_bytes[2] & 0x7F) << 8) + the_bytes[3]) / 10

            # ok, we have valid data, return it
            if (c > 125):
                c = the_bytes[2]

            if (the_bytes[2] & 0x80):
                c = -c

            result = DHTResult(DHTResult.ERR_NO_ERROR, c, ((the_bytes[0] << 8) + the_bytes[1]) / 10.00)
            return result.error_code, result.humidity, result.temperature
        else:
            # ok, we have valid data, return it
            result = DHTResult(DHTResult.ERR_NO_ERROR, the_bytes[2], the_bytes[0])
            return result.error_code, result.humidity, result.temperature

    def __send_and_sleep(self, output, sleep):
        GPIO.output(self.__pin, output)
        time.sleep(sleep)

    def __collect_input(self):
        # collect the data while unchanged found
        unchanged_count = 0

        # this is used to determine where is the end of the data
        max_unchanged_count = 100

        last = -1
        data = []
        while True:
            current = GPIO.input(self.__pin)
            data.append(current)
            if last != current:
                unchanged_count = 0
                last = current
            else:
                unchanged_count += 1
                if unchanged_count > max_unchanged_count:
                    break

        return data

    def __parse_data_pull_up_lengths(self, data):
        STATE_INIT_PULL_DOWN = 1
        STATE_INIT_PULL_UP = 2
        STATE_DATA_FIRST_PULL_DOWN = 3
        STATE_DATA_PULL_UP = 4
        STATE_DATA_PULL_DOWN = 5

        state = STATE_INIT_PULL_DOWN

        lengths = []  # will contain the lengths of data pull up periods
        current_length = 0  # will contain the length of the previous period

        for i in range(len(data)):

            current = data[i]
            current_length += 1

            if state == STATE_INIT_PULL_DOWN:
                if current == 0:
                    # ok, we got the initial pull down
                    state = STATE_INIT_PULL_UP
                    continue
                else:
                    continue
            if state == STATE_INIT_PULL_UP:
                if current == 1:
                    # ok, we got the initial pull up
                    state = STATE_DATA_FIRST_PULL_DOWN
                    continue
                else:
                    continue
            if state == STATE_DATA_FIRST_PULL_DOWN:
                if current == 0:
                    # we have the initial pull down, the next will be the data
                    # pull up
                    state = STATE_DATA_PULL_UP
                    continue
                else:
                    continue
            if state == STATE_DATA_PULL_UP:
                if current == 1:
                    # data pulled up, the length of this pull up will determine
                    # whether it is 0 or 1
                    current_length = 0
                    state = STATE_DATA_PULL_DOWN
                    continue
                else:
                    continue
            if state == STATE_DATA_PULL_DOWN:
                if current == 0:
                    # pulled down, we store the length of the previous pull up
                    # period
                    lengths.append(current_length)
                    state = STATE_DATA_PULL_UP
                    continue
                else:
                    continue

        return lengths

    def __calculate_bits(self, pull_up_lengths):
        # find shortest and longest period
        shortest_pull_up = 1000
        longest_pull_up = 0

        for i in range(0, len(pull_up_lengths)):
            length = pull_up_lengths[i]
            if length < shortest_pull_up:
                shortest_pull_up = length
            if length > longest_pull_up:
                longest_pull_up = length

        # use the halfway to determine whether the period it is long or short
        halfway = shortest_pull_up + (longest_pull_up - shortest_pull_up) / 2
        bits = []

        for i in range(0, len(pull_up_lengths)):
            bit = False
            if pull_up_lengths[i] > halfway:
                bit = True
            bits.append(bit)

        return bits

    def __bits_to_bytes(self, bits):
        the_bytes = []
        byte = 0

        for i in range(0, len(bits)):
            byte = byte << 1
            if (bits[i]):
                byte = byte | 1
            else:
                byte = byte | 0
            if ((i + 1) % 8 == 0):
                the_bytes.append(byte)
                byte = 0

        return the_bytes

    def __calculate_checksum(self, the_bytes):
        return the_bytes[0] + the_bytes[1] + the_bytes[2] + the_bytes[3] & 255

def read(sensor, pin):
    # Validate GPIO and map it to GPIO base and number.
    # Validate pin is a valid GPIO.
    if pin is None or int(pin) < 0 or int(pin) > 31:
        raise ValueError('Pin must be a valid GPIO number 0 to 31.')
    # Get a reading from C driver code.
    result, humidity, temp = DHT.read(sensor, int(pin))
    if result in common.TRANSIENT_ERRORS:
        # Signal no result could be obtained, but the caller can retry.
        return (None, None)
    elif result == common.DHT_ERROR_GPIO:
        raise RuntimeError('Error accessing GPIO.')
    elif result != common.DHT_SUCCESS:
        # Some kind of error occured.
        raise RuntimeError('Error calling DHT test driver read: {0}'.format(result))
    return (humidity, temp)
