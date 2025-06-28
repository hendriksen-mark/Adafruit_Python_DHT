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

# Configure GPIO for Orange Pi Zero 2
# Use orangepi.zero2 mapping for better pin availability
try:
    import orangepi.zero2
    GPIO.setmode(orangepi.zero2.BOARD)
    AVAILABLE_PINS = [3, 5, 7, 8, 10, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26]
except ImportError:
    # Fallback to standard BOARD mode
    GPIO.setmode(GPIO.BOARD)
    AVAILABLE_PINS = [8, 10]  # Only pins available with standard mapping


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
        
        # Validate pin number based on available pins
        if pin not in AVAILABLE_PINS:
            available_pins_str = ", ".join(map(str, AVAILABLE_PINS))
            raise ValueError(f'Pin {pin} is not available for GPIO on Orange Pi Zero 2. Available pins: {available_pins_str}')
        
        self.__pin = pin
        # Sensor should be set to DHT11 or DHT22.
        if sensor in [22, 11]:
            self.__sensor = sensor
        else:
            raise ValueError('invalid sensor dht')
        

    def set_GPIO_mode(self, mode=GPIO.OUT, pull_up_down=None):
        try:
            # Ensure GPIO mode is set and disable warnings
            GPIO.setwarnings(False)
            
            # Clean up any existing configuration for this pin
            try:
                GPIO.cleanup(self.__pin)
            except:
                print(f"Debug: Failed to clean up pin {self.__pin}, continuing setup.")
                pass
            
            try:
                GPIO.setmode(orangepi.zero2.BOARD)
                print(f"Debug: Using orangepi.zero2.BOARD mapping for pin {self.__pin}")
            except ImportError:
                GPIO.setmode(GPIO.BOARD)
                print(f"Debug: Using standard GPIO.BOARD mapping for pin {self.__pin}")
            
            # Add a small delay to ensure pin is ready
            #time.sleep(0.1)
            
            # Setup GPIO with optional pull-up/down
            if pull_up_down is not None:
                # Convert pull_up_down value to readable string
                pud_strings = {
                    GPIO.PUD_OFF: "PUD_OFF",
                    GPIO.PUD_DOWN: "PUD_DOWN", 
                    GPIO.PUD_UP: "PUD_UP"
                }
                pud_text = pud_strings.get(pull_up_down, f"UNKNOWN({pull_up_down})")
                
                GPIO.setup(self.__pin, mode, pull_up_down=pull_up_down)
                print(f"Debug: Pin {self.__pin} configured as {'OUTPUT' if mode == GPIO.OUT else 'INPUT'} with pull_up_down={pud_text}")
            else:
                GPIO.setup(self.__pin, mode)
                print(f"Debug: Pin {self.__pin} configured as {'OUTPUT' if mode == GPIO.OUT else 'INPUT'} without pull-up/down")
        except Exception as e:
            print(f"Debug: Error in initial GPIO setup: {e}")
            raise

    def read(self):
        self.set_GPIO_mode()
        
        # send initial high
        self.__send_and_sleep(GPIO.HIGH, 0.5)

        # pull down to low
        self.__send_and_sleep(GPIO.LOW, 0.02)

        # change to input using pull up
        # Clean up the pin configuration before changing mode
        self.set_GPIO_mode(mode=GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # collect data into an array
        data = self.__collect_input()
        
        # Debug: analyze the data pattern
        high_count = data.count(1)
        low_count = data.count(0)
        print(f"Debug: Collected {len(data)} data points - HIGH: {high_count}, LOW: {low_count}")
        
        # Check if we have proper signal levels
        if low_count == 0:
            print("Debug: ERROR - No LOW states detected!")
            print("Debug: DHT22 has 4.7kΩ pull-up (resistor marked '472') + Orange Pi weak pull-down = insufficient")
            print("Debug: Still getting 2.5V 'low' which is too high for digital logic")
            print("Debug: SOLUTION: Add external pull-down resistor:")
            print("  - 1.0kΩ pull-down → 0.58V LOW (ideal)")
            print("  - 1.5kΩ pull-down → 0.80V LOW (good)")
            print("  - 2.2kΩ pull-down → 1.06V LOW (marginal)")
            print("  Circuit: DHT Data pin → Orange Pi pin 8")
            print("                           ↓")
            print("                    1kΩ-1.5kΩ resistor")
            print("                           ↓")
            print("                          GND")
            result = DHTResult(DHTResult.ERR_MISSING_DATA, 0, 0)
            GPIO.cleanup(self.__pin)
            return result.error_code, result.humidity, result.temperature
            
        if high_count == 0:
            print("Debug: ERROR - No HIGH states detected!")
            print("Debug: External pull-down resistor is too strong for 4.7kΩ pull-up")
            print("Debug: With 4.7kΩ pull-up, you can use stronger pull-down:")
            print("  - If using <1kΩ: Try 1kΩ-1.5kΩ instead")
            print("  - If using 1kΩ: Circuit should work, check connections")
            print("  - Calculate: V_LOW = 3.3V × R_down / (4.7kΩ + R_down)")
            result = DHTResult(DHTResult.ERR_MISSING_DATA, 0, 0)
            GPIO.cleanup(self.__pin)
            return result.error_code, result.humidity, result.temperature
            
        print(f"Debug: Good signal levels detected - proceeding with DHT decoding")

        # parse lengths of all data pull up periods
        pull_up_lengths = self.__parse_data_pull_up_lengths(data)

        # if bit count mismatch, return error (4 byte data + 1 byte checksum)
        # Fix issue on my Board with AM2301 to ensure at least the data is
        # available
        pull_up_lengths_size = len(pull_up_lengths)
        if (self.__sensor == 22 and pull_up_lengths_size < 40) or (self.__sensor == 11 and pull_up_lengths_size != 40):
            print(f"Debug: Insufficient data - expected 40 bits, got {pull_up_lengths_size}")
            result = DHTResult(DHTResult.ERR_MISSING_DATA, 0, 0)
            GPIO.cleanup(self.__pin)  # Clean up on error
            return result.error_code, result.humidity, result.temperature

        # calculate bits from lengths of the pull up periods
        bits = self.__calculate_bits(pull_up_lengths)

        # we have the bits, calculate bytes
        the_bytes = self.__bits_to_bytes(bits)

        # calculate checksum and check
        checksum = self.__calculate_checksum(the_bytes)
        if the_bytes[4] != checksum:
            result = DHTResult(DHTResult.ERR_CRC, 0, 0)
            GPIO.cleanup(self.__pin)  # Clean up on error
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
            GPIO.cleanup(self.__pin)
            return result.error_code, result.humidity, result.temperature
        else:
            # ok, we have valid data, return it
            result = DHTResult(DHTResult.ERR_NO_ERROR, the_bytes[2], the_bytes[0])
            GPIO.cleanup(self.__pin)
            return result.error_code, result.humidity, result.temperature

    def __send_and_sleep(self, output, sleep):
        '''Send output to the GPIO pin and sleep for a specified duration.
        This method is used to control the timing of the signal sent to the DHT sensor.
        Args:
            output (int): The GPIO output value to send (GPIO.HIGH or GPIO.LOW).
            sleep (float): The duration to sleep after sending the output, in seconds.
        '''

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
        
        # Debug: track state transitions
        state_changes = []

        for i in range(len(data)):

            current = data[i]
            current_length += 1

            if state == STATE_INIT_PULL_DOWN:
                if current == 0:
                    # ok, we got the initial pull down
                    state = STATE_INIT_PULL_UP
                    state_changes.append(f"i={i}: INIT_PULL_DOWN -> INIT_PULL_UP")
                    continue
                else:
                    continue
            if state == STATE_INIT_PULL_UP:
                if current == 1:
                    # ok, we got the initial pull up
                    state = STATE_DATA_FIRST_PULL_DOWN
                    state_changes.append(f"i={i}: INIT_PULL_UP -> DATA_FIRST_PULL_DOWN")
                    continue
                else:
                    continue
            if state == STATE_DATA_FIRST_PULL_DOWN:
                if current == 0:
                    # we have the initial pull down, the next will be the data
                    # pull up
                    state = STATE_DATA_PULL_UP
                    state_changes.append(f"i={i}: DATA_FIRST_PULL_DOWN -> DATA_PULL_UP")
                    continue
                else:
                    continue
            if state == STATE_DATA_PULL_UP:
                if current == 1:
                    # data pulled up, the length of this pull up will determine
                    # whether it is 0 or 1
                    current_length = 0
                    state = STATE_DATA_PULL_DOWN
                    state_changes.append(f"i={i}: DATA_PULL_UP -> DATA_PULL_DOWN")
                    continue
                else:
                    continue
            if state == STATE_DATA_PULL_DOWN:
                if current == 0:
                    # pulled down, we store the length of the previous pull up
                    # period
                    lengths.append(current_length)
                    state = STATE_DATA_PULL_UP
                    state_changes.append(f"i={i}: DATA_PULL_DOWN -> DATA_PULL_UP (length={current_length})")
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
    # Validate GPIO pin for Orange Pi Zero 2
    if pin is None:
        raise ValueError('Pin cannot be None.')
    
    pin_int = int(pin)
    valid_pins = [3, 5, 7, 8, 10, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26]  # Confirmed working GPIO pins
    
    if pin_int not in valid_pins:
        raise ValueError(f'Pin {pin_int} is not a valid/working GPIO pin for Orange Pi Zero 2. Valid pins: {valid_pins}')
    
    # Create DHT instance and get a reading
    try:
        dht = DHT(sensor, pin_int)
        result, humidity, temp = dht.read()
        
        if result in common.TRANSIENT_ERRORS:
            # Signal no result could be obtained, but the caller can retry.
            return (None, None)
        elif result == common.DHT_ERROR_GPIO:
            raise RuntimeError('Error accessing GPIO.')
        elif result != common.DHT_SUCCESS:
            # Some kind of error occured.
            raise RuntimeError('Error calling DHT test driver read: {0}'.format(result))
        return (humidity, temp)
    except Exception as e:
        # Debug: print the actual error to help diagnose
        import traceback
        traceback.print_exc()
        raise