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

# Try to import gpiod for direct GPIO access
try:
    import gpiod
    GPIOD_AVAILABLE = True
except ImportError:
    GPIOD_AVAILABLE = False
    print("Debug: gpiod not available, DHT reads will fail")

BOARD = {
    3:    229,   # PH5/I2C3_SDA
    5:    228,   # PH4/I2C3_SCK
    7:    73,    # PC9
    8:    226,   # PH2/UART5_TX
    10:   227,   # PH3/UART5_RX
    11:   70,    # PC6
    12:   75,    # PC11
    13:   69,    # PC5
    15:   72,    # PC8
    16:   79,    # PC15
    18:   78,    # PC14
    19:   231,   # PH7,SPI1_MOSI
    21:   232,   # PH8,SPI1_MISO
    22:   71,    # PC7
    23:   230,   # PH6,SPI1_CLK
    24:   233,   # PH9,SPI1_CS
    26:   74,    # PC10
}

def _read_with_gpiod(sensor, gpio_line):
    """Read DHT sensor using gpiod library with proper DHT22 protocol"""
    
    if not GPIOD_AVAILABLE:
        raise RuntimeError("gpiod library not available - install with: pip3 install gpiod")
    
    print(f"Debug: Using gpiod to read GPIO line {gpio_line}")
    
    try:
        # Open GPIO chip 1 (where our GPIO lines are)
        try:
            chip = gpiod.Chip('gpiochip1')
        except (FileNotFoundError, OSError) as e:
            raise RuntimeError(f"Cannot access gpiochip1: {e}. Check if GPIO devices exist: ls -la /dev/gpio*")
        
        try:
            line = chip.get_line(gpio_line)
        except (ValueError, OSError) as e:
            chip.close()
            raise RuntimeError(f"Cannot access GPIO line {gpio_line}: {e}")
        
        # DHT22 Communication Protocol:
        # 1. Send start signal: pull low for 1-10ms, then high for 20-40us
        # 2. DHT responds with: low 80us, high 80us
        # 3. Data: 40 bits (5 bytes) - each bit starts with 50us low
        #    - 0 bit: 26-28us high
        #    - 1 bit: 70us high
        
        # Step 1: Send start signal
        line.request(consumer="dht_sensor", type=gpiod.LINE_REQ_DIR_OUT)
        line.set_value(1)  # Initially high
        time.sleep(0.5)   # Wait 500ms

        line.set_value(0)  # Pull low
        time.sleep(0.02)  # Hold low for 20ms (>1ms required)
        
        #line.set_value(1)  # Release to high
        #time.sleep(0.00004)  # Wait 40us
        
        # Step 2: Switch to input and read DHT response
        line.release()
        line.request(consumer="dht_sensor", type=gpiod.LINE_REQ_DIR_IN)
        
        # Wait for DHT to pull low (start of response)
        timeout_start = time.time()
        while line.get_value() == 1:
            if time.time() - timeout_start > 0.1:  # 100ms timeout
                #raise RuntimeError("DHT22 did not respond (no initial low)")
                return common.DHT_ERROR_GPIO, None, None
        
        # Wait for DHT to go high (end of initial low pulse ~80us)
        timeout_start = time.time()
        while line.get_value() == 0:
            if time.time() - timeout_start > 0.1:
                #raise RuntimeError("DHT22 stuck low")
                return common.DHT_ERROR_GPIO, None, None
        
        # Wait for DHT to go low again (end of initial high pulse ~80us)
        timeout_start = time.time()
        while line.get_value() == 1:
            if time.time() - timeout_start > 0.1:
                #raise RuntimeError("DHT22 stuck high")
                return common.DHT_ERROR_GPIO, None, None

        # Now read 40 bits of data
        bits = []
        for i in range(40):
            # Wait for start of bit (low to high transition)
            timeout_start = time.time()
            while line.get_value() == 0:
                if time.time() - timeout_start > 0.001:  # 1ms timeout per bit
                    #raise RuntimeError(f"Timeout waiting for bit {i} start")
                    return common.DHT_ERROR_TIMEOUT, None, None
            
            # Measure high pulse duration
            pulse_start = time.time()
            while line.get_value() == 1:
                if time.time() - pulse_start > 0.001:  # 1ms timeout
                    #raise RuntimeError(f"Bit {i} pulse too long")
                    return common.DHT_ERROR_TIMEOUT, None, None
            
            pulse_duration = time.time() - pulse_start
            
            # Determine if bit is 0 or 1 based on pulse duration
            # 0 bit: ~26-28us, 1 bit: ~70us
            # Use 50us as threshold
            if pulse_duration > 0.00005:  # 50us
                bits.append(1)
            else:
                bits.append(0)
        
        # Clean up
        line.release()
        chip.close()
        
        # Convert bits to bytes
        data = []
        for i in range(0, 40, 8):
            byte = 0
            for j in range(8):
                byte = (byte << 1) | bits[i + j]
            data.append(byte)
        
        # Validate checksum
        checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF
        if checksum != data[4]:
            #raise RuntimeError(f"DHT22 checksum error: calculated {checksum}, received {data[4]}")
            return common.DHT_ERROR_CHECKSUM, None, None
        
        # Convert to humidity and temperature
        humidity = ((data[0] << 8) | data[1]) / 10.0
        temperature = (((data[2] & 0x7F) << 8) | data[3]) / 10.0
        
        # Handle negative temperatures
        if data[2] & 0x80:
            temperature = -temperature
        
        return (common.DHT_SUCCESS, humidity, temperature)

    except Exception as e:
        raise RuntimeError(f"DHT22 protocol error: {e}")

def read(sensor, pin):
    # Validate pin is a valid GPIO.
    pin_int = int(pin)
    valid_pins = list(BOARD.keys())
    
    if pin_int not in valid_pins:
        raise ValueError(f'Pin {pin_int} is not a valid/working GPIO pin for Orange Pi. Valid pins: {valid_pins}')
    
    gpio_line = BOARD[pin_int]

    result, humidity, temp = _read_with_gpiod(sensor, gpio_line)
    if result in common.TRANSIENT_ERRORS:
        # Signal no result could be obtained, but the caller can retry.
        return (None, None)
    elif result == common.DHT_ERROR_GPIO:
        raise RuntimeError('Error accessing GPIO.')
    elif result != common.DHT_SUCCESS:
        # Some kind of error occured.
        raise RuntimeError('Error calling DHT test driver read: {0}'.format(result))
    return (humidity, temp)
