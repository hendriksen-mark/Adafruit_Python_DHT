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

# This is a direct copy of what's in the Adafruit Python GPIO library:
#  https://raw.githubusercontent.com/adafruit/Adafruit_Python_GPIO/master/Adafruit_GPIO/Platform.py
# TODO: Add dependency on Adafruit Python GPIO and use its platform detect
# functions.

import platform
import re

# Platform identification constants.
UNKNOWN          = 0
RASPBERRY_PI     = 1
BEAGLEBONE_BLACK = 2
ORANGE_PI        = 3


def platform_detect():
    """Detect if running on the Raspberry Pi, Beaglebone Black or Orange Pi and return the
    platform type.  Will return RASPBERRY_PI, BEAGLEBONE_BLACK, ORANGE_PI or UNKNOWN."""
    # Handle Raspberry Pi and Orange Pi
    pi = pi_version()
    if pi is not None:
        return RASPBERRY_PI
    opi = opi_version()
    if opi is not None:
        return ORANGE_PI

    # Handle Beaglebone Black
    # TODO: Check the Beaglebone Black /proc/cpuinfo value instead of reading
    # the platform.
    plat = platform.platform()
    if plat.lower().find('armv7l-with-debian') > -1:
        return BEAGLEBONE_BLACK
    elif plat.lower().find('armv7l-with-ubuntu') > -1:
        return BEAGLEBONE_BLACK
    elif plat.lower().find('armv7l-with-glibc2.4') > -1:
        return BEAGLEBONE_BLACK
    elif plat.lower().find('armv7l-with-arch') > -1:
        return BEAGLEBONE_BLACK

    # Couldn't figure out the platform, just return unknown.
    return UNKNOWN


def pi_revision():
    """Detect the revision number of a Raspberry Pi, useful for changing
    functionality like default I2C bus based on revision."""
    # Revision list available at: http://elinux.org/RPi_HardwareHistory#Board_Revision_History
    with open('/proc/cpuinfo', 'r') as infile:
        for line in infile:
            # Match a line of the form "Revision : 0002" while ignoring extra
            # info in front of the revsion (like 1000 when the Pi was over-volted).
            match = re.match('Revision\s+:\s+.*(\w{4})$', line, flags=re.IGNORECASE)
            if match and match.group(1) in ['0000', '0002', '0003']:
                # Return revision 1 if revision ends with 0000, 0002 or 0003.
                return 1
            elif match:
                # Assume revision 2 if revision ends with any other 4 chars.
                return 2
        # Couldn't find the revision, throw an exception.
        raise RuntimeError('Could not determine Raspberry Pi revision.')


def opi_version():
    """Detect the version of the Orange Pi.  Returns either 1 or None
    depending on if it's an Orange Pi Zero 2 or not an Orange Pi.
    Note: This function currently only detects the Orange Pi Zero 2.
    If you need to detect other Orange Pi models, you can extend this function.
    Returns 1 for Orange Pi Zero 2, None for other models or if not an Orange Pi.
    """
    try:
        with open('/proc/device-tree/model', 'r') as infile:
            model = infile.read().strip().rstrip('\x00')
        if model == 'OrangePi Zero2':
            # Orange Pi Zero 2
            return 1
    except (IOError, OSError):
        pass
    return None

def pi_version():
    """Detect the version of the Raspberry Pi.  Returns either 1, 2, 3, 4, or None
    depending on if it's a Raspberry Pi 1 (model A, B, A+, B+),
    Raspberry Pi 2 (model B+), Raspberry Pi 3, Raspberry Pi 3 (model B+), Raspberry Pi 4
    or not a Raspberry Pi (None).
    """
    try:
        # Check /proc/cpuinfo for the Hardware field value.
        # 2708 is pi 1
        # 2709 is pi 2
        # 2835 is pi 3 or pi 2 (on some Pi 2 models)
        # 2837 is pi 3b+
        # 2711 is pi 4
        # Anything else is not a pi.
        with open('/proc/cpuinfo', 'r') as infile:
            cpuinfo = infile.read()
    except (IOError, OSError):
        return None
    
    # Match a line like 'Hardware   : BCM2709'
    match = re.search('^Hardware\s+:\s+(\w+)$', cpuinfo,
                      flags=re.MULTILINE | re.IGNORECASE)
    if match:
        # We have a Hardware field, use it for detection
        if match.group(1) == 'BCM2708':
            # Pi 1
            return 1
        elif match.group(1) == 'BCM2709':
            # Pi 2
            return 2
        elif match.group(1) == 'BCM2835':
            # Could be Pi 3 or Pi 2 (some Pi 2 models report BCM2835)
            # Check Model field to distinguish
            model_match = re.search('^Model\s+:\s+(.+)$', cpuinfo,
                                  flags=re.MULTILINE | re.IGNORECASE)
            if model_match and 'Pi 2' in model_match.group(1):
                return 2
            else:
                return 3
        elif match.group(1) == 'BCM2837':
            # Pi 3b+
            return 3
        elif match.group(1) == 'BCM2711':
            # Pi 4B
            return 4
        else:
            # Something else, not a pi
            return None
    else:
        # No Hardware field, try to match Model line for newer Pi models (like Pi 4)
        match = re.search('^Model\s+:\s+(.+)$', cpuinfo,
                      flags=re.MULTILINE | re.IGNORECASE)
        if match:
            # Check if it's a Raspberry Pi 4
            if 'Raspberry Pi' in match.group(1):
                # Try to extract version from model string
                if 'Pi 2' in match.group(1):
                    return 2
                elif 'Pi 3' in match.group(1):
                    return 3
                elif 'Pi 4' in match.group(1):
                    return 4
                else:
                    return 1  # Assume Pi 1 as fallback
            else:
                # Not a recognized Raspberry Pi model
                return None
        else:
            return None
