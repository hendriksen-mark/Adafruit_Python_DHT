from setuptools import setup, find_packages, Extension
import os
import sys

import Adafruit_DHT.platform_detect as platform_detect


BINARY_COMMANDS = [
    'build_ext',
    'build_clib',
    'bdist',
    'bdist_dumb',
    'bdist_rpm',
    'bdist_wininst',
    'bdist_wheel',
    'install'
]


def is_binary_install():
    do_binary = [command for command in BINARY_COMMANDS if command in sys.argv]
    return len(do_binary) > 0


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

# Check if an explicit platform was chosen with a command line parameter.
# Kind of hacky to manipulate the argument list before calling setup, but it's
# the best simple option for adding optional config to the setup.
platform = platform_detect.UNKNOWN
pi_version = None
if '--force-pi' in sys.argv:
    platform = platform_detect.RASPBERRY_PI
    pi_version = 1
    sys.argv.remove('--force-pi')
elif '--force-pi2' in sys.argv:
    platform = platform_detect.RASPBERRY_PI
    pi_version = 2
    sys.argv.remove('--force-pi2')
elif '--force-bbb' in sys.argv:
    platform = platform_detect.BEAGLEBONE_BLACK
    sys.argv.remove('--force-bbb')
elif '--force-opi' in sys.argv:
    platform = platform_detect.ORANGE_PI
    sys.argv.remove('--force-opi')
elif '--force-test' in sys.argv:
    platform = 'TEST'
    sys.argv.remove('--force-test')
else:
    # No explicit platform chosen, detect the current platform.
    platform = platform_detect.platform_detect()

# Pick the right extension to compile based on the platform.
extensions = []
install_requires = []
if not is_binary_install():
    print('Skipped loading platform-specific extensions for Adafruit_DHT (we are generating a cross-platform source distribution).')
elif platform == platform_detect.RASPBERRY_PI:
    # Get the Pi version (1 or 2)
    if pi_version is None:
        pi_version = platform_detect.pi_version()
    # Build the right extension depending on the Pi version.
    if pi_version == 1:
        extensions.append(Extension("Adafruit_DHT.Raspberry_Pi_Driver",
                                    ["source/_Raspberry_Pi_Driver.c", "source/common_dht_read.c", "source/Raspberry_Pi/pi_dht_read.c", "source/Raspberry_Pi/pi_mmio.c"],
                                    libraries=['rt'],
                                    extra_compile_args=['-std=gnu99']))
    elif pi_version == 2:
        extensions.append(Extension("Adafruit_DHT.Raspberry_Pi_2_Driver",
                                    ["source/_Raspberry_Pi_2_Driver.c", "source/common_dht_read.c", "source/Raspberry_Pi_2/pi_2_dht_read.c", "source/Raspberry_Pi_2/pi_2_mmio.c"],
                                    libraries=['rt'],
                                    extra_compile_args=['-std=gnu99']))
    elif (pi_version == 3) | (pi_version == 4):
        extensions.append(Extension("Adafruit_DHT.Raspberry_Pi_2_Driver",
                                    ["source/_Raspberry_Pi_2_Driver.c", "source/common_dht_read.c", "source/Raspberry_Pi_2/pi_2_dht_read.c", "source/Raspberry_Pi_2/pi_2_mmio.c"],
                                    libraries=['rt'],
                                    extra_compile_args=['-std=gnu99']))
    else:
        raise RuntimeError('Detected Pi version that has no appropriate driver available.')
elif platform == platform_detect.BEAGLEBONE_BLACK:
    extensions.append(Extension("Adafruit_DHT.Beaglebone_Black_Driver",
                                ["source/_Beaglebone_Black_Driver.c", "source/common_dht_read.c", "source/Beaglebone_Black/bbb_dht_read.c", "source/Beaglebone_Black/bbb_mmio.c"],
                                libraries=['rt'],
                                extra_compile_args=['-std=gnu99']))
elif platform == platform_detect.ORANGE_PI:
    pass
elif platform == 'TEST':
    extensions.append(Extension("Adafruit_DHT.Test_Driver",
                                ["source/_Test_Driver.c", "source/Test/test_dht_read.c"],
                                extra_compile_args=['-std=gnu99']))
else:
    print('Could not detect if running on the Raspberry Pi, Beaglebone Black or Orange Pi. If this failure is unexpected, you can run again with --force-pi, --force-bbb or --force-opi parameter to force using the Raspberry Pi, Beaglebone Black or Orange Pi respectively.')
    sys.exit(1)

classifiers = ['Development Status :: 4 - Beta',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

# Determine install requirements based on platform
install_requires = []

# Add platform-specific dependencies
if platform == platform_detect.ORANGE_PI:
    install_requires.append('gpiod>=1.5.0')  # Required for Orange Pi GPIO control

# Call setuptools setup function to install package.
setup(name              = 'Adafruit_DHT',
      version           = '1.4.0',
      author            = 'Tony DiCola',
      author_email      = 'tdicola@adafruit.com',
      description       = 'Library to get readings from the DHT11, DHT22, and AM2302 humidity and temperature sensors on a Raspberry Pi, Beaglebone Black, or Orange Pi. Enhanced with gpiod support for Orange Pi Zero 2.',
      long_description  = read('README.md'),
      license           = 'MIT',
      classifiers       = classifiers,
      url               = 'https://github.com/adafruit/Adafruit_Python_DHT/',
      packages          = find_packages(),
      install_requires  = install_requires,
      ext_modules       = extensions)
