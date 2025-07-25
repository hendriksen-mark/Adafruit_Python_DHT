from setuptools import setup, Extension
import sys
import os

# Try to import platform_detect, but handle the case where it's not available during build
try:
    import Adafruit_DHT.platform_detect as platform_detect
except ImportError:
    # During build, the package might not be available yet
    # Define constants locally
    class PlatformDetect:
        UNKNOWN = 0
        RASPBERRY_PI = 1
        BEAGLEBONE_BLACK = 2
        ORANGE_PI = 3
        
        @staticmethod
        def platform_detect():
            # Default to unknown during build - will be overridden by env var or flags
            return PlatformDetect.UNKNOWN
            
        @staticmethod
        def pi_version():
            return 2  # Default to Pi 2 driver for compatibility
    
    platform_detect = PlatformDetect()


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

# Check if an explicit platform was chosen with a command line parameter or environment variable.
# Kind of hacky to manipulate the argument list before calling setup, but it's
# the best simple option for adding optional config to the setup.
platform = platform_detect.UNKNOWN
pi_version = None

# Check environment variables first (more modern approach)
force_platform = os.environ.get('ADAFRUIT_DHT_FORCE_PLATFORM', '').lower()
if force_platform == 'pi':
    platform = platform_detect.RASPBERRY_PI
    pi_version = 1
elif force_platform == 'pi2':
    platform = platform_detect.RASPBERRY_PI
    pi_version = 2
elif force_platform == 'bbb':
    platform = platform_detect.BEAGLEBONE_BLACK
elif force_platform == 'opi':
    platform = platform_detect.ORANGE_PI
elif force_platform == 'test':
    platform = 'TEST'
# Then check command line arguments (legacy support)
elif '--force-pi' in sys.argv:
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
    try:
        platform = platform_detect.platform_detect()
    except Exception as e:
        # If platform detection fails during build, default to unknown
        print(f"Platform detection failed: {e}")
        platform = platform_detect.UNKNOWN

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
    if platform == platform_detect.UNKNOWN:
        print('Could not detect platform. For testing or cross-compilation, set ADAFRUIT_DHT_FORCE_PLATFORM environment variable or use --force-* flags.')
        print('Available options:')
        print('  ADAFRUIT_DHT_FORCE_PLATFORM=test (for testing)')
        print('  ADAFRUIT_DHT_FORCE_PLATFORM=pi (for Raspberry Pi)')
        print('  ADAFRUIT_DHT_FORCE_PLATFORM=pi2 (for Raspberry Pi 2+)')
        print('  ADAFRUIT_DHT_FORCE_PLATFORM=bbb (for Beaglebone Black)')
        print('  ADAFRUIT_DHT_FORCE_PLATFORM=opi (for Orange Pi)')
        # Don't exit during build - just skip extensions
        print('Continuing without platform-specific extensions...')
    else:
        print('Could not detect if running on the Raspberry Pi, Beaglebone Black or Orange Pi. If this failure is unexpected, you can run again with --force-pi, --force-bbb or --force-opi parameter to force using the Raspberry Pi, Beaglebone Black or Orange Pi respectively.')
        sys.exit(1)

# Call setuptools setup function to install package.
# Most metadata is now in pyproject.toml, setup.py only handles extensions
setup(ext_modules=extensions)
