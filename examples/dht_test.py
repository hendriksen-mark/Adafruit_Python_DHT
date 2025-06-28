import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(lineno)d - %(levelname)s - %(message)s'
)

try:
    import Adafruit_DHT  # type: ignore
except ImportError:
    logging.warning("Adafruit_DHT module not found. Using dummy implementation.")
    class DummyDHT:
        DHT22 = None
        @staticmethod
        def read_retry(sensor, pin):
            return 22.0, 50.0
    Adafruit_DHT = DummyDHT()

# NOTE: DHT_PIN uses orangepi.zero2.BOARD numbering for Orange Pi Zero 2
# This gives access to many more GPIO pins than standard BOARD mapping
sensor = Adafruit_DHT.DHT22
# DHT_PIN is the BOARD pin number for Orange Pi Zero 2
# With orangepi.zero2 mapping, many pins are available: 3, 5, 7, 8, 10, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26
DHT_PIN = 7  # Physical pin 7 - works with orangepi.zero2.BOARD mapping

def read_dht_temperature() -> tuple:
    """
    Read the temperature and humidity from the DHT sensor.
    Returns (humidity, temperature) or (None, None) if failed.
    """
    try:
        humidity, temperature = Adafruit_DHT.read_retry(sensor, DHT_PIN)
        logging.debug(f"Raw sensor values - Humidity: {humidity}, Temperature: {temperature}")

        if humidity is None and temperature is None:
            logging.warning("Both humidity and temperature are None. Check wiring, power, and pin number.")

        if temperature is not None and -40.0 < temperature < 80.0:
            logging.debug(f"Valid Temperature: {temperature}")
        else:
            logging.warning("Temperature value not updated (None or out of range)")
            return None, None

        if humidity is not None and 0.0 <= humidity <= 100.0:
            logging.debug(f"Valid Humidity: {humidity}")
        else:
            logging.warning("Humidity value not updated (None or out of range)")
            return None, None

        return humidity, temperature
    except Exception as e:
        logging.error(f"Error reading DHT sensor: {e}")
        return None, None

def main():
    """
    Main function to read and log the DHT sensor data.
    """
    logging.info(f"Starting DHT sensor reading on pin {DHT_PIN} (orangepi.zero2.BOARD numbering)...")
    for attempt in range(5):
        humidity, temperature = read_dht_temperature()
        if humidity is not None and temperature is not None:
            logging.info(f"Humidity: {humidity}%, Temperature: {temperature}°C")
            break
        else:
            logging.warning(f"Attempt {attempt+1}: Failed to read from DHT sensor. Retrying in 2 seconds...")
            import time
            time.sleep(2)
    else:
        logging.error("All attempts to read from DHT sensor failed. Check wiring, power, and pin number.")

if __name__ == "__main__":
    main()
