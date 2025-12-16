import math
import time
from smbus2 import SMBus

class SEN0291:
    def __init__(self, i2c_bus=3, i2c_address=0x45):
        self.address = i2c_address
        self.bus = SMBus(i2c_bus)

    def read_voltage(self):
        """
        Read bus voltage from INA219 on SEN0291.
        Bus voltage register is at 0x02, returns voltage in 4 mV units.
        """
        reg = 0x02  # Bus voltage register
        raw = self.bus.read_word_data(self.address, reg)

        # Swap bytes (INA219 sends data in little-endian format)
        raw_swapped = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)

        # Bits [0:2] are flags, so shift right 3 bits
        voltage_raw = raw_swapped >> 3

        # Each unit = 4 mV
        voltage = voltage_raw * 0.004
        return voltage
    
    def estimate_soc(self, batt_voltage):
        v_cell = batt_voltage / 4
        if v_cell <= 3.2:
            return 0
        elif v_cell >= 4.2:
            return 100
        else:
            return round(100 * (v_cell - 3.2), 1)

# Example usage
if __name__ == "__main__":
    sensor = SEN0291()
    while True:
        voltage = sensor.read_voltage()
        remain_cap = sensor.estimate_soc(voltage)
        print(f"Voltage: {voltage:.1f} V | Capacity: {remain_cap:.1f}")
        time.sleep(1)