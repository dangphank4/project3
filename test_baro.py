import ms5837 
import time 

sensor = ms5837.MS5837_30BA(bus=3)
if not sensor.init():
    print("❌ Failed to initialize MS5837 sensor")
else: 
    sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)  # freshwater
    print("✅ Depth sensor initialized (MS5837-30BA)")

    try:
        while True:
            if sensor.read():
                latest_depth = sensor.depth()
                print("Depth: ", latest_depth)
            time.sleep(0.05)
    except:
        print("⛔ Kết thúc")


