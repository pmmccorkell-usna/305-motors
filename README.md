# 305-motors

Developed on Circuitpython 7.0.0-602 for RP2040

1. Drop python files on raspberry pico
2. Open matlab

3. In matlab:
a = ew305(##)    where ## is the COM ## of the serial port
a.send_commands(target)

______________________
<b>Example:</b>

+ ie:
    + a = ew305(15)
        + for COM 15
    + a.send_commands(100)

+ to see the data:
    + a.data
        + (struct array of position and time, typically 300-500 samples)



______________________
<b>Advanced:</b>

+ optional send_commands variables:
    + a.send_commands(target, runtime, Kp, Ki, Kd, dc_bias)

+ restart rp2040 through matlab:
    + a.reset_rp2040()

+ Uncomment lines 128-131 for automatic graphing of a.data

