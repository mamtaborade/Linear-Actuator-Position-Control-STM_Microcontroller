# Linear-Actuator-Position-Control-STM_Microcontroller
The Serial Monitor is showing live actuator control status:
![image alt](https://github.com/mamtaborade/Linear-Actuator-Position-Control-STM_Microcontroller/blob/1f039ef1abb8b4a928e0ae5e20196f3975a1ab18/Task1/OP_1.png)
🔹 Current Position = 3079
    This value comes from the position sensor (ADC input).
    It represents the actuator’s position reading.
🔹 Current Pressure = 2958
    This value comes from the pressure sensor (ADC input).
    It represents the pressure in the system.
🔹 Actuator: ON
    The motor is active (green LED ON).
    This means the actuator has not yet reached the defined POSITION_THRESHOLD (2000).
🔹 PPR Valve: OPEN
    The valve status (red LED ON) is OPEN, because the pressure (2958) is below the PRESSURE_THRESHOLD (2000).
    If the pressure exceeds the threshold, the valve status would change to CLOSED.

🔹Visual Output (LED Indicators on Board)
  Green LED (Motor LED): ON → actuator running.
  Red LED (Valve LED): ON → valve is open.

