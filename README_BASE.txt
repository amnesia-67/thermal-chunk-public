CSV OUTPUT MAPPED:  time_s, PV, SP, A[0..], B[0..], mode, duty%

Field	Description
time_s	Elapsed time since boot (in seconds).
PV	Process Variable — the temperature currently being controlled (from the selected sensor).
SP	Setpoint — the user-defined target temperature in °C.
A[0..]	Temperatures (°C) from all sensors on Bus A, separated by the `
B[0..]	Temperatures (°C) from all sensors on Bus B, separated by the `
mode	Current operating mode: H = Heating, C = Cooling, S = Stopped/Idle.
duty%	PWM duty cycle applied to the BTS7960 driver (0–100 %).