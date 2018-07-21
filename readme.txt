Arduino-based PWM fan-controller

mode: cooling mode (0 - quiet / logarithmic, 1 - default / linear, 2 - performance / exponential)
start: temperature to start PWM fan
max: temperature to trigger fullspeed
poll: polling interval (ms)
err: max number of accepted failed polls
stats: monitoring (0 - disable, 1 - one time, 2 - continuous)
echo: print back input buffer (0 - disable, 1 - enable)
info: show current config
help: show built-in help

depends on:
OneWire - https://github.com/PaulStoffregen/OneWire
DallasTemperature - https://github.com/milesburton/Arduino-Temperature-Control-Library/
