# mruby-raspberry
mruby gem for accessing Raspberry IO.

##Intro
mruby-raspberry is a wrapper around [wiringpi library](http://wiringpi.com). At the moment, it support most of the original basic functionalities, excluding I2C, SPI, Shift, Software PWM, and Software Tone.

##Requirements and Building
The [wiringpi library v2](http://wiringpi.com) needs to be installed before using this mruby-gem.

### Testing
To do a quick test, after installing WiringPi, proceed as follows:

```sh
$ git clone --depth 1 https://github.com/uniTN-Mechatronics/mruby-raspberry.git
$ cd mruby-raspberry
$ ./run_test.rb           # this also clones mruby master
$ sudo tmp/mruby/bin/mirb # sudo needed for accessing hw pins
```

### Adding to your mruby
To use it within your custom mruby, just add `conf.gem :github => 'UniTN-Mechatronics/mruby-raspberry', :branch => 'master'` in your `build_config.rb`

##Usage
The C functions of the original [WiringPi library](http://wiringpi.com/reference/) are grouped under the module `Raspberry`. Each of the categories in which the WiringPi functions are collected is implemented either as a submodule of `Raspberry` or as a class when it makes sense:

- `Raspberry::Core`: module
- `Raspberry::Specifics`: module
- `Raspberry::Timing`: module
- `Raspberry::Serial`: class

The `Raspberry` module only has one function, `setup()`, which maps the `wiringPiSetup` C call. **Note**: if you `include Raspberry`, the `setup` function is automatically called; otherwise, you explicitly need to call it for the rest to work.

The following is the list of the implemented functions/methods. For details, see the original WiringPi documentation.

### Raspberry::Core
- `pin_mode(pin, mode)`, mode is one of the constants `INPUT`, `OUTPUT`, `PWM_OUTPUT`, or `GPIO_CLOCK`
- `pull_mode(pin, mode)`, mode is one of the constants `PUD_OFF`, `PUD_DOWN` or `PUD_UP`
- `digital_write(pin, level)`, level is one of the constants `HIGH`, `LOW`
- `pwm_write(pin, value)`
- `digital_read(pin)`
- `analog_write(pin, value)`
- `analog_read(pin)`

### Raspberry::Specifics
- `write_byte(value)`, value is casted into a 8 bit int
- `pwd_set_mode()`, mode is one of the constants `PWM_MODE_BAL` or `PWM_MODE_MS`
- `pwm_set_range(value)`
- `pwm_set_clock(value)`
- `board_rev()`, returns either 1 or 2
- `wpi_pin()`, converts WirnigPI pin number into GPIO pin number
- `phys_pin()`, converts physical pin number into GPIO pin number

### Raspberry::Timing
- `millis()`
- `micro()`
- `delay(value)`
- `delay_micro(value)`

### Raspberry::Serial
- `Serial.valid_rate?(rate)`, returns true if rate is a standard valid baud rate
- `Serial.new(port, baud)`, creates and opens a serial port connection
- `Serial#close()`
- `Serial#put_char(chr)`, only the first character of `chr` is used
- `Serial#puts(str)`
- `Serial#data_avail()`
- `Serial#get_char()`
- `Serial#flush()`
- `Serial#printf(fmt, ...)`