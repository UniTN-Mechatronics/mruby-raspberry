/***************************************************************************/
/*                                                                         */
/* raspberry.c - mruby gem provoding access to Raspberry Pi IOs            */
/* Copyright (C) 2015 Paolo Bosetti and Matteo Ragni,                      */
/* paolo[dot]bosetti[at]unitn.it and matteo[dot]ragni[at]unitn.it          */
/* Department of Industrial Engineering, University of Trento              */
/*                                                                         */
/* This library is free software.  You can redistribute it and/or          */
/* modify it under the terms of the GNU GENERAL PUBLIC LICENSE 2.0.        */
/*                                                                         */
/* This library is distributed in the hope that it will be useful,         */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/* Artistic License 2.0 for more details.                                  */
/*                                                                         */
/* See the file LICENSE                                                    */
/*                                                                         */
/***************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <sched.h>

#include "mruby.h"
#include "mruby/variable.h"
#include "mruby/string.h"
#include "mruby/data.h"
#include "mruby/class.h"
#include "mruby/value.h"

static int g_setup = 0;

/*
  ____                 _
 |  _ \ __ _ ___ _ __ | |__   ___ _ __ _ __ _   _
 | |_) / _` / __| '_ \| '_ \ / _ \ '__| '__| | | |
 |  _ < (_| \__ \ |_) | |_) |  __/ |  | |  | |_| |
 |_| \_\__,_|___/ .__/|_.__/ \___|_|  |_|   \__, |
                |_|                         |___/
*/
static mrb_value mrb_wiringPiSetup(mrb_state *mrb, mrb_value self) {
  mrb_value result;
  if ( 0 != g_setup) {
    result = mrb_false_value();
  }
  else {
    wiringPiSetup();
    g_setup = 1;
    result = mrb_true_value();
  }
  return result;
}

/*
   ____
  / ___|___  _ __ ___
 | |   / _ \| '__/ _ \
 | |__| (_) | | |  __/
  \____\___/|_|  \___|

*/
static mrb_value mrb_core_pinMode(mrb_state *mrb, mrb_value self) {
  mrb_int pin, mode;
  mrb_get_args(mrb, "ii", &pin, &mode);
  if (mode != INPUT && 
      mode != OUTPUT &&
      mode != PWM_OUTPUT &&
      mode != GPIO_CLOCK) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "Only INPUT, OUTPUT, PWM_OUTPUT, or GPIO_CLOCK allowed!");
  }
  pinMode(pin, mode);
  return mrb_nil_value();
}

static mrb_value mrb_core_pullUpDnControl(mrb_state *mrb, mrb_value self) {
  mrb_int pin, pud;
  mrb_get_args(mrb, "ii", &pin, &pud);
  if (pud != PUD_OFF && 
      pud != PUD_DOWN &&
      pud != PUD_UP) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "Only PUD_OFF, PUD_DOWN or PUD_UP allowed!");
  }
  pullUpDnControl(pin, pud);
  return mrb_nil_value();
}

static mrb_value mrb_core_digitalWrite(mrb_state *mrb, mrb_value self) {
  mrb_int pin, value;
  mrb_get_args(mrb, "ii", &pin, &value);
  if (value != HIGH && 
      value != LOW) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "Only HIGH or LOW allowed!");
  }
  digitalWrite(pin, value);
  return mrb_nil_value();
}

static mrb_value mrb_core_pwmWrite(mrb_state *mrb, mrb_value self) {
  mrb_int pin, value;
  mrb_get_args(mrb, "ii", &pin, &value);
  pwmWrite(pin, value);
  return mrb_nil_value();
}

static mrb_value mrb_core_digitalRead(mrb_state *mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);
  return mrb_fixnum_value(digitalRead(pin));
}

static mrb_value mrb_core_analogWrite(mrb_state *mrb, mrb_value self) {
  mrb_int pin, value;
  mrb_get_args(mrb, "ii", &pin, &value);
  analogWrite(pin, value);
  return mrb_nil_value();
}

static mrb_value mrb_core_analogRead(mrb_state *mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);
  return mrb_fixnum_value(analogRead(pin));
}

/*
  _____ _           _
 |_   _(_)_ __ ___ (_)_ __   __ _
   | | | | '_ ` _ \| | '_ \ / _` |
   | | | | | | | | | | | | | (_| |
   |_| |_|_| |_| |_|_|_| |_|\__, |
                            |___/

*/
static mrb_value mrb_timing_millis(mrb_state *mrb, mrb_value self) {
  return mrb_fixnum_value(millis());
}

static mrb_value mrb_timing_micros(mrb_state *mrb, mrb_value self) {
  return mrb_fixnum_value(micros());
}

static mrb_value mrb_timing_delay(mrb_state *mrb, mrb_value self) {
  mrb_int howLong;
  mrb_get_args(mrb, "i", &howLong);
  delay(howLong);
  return mrb_nil_value();
}

static mrb_value mrb_timing_delayMicroseconds(mrb_state *mrb, mrb_value self) {
  mrb_int howLong;
  mrb_get_args(mrb, "i", &howLong);
  delayMicroseconds(howLong);
  return mrb_nil_value();
}

/*
                  ____                  _  __ _          
                 / ___| _ __   ___  ___(_)/ _(_) ___ ___ 
                 \___ \| '_ \ / _ \/ __| | |_| |/ __/ __|
                  ___) | |_) |  __/ (__| |  _| | (__\__ \
                 |____/| .__/ \___|\___|_|_| |_|\___|___/
                       |_|                               
*/
static mrb_value mrb_specifics_digitalWriteByte(mrb_state *mrb, mrb_value self) {
  mrb_int value;
  mrb_get_args(mrb, "i", &value);
  digitalWriteByte((char)value);
  return mrb_nil_value();
}

static mrb_value mrb_specifics_pwmSetMode(mrb_state *mrb, mrb_value self) {
  mrb_int mode;
  mrb_get_args(mrb, "i", &mode);
  if (mode != PWM_MODE_BAL && mode != PWM_MODE_MS) {
    mrb_raise(mrb, E_RUNTIME_ERROR, "Only PWM_MODE_BAL or PWM_MODE_MS allowed!");
  }
  digitalWriteByte(mode);
  return mrb_nil_value();
}

static mrb_value mrb_specifics_pwmSetRange(mrb_state *mrb, mrb_value self) {
  unsigned int range;
  mrb_get_args(mrb, "i", &range);
  pwmSetRange(range);
  return mrb_nil_value();
}

static mrb_value mrb_specifics_pwmSetClock(mrb_state *mrb, mrb_value self) {
  mrb_int divisor;
  mrb_get_args(mrb, "i", &divisor);
  pwmSetRange(divisor);
  return mrb_nil_value();
}

static mrb_value mrb_specifics_piBoardRev(mrb_state *mrb, mrb_value self) {
  return mrb_fixnum_value(piBoardRev());
}

static mrb_value mrb_specifics_wpiPinToGpio(mrb_state *mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);
  return mrb_fixnum_value(wpiPinToGpio(pin));
}

static mrb_value mrb_specifics_physPinToGpio(mrb_state *mrb, mrb_value self) {
  mrb_int pin;
  mrb_get_args(mrb, "i", &pin);
  return mrb_fixnum_value(physPinToGpio(pin));
}





/*
 ____            _       _ 
/ ___|  ___ _ __(_) __ _| |
\___ \ / _ \ '__| |/ _` | |
 ___) |  __/ |  | | (_| | |
|____/ \___|_|  |_|\__,_|_|
                           
*/

static mrb_value mrb_serial_open(mrb_state *mrb, mrb_value self) {
  char *port_name = (char *)NULL;
  mrb_int port_len, baud;
  mrb_int device;
  mrb_value valid_rate;
  mrb_get_args(mrb, "si", &port_name, &port_len, &baud);
  device = serialOpen(port_name, baud);
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@baud"), mrb_fixnum_value(baud));
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@port"), mrb_str_new_cstr(mrb, port_name));  
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@device"), mrb_fixnum_value(device));
  valid_rate = mrb_funcall(mrb, self, "valid_rate?", 1, mrb_fixnum_value(baud));
  if (mrb_obj_eq(mrb, valid_rate, mrb_false_value())) {
    mrb_funcall(mrb, self, "close", 0);
    mrb_raise(mrb, E_RUNTIME_ERROR, "Unsupported baud rate");
  }
  return mrb_funcall(mrb, self, "open?", 0);
}

static mrb_value mrb_serial_close(mrb_state *mrb, mrb_value self) {
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  serialClose(dev);
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@device"), mrb_nil_value());
  return mrb_nil_value();
}

static mrb_value mrb_serial_putchar(mrb_state *mrb, mrb_value self) {
  char *str = (char *)NULL;
  char *str_len = 0;
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  mrb_get_args(mrb, "s", &str, &str_len);
  serialPutchar(dev, str[0]);
  return mrb_nil_value();
}

static mrb_value mrb_serial_puts(mrb_state *mrb, mrb_value self) {
  char *str = (char *)NULL;
  char *str_len = 0;
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  mrb_get_args(mrb, "s", &str, &str_len);
  serialPuts(dev, str);
  return mrb_nil_value();
}

static mrb_value mrb_serial_dataAvail(mrb_state *mrb, mrb_value self) {
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  return mrb_fixnum_value(serialDataAvail(dev));
}

static mrb_value mrb_serial_getchar(mrb_state *mrb, mrb_value self) {
  char str;
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  str = (char)serialGetchar(dev);
  return mrb_str_new_cstr(mrb, &str);
}

static mrb_value mrb_serial_flush(mrb_state *mrb, mrb_value self) {
  int dev = mrb_fixnum(mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, "@device")));
  serialFlush(dev);
  return mrb_nil_value();
}

/*
 ____       _            _ _         
|  _ \ _ __(_) ___  _ __(_) |_ _   _ 
| |_) | '__| |/ _ \| '__| | __| | | |
|  __/| |  | | (_) | |  | | |_| |_| |
|_|   |_|  |_|\___/|_|  |_|\__|\__, |
                               |___/ 
*/
static mrb_value mrb_priority_SetPri(mrb_state *mrb, mrb_value self) {
  mrb_int pri, result;
  struct sched_param sched ;
  int policy;
  
  mrb_get_args(mrb, "i", &pri);
  if (pri == 0) {
    policy = SCHED_OTHER;
    pri = 0;
  }
  else if(pri > 0 && pri < 100 ) {
    policy = SCHED_FIFO;
  }
  else  {
    mrb_raise(mrb, E_ARGUMENT_ERROR, "Priority level must be in [0, 99]");
  }
  memset (&sched, 0, sizeof(sched)) ;
  if (pri > sched_get_priority_max(policy))
    sched.sched_priority = sched_get_priority_max(policy) ;
  else
    sched.sched_priority = pri ;
  
  result = sched_setscheduler(0, policy, &sched);
  if (result != 0) {
    perror("System message");
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not set value");
  }
  return mrb_true_value();
}

static mrb_value mrb_priority_GetPri(mrb_state *mrb, mrb_value self) {
  mrb_int result;
  struct sched_param sched ;

  memset (&sched, 0, sizeof(sched)) ;
  
  result = sched_getparam(0, &sched);
  if (result != 0) {
    perror("System message");
    mrb_raise(mrb, E_RUNTIME_ERROR, "Could not get value");
  }
  return mrb_fixnum_value(sched.sched_priority);
}

/*
 ___ ____   ____ 
|_ _|___ \ / ___|
 | |  __) | |    
 | | / __/| |___ 
|___|_____|\____|
*/                 

#define IV_GET(name) mrb_iv_get(mrb, self, mrb_intern_cstr(mrb, (name)))
#define IV_SET(name, value)                                                    \
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, (name)), value)

static mrb_value mrb_i2c_init(mrb_state *mrb, mrb_value self) {
  mrb_int devId = 0, res = 0, fd = 0;  
  mrb_get_args(mrb, "i", &devId);
  res = wiringPiI2CSetup(devId);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  IV_SET("@fd", mrb_fixnum_value(fd));
  IV_SET("@device", mrb_fixnum_value(devId));
  return mrb_true_value();
}

static mrb_value mrb_i2c_read(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  mrb_int res;
  res = wiringPiI2CRead(fd);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}

static mrb_value mrb_i2c_write(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  int data;
  mrb_int res;
  mrb_get_args(mrb, "i", &data);
  res = wiringPiI2CWrite(fd, data);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}

static mrb_value mrb_i2c_read_reg_8(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  mrb_int res;
  int reg = 0;
  mrb_get_args(mrb, "i", &reg);
  res = wiringPiI2CReadReg8(fd, reg);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}

static mrb_value mrb_i2c_read_reg_16(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  mrb_int res;
  int reg = 0;
  mrb_get_args(mrb, "i", &reg);
  res = wiringPiI2CReadReg16(fd, reg);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}

static mrb_value mrb_i2c_write_reg_8(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  mrb_int res;
  int reg = 0, data = 0;
  mrb_get_args(mrb, "ii", &reg, &data);
  res = wiringPiI2CWriteReg8(fd, reg, data);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}

static mrb_value mrb_i2c_write_reg_16(mrb_state *mrb, mrb_value self) {
  int fd = mrb_fixnum(IV_GET("@fd"));
  mrb_int res;
  int reg = 0, data = 0;
  mrb_get_args(mrb, "ii", &reg, &data);
  res = wiringPiI2CWriteReg16(fd, reg, data);
  if (res < 0) {
    mrb_raise(mrb, E_RUNTIME_ERROR, strerror(errno));
  }
  return mrb_fixnum_value(res);
}



void mrb_mruby_raspberry_gem_init(mrb_state *mrb) {
  struct RClass *rasp, *core, *specifics, *timing, *serial, *i2c;
  rasp = mrb_define_module(mrb, "Raspberry");
  mrb_define_class_method(mrb, rasp, "setup", mrb_wiringPiSetup, MRB_ARGS_NONE());
  mrb_define_class_method(mrb, rasp, "set_priority", mrb_priority_SetPri, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, rasp, "priority", mrb_priority_GetPri, MRB_ARGS_NONE());
  
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "INPUT"), mrb_fixnum_value(INPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "OUTPUT"), mrb_fixnum_value(OUTPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PWM_OUTPUT"), mrb_fixnum_value(PWM_OUTPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "GPIO_CLOCK"), mrb_fixnum_value(GPIO_CLOCK));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "HIGH"), mrb_fixnum_value(HIGH));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "LOW"), mrb_fixnum_value(LOW));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_OFF"), mrb_fixnum_value(PUD_OFF));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_UP"), mrb_fixnum_value(PUD_UP));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_DOWN"), mrb_fixnum_value(PUD_DOWN));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_DOWN"), mrb_fixnum_value(PUD_DOWN));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PWM_MODE_BAL"), mrb_fixnum_value(PWM_MODE_BAL));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PWM_MODE_MS"), mrb_fixnum_value(PWM_MODE_MS));


  core = mrb_define_module_under(mrb, rasp, "Core");
  mrb_define_class_method(mrb, core, "pin_mode", mrb_core_pinMode, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "pull_mode", mrb_core_pullUpDnControl, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "digital_write", mrb_core_digitalWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "pwm_write", mrb_core_pwmWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "digital_read", mrb_core_digitalRead, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, core, "analog_write", mrb_core_analogWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "analog_read", mrb_core_analogRead, MRB_ARGS_REQ(1));
  
  specifics = mrb_define_module_under(mrb, rasp, "Specifics");
  mrb_define_class_method(mrb, specifics, "write_byte", mrb_specifics_digitalWriteByte, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, specifics, "pwd_set_mode", mrb_specifics_pwmSetMode, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, specifics, "pwm_set_range", mrb_specifics_pwmSetRange, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, specifics, "pwm_set_clock", mrb_specifics_pwmSetClock, MRB_ARGS_REQ(1));

  mrb_define_class_method(mrb, specifics, "board_rev", mrb_specifics_piBoardRev, MRB_ARGS_NONE());
  mrb_define_class_method(mrb, specifics, "wpi_pin", mrb_specifics_wpiPinToGpio, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, specifics, "phys_pin", mrb_specifics_physPinToGpio, MRB_ARGS_REQ(1));
  
  timing = mrb_define_module_under(mrb, rasp, "Timing");
  mrb_define_class_method(mrb, timing, "millis", mrb_timing_millis, MRB_ARGS_NONE());
  mrb_define_class_method(mrb, timing, "micros", mrb_timing_micros, MRB_ARGS_NONE());
  mrb_define_class_method(mrb, timing, "delay", mrb_timing_delay, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, timing, "delay_micro", mrb_timing_delayMicroseconds, MRB_ARGS_REQ(1));
  
  serial = mrb_define_class_under(mrb, rasp, "Serial", mrb->object_class);
  mrb_define_method(mrb, serial, "initialize", mrb_serial_open, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, serial, "close", mrb_serial_close, MRB_ARGS_NONE());
  mrb_define_method(mrb, serial, "put_char", mrb_serial_putchar, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serial, "puts", mrb_serial_puts, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serial, "data_avail", mrb_serial_dataAvail, MRB_ARGS_NONE());
  mrb_define_method(mrb, serial, "get_char", mrb_serial_getchar, MRB_ARGS_NONE());
  mrb_define_method(mrb, serial, "flush", mrb_serial_flush, MRB_ARGS_NONE());

  i2c = mrb_define_class_under(mrb, rasp, "I2C", mrb->object_class);
  mrb_define_method(mrb, i2c, "initialize", mrb_i2c_init, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, i2c, "read", mrb_i2c_read, MRB_ARGS_NONE());
  mrb_define_method(mrb, i2c, "write", mrb_i2c_write, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, i2c, "read_reg_8", mrb_i2c_read_reg_8, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, i2c, "read_reg_16", mrb_i2c_read_reg_16, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, i2c, "write_reg_8", mrb_i2c_write_reg_8, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, i2c, "write_reg_16", mrb_i2c_write_reg_16, MRB_ARGS_REQ(2));
}

void mrb_mruby_raspberry_gem_final(mrb_state *mrb) {}
