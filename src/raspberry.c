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
#include <wiringPi.h>
#include <wiringSerial.h>

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
  pinMode(pin, mode);
  return mrb_nil_value();
}

static mrb_value mrb_core_pullUpDnControl(mrb_state *mrb, mrb_value self) {
  mrb_int pin, pud;
  mrb_get_args(mrb, "ii", &pin, &pud);
  pullUpDnControl(pin, pud);
  return mrb_nil_value();
}

static mrb_value mrb_core_digitalWrite(mrb_state *mrb, mrb_value self) {
  mrb_int pin, value;
  mrb_get_args(mrb, "ii", &pin, &value);
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
static mrb_value mrb_specifics_piBoardRev(mrb_state *mrb, mrb_value self) {
  return mrb_fixnum_value(piBoardRev());
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
  mrb_get_args(mrb, "si", &port_name, &port_len, &baud);
  device = serialOpen(port_name, baud);
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@baud"), mrb_fixnum_value(baud));
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@port"), mrb_str_new_cstr(mrb, port_name));  
  mrb_iv_set(mrb, self, mrb_intern_cstr(mrb, "@device"), mrb_fixnum_value(device));
  return self;
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

// TO BE IMPROVED (VARARGS)
static mrb_value mrb_serial_printf(mrb_state *mrb, mrb_value self) {
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


void mrb_mruby_raspberry_gem_init(mrb_state *mrb) {
  struct RClass *rasp, *core, *specifics, *timing, *serial;
  rasp = mrb_define_module(mrb, "Raspberry");
  mrb_define_class_method(mrb, rasp, "setup", mrb_wiringPiSetup, MRB_ARGS_NONE());
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "INPUT"), mrb_fixnum_value(INPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "OUTPUT"), mrb_fixnum_value(OUTPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PWM_OUTPUT"), mrb_fixnum_value(PWM_OUTPUT));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "GPIO_CLOCK"), mrb_fixnum_value(GPIO_CLOCK));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "HIGH"), mrb_fixnum_value(HIGH));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "LOW"), mrb_fixnum_value(LOW));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_OFF"), mrb_fixnum_value(PUD_OFF));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_UP"), mrb_fixnum_value(PUD_UP));
  mrb_const_set(mrb, mrb_obj_value(rasp), mrb_intern_lit(mrb, "PUD_DOWN"), mrb_fixnum_value(PUD_DOWN));


  core = mrb_define_module_under(mrb, rasp, "Core");
  mrb_define_class_method(mrb, core, "pin_mode", mrb_core_pinMode, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "pull_mode", mrb_core_pullUpDnControl, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "digital_write", mrb_core_digitalWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "pwm_write", mrb_core_pwmWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "digital_read", mrb_core_digitalRead, MRB_ARGS_REQ(1));
  mrb_define_class_method(mrb, core, "analog_write", mrb_core_analogWrite, MRB_ARGS_REQ(2));
  mrb_define_class_method(mrb, core, "analog_read", mrb_core_analogRead, MRB_ARGS_REQ(1));
  
  specifics = mrb_define_module_under(mrb, rasp, "Specifics");
  mrb_define_class_method(mrb, specifics, "board_rev", mrb_specifics_piBoardRev, MRB_ARGS_NONE());
  
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
  // TO BE IMPROVED (VARARGS)
  mrb_define_method(mrb, serial, "printf", mrb_serial_printf, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, serial, "data_avail", mrb_serial_dataAvail, MRB_ARGS_NONE());
  mrb_define_method(mrb, serial, "get_char", mrb_serial_getchar, MRB_ARGS_NONE());
  mrb_define_method(mrb, serial, "flush", mrb_serial_flush, MRB_ARGS_NONE());
}

void mrb_mruby_raspberry_gem_final(mrb_state *mrb) {}
