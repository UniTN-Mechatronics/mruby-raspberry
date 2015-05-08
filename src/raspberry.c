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
#include "mruby.h"
#include "mruby/variable.h"
#include "mruby/string.h"
#include "mruby/data.h"
#include "mruby/class.h"
#include "mruby/value.h"

static int g_setup = 0;

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

/* ------------------------------------------------------------------------*/
void mrb_mruby_raspberry_gem_init(mrb_state *mrb) {
  struct RClass *rasp, *core;
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
}

void mrb_mruby_raspberry_gem_final(mrb_state *mrb) {}
