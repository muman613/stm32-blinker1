# Readme

## Project Overview

This repository contains sample STM32 firmware for button detection.

This firmware demonstrates programming using FreeRTOS along with STM32 timers and PWM control of LED.

1) Press the blue button.
   1) LED light turns on
2) Release the blue button.
   1) LED remains on
3) Long press (~2s) blue button.
   1) LED light fades from full on to full off

### Timers used

| Timer | Description |
| ----- | ----------- |
| TIM2  | PWM Generation for LED fade. |
| TIM3  | Used to determine long-press. |
| TIM4  | Used to fade the LED |

## Hardware Used

* STM32 Nucleo-F446RE Evaluation Kit

