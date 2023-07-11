.. _lora-sample:

LoRa Radio Sample
###########################

Overview
********

This is a simple application demonstrating a LoRa Radio functionality.

Upon pressing a button the device will transmit a message with a unique byte.

When not transmitting any data the device will be listening for incoming LoRa frames.

Upon receiving a frame if it contains the unique byte the device will toggle its LED.


Requirements
************

This sample has been tested on the LoRa-E5 Dev Board.

Building and Running
********************

The following commands build the application.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/nrf/mesh/onoff-app
   :board: lora_e5_dev_board
   :goals: build flash
   :compact:
