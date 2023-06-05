# Stepper-Indexer
ESP32 Stepper Indexer for a ESP32 WiFi Heltec board

:Stable release: V1.a.1.

:Status: Released.

:Maintainer: sjames@hydrasystems.com

:Description: Stepper motor indexer to produce Step, Direction and Enable signals to a driver module. Note that this code generates only Enable/Step & Direction signals, it does not generate phased coil signals.

Key Features

    Written using the Eclipse IDE and the ESP IDF framework including FreeRTOS:
        RS232 and Bluetooth interfaces for all motor commands - Absolute/Incremental/Jog.
        Two button interface for simple Jog movement.
        Includes simple Heltec display functions.
        'S' curve generation for correct acceleration/deceleration profiles.

    Hardware:
        Heltec WiFi Kit 32 V1.0-V2.1.

To Do
Known Issues

    None.

Required Repositories

    None

Required Hardware

    None. Recommended: Mobile phone with BLE Monitor installed and/or PC with RS232 capability.

Note

I use a tab size of 2 for all source code. Using anything else may look weird.

Support

Problems can be reported using the issues tab in github.
