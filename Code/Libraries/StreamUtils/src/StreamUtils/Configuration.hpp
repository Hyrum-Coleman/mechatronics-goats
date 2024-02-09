// StreamUtils - github.com/bblanchon/ArduinoStreamUtils
// Copyright Benoit Blanchon 2019-2023
// MIT License

#pragma once

#ifndef STREAMUTILS_PRINT_FLUSH_EXISTS
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_SAMD) ||    \
    defined(ARDUINO_ARCH_AVR) ||                                      \
    (defined(ARDUINO_ARCH_ESP32) && ESP_ARDUINO_VERSION_MAJOR >= 2 && \
     ESP_ARDUINO_VERSION_PATCH >= 3) ||                               \
    (defined(ARDUINO_ARCH_STM32) && STM32_CORE_VERSION_MAJOR >= 2)
#define STREAMUTILS_PRINT_FLUSH_EXISTS 1
#else
#define STREAMUTILS_PRINT_FLUSH_EXISTS 0
#endif
#endif

#ifndef STREAMUTILS_STREAM_READBYTES_IS_VIRTUAL
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || \
    defined(ARDUINO_ARCH_STM32)
#define STREAMUTILS_STREAM_READBYTES_IS_VIRTUAL 1
#else
#define STREAMUTILS_STREAM_READBYTES_IS_VIRTUAL 0
#endif
#endif

#ifndef STREAMUTILS_PRINT_WRITE_VOID_UINT32
#if defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F4)
#define STREAMUTILS_PRINT_WRITE_VOID_UINT32 1
#else
#define STREAMUTILS_PRINT_WRITE_VOID_UINT32 0
#endif
#endif

#ifndef STREAMUTILS_ENABLE_EEPROM
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266) || \
    defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_PICO_MAJOR) || \
    defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY) ||        \
    defined(ARDUINO_ARCH_MEGAAVR)
#define STREAMUTILS_ENABLE_EEPROM 1
#else
#define STREAMUTILS_ENABLE_EEPROM 0
#endif
#endif

#ifndef STREAMUTILS_USE_EEPROM_COMMIT
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || \
    defined(ARDUINO_ARCH_RP2040)
#define STREAMUTILS_USE_EEPROM_COMMIT 1
#else
#define STREAMUTILS_USE_EEPROM_COMMIT 0
#endif
#endif

#ifndef STREAMUTILS_USE_EEPROM_UPDATE
#if defined(ARDUINO_ARCH_AVR) || defined(CORE_TEENSY) || \
    defined(ARDUINO_ARCH_MEGAAVR)
#define STREAMUTILS_USE_EEPROM_UPDATE 1
#else
#define STREAMUTILS_USE_EEPROM_UPDATE 0
#endif
#endif

#ifndef STREAMUTILS_STACK_BUFFER_MAX_SIZE
#define STREAMUTILS_STACK_BUFFER_MAX_SIZE 32
#endif
