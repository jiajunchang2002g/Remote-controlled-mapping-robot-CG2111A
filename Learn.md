# Learning Outcomes

This project combines a Linux host (Raspberry Pi) and an Arduino MCU to control a mobile robot over a custom serial protocol. The outcomes below map directly to the codebase.

## C++ and Embedded C/C++
- Multi-file organization with clear header/source separation on the Pi and Arduino sides.
- Fixed-size packet layout and struct-based messaging for consistent cross-device data exchange.
- Low-level interrupt handling and register-level setup for encoder inputs on the MCU.

## Communication and Protocol Design
- Serial framing with magic numbers and checksums to detect corruption.
- Command/response protocol with explicit packet types, commands, and status payloads.
- Threaded host-side receive loop to process asynchronous device responses.

## Low-Latency and Real-Time Control
- Interrupt-driven encoder ticks to minimize latency in motion feedback.
- Deterministic packet sizes and simple checksum to reduce parsing overhead.
- Direct motor control primitives to keep actuation responsive.

## Systems Programming (Linux + MCU)
- POSIX serial configuration and I/O on Linux using termios and file descriptors.
- Concurrency primitives on the host (pthread, semaphores) for non-blocking I/O.
- Atomic sections on the MCU to protect shared buffers and counters.

## Engineering Practice
- Clear partitioning of responsibilities between host orchestration and embedded control.
- Protocol consistency shared across platforms for integration reliability.
- Built-in telemetry and status reporting to support testing and verification.
