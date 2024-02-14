# MKS Control for Arctos Robot Arm

Convenience functionality for controlling the Arctos robot arm using
MKS Servo Drivers.

## Usage

See main.py for an example of how to use the library.

Currently, it's a couple of classes that wrap python-can and do some
interpretation for you. This might change.

Todos:
- [ ] Use `struct` package to parse CAN data
- [ ] Check possibility of simplifying to simple `send` and `receive` functions
- [ ] TBD