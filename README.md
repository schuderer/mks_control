# MKS Control for Arctos Robot Arm

Convenience functionality for controlling the Arctos robot arm using
MKS Servo Drivers.

# Installation

Clone the repository.
```bash
git clone https://github.com/schuderer/mks_control.git
cd mks_control
```

If you have a virtual environment activated, continue to the next step.
If not, create a virtual environment and activate it.
```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the dependencies using pip.
```bash
pip install -r requirements.txt
```

## How to use

The file main.py contains a simple UI for controlling the robot arm.
```bash
python3 main.py
```

If you ignore all the user interface stuff and the state machine, you
can see that importing the mks_bus module and using the
Bus class is just a few commands like:
```python
from mks_bus import Bus
with Bus() as bus:  # for any optional params see mks_bus.py
    # Tell can_id 1 device to move (0xF4)
    # Speed 300, accel 2, relative angle of 20deg
    bus.send(1, "move_by", [300, 2, 20 / 360 * 16384])
    # See COMMANDS in mks_bus.py for a list of commands and their parameters
    msg = bus.receive()  # default timeout = 0.01s
    print(f"Received: {msg.cmd_str} with values {msg.values} (0: fail, 1: starting/stopping, 2: complete, 3: stopped by limit)")
    print(msg)
    for axis in range(1, 7):
        bus.send(axis, "encoder")
    for msg in bus.receive_all():
        angle = msg.values[0] / 16384 * 360
        print(msg)
```

Currently, it's a couple of classes that wrap python-can and do some
interpretation for you (particularly parsing the bytes into values).
