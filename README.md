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
pip3 install -r requirements.txt
```

## How to use

The file main.py contains a simple UI for controlling the robot arm.
```bash
python3 main.py
```

Besides the UI, the mks_bus module can be used to control the robot arm from code.

Currently, it's a couple of classes that wrap python-can and do some
interpretation and convenience functionality for you (particularly parsing 
the bytes into/from values, and transactional messaging with `ask()`).

Some examples:
```python
from mks_bus import Bus
with Bus() as bus:  # for any optional params see mks_bus.py
    hundred_degrees = 20 / 360 * 16384  # 100 degrees in encoder units
    
    # Example: Send a command to which you expect a response
    resp = bus.ask(1, 'encoder')
    print(f"Values property containing 1 value for encoder response: {resp.values}")
    
    # Example: Send a command to which you expect a more specific response
    # You can also specify a timeout (default 5s)
    resp = bus.ask(1, 'move_by', [300, 20, hundred_degrees], answer_pattern=[2])
    print(f"Move complete (got 'move' message with value=2): {resp.values}")
    
    # If the timeout is exceeded, a TimeoutError is raised.
    try:
        # This will wait for a response with the status 2 (complete)
        bus.ask(1, 'move_by', [300, 20, -hundred_degrees], answer_pattern=[2], timeout=0.1)
        print("Motor move won the race against the timeout!")
    except TimeoutError as e:
        print(f"Movement took longer than 0.1 seconds: {e}")
    
    # After an undetermined outcome like this, you might want to flush out any 
    # remaining messages that are left from previous commands
    bus.flush()  # Gets rid of any 'move complete' message that came in later
    
    # Not recommended but possible: Send a command without waiting for a response
    # (this is similar to python-can's functionality, but with human-readable values)
    bus.send(1, 'move_by', [300, 2, hundred_degrees])  # Speed 300, accel 2, relative angle
    # See COMMANDS in mks_bus.py for a list of commands and their parameters
    msg = bus.receive()  # default timeout = 0.01s
    print(f"Received: '{msg.cmd_str}' with values {msg.values} (0: fail, 1: starting/stopping, 2: complete, 3: stopped by limit)")
    print(msg)
    
    # Using bus.receive_all(), you can slurp in all messages that are currently available
    for msg in bus.receive_all():
        print(f"Received: '{msg.cmd_str}' with values {msg.values}")
    
    # You can also wait for a specific response to a message sent with `send` earlier.
    # E.g. monitor the motor use by some other system component
    # (this will wait for the next message with the same command string)
    bus.flush()
    bus.send(1, 'move_by', [300, 2, -10*hundred_degrees])
    try:
        while True:
            resp = bus.wait_for(1, 'move_by')
            print(f"Motor move status updated: {resp.values}")
    except TimeoutError as e:
        # Remember that exceptions are nothing bad in Python, but another
        # way to control the flow of your program
        print(f"No updates from motor in the last 5s")
        
    # In `wait_for()`, similarly to `ask()`, you can also specify a timeout or a 
    # pattern for the response (optional parameters timeout and value_pattern)
```
