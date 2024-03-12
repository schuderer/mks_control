# Axes
NUM_AXES = 6
AXES_ANGLE_RANGE = [4896, -32200, 22700, 1990.8, 2120, 2120]  # between 0 and this angle in degrees (may be negative)

AXES_CURRENT_LIMIT = [2000, 1200, 500, 600, 600, 600]  # in mA
AXES_SPEED_LIMIT = [600, 600, 600, 500, 300, 300, 300]  # in RPM
AXES_ACCEL_LIMIT = [150, 150, 150, 150, 150, 150, 150]  # change of RPM per second
AXES_MOVE_TIMEOUT = [30, 25, 25, 15, 10, 10]  # in seconds

AXES_ANGLE_RATIO = [  # motor degrees per joint degree (or motor turns per joint turn)
    # row: input at motor shaft, column: result at joint
    [13.6,    0,    0,  0   ,  0   , 0   ],
    [ 0  ,  144,    0,  0   ,  0   , 0   ],
    [ 0  ,    0,  114, 22.60,  0   , 0   ],  # 15468.8-5056.8=10412 moves joint 4 by 90 joint degrees = 1441.0-980.2=460.8 motor degrees; 10412/460.8=22.60
    [ 0  ,    0,    0,  5.53,  0   , 0   ],
    [ 0  ,    0,    0,  0   ,  4.65, 4.65],  # 1873-1060=813 moves joint 5 by 90 joint degrees = 813/4.65=175 motor degrees
    [ 0  ,    0,    0,  0   ,  4.65, 4.65],
]

KINEMATIC_CONVENTION = 'classic'  # 'classic', 'modified' or 'offset'
KINEMATIC_CHAIN = [
    # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    # alpha,       a,  theta_offset,      d
    #(degrees)   (mm)      (degrees)    (mm)
    # From reconstruction/own drawings of coordinate systems (classic)
    [-90,   20.174,    0,  287.87 ],  # Link 1
    [  0,  260.986,  -90,    0    ],  # Link 2  # changed theta to -90 for a better default pose
    [-90,   19.219,  -90,    0    ],  # Link 3
    [ 90,    0    ,    0,  260.753],  # Link 4
    [-90,    0    ,    0,    0    ],  # Link 5
    [  0,    0    ,    0,   74.745],  # Fixed link

    # original from the thesis (modified DH parameter convention)
    #     [      0,      0,             0, 287.87],  # Link 1
    #     [    -90, 20.174,           -90,      0],  # Link 2
    #     [      0,260.986,             0,      0],  # Link 3
    #     [      0, 19.219,             0,260.753],  # Link 4
    #     [     90,      0,             0,      0],  # Link 5
    #     [    -90,      0,           180, 74.745],  # Link 6
]
FIXED_LINKS = []  # Indices of links that will be hidden and do not change with theta (just for fixed transformations, e.g. extending the end effector position)
JOINT_BOUND = [(-45, 45), (-45, 45), (-45, 45), (-45, 45), (-45, 45), (-45, 45)]  # in joint degrees
JOINT_ZERO_OFFSET = [0, 0, 0, 0, 0, 0]  # Distance between the homing zero pose and the 0 joint degrees stance (in joint degrees)
# Vertical pose (ca.)
# 6: can_id: None, state: stopped (1) , angle: 1059.9, timestamp: 20:55:35.407, shaft_lock: False
# 5: can_id: None, state: stopped (1) , angle: 1119.7, timestamp: 20:55:35.368, shaft_lock: False
# 4: can_id: None, state: stopped (1) , angle: 1441.0, timestamp: 20:55:35.329, shaft_lock: False
# 3: can_id: None, state: stopped (1) , angle: 5056.8, timestamp: 20:55:35.293, shaft_lock: False
# 2: can_id: None, state: stopped (1) , angle: -12273.7, timestamp: 20:55:35.257, shaft_lock: False
# 1: can_id: None, state: stopped (1) , angle:    0.0, timestamp: 20:55:35.221, shaft_lock: False

# Homing
# We support a mixture of native endstop homing (value -1), sensorless homing (0, 1) or deactivated homing (None)
ENDSTOP = -1; CW = 1; CCW = 0; NONE = None
AXES_HOMING_DIRECTION = [ENDSTOP, CW, CCW, ENDSTOP, CW, CW]
AXES_HOMING_CURRENT_LIMIT = [450, 450, 450, 450, 450, 450]  # in mA
AXES_SAFE_HOMING_ANGLE = [AXES_ANGLE_RANGE[i] * factor for i, factor in enumerate([0, 0.25, 0.75, 0.5, 0.5, 0.5])]
# order in which to home the axes (homing 3 twice to prevent collisions at first and to make it look neat in the end)
HOMING_ORDER = [5, 4, 3, 2, 1, 3, 0]

# Others
ARM_DEMO_SEQUENCE = [[0.0, -5733.17138671875, 20418.1787109375, 800.244140625, 1059.71923828125, 78.99169921875], [-1793.60595703125, -10919.2236328125, 6865.927734375, 1360.458984375, -619.541015625, -812.2412109375], [-1793.6279296875, -10919.24560546875, 6865.90576171875, 1360.458984375, -1459.51171875, -1727.666015625]]
# [[0.0, -8050.60546875, 17025.7763671875, 678.8671875, 1838.03466796875, 1059.873046875], [-1996.28173828125, -8086.46484375, 13008.955078125, 893.51806640625, 642.48046875, 110.98388671875]]
# [[-8.0419921875, -5762.63671875, 17774.97802734375, 910, 447.42919921875, 1060.13671875], [-1570.62744140625, -11576.75537109375, 10465.0927734375, 255.34423828125, 1561.13525390625, 2055.498046875]]