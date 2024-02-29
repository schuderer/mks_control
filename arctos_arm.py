# Axes
NUM_AXES = 6
AXES_ANGLE_RANGE = [4700, -32200, 22700, 1820, 2120, 2120]  # between 0 and this angle in degrees (may be negative)

AXES_CURRENT_LIMIT = [2000, 1200, 500, 600, 600, 600]  # in mA
AXES_SPEED_LIMIT = [600, 600, 600, 500, 300, 300, 300]  # in RPM
AXES_ACCEL_LIMIT = [150, 150, 150, 150, 150, 150, 150]  # change of RPM per second
AXES_MOVE_TIMEOUT = [25, 25, 25, 15, 10, 10]  # in seconds


# Homing
NATIVE = -1; CW = 1; CCW = 0; NONE = None
# We support a mixture of native endstop homing (value -1), sensorless homing (0, 1) or deactivated homing (None)
AXES_HOMING_DIRECTION = [NATIVE, CW, CCW, NATIVE, CW, CW]  # 1 for CW, 0 for CCW, None for no homing, -1 for endstop homing
AXES_HOMING_CURRENT_LIMIT = [550, 550, 500, 500, 500, 500]  # in mA
AXES_SAFE_HOMING_ANGLE = [AXES_ANGLE_RANGE[i] * factor for i, factor in enumerate([0, 0.25, 0.75, 0.5, 0.5, 0.5])]
# AXES_SAFE_HOMING_ANGLE[3] = -440  # special case for axis 3 to make it look neater after homing all axes
HOMING_ORDER = [5, 4, 2, 1, 3, 0]  # order in which to home the axes

# Others
ARM_DEMO_SEQUENCE = [[0.0, -8050.60546875, 17025.7763671875, 678.8671875, 1838.03466796875, 1059.873046875], [-1996.28173828125, -8086.46484375, 13008.955078125, 893.51806640625, 642.48046875, 110.98388671875]]
# [[-8.0419921875, -5762.63671875, 17774.97802734375, 910, 447.42919921875, 1060.13671875], [-1570.62744140625, -11576.75537109375, 10465.0927734375, 255.34423828125, 1561.13525390625, 2055.498046875]]