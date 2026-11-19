"""Parameter declaration shared by the road-following nodes (stdlib only).

Both `road_follow_node` and `road_goal_node` consume the SAME tuning dicts from
`road_follow_math` (TRACE_/FOLLOW_/CORNER_/GOAL_DEFAULTS). Hand-copying a
parallel `PARAM_TYPES` table per node drifted twice: `bridge_frac` and
`open_frac` were added to the defaults on the board and were missing from the
node's type table, so the node raised KeyError at startup until patched by hand
(2026-11-18), and the 2026-11-19 perspective params would have done it again.

The declared ROS type is derivable from the default value, so there is nothing
to copy. The only subtlety is that `bool` is a subclass of `int` in Python, so
it must be tested first -- otherwise every boolean parameter would be declared
as an integer and `--ros-args -p x:=true` would fail to parse.
"""

from rcl_interfaces.msg import ParameterDescriptor


def param_type(default):
    """The Python type rclpy should declare `default` as."""
    if isinstance(default, bool):
        return bool
    if isinstance(default, int):
        return int
    if isinstance(default, float):
        return float
    if isinstance(default, str):
        return str
    raise TypeError(f"unsupported parameter default {default!r} ({type(default)})")


def declare_all(node, defaults, read=True):
    """Declare every entry of `defaults` with its derived type.

    Returns the dict of read values when `read` is true (the nodes keep one
    plain dict and pass it to the pure-math functions unchanged).
    """
    values = {}
    for name, default in defaults.items():
        t = param_type(default)
        node.declare_parameter(name, t(default), ParameterDescriptor())
        if read:
            value = node.get_parameter(name).value
            # rclpy hands floats back as float and ints as int; str/bool are
            # exact. Coerce so a parameter file cannot smuggle a type the pure
            # math does not expect (e.g. an int where a division needs float).
            values[name] = t(value)
    return values
