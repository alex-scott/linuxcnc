#!/usr/bin/env python
import linuxcnc
import gcode

import fastcanon

class MinimalCanon(object):
    def __init__(self):
        linuxcnc.stat().poll() ## without this we get core dump of lcnc not running

    def comment(self, msg):   pass

    def message(self, msg):   pass

    def check_abort(self): return False

    def next_line(self, st): pass

    def set_g5x_offset(self, index, x, y, z, a, b, c, u, v, w): pass

    def set_g92_offset(self, x, y, z, a, b, c, u, v, w): pass

    def set_xy_rotation(self, rotation): pass

    def tool_offset(self, xo, yo, zo, ao, bo, co, uo, vo, wo): pass

    def set_spindle_rate(self, speed):   pass

    def set_feed_rate(self, feed_rate): pass

    def select_plane(self, plane): pass

    def change_tool(self, pocket): pass

    def get_tool(self, pocket):
        return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0

    def straight_traverse(self, x, y, z, a, b, c, u, v, w): pass

    def rigid_tap(self, x, y, z): pass

    def set_plane(self, plane): pass

    def arc_feed(self, end_x, end_y, center_x, center_y, rot, end_z, a, b, c, u, v, w): pass

    def straight_arcsegments(self, segs): pass

    def straight_feed(self, x, y, z, a, b, c, u, v, w): pass

    def straight_probe(self, x, y, z, a, b, c, u, v, w): pass

    def user_defined_function(self, i, p, q): pass

    def dwell(self, arg): pass

    def get_external_angular_units(self):
        return 1.0

    def get_external_length_units(self):
        return 1.0

    def get_axis_mask(self):
        return 7

    def get_block_delete(self):
        return 0

    def get_optimized_callbackp(self):
        return {
            #"get_block_delete" : None
        }


canon = MinimalCanon()
canon.parameter_file = "/home/alex/dev/linuxcnc/configs/sim/axis/sim_mm.var"
fn = "/home/alex/dev/test3.ngc"
result, seq = gcode.parse(fn, canon, "G0", "G21")
print("finished", result, seq)

#v = fastcanon.Fastcanon()
#v.first = "111"
#print(v.first)