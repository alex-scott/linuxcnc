#!/usr/bin/env python
import ctypes
from gcode import calc_vertex_extents
from enum import IntEnum

class TrajLineType(IntEnum):
    NONE = 0
    TRAVERSE = 1
    FEED = 2
    RIGID_TAP = 3
    ARC_SEGMENT = 4
    DWELL = 5
    ## special record type inserted to change tool offsets
    TOOLOFFSET = 254

class TrajPointCtype(ctypes.Structure): ## 16 bytes
    _pack_ = 1 # disable memory align=pack
    _fields_ = [('x', ctypes.c_float),
                ('y', ctypes.c_float),
                ('z', ctypes.c_float),
                ## the following expected be packed to 32 bit
                ('line_type', ctypes.c_char),
                ('lineno_hi',  ctypes.c_uint16),
                ('lineno_lo',  ctypes.c_uint8 )
                ]

class TrajBufferFullException(Exception): pass

class TrajBufferChunk:
    Len = 1024 * 1024 ## 16Mb buffer

    def __init__(self):
        TrajBufferType = TrajPointCtype * self.Len
        self._buffer = TrajBufferType()
        self._last = 0

    def append(self, x:float, y:float, z:float, line_type:int, line_no:int):
        r = self._buffer[self._last]
        r.x = x; r.y = y; r.z = z
        r.line_type = line_type
        r.lineno_lo = line_no & 0xFFFF
        r.lineno_hi = line_no & 0xFF0000
        self._last += 1
        if self._last>=self.Len: raise TrajBufferFullException

    def dump(self):
        print("======= DUMP")
        for i in range(self._last):
            b = self._buffer[i]
            print(b.x, b.y, b.z, b.line_type, (b.lineno_hi * 65536 + b.lineno_lo) )

    _extents_cache = None
    _extents_cache_created_on = -1
    # it will return min/max values grouped by axis, then by line_type
    def calc_extents(self):
        if self._extents_cache_created_on != self._last:
            self._extents_cache = calc_vertex_extents( ( self._get_capsule(), self._last) )
            self._extents_cache_created_on = self._last
        return self._extents_cache

    def __len__(self):
        return self._last

    _capsule = None
    # return pointer to buffer packed for passing into C API (gcodemodule.cc)
    def _get_capsule(self):
        if not self._capsule:
            PyCapsule_Destructor = ctypes.CFUNCTYPE(None, ctypes.py_object)
            PyCapsule_New = ctypes.pythonapi.PyCapsule_New
            PyCapsule_New.restype = ctypes.py_object
            PyCapsule_New.argtypes = (ctypes.c_void_p, ctypes.c_char_p, PyCapsule_Destructor)
            self._capsule = PyCapsule_New(ctypes.byref( self._buffer) , None, PyCapsule_Destructor(0))
        return self._capsule

class TrajBuffer:
    def __init__(self):
        self._buffers = [TrajBufferChunk()]
        self._current = self._buffers[0]
        self._vertex_count = 0
        self._tool_offset = (0.0, 0.0, 0.0, 0, 0)

    def append(self, x:float, y:float, z:float, line_type:int, line_no:int):
        try:
            self._vertex_count += 1
            if line_type == TrajLineType.TOOLOFFSET:
                self._tool_offset = (x, y, z, line_type, line_no)
            self._current.append(x, y, z, line_type, line_no)
        except TrajBufferFullException:
           n = TrajBufferChunk()
           self._buffers.append(n)
           self._current = n
           n.append(*self._tool_offset) ## it is sticky will be applied to next buffer too

    def calc_extents(self):
        vmin = [ 3e33 for i in range(3)]
        vmax = [ -3e33 for i in range(3)]
        vmint = [ 3e33 for i in range(3)]
        vmaxt = [ -3e33 for i in range(3)]
        for b in self._buffers:
             nmin, nmax, nmint, nmaxt = b.calc_extents()
             for axis in range(3):
                 vmin[axis] = min(vmin[axis], nmin[axis])
                 vmax[axis] = max(vmax[axis], nmax[axis])
                 vmint[axis] = min(vmint[axis], nmint[axis])
                 vmaxt[axis] = max(vmaxt[axis], nmaxt[axis])
        return vmin, vmax, vmint, vmaxt

    def dump(self):
        for b in self._buffers: b.dump()


if __name__ == '__main__':
    TrajBufferChunk.Len = 3
    vb = TrajBuffer()
    vb.append(1, 2, 3, 1, 1)
    vb.append(1, 2, 3, TrajLineType.TOOLOFFSET, 2)
    vb.append(4, 5, 6, 1, 2)
    vb.append(7, 8, 9, 0, 3)
    vb.append(10, 11, 12, 1, 4)
    vb.append(13, 14, 15, 1, 5)
    #vb.dump()
    print(vb.calc_extents())

