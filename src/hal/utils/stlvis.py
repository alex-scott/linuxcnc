#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os,sys
from optparse import Option, OptionParser
import stltool
import numpy as np
import matplotlib.pyplot as pp


options = [Option( '-d','--debug', action='store_true', dest='debug', help="enable debug output"),
	   Option( '-b','--boundary', dest='boundary', help="set plotting axes, like -b x0,y0,x1,y1"),
          ]

def main():
    global opts
    (progdir, progname) = os.path.split(sys.argv[0])

    usage = "usage: %prog [options] [stl file]"
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_options(options)
    (opts, args) = parser.parse_args()

    filename = None
    if len(args):
        filename = args[0]
    a = stltool.stl(filename)
    (points, faces) = a.unique_vertices(a.facets)

    if opts.boundary:
        (pxmin,pymin,pxmax,pymax) = list(map(float,opts.boundary.split(',')))

    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    z = np.array([p[2] for p in points])

    fig = pp.figure()
    if opts.boundary:
        pp.axis([pxmin, pxmax, pymin, pymax])
    pp.tricontourf(x,y, z,256)
    pp.colorbar()
    pp.triplot(x, y, faces, 'k.', linewidth=0.5)
    pp.title('Correction mesh contour')
    pp.show()

if __name__ == '__main__':
    main()
