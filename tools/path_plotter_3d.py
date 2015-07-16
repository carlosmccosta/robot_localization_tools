#!/usr/bin/env python
# coding=UTF-8


import argparse
import ntpath
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as tk
from mpl_toolkits.mplot3d import axis3d
from math import atan2, asin
from numpy import rad2deg



def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")


if __name__ == "__main__":
    ##########################################################################
    # args
    parser = argparse.ArgumentParser(description='PLots paths from CSV files with pose information')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILES', type=str, required=True, help='CSV input file paths separated by +')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and pdf)')
    parser.add_argument('-p', metavar='FILE_POSITION_X_AXIS_COLUNM', type=int, required=False, default=0, help='CSV data column where the arrows x axis position starts')
    parser.add_argument('-v', metavar='FILE_VECTOR_X_AXIS_COLUNM', type=int, required=False, default=2, help='CSV data column where the arrows x axis vector starts')
    parser.add_argument('-a', metavar='ARROW_SCALE', type=float, required=False, default=0.005, help='Arrow scale')
    parser.add_argument('-r', metavar='ARROW_HEAD_RATION', type=float, required=False, default=0.3, help='Arrow head ratio')
    parser.add_argument('-l', metavar='ARROW_LINE_WIDTH', type=float, required=False, default=0.05, help='Arrow line width')
    parser.add_argument('-c', metavar='ARROWS_COLORS', type=str, required=False, default='g+b', help='Arrows colors for each file (separated by + in hex format #rrggbb)')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Paths', help='Graph title')
    parser.add_argument('-b', metavar='X_AXIS_LABEL', type=str, required=False, default='x position (meters)', help='X axis label')
    parser.add_argument('-m', metavar='Y_AXIS_LABEL', type=str, required=False, default='y position (meters)', help='Y axis label')
    parser.add_argument('-z', metavar='Z_AXIS_LABEL', type=str, required=False, default='z position (meters)', help='Z axis label')
    parser.add_argument('-s', metavar='SAVE_GRAPH', type='bool', required=False, default=True, help='Save graphs to files using the name prefix specified with -o')
    parser.add_argument('-q', metavar='ADD_FILE_EXTENSION_TO_PATH', type='bool', required=False, default=False, help='Prepend to path the extension of the output file')
    parser.add_argument('-d', metavar='DISPLAY_GRAPH', type='bool', required=False, default=False, help='Show graph')
    args = parser.parse_args()



    ##########################################################################
    # graph setup
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect("equal")

    ax.set_xlabel(args.b)
    ax.set_ylabel(args.m)
    ax.set_zlabel(args.z)
    if args.t != '':
        graph_title = plt.title(args.t, fontsize=16)
        graph_title.set_y(1.01)

    plt.minorticks_on()


    ##########################################################################
    # path plotting
    file_names = args.i.split('+')
    arrow_colors = args.c.split('+')

    for idx, file in enumerate(file_names):
        arrow_positions_x = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.p,))
        arrow_positions_y = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.p + 1,))
        arrow_positions_z = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.p + 2,))
        arrow_directions_x = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.v,))
        arrow_directions_y = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.v + 1,))
        arrow_directions_z = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.v + 2,))
        number_arrows = min(len(arrow_positions_x), len(arrow_positions_y), len(arrow_positions_z), len(arrow_directions_x), len(arrow_directions_y), len(arrow_directions_z))

        print "Plotting path for file", file, "with", number_arrows, "poses"

        for i in range(0, number_arrows):
            ax.quiver(arrow_positions_x[i], arrow_positions_y[i], arrow_positions_z[i], arrow_directions_x[i], arrow_directions_y[i], arrow_directions_z[i],
                      length=args.a, arrow_length_ratio=args.r, lw=args.l, color=arrow_colors[idx], alpha=0.5)

    plt.tight_layout()
    plt.draw()



    ##########################################################################
    # output
    if args.s:
        if args.q:
            output_path = ntpath.dirname(args.o)
            output_file_name=ntpath.basename(args.o)
            plt.savefig('%s/svg/%s.svgz' % (output_path, output_file_name), bbox_inches='tight')
            plt.savefig('%s/eps/%s.eps' % (output_path, output_file_name), bbox_inches='tight')
            plt.savefig('%s/pdf/%s.pdf' % (output_path, output_file_name), bbox_inches='tight')
#            plt.savefig('%s/png/%s.png' % (output_path, output_file_name), dpi=1500, bbox_inches='tight')
        else:
            plt.savefig('%s.svgz' % args.o, bbox_inches='tight')
            plt.savefig('%s.eps' % args.o, bbox_inches='tight')
            plt.savefig('%s.pdf' % args.o, bbox_inches='tight')
#            plt.savefig('%s.png' % args.o, dpi=1500, bbox_inches='tight')

    if args.d:
        plt.show()

    exit(0)
