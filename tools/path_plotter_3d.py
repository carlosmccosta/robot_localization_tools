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



def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = x_limits[1] - x_limits[0]; x_mean = np.mean(x_limits)
    y_range = y_limits[1] - y_limits[0]; y_mean = np.mean(y_limits)
    z_range = z_limits[1] - z_limits[0]; z_mean = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_mean - plot_radius, x_mean + plot_radius])
    ax.set_ylim3d([y_mean - plot_radius, y_mean + plot_radius])
    ax.set_zlim3d([z_mean - plot_radius, z_mean + plot_radius])



if __name__ == "__main__":
    ##########################################################################
    # args
    parser = argparse.ArgumentParser(description='PLots paths from CSV files with pose information')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILES', type=str, required=True, help='CSV input file paths separated by +')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and pdf)')
    parser.add_argument('-p', metavar='FILE_POSITION_X_AXIS_COLUNM', type=int, required=False, default=0, help='CSV data column where the arrows x axis position starts')
    parser.add_argument('-v', metavar='FILE_VECTOR_X_AXIS_COLUNM', type=int, required=False, default=2, help='CSV data column where the arrows x axis vector starts')
    parser.add_argument('-a', metavar='ARROW_SCALE', type=float, required=False, default=0.0125, help='Arrow scale')
    parser.add_argument('-r', metavar='ARROW_HEAD_RATIO', type=float, required=False, default=0.5, help='Arrow head ratio')
    parser.add_argument('-l', metavar='ARROW_LINE_WIDTH', type=float, required=False, default=0.75, help='Arrow line width')
    parser.add_argument('-c', metavar='ARROWS_COLORS', type=str, required=False, default='g+b', help='Arrows colors for each file (separated by + in hex format #rrggbb)')
    parser.add_argument('-j', metavar='MARGIN_DIFF_PERCENTAGE', type=float, required=False, default=0.025, help='Margin percentage around data')
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

    ax.set_xlabel(args.b)
    ax.set_ylabel(args.m)
    ax.set_zlabel(args.z)
    if args.t != '':
        graph_title = plt.title(args.t, fontsize=16)
        graph_title.set_y(1.01)

    plt.minorticks_on()
    
    x_min = sys.maxint
    x_max = -sys.maxint
    y_min = sys.maxint
    y_max = -sys.maxint
    z_min = sys.maxint
    z_max = -sys.maxint


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

        x_min = np.min([np.min(arrow_positions_x), x_min])
        x_max = np.max([np.max(arrow_positions_x), x_max])
        y_min = np.min([np.min(arrow_positions_y), y_min])
        y_max = np.max([np.max(arrow_positions_y), y_max])
        z_min = np.min([np.min(arrow_positions_z), z_min])
        z_max = np.max([np.max(arrow_positions_z), z_max])

        for i in range(0, number_arrows):
            ax.quiver(arrow_positions_x[i], arrow_positions_y[i], arrow_positions_z[i], arrow_directions_x[i], arrow_directions_y[i], arrow_directions_z[i],
                      length=args.a, arrow_length_ratio=args.r, lw=args.l, color=arrow_colors[idx], alpha=0.5)


    ax.set_aspect("equal", adjustable="datalim")
    ax.axis('scaled')

    diff_x = (x_max - x_min)
    diff_y = (y_max - y_min)
    diff_z = (z_max - z_min)
    plot_radius = (np.max([diff_x, diff_y, diff_z]) * (1.0 + args.j)) * 0.5

#     ax.set_xlim3d([x_min - diff_x * args.j, x_max + diff_x * args.j])
#     ax.set_ylim3d([y_min - diff_y * args.j, y_max + diff_y * args.j])
#     ax.set_zlim3d([z_min - diff_z * args.j, z_max + diff_z * args.j])

#     ax.auto_scale_xyz([x_min - diff_x * args.j, x_max + diff_x * args.j],
#                       [y_min - diff_y * args.j, y_max + diff_y * args.j],
#                       [z_min - diff_z * args.j, z_max + diff_z * args.j])

    x_mean = x_min + diff_x * 0.5
    y_mean = y_min + diff_y * 0.5
    z_mean = z_min + diff_z * 0.5
    ax.set_xlim3d([x_mean - plot_radius, x_mean + plot_radius])
    ax.set_ylim3d([y_mean - plot_radius, y_mean + plot_radius])
    ax.set_zlim3d([z_mean - plot_radius, z_mean + plot_radius])

#     set_axes_equal(ax)

    print "Limits: [%f, %f] [%f, %f] [%f, %f] | Plot radius: %f" % (x_min, x_max, y_min, y_max, z_min, z_max, plot_radius)

#     ax.pbaspect=[1,1,1]
#     ax.localPbAspect=[1,1,1]
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
