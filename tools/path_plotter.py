#!/usr/bin/env python
# coding=UTF-8


import argparse
import ntpath
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as tk
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
    parser.add_argument('-p', metavar='FILE_POSITION_START_COLUNM', type=int, required=False, default=0, help='CSV data column where the arrows position starts')
    parser.add_argument('-v', metavar='FILE_VECTOR_START_COLUNM', type=int, required=False, default=2, help='CSV data column where the arrows vector starts')
    parser.add_argument('-a', metavar='ARROW_SCALE', type=float, required=False, default=0.005, help='Arrow scale')
    parser.add_argument('-w', metavar='ARROW_WIDTH', type=float, required=False, default=0.002, help='Arrow width')
    parser.add_argument('-l', metavar='ARROW_LINE_WIDTH', type=float, required=False, default=0.002, help='Arrow line width')
    parser.add_argument('-y', metavar='ARROW_HEAD_WIDTH', type=float, required=False, default=0.004, help='Arrow head width')
    parser.add_argument('-u', metavar='ARROW_HEAD_LENGTH', type=float, required=False, default=0.003, help='Arrow head length')
    parser.add_argument('-c', metavar='ARROWS_COLORS', type=str, required=False, default='g+b', help='Arrows colors for each file (separated by + in hex format #rrggbb)')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Paths', help='Graph title')
    parser.add_argument('-s', metavar='SAVE_GRAPH', type='bool', required=False, default=True, help='Save graphs to files using the name prefix specified with -o')
    parser.add_argument('-q', metavar='ADD_FILE_EXTENSION_TO_PATH', type='bool', required=False, default=False, help='Prepend to path the extension of the output file')
    parser.add_argument('-d', metavar='DISPLAY_GRAPH', type='bool', required=False, default=False, help='Show graph')
    args = parser.parse_args()



    ##########################################################################
    # graph setup
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)

    plt.xlabel('x position (meters)')
    plt.ylabel('y position (meters)')
    graph_title = plt.title(args.t, fontsize=16)
    graph_title.set_y(1.01)

    plt.minorticks_on()
#     plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.3, alpha=0.7)
#     plt.grid(b=True, which='minor', color='k', linestyle='--', linewidth=0.1, alpha=0.7)
#     majorLocator = tk.MultipleLocator(1.0)
#     minorLocator = tk.MultipleLocator(0.25)
#     ax.xaxis.set_major_locator(majorLocator)
#     ax.xaxis.set_minor_locator(minorLocator)

    x_min = sys.maxint
    x_max = -sys.maxint
    y_min = sys.maxint
    y_max = -sys.maxint

    ##########################################################################
    # path plotting
    file_names = args.i.split('+')
    arrow_colors = args.c.split('+')

    for idx, file in enumerate(file_names):
        arrow_positions_x = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.p,))
        arrow_positions_y = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.p + 1,))
        arrow_directions_x = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.v,))
        arrow_directions_y = np.loadtxt(file, dtype=float, delimiter=' ', skiprows=2, usecols=(args.v + 1,))
        number_arrows = min(len(arrow_positions_x), len(arrow_positions_y), len(arrow_directions_x), len(arrow_directions_y))

        x_min = np.min([np.min(arrow_positions_x), x_min])
        x_max = np.max([np.max(arrow_positions_x), x_max])
        y_min = np.min([np.min(arrow_positions_y), y_min])
        y_max = np.max([np.max(arrow_positions_y), y_max])

        print "Plotting path for file", file, "with", number_arrows, "poses"

        for i in range(0, number_arrows):
#             print arrow_positions_x[i], arrow_positions_y[i], arrow_directions_x[i], arrow_directions_y[i]

            ax.arrow(arrow_positions_x[i], arrow_positions_y[i], arrow_directions_x[i] * args.a, arrow_directions_y[i] * args.a,
                     shape='full', width=args.w, linewidth=args.l, length_includes_head=True, head_width=args.y, head_length=args.u, color=arrow_colors[idx], alpha=0.5)

#             ax.annotate(str(i), fontsize=0.1,
#                         xy=(arrow_positions_x[i] + arrow_directions_x[i] * args.a, arrow_positions_y[i] + arrow_directions_y[i] * args.a),
#                         xytext=(arrow_positions_x[i], arrow_positions_y[i]),
# #                         arrowprops=dict(arrowstyle="->", linewidth=0.05, color=arrow_colors[idx])
#                         arrowprops=dict(width=0.05, headwidth=0.15, frac=0.3, linewidth=0.05, color=arrow_colors[idx])
#                         )

#             ax.text(arrow_positions_x[i], arrow_positions_y[i], str(i),
#                     ha="left", va="center", rotation=rad2deg(asin(arrow_directions_y[i])), size=0.1,
#                     bbox=dict(boxstyle="rarrow,pad=0.05", color=arrow_colors[idx], lw=0.05, alpha=0.2, width=0.1, mutation_scale=0.1, mutation_aspect=1.0))

    plt.axis('scaled')
    axlim = list(plt.axis())
    diff_x = (x_max - x_min)
    diff_y = (y_max - y_min)
    axlim[0] = x_min - diff_x * 0.025
    axlim[1] = x_max + diff_x * 0.025
    axlim[2] = y_min - diff_y * 0.025
    axlim[3] = y_max + diff_y * 0.025
    if axlim[0] == axlim[1]:
        axlim[0] = -1
        axlim[1] = 1
    if axlim[2] == axlim[3]:
        axlim[2] = -1
        axlim[3] = 1
    plt.axis(axlim)
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
