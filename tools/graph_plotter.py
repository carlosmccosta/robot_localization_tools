#!/usr/bin/env python
# coding=UTF-8


import argparse
import ntpath
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as tk


def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")




if __name__ == "__main__":
    ##########################################################################
    # args
    parser = argparse.ArgumentParser(description='PLots line graphs from CSV file')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILE', type=str, required=True, help='CSV input file name')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and pdf)')
    parser.add_argument('-x', metavar='FILE_X_COLUNM', type=str, required=False, default=0, help='CSV data column with the x data for each file split by -')
    parser.add_argument('-y', metavar='FILE_Y_COLUNMS', type=str, required=False, default=1, help='CSV data columns with the y data separated with + within file and split by - for each file')
    parser.add_argument('-z', metavar='FILE_VALUE_DELIMITER', type=str, required=False, default=',', help='Value delimiter in each line')
    parser.add_argument('-e', metavar='FILE_N_SKIP_ROWS', type=int, required=False, default=1, help='Number of rows to skip when reading files')
    parser.add_argument('-w', metavar='PLOT_LINE_WIDTH', type=float, required=False, default=0.25, help='Plot line width')
    parser.add_argument('-u', metavar='PLOT_LINE_STYLE', type=str, required=False, default='-', help='Plot line style')
    parser.add_argument('-a', metavar='PLOT_LINE_STYLE_ALPHA', type=float, required=False, default=0.75, help='Plot line alpha')
    parser.add_argument('-j', metavar='PLOT_LINE_MARKER', type=str, required=False, default='.', help='Plot line marker')
    parser.add_argument('-k', metavar='PLOT_LINE_MARKER_SIZE_WIDTH_MULTIPLIER', type=float, required=False, default=0.75, help='Plot line marker size will be PLOT_LINE_WIDTH * PLOT_LINE_MARKER_SIZE_WIDTH_MULTIPLIER')
    parser.add_argument('-m', metavar='X_AXIS_SCALE', type=float, required=False, default=0.000000001, help='X axis scale')
    parser.add_argument('-n', metavar='Y_AXIS_SCALE', type=float, required=False, default=1, help='Y axis scale')
    parser.add_argument('-b', metavar='X_AXIS_LABEL', type=str, required=False, default='X', help='X axis label')
    parser.add_argument('-v', metavar='Y_AXIS_LABEL', type=str, required=False, default='Y', help='Y axis label')
    parser.add_argument('-l', metavar='Y_LINES_LABELS', type=str, required=False, default='Data', help='Legend for each y plot line, separated by +')
    parser.add_argument('-c', metavar='Y_AXIS_COLORS', type=str, required=False, default='g', help='Y axis colors, separated by + in hex format #rrggbb')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Paths', help='Graph title')
    parser.add_argument('-r', metavar='RESET_X_VALUES', type='bool', required=False, default=False, help='Reset the x values so that they are in range [0..(max-min)]')
    parser.add_argument('-g', metavar='DISPLAY_GRID', type='bool', required=False, default=True, help='Show graph grid')
    parser.add_argument('-s', metavar='SAVE_GRAPH', type='bool', required=False, default=True, help='Save graphs to files using the name prefix specified with -o')
    parser.add_argument('-q', metavar='ADD_FILE_EXTENSION_TO_PATH', type='bool', required=False, default=False, help='Prepend to path the extension of the output file')
    parser.add_argument('-d', metavar='DISPLAY_GRAPH', type='bool', required=False, default=False, help='Show graph')
    args = parser.parse_args()



    ##########################################################################
    # graph setup
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)

    plt.xlabel(args.b)
    plt.ylabel(args.v)
    graph_title = plt.title(args.t, fontsize=16)
    graph_title.set_y(1.01)

    plt.minorticks_on()
    if args.g:
        plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.30, alpha=0.5)
        plt.grid(b=True, which='minor', color='k', linestyle=':', linewidth=0.01, alpha=0.2)

    x_min = sys.maxint
    x_max = -sys.maxint
    y_min = sys.maxint
    y_max = -sys.maxint



    ##########################################################################
    # graph plotting
    file_names = args.i.split('+')
    x_columns = args.x.split('-')
    y_columns_per_file = args.y.split('-')
    y_colors = args.c.split('+')
    y_labels = args.l.split('+')

    current_column = 0
    for idx_file, file in enumerate(file_names):
        x_values = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(x_columns[idx_file]),))
        x_values_sorted_indexs = np.argsort(x_values)
        x_values = x_values[x_values_sorted_indexs]
        
        if args.m != 1:
            x_values *= args.m
        if args.r:
            x_values -= np.min(x_values)
        
        y_columns = y_columns_per_file[idx_file].split('+')
        for idx_colomn, y_column in enumerate(y_columns):
            y_values = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(y_column),))
            y_values = y_values[x_values_sorted_indexs]
            
            if args.n != 1:
                y_values *= args.n

            x_min = np.min([np.min(x_values), x_min])
            x_max = np.max([np.max(x_values), x_max])
            y_min = np.min([np.min(y_values), y_min])
            y_max = np.max([np.max(y_values), y_max])

            plt.plot(x_values, y_values, y_colors[current_column], linewidth=args.w, label=y_labels[current_column], alpha=args.a, linestyle=args.u, marker=args.j, markersize=args.w * args.k)
            current_column += 1


    plt.axis('tight')
    axlim = list(plt.axis())
    diff_x = abs(x_max - x_min)
    diff_y = abs(y_max - y_min)
    axlim[0] = x_min - diff_x * 0.01
    axlim[1] = x_max + diff_x * 0.01
    axlim[2] = y_min - diff_y * 0.02
    axlim[3] = y_max + diff_y * (0.042 * len(y_labels))
    if axlim[0] == axlim[1]:
        axlim[0] -= 1
        axlim[1] += 1
    if axlim[2] == axlim[3]:
        axlim[2] -= 1
        axlim[3] += 1
    plt.axis(axlim)

    graph_legend = plt.legend(fancybox=True, prop={'size':12})
    graph_legend.get_frame().set_alpha(0.75)
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
#            plt.savefig('%s/png/%s.png' % (output_path, output_file_name), dpi=300, bbox_inches='tight')
        else:
            plt.savefig('%s.svgz' % args.o, bbox_inches='tight')
            plt.savefig('%s.eps' % args.o, bbox_inches='tight')
            plt.savefig('%s.pdf' % args.o, bbox_inches='tight')
#            plt.savefig('%s.png' % args.o, dpi=300, bbox_inches='tight')


    if args.d:
        plt.show()

    exit(0)
