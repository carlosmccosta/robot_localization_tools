#!/usr/bin/env python


import argparse
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
    parser = argparse.ArgumentParser(description='PLots line graphs from CSV file')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILE', type=str, required=True, help='CSV input file name')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and png)')
    parser.add_argument('-x', metavar='FILE_X_COLUNM', type=int, required=False, default=1, help='CSV data column with the x data')
    parser.add_argument('-y', metavar='FILE_Y_COLUNMS', type=str, required=False, default=2, help='CSV data column with the y data')
    parser.add_argument('-m', metavar='X_AXIS_SCALE', type=float, required=False, default=0.000000001, help='X axis scale')
    parser.add_argument('-n', metavar='Y_AXIS_SCALE', type=float, required=False, default=1, help='Y axis scale')
    parser.add_argument('-b', metavar='X_AXIS_LABEL', type=str, required=False, default='X', help='X axis label')
    parser.add_argument('-v', metavar='Y_AXIS_LABEL', type=str, required=False, default='Y', help='Y axis label')
    parser.add_argument('-l', metavar='Y_LINES_LABELS', type=str, required=False, default='Data', help='Legend for each y plot line')
    parser.add_argument('-c', metavar='Y_AXIS_COLORS', type=str, required=False, default='g', help='Y axis colors')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Paths', help='Graph title')
    parser.add_argument('-s', metavar='SAVE_GRAPH', type='bool', required=False, default=True, help='Save graphs to files using the name prefix specified with -o')
    parser.add_argument('-d', metavar='DISPLAY_GRAPH', type='bool', required=False, default=False, help='Show graph')
    args = parser.parse_args()



    ##########################################################################
    # graph setup
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)

    plt.xlabel(args.b)
    plt.ylabel(args.v)
    plt.title(args.t)

    plt.minorticks_on()
    plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.3, alpha=0.5)
    plt.grid(b=True, which='minor', color='k', linestyle='--', linewidth=0.1, alpha=0.5)

    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0

    ##########################################################################
    # graph plotting
    y_columns = args.y.split('+')
    y_colors = args.c.split('+')
    y_labels = args.l.split('+')

    for idx, y_column in enumerate(y_columns):
        x_values = np.loadtxt(args.i, dtype=float, delimiter=',', skiprows=1, usecols=(args.x,)) * args.m
        y_values = np.loadtxt(args.i, dtype=float, delimiter=',', skiprows=1, usecols=(int(y_column),))
        if args.n != 1:
            y_values *= args.n

        x_min = np.min([np.min(x_values), x_min])
        x_max = np.max([np.max(x_values), x_max])
        y_min = np.min([np.min(y_values), y_min])
        y_max = np.max([np.max(y_values), y_max])
        
        plt.plot(x_values, y_values, y_colors[idx], linewidth=1, label=y_labels[idx])

    plt.axis('tight')
    axlim = list(plt.axis())
    axlim[0] = x_min - abs(x_min * 0.05)
    axlim[1] = x_max + abs(x_max * 0.05)
    axlim[2] = y_min - abs(y_min * 0.05)
    axlim[3] = y_max + abs(y_max * (0.1 * len(y_labels)))
    plt.axis(axlim)
    graph_legend = plt.legend(fancybox=True)
    graph_legend.get_frame().set_alpha(0.5)
    plt.draw()



    ##########################################################################
    # output
    if args.s:
        plt.savefig('%s.svg' % args.o)
        plt.savefig('%s.eps' % args.o)
        plt.savefig('%s.pdf' % args.o)
        plt.savefig('%s.png' % args.o, dpi=1000, bbox_inches='tight')

    if args.d:
        plt.show()

    exit(0)
