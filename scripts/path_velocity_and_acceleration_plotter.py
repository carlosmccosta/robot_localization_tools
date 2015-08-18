#!/usr/bin/env python
# coding=UTF-8

# Requires https://github.com/moble/quaternion

import argparse
import ntpath
import sys
import math
import numpy as np
import numpy.linalg
import matplotlib.pyplot as plt
import matplotlib.ticker as tk
import quaternion


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


def euclidean_distance(point1, point2):
    return numpy.sqrt(numpy.sum((point2 - point1)**2))


def angular_difference_degrees(quaternion1, quaternion2):
    diff_quaternion = (quaternion1 * quaternion2.conjugate()).normalized()
    if (diff_quaternion.w < 0.0): # shortest path angle
        diff_quaternion = -diff_quaternion
    return math.degrees(2.0 * math.acos(diff_quaternion.w))



##########################################################################
# Velocity and acceleration Savitzky-Golay filtering computation (http://dsp.stackexchange.com/questions/9498/have-position-want-to-calculate-velocity-and-acceleration)

def sg_filter(time_values, polynomial_order, derivative=0):
    """
    time_values = Vector of sample times
    polynomial_order = Order of the smoothing polynomial
    derivative = Which derivative
    """
    mid = len(time_values) / 2
    a = time_values - time_values[mid]
    expa = lambda time_values: map(lambda i: i**time_values, a)
    A = np.r_[map(expa, range(0,polynomial_order+1))].transpose()
    Ai = np.linalg.pinv(A)

    return Ai[derivative]

def smooth(x, y, window_size=5, polynomial_order=2, derivative=0):

    if derivative > polynomial_order:
        raise Exception, "derivative must be <= polynomial_order"

    data_length = len(x)

    result = np.zeros(data_length)

    for i in xrange(window_size, data_length - window_size):
        start, end = i - window_size, i + window_size + 1
        f = sg_filter(x[start:end], polynomial_order, derivative)
        result[i] = np.dot(f, y[start:end])

    if derivative > 1:
        result *= math.factorial(derivative)

    return result




if __name__ == "__main__":
    ##########################################################################
    # args
    parser = argparse.ArgumentParser(description='PLots line graphs from CSV file')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILE', type=str, required=True, help='CSV input file name')
    parser.add_argument('-f', metavar='DERIVATIVE_ORDER', type=int, required=False, default=1, help='Order of derivative (0 -> original data, 1 -> velocity, 2 -> acceleration)')
    parser.add_argument('-r', metavar='NUMBER_POINTS_FOR_SMOOTHING', type=int, required=False, default=7, help='Number of data points that will be used to perform Savitzky-Golay filtering (0 disables smoothing, and result is the max of -r and data_points_nr * --smooth)')
    parser.add_argument('--smooth', metavar='NUMBER_PERCENTAGE_OF_POINTS_FOR_SMOOTHING', type=float, required=False, default=0.05, help='Percentage of data points that will be used to perform Savitzky-Golay filtering')
    parser.add_argument('-g', metavar='POLYNOMIAL_ORDER_FOR_SMOOTHING', type=int, required=False, default=2, help='Polynomial order that will be used to perform Savitzky-Golay filtering')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and pdf)')
    parser.add_argument('-p', metavar='INPUT_FILE_WITH_POSITIONS', type='bool', required=False, default=True, help='Whether the file has position or quaternion orientations (computes either the linear or angular velocity)')
    parser.add_argument('-x', metavar='FILES_TIME_COLUNM', type=str, required=False, default=0, help='CSV data column with the x data for each file split by -')
    parser.add_argument('--sort', metavar='SORT_TIME_COLUNM', type='bool', required=False, default=1, help='Sort all data by their time stamp')
    parser.add_argument('-y', metavar='FILES_POSE_START_COLUNMS', type=str, required=False, default=1, help='CSV data columns with the y data separated with + within file and split by - for each file')
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
    parser.add_argument('--reset', metavar='RESET_X_VALUES', type='bool', required=False, default=False, help='Reset the x values so that they are in range [0..(max-min)]')
    parser.add_argument('--grid', metavar='DISPLAY_GRID', type='bool', required=False, default=True, help='Show graph grid')
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
    if args.grid:
        plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.30, alpha=0.5)
        plt.grid(b=True, which='minor', color='k', linestyle=':', linewidth=0.01, alpha=0.2)

    x_min = sys.maxint
    x_max = -sys.maxint
    y_min = sys.maxint
    y_max = -sys.maxint



    ##########################################################################
    # graph plotting
    file_names = args.i.split('+')
    time_columns = args.x.split('-')
    pose_start_columns_per_file = args.y.split('-')
    y_colors = args.c.split('+')
    y_labels = args.l.split('+')

    for idx_file, file in enumerate(file_names):
        time_values = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(time_columns[idx_file]),))
        
        if (args.sort):
            time_values_sorted_indexs = np.argsort(time_values)
            time_values = time_values[time_values_sorted_indexs]
        
        if args.m != 1:
            time_values *= args.m
        if args.reset:
            time_values -= np.min(time_values)
        
        pose_x = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(pose_start_columns_per_file[idx_file]),))
        pose_y = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(pose_start_columns_per_file[idx_file]) + 1,))
        pose_z = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(pose_start_columns_per_file[idx_file]) + 2,))
        if (args.sort):
            pose_x = pose_x[time_values_sorted_indexs]
            pose_y = pose_y[time_values_sorted_indexs]
            pose_z = pose_z[time_values_sorted_indexs]
        values_count = np.min([pose_x.size, pose_y.size, pose_z.size])
        y_values = np.zeros(values_count)

        if args.p:
            for i in xrange(1, values_count):
                y_values[i] = euclidean_distance(np.array([ pose_x[i-1], pose_y[i-1], pose_z[i-1] ]), np.array([ pose_x[i], pose_y[i], pose_z[i] ]))
        else:
            pose_w = np.loadtxt(file, dtype=float, delimiter=args.z, skiprows=args.e, usecols=(int(pose_start_columns_per_file[idx_file]) + 3,))
            if (args.sort):
                pose_w = pose_w[time_values_sorted_indexs]
            for i in range(1, values_count):
                y_values[i] = angular_difference_degrees( np.quaternion(pose_w[i-1], pose_x[i-1], pose_y[i-1], pose_z[i-1]), np.quaternion(pose_w[i], pose_x[i], pose_y[i], pose_z[i]) )

        if args.n != 1:
            y_values *= args.n

        if args.f >= 0:
            if args.r == 0:
                for i in xrange(0, values_count-1):
                    time_diff = time_values[i+1] - time_values[i]
                    y_values[i] = y_values[i] / (time_diff ** args.f)
            else:
                y_values = np.cumsum(y_values)
                if args.f >= 1:
                    number_smooth_points = int(np.max([args.r, values_count * args.smooth]))
                    y_values = smooth(time_values, y_values, number_smooth_points, args.g, args.f)

        x_min = np.min([np.min(time_values), x_min])
        x_max = np.max([np.max(time_values), x_max])
        y_min = np.min([np.min(y_values), y_min])
        y_max = np.max([np.max(y_values), y_max])

        plt.plot(time_values, y_values, y_colors[idx_file], linewidth=args.w, label=y_labels[idx_file], alpha=args.a, linestyle=args.u, marker=args.j, markersize=args.w * args.k)


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
