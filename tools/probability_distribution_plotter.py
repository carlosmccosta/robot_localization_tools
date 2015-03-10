#!/usr/bin/env python
# coding=UTF-8

# Be warned, in Ubuntu 12.04 the fitting of the lognorm and genextreme distribution doesn't work with the default scipy 0.9.0 package
# Either update your scipy installation to version 0.14.0
# or install anaconda (from http://continuum.io/downloads) and prepend to your PATH the anaconda installation folder (usually ~/anaconda/bin)


import argparse
import ntpath
import sys
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import matplotlib.ticker as tk



def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")




if __name__ == "__main__":
    ##########################################################################
    # args
    parser = argparse.ArgumentParser(description='Fits probability distributions to data coming from a csv file and outputs their graphs')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_FILE', type=str, required=True, help='CSV input file')
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and pdf)')
    parser.add_argument('-c', metavar='FILE_COLUNM', type=int, required=False, default=0, help='CSV data column to use')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Probability distributions', help='Graph title')
    parser.add_argument('-x', metavar='GRAPH_X_AXIS_LABEL', type=str, required=False, default='Values', help='Graph x axis label')
    parser.add_argument('-z', metavar='FILE_VALUE_DELIMITER', type=str, required=False, default=',', help='Value delimiter in each line')
    parser.add_argument('-b', metavar='BIN_WIDTH', type=float, required=False, default=-1, help='Histogram bin width. If < 0, it will use the number of bins specified with -n')
    parser.add_argument('-n', metavar='NUMBER_OF_BINS', type=float, required=False, default=100, help='Number of bins to use (only used if -b is < 0)')
    parser.add_argument('-e', metavar='VALUES_SCALE', type=float, required=False, default=1, help='X axis scale')
    parser.add_argument('-m', metavar='AXIS_MAX_LOCATOR_WIDTH', type=int, required=False, default=-1, help='Width for the major tick locator. If <= 0 lets matplotlib choose one')
    parser.add_argument('-l', metavar='AXIS_MAX_LOCATOR_LINEAR', type=int, required=False, default=11, help='Divides the input range in -l major ticks. If <= 0 lets matplotlib choose one. Overrides -m')
    parser.add_argument('-a', metavar='AXIS_MAX_LOCATOR_SUB_SIVISIONS', type=int, required=False, default=10, help='Number of major tick subdivisions for minor locator. If <= 0 lets matplotlib choose one')
    parser.add_argument('-w', metavar='PLOT_LINE_WIDTH', type=float, required=False, default=0.25, help='Plot line width')
    parser.add_argument('-g', metavar='GRAPH_STD_DEV_VALUES', type=int, required=False, default=4,
                        help='Show only values that are lower than -g standard deviations (from the mean of the fitted normal distribution). If 0, shows all values. Note: this doesnt affect the estimated distribution values (it only trims the displayed values)')
    parser.add_argument('-r', metavar='DISPLAY_POSITIVE_VALUES_ONLY', type='bool', required=False, default=False, help='Resets the display axis so that only positive values are shown')
    parser.add_argument('-s', metavar='SAVE_GRAPH', type='bool', required=False, default=True, help='Save graph to files using the name prefix specified with -o')
    parser.add_argument('-q', metavar='ADD_FILE_EXTENSION_TO_PATH', type='bool', required=False, default=False, help='Prepend to path the extension of the output file')
    parser.add_argument('-d', metavar='DISPLAY_GRAPH', type='bool', required=False, default=False, help='Show graph')
    parser.add_argument('-f', metavar='FORMATED_OUTPUT', type='bool', required=False, default=True, help='Console output in readable format or in csv style (if False)')
    args = parser.parse_args()
    if args.z == 'sp':
        args.z = ' '


    ##########################################################################
    # input
    data = np.loadtxt(args.i, dtype=float, delimiter=args.z, skiprows=1, usecols=(args.c,))
    if args.e != 1:
            data *= args.e


    ##########################################################################
    # graph setup
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
    bin_width = args.b
    x_min = np.min(data)
    x_max = np.max(data)

    if args.n <= 0:
        args.n = 100

    if args.b <= 0:
        bin_width = (x_max - x_min) / args.n

    n, bins = np.histogram(data, args.n, range=(x_min, x_max), density=1)

    max_bin_count = -sys.maxint
    max_bin_count_x = -sys.maxint
    for idx, bin_count in enumerate(n):
        if bin_count > max_bin_count:
            max_bin_count = bin_count
            max_bin_count_x = x_min + (idx * bin_width) + bin_width * 0.5

    plt.xlabel(args.x)
    plt.ylabel('Histogram bin count percentage [0..1] | Probability distribution function value')
    graph_title = plt.title(args.t, fontsize=16)
    graph_title.set_y(1.01)

    plt.minorticks_on()
    plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.30, alpha=0.5)
    plt.grid(b=True, which='minor', color='k', linestyle=':', linewidth=0.01, alpha=0.2)



    ##########################################################################
    # probability distribution fitting
    output_str = ''
    output_file_name=ntpath.basename(args.o)
    if args.f:
        output_str += '- Distribution fitting for file '
    output_str += output_file_name

    distr_names = ['Normal', 'Log\ Normal', 'Generalized\ Extreme\ Value']
    distr_colors = ['-r', '-g', '-b']

    normal_ditribution_mean = 0
    normal_ditribution_std_dev = 0

    for idx, distr in enumerate([stats.norm, stats.lognorm, stats.genextreme]):
        par_est = distr.fit(data, loc=max_bin_count_x)
        loc_est = par_est[-2]
        scale_est = par_est[-1]

        if idx == 0:
            normal_ditribution_mean = loc_est
            normal_ditribution_std_dev = scale_est

        if args.f:
            output_str += ('\n  - Estimated parameters for %s distribution:' % distr.name)
            output_str += ('\n    -> Location: ' + str(loc_est))
            output_str += ('\n    ->    Scale: ' + str(scale_est))
        else:
#             output_str += (',' + distr.name)
            output_str += (',' + str(loc_est))
            output_str += (',' + str(scale_est))

        x_values = np.linspace(x_min, x_max, 100000)
        y_values = distr.pdf(x_values, *par_est)

        plot_label = ''
        if len(par_est) == 2:
            plot_label = '$\mathrm{%s\ distribution:}\ location=%s,\ scale=%s$' % (distr_names[idx], str(loc_est), str(scale_est))
        elif len(par_est) == 3:
            if args.f:
                output_str += ('\n    ->    Shape: ' + str(par_est[0]))
            else:
                output_str += (',' + str(par_est[0]))
            plot_label='$\mathrm{%s\ distribution:}\ location=%s,\ scale=%s,\ shape=%s$' % (distr_names[idx], str(loc_est), str(scale_est), str(par_est[0]))
        ax.plot(x_values, y_values, distr_colors[idx], linewidth=args.w, label=plot_label, alpha=0.75)



    ##########################################################################
    # graph plotting
    if args.g > 0 and abs(normal_ditribution_std_dev) > 0:
        x_min = np.max([normal_ditribution_mean - normal_ditribution_std_dev * args.g, x_min])
        x_max = np.min([normal_ditribution_mean + normal_ditribution_std_dev * args.g, x_max])

    if args.b <= 0:
        bin_width = (x_max - x_min) / args.n

    if args.b <= 0:
        x_min_final = x_min
        x_max_final = x_max
        number_bins = args.n
    else:
        x_min_final = int(x_min // bin_width)
        x_max_final = int(np.ceil(x_max / bin_width))
        number_bins = np.max([x_max_final - x_min_final, 1])
        x_min_final *= bin_width
        x_max_final *= bin_width

    plt.axis('tight')
    axlim = list(plt.axis())

    axlim[0] = x_min_final
    axlim[1] = x_max_final

    if args.r:
        axlim[0] = np.max([axlim[0], 0.0])

    if axlim[0] == axlim[1]:
        axlim[0] -= 1
        axlim[1] += 1

    n, bins, patches = plt.hist(data, number_bins, range=(x_min_final, x_max_final), normed=1, histtype='bar', facecolor='grey', linewidth=args.w, alpha=1.0)
    ax.yaxis.set_major_formatter(tk.FuncFormatter(lambda v, pos: "{:4.2f}".format(v * bin_width) + ' | ' + "{:4.2f}".format(v)))

    if args.a > 0:
        minorLocator = tk.AutoMinorLocator(args.a)
        ax.xaxis.set_minor_locator(minorLocator)

    if args.m > 0 and args.l <= 0:
        majorLocator = tk.MultipleLocator(args.m)
        ax.xaxis.set_major_locator(majorLocator)

    if args.l > 0:
        majorLocator = tk.LinearLocator(numticks=args.l)
        ax.xaxis.set_major_locator(majorLocator)

    axlim[3] = np.max(n) * 1.15
    if axlim[2] == axlim[3]:
        axlim[2] -= 1
        axlim[3] += 1

    plt.axis(axlim)

    graph_legend = plt.legend(fancybox=True)
    graph_legend.get_frame().set_alpha(0.75)
    plt.draw()
    print output_str



    ##########################################################################
    # output
    if args.s:
        if args.q:
            output_path = ntpath.dirname(args.o)
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
