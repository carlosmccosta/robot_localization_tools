#!/home/carloscosta/anaconda/bin/python

# To use your system scipy package change to #!/usr/bin/python
# Be warned, in Ubuntu 12.04 the fitting of the lognorm and genextreme distribution doesn't work (if using the default scipy 0.9.0 package)
# If you don't want to install anaconda (from http://continuum.io/downloads), then update your scipy installation to version 0.14.0


import argparse
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
    parser.add_argument('-o', metavar='OUTPUT_FILE_NAME', type=str, required=False, default='results', help='Output file name (exports in svg, eps and png)')
    parser.add_argument('-b', metavar='BIN_WIDTH', type=float, required=False, default=0.5, help='Histogram bin width')
    parser.add_argument('-a', metavar='AXIS_TICKER_WIDTH', type=int, required=False, default=4, help='How many bins fit in each x axis ticker')
    parser.add_argument('-c', metavar='FILE_COLUNM', type=int, required=False, default=1, help='CSV data column to use')
    parser.add_argument('-t', metavar='GRAPH_TITLE', type=str, required=False, default='Probability distributions', help='Graph title')
    parser.add_argument('-x', metavar='GRAPH_X_AXIS_LABEL', type=str, required=False, default='Values', help='Graph x axis label')
    parser.add_argument('-s', metavar='SAVE_GRAPHS', type='bool', required=False, default=True, help='Save graphs to files using the name prefix specified with -o')
    parser.add_argument('-d', metavar='DISPLAY_GRAPHS', type='bool', required=False, default=False, help='Show graphs')
    parser.add_argument('-f', metavar='FORMATED_OUTPUT', type='bool', required=False, default=True, help='Console output in readable format or in csv style (if False)')
    args = parser.parse_args()



    ##########################################################################
    # input
    data = np.loadtxt(args.i, dtype=float, delimiter=',', skiprows=1, usecols=(args.c,))



    ##########################################################################
    # graph setup
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
    bin_width = args.b
    number_bins = (np.max(data) // bin_width) + 1
    x_min = 0.0
    x_max = number_bins * bin_width
    n, bins, patches = plt.hist(data, number_bins, range=(x_min, x_max), normed=1, histtype='bar', facecolor='grey', alpha=1.0)

    max_bin_count = 0
    max_bin_count_x = 0
    for idx, bin_count in enumerate(n):
        if bin_count > max_bin_count:
            max_bin_count = bin_count
            max_bin_count_x = (idx * bin_width) + bin_width * 0.5

    plt.axis('tight')
    axlim = list(plt.axis())
    axlim[0] = 0.0
    axlim[3] *= 1.2
    plt.axis(axlim)

    plt.xlabel(args.x)
    plt.ylabel('Probability')
    plt.title(args.t)

    plt.minorticks_on()
    plt.grid(b=True, which='major', color='k', linestyle='--', linewidth=0.3)
    plt.grid(b=True, which='minor', color='k', linestyle='--', linewidth=0.1)
    majorLocator = tk.MultipleLocator(bin_width * args.a)
    minorLocator = tk.MultipleLocator(bin_width)
    ax.xaxis.set_major_locator(majorLocator)
    ax.xaxis.set_minor_locator(minorLocator)



    ##########################################################################
    # probability distribution fitting
    output_str = ''
    if args.f:
        output_str += '- Distribution fitting for file ' + args.i
    else:
        output_str += args.i

    distr_names = ['Normal', 'Log\ Normal', 'Generalized\ Extreme\ Value']
    distr_colors = ['-r', '-g', '-b']

    for idx, distr in enumerate([stats.norm, stats.lognorm, stats.genextreme]):
        par_est = distr.fit(data, loc=max_bin_count_x)
        loc_est = par_est[-2]
        scale_est = par_est[-1]
        if args.f:
            output_str += ('\n  - Estimated parameters for %s distribution:' % distr.name)
            output_str += ('\n    -> Location: ' + str(loc_est))
            output_str += ('\n    ->    Scale: ' + str(scale_est))
        else:
            output_str += (',' + distr.name)
            output_str += (',' + str(loc_est))
            output_str += (',' + str(scale_est))

        x_values = np.linspace(x_min, x_max, 2000)
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
        ax.plot(x_values, y_values, distr_colors[idx], linewidth=1, label=plot_label)

    plt.legend()
    print output_str



    ##########################################################################
    # output
    if args.s:
        plt.savefig('%s.svg' % args.o)
        plt.savefig('%s.eps' % args.o)
        plt.savefig('%s.png' % args.o, dpi=300, bbox_inches='tight')

    if args.d:
        plt.show()

    exit(0)
