import argparse
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import pylatex
import subprocess
import time


def parse_arguments():
    argument_parser = argparse.ArgumentParser(description='Run track performance test on datasets and '
                                                          'generate PDF file with demonstrative results')

    argument_parser.add_argument('run_track_performance_sh_path', help='path to run_track_performance.sh')
    argument_parser.add_argument('dsopp_main_path', help='path to dsopp_main')
    argument_parser.add_argument('track2trajectory_path', help='path to track2trajectory')
    argument_parser.add_argument('evaluate_ate_py_path', help='path to evaluate_ate.py')
    argument_parser.add_argument('datasets_directory_path', help='path to directory with datasets')

    return argument_parser.parse_args()


def init_document():
    document = pylatex.Document(document_options='landscape', page_numbers=False, geometry_options={'margin': '1cm'})
    document.packages.append(pylatex.Package('float'))
    return document


def add_col_trans_error(table_cols, dataset_name, elapsed_time, trans_error):
    fmt = '%.3f'
    table_cols.append([dataset_name, fmt % elapsed_time, len(trans_error),
                       fmt % np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)), fmt % np.mean(trans_error),
                       fmt % np.median(trans_error), fmt % np.std(trans_error), fmt % np.min(trans_error),
                       fmt % np.max(trans_error)])


def write_plot_cdf_trans_error(document, caption, trans_error, x_right):
    trans_error.sort()
    trans_error = np.concatenate(([0], trans_error))

    plt.plot(trans_error, np.arange(len(trans_error)) / (len(trans_error) - 1))
    plt.xlim(right=x_right)
    plt.grid()
    plt.xlabel('absolute translational error, m')

    with document.create(pylatex.SubFigure(width=pylatex.NoEscape(r'0.25\textwidth'))) as subfigure:
        subfigure.add_plot()
        subfigure.add_caption(caption)
        write_plot_cdf_trans_error.subfigures_number += 1

    if write_plot_cdf_trans_error.subfigures_number % 4 == 0:
        document.append('\n')

    plt.clf()


write_plot_cdf_trans_error.subfigures_number = 0


def main():
    arguments = parse_arguments()
    document = init_document()
    datasets_directory = pathlib.Path(arguments.datasets_directory_path)
    trans_errors = []
    elapsed_times = 0
    table_cols = [['dataset_name', 'elapsed_time, s', 'compared_pose_pairs', 'rmse, m', 'mean, m', 'median, m',
                   'std, m', 'min, m', 'max, m']]
    for dataset_directory in datasets_directory.iterdir():
        if not dataset_directory.is_dir():
            continue

        config_file_path = dataset_directory / 'mono.yaml'
        track_bin_path = dataset_directory / 'track.bin'
        output_track_positions_tum_path = dataset_directory / 'output_track_positions.tum'
        output_ecef_poses_enu_path = dataset_directory / 'output_ecef_poses.enu'
        gt_tum_path = dataset_directory / 'gt.tum'
        gt_enu_path = dataset_directory / 'gt.enu'
        trans_error_odometry_txt_path = dataset_directory / 'trans_error_odometry.txt'
        trans_error_ecef_poses_txt_path = dataset_directory / 'trans_error_ecef_poses.txt'
        results_odometry_txt_path = dataset_directory / 'results_odometry.txt'
        results_ecef_poses_txt_path = dataset_directory / 'results_ecef_poses.txt'

        track_bin_path.unlink(True)
        output_track_positions_tum_path.unlink(True)
        output_ecef_poses_enu_path.unlink(True)
        trans_error_odometry_txt_path.unlink(True)
        trans_error_ecef_poses_txt_path.unlink(True)
        results_odometry_txt_path.unlink(True)
        results_ecef_poses_txt_path.unlink(True)

        print(dataset_directory.name)
        start = time.time()
        completed_process = subprocess.run([arguments.run_track_performance_sh_path, arguments.dsopp_main_path,
                                            track_bin_path, arguments.track2trajectory_path,
                                            output_track_positions_tum_path, output_ecef_poses_enu_path,
                                            arguments.evaluate_ate_py_path, gt_tum_path, gt_enu_path,
                                            results_odometry_txt_path, results_ecef_poses_txt_path, config_file_path,
                                            trans_error_odometry_txt_path, trans_error_ecef_poses_txt_path])
        end = time.time()
        elapsed_time = end - start
        elapsed_times += elapsed_time

        if completed_process.returncode:
            table_cols.append([dataset_directory.name, '%.3f' % elapsed_time, *(['error'] * 7)])
            continue

        trans_error = np.loadtxt(trans_error_odometry_txt_path)
        trans_errors = np.append(trans_errors, trans_error)
        add_col_trans_error(table_cols, dataset_directory.name, elapsed_time, trans_error)

    add_col_trans_error(table_cols, 'overall', elapsed_times, trans_errors)

    with document.create(pylatex.Center()) as centered:
        with centered.create(pylatex.Tabular('|c' * len(table_cols) + '|')) as table:
            table.add_hline()
            for j in range(len(table_cols[0])):
                cells = []
                for i in range(len(table_cols)):
                    cells.append(table_cols[i][j])
                table.add_row(cells, color='lightgray' if j % 2 else None, mapper=pylatex.utils.bold if not j else None)
                table.add_hline()

        with centered.create(pylatex.Figure(position='H')) as figure:
            for dataset_directory in datasets_directory.iterdir():
                if not dataset_directory.is_dir():
                    continue

                trans_error_odometry_txt_path = dataset_directory / 'trans_error_odometry.txt'

                try:
                    trans_error = np.loadtxt(trans_error_odometry_txt_path)
                    write_plot_cdf_trans_error(centered, dataset_directory.name, trans_error, np.max(trans_errors))
                except IOError:
                    pass

            write_plot_cdf_trans_error(centered, 'overall', trans_errors, np.max(trans_errors))

            figure.add_caption('CDF of absolute odometry translational error')

    document.generate_pdf('results')


if __name__ == '__main__':
    main()
