import argparse

from runner_tool.runner_trajectroy import run_trajectroy
from runner_tool.runner_area import run_area
from runner_tool.runner_dispersion import run_dispersion

ver_runner_tool = '1.0.0'

def get_args():
    argparser = argparse.ArgumentParser(prog='RunnerTool')

    argparser.add_argument('-s', '--solver-config-json', help="Solver config json file name", type=str, required=True)

    argparser.add_argument('-a', '--area-config-json', help="Area config json file name", type=str)
    argparser.add_argument('-d', '--dispersion-config-json', help="Dispersion config json file name", type=str)

    argparser.add_argument('-X', '--use-max-thread', action='store_true', help='Using max cpu thread in area and dispersion calculate')

    argparser.add_argument('-v', '--version', action='version', version='%(prog)s '+ver_runner_tool)

    args = argparser.parse_args()
    return args


if __name__ == '__main__':
    args = get_args()

    print('ForRocket Runner Start.')

    
    if args.dispersion_config_json:
        print('== Impact Point Dispersion Calcuration Mode ==')
        # run_dispersion()
    elif args.area_config_json:
        print('== Impact Point Area Calcuration Mode ==')
        run_area(args.solver_config_json, args.area_config_json, args.use_max_thread)
    else:
        print('== Trajectory Calculation Mode ==')   
        run_trajectroy(args.solver_config_json)


