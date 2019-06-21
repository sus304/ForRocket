import os
import sys
import datetime
import argparse
import json
from Simulator.rocket_param import Rocket
import Simulator.solver as solver

ver_ForRocket = '3.0.2'

def forrocket(args):
    if args.quiet:
        print('Welcome to ForRocket!')
    time_start_calc = datetime.datetime.now()

    config_file = args.JsonConfigFile

    json_config = json.load(open(config_file))
    json_engine = json.load(open(json_config.get('System').get('Engine Config json')))
    model_name = json_config.get('System').get('Name')

    if args.quiet:
        print('Config File: ', config_file)
        print('Config File: ', json_config.get('System').get('Engine Config json'))
        print('Model: ', model_name)

    ## Make Result Directory
    result_dir = './Result_single_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)

    ## make rocket instance
    rocket = Rocket(json_config, json_engine, result_dir)

    ## run solver
    if args.quiet:
        print('Running Solver...')
    solver.solve_trajectory(rocket)

    if args.full:
        rocket.result.output_full(rocket)
    elif args.minimum:
        rocket.result.output_min(rocket)
    else:
        rocket.result.output(rocket)

    if args.quiet:
        print('Completed solve!')
        print('Output Result Directory: ', result_dir)
    
    if args.summary:
        txt = open(result_dir + '/result.txt', mode='r').read()
        print(txt)

    time_end_calc = datetime.datetime.now()
    if args.quiet:
        print('Calculate Time: ', time_end_calc - time_start_calc)

def get_args():
    argparser = argparse.ArgumentParser(prog='ForRocket')

    argparser.add_argument('JsonConfigFile', help="rocket configuration json file.", type=str)

    argparser.add_argument('-f', '--full', action="store_true", help='set full output mode (result summary, log csv file, graph picture, kml file). Prior to "minimum output".')
    argparser.add_argument('-m', '--minimum', action="store_true", help='set minimum output mode (result summary).')
    argparser.add_argument('-s', '--summary', action="store_true", help='display result summary.')
    argparser.add_argument('-q', '--quiet', action="store_false", help='disable display configuration messeage.')
    argparser.add_argument('-v', '--version', action='version', version='%(prog)s '+ver_ForRocket)

    args = argparser.parse_args()
    return args


if __name__ == '__main__':
    args = get_args()

    forrocket(args)