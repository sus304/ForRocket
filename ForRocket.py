import os
import sys
import datetime

import json
from Simulator.rocket_param import Rocket
import Simulator.solver as solver

def forrocket(config_file, mode_output):
    time_start_calc = datetime.datetime.now()

    # config file to json
    json_config = json.load(open(config_file))
    print('Config File: ', config_file)
    json_engine = json.load(open(json_config.get('System').get('Engine Config json')))
    print('Config File: ', json_config.get('System').get('Engine Config json'))

    model_name = json_config.get('System').get('Name')
    print('Model: ', model_name)

    # Make Result Directory
    result_dir = './Result_single_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)

    # make rocket instance
    rocket = Rocket(json_config, json_engine, result_dir)

    # run solver
    print('Running Solver...')
    solver.solve_trajectory(rocket)
    if mode_output == 'full':
        rocket.result.output_full(rocket)
    elif mode_output == 'min':
        rocket.result.output_min(rocket)
    else:
        rocket.result.output(rocket)
    print('Completed solve!')
    print('Output Result Directory: ', result_dir)    

    time_end_calc = datetime.datetime.now()
    print('Calculate Time: ', time_end_calc - time_start_calc)


if __name__ == '__main__':
    print('Welcome to ForRocket!')
    # config file arg
    argv = sys.argv
    if len(argv) < 2:
        print('Error!! config file is missing')
        print('Usage: python ForRocket.py configFileName.json -[output mode]')
        sys.exit()  

    arg_file = argv[1]
    if '.json' not in arg_file:
        print('Error!! Secondary arg is config file name with ".json" extention')
        sys.exit()
    
    try:
        output = argv[2]
    except:
        output = 'default'
    if output == '-f' or output == '--full':
        output = 'full'
    elif output == '-m' or output == '--min':
        output = 'min'

    forrocket(arg_file, output)