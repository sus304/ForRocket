# 1条件計算ランナー

import os
import json

from json_api import get_stage_count, get_stage_config, get_stage_config_file_name
from json_api import get_soe, get_soe_file_name
from json_api import is_enable_parachute, is_enable_secondary_parachute
from ballistic_generator import generate_ballistic_config_json_file_path

from runner_solver import run_solver

def run_trajectroy(solver_config_json_file_path):
    solver_config = json.load(open(solver_config_json_file_path))

    # 要否判断のためのフラグ
    enable_decent = False
    
    stage_config = get_stage_config(solver_config, 1)
    soe = get_soe(stage_config)

    # パラシュートが有効の場合は無効版のjsonを生成
    if is_enable_parachute(soe):            
        soe['Enable Parachute Open'] = False
        if is_enable_secondary_parachute(soe):
            soe['Enable Secondary Parachute Open'] = False
        
        # jsonを別名保存
        soe_file_path_ballistic = generate_ballistic_config_json_file_path(get_soe_file_name(stage_config))
        stage_config['Sequence of Event File Path'] = soe_file_path_ballistic
        
        stage_config_file_path_ballistic = generate_ballistic_config_json_file_path(get_stage_config_file_name(solver_config, 1))
        solver_config['Stage1 Config File List'] = stage_config_file_path_ballistic

        if not os.path.exists(soe_file_path_ballistic):
            with open(soe_file_path_ballistic, 'w') as f:
                json.dump(soe, f, indent=4)
        if not os.path.exists(stage_config_file_path_ballistic):
            with open(stage_config_file_path_ballistic, 'w') as f:
                json.dump(stage_config, f, indent=4)
        
        enable_decent = True


    if enable_decent:
        # Model IDを無効版に変更
        solver_config['Model ID'] = solver_config.get('Model ID') + '_ballistic'
        # jsonを別名保存
        solver_config_file_path_ballistic = generate_ballistic_config_json_file_path(solver_config_json_file_path)
        with open(solver_config_file_path_ballistic, 'w') as f:
            json.dump(solver_config, f, indent=4)

        run_solver(solver_config_file_path_ballistic)

    run_solver(solver_config_json_file_path)

    return True
    
