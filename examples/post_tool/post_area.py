import os
import glob
import numpy as np
from tqdm import tqdm

from post_tool.post_df import csv2df
from post_tool.post_summary import post_summary
from post_tool.post_kml import dump_area_kml


def post_area(area_work_dir):
    os.chdir(area_work_dir)

    # ディレクトリ内の*_flight_log.csvをリストアップする
    log_file_list = glob.glob('*_flight_log.csv')

    # stage毎に振り分け
    # 弾道と減速を振り分け
    # Stage1のみ対応
    stage1_log_file_list = []
    stage1_ballistic_log_file_list = []
    stage2_log_file_list = []
    stage3_log_file_list = []
    for file in tqdm(log_file_list):
        if '_stage1_' in file:
            if '_ballistic_' in file:
                stage1_ballistic_log_file_list.append(file)
            else:
                stage1_log_file_list.append(file)
        elif '_stage2_' in file:
            stage2_log_file_list.append(file)
        elif '_stage3_' in file:
            stage3_log_file_list.append(file)


    # ケース条件から風向風速の数を検出
    case_param_list = np.loadtxt('wind_case_list.csv', delimiter=',', skiprows=1)
    previous_speed = case_param_list[0][1]
    direction_count = 0
    for i in range(len(case_param_list)):
        # 風向数を検出
        if previous_speed != case_param_list[i][1]:
            direction_count = i
            break
    speed_count = int(len(case_param_list) / direction_count)  # 風速数を検出

    # ケースナンバーからログファイルを振り分け
    direction_loop_list = []
    speed_loop_list = []
    for i in tqdm(range(len(case_param_list))):
        case_num = int(case_param_list[i][0])
        for file in stage1_log_file_list:
            if '_wind'+str(case_num)+'_' in file:
                direction_loop_list.append(file)
                break
        
        if len(direction_loop_list) == direction_count:
            speed_loop_list.append(direction_loop_list)
            direction_loop_list = []

    ballicstic_speed_loop_list = []
    if len(stage1_ballistic_log_file_list) != 0:
        for i in tqdm(range(len(case_param_list))):
            case_num = int(case_param_list[i][0])
            for file in stage1_ballistic_log_file_list:
                if '_wind'+str(case_num)+'_' in file:
                    direction_loop_list.append(file)
                    break
            
            if len(direction_loop_list) == direction_count:
                ballicstic_speed_loop_list.append(direction_loop_list)
                direction_loop_list = []

    # 着地点を抽出してkmlへ
    speed_impact_points_latlon = []
    direction_impact_points_latlon = []
    for direction_log_files in tqdm(speed_loop_list):
        for log_file in direction_log_files:
            case_name = os.path.split(log_file)[-1]
            df, _, _ = csv2df(log_file)
            _, latlon = post_summary(df, case_name)
            direction_impact_points_latlon.append(latlon)
        speed_impact_points_latlon.append(direction_impact_points_latlon)
        direction_impact_points_latlon = []
    if len(stage1_ballistic_log_file_list) == 0:
        dump_area_kml(speed_impact_points_latlon, 'ballistic')
    else:
        dump_area_kml(speed_impact_points_latlon, 'decent')
        speed_impact_points_latlon = []
        direction_impact_points_latlon = []
        for direction_log_files in tqdm(ballicstic_speed_loop_list):
            for log_file in direction_log_files:
                case_name = os.path.split(log_file)[-1]
                df, _, _ = csv2df(log_file)
                _, latlon = post_summary(df, case_name)
                direction_impact_points_latlon.append(latlon)
            speed_impact_points_latlon.append(direction_impact_points_latlon)
            direction_impact_points_latlon = []
        dump_area_kml(speed_impact_points_latlon, 'ballistic')


    os.chdir('../')
