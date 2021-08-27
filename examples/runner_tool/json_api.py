import json
import os
import shutil

def get_file_name_from_path(file_path):
    return os.path.split(file_path)[-1]

## Stage Config ####################################
def get_stage_count(solver_config):
    count_stage = solver_config.get('Number of Stage')
    return count_stage


def get_stage_config_file_path(solver_config, stage_number):
    if stage_number > get_stage_count(solver_config):
        return
    stage_config_file_item = 'Stage'+str(stage_number)+' Config File List'
    stage_config_file_path = solver_config.get(stage_config_file_item)
    return stage_config_file_path

def get_stage_config_file_name(solver_config, stage_number):
    file_path = get_stage_config_file_path(solver_config, stage_number)
    return get_file_name_from_path(file_path)

def get_stage_config(solver_config, stage_number):
    stage_config_file_path = get_stage_config_file_path(solver_config, stage_number)
    stage_config = json.load(open(stage_config_file_path))
    return stage_config


## Rocket Parameter ####################################
def get_rocket_param_file_path(stage_config):
    return stage_config.get('Rocket Configuration File Path')

def get_rocket_param_file_name(stage_config):
    file_path = get_rocket_param_file_path(stage_config)
    return get_file_name_from_path(file_path)

def get_rocket_param(stage_config):
    rocket_param_file_path = get_rocket_param_file_path(stage_config)
    rocket_param = json.load(open(rocket_param_file_path))
    return rocket_param


## Engine Parameter ####################################
def get_engine_param_file_path(stage_config):
    return stage_config.get('Engine Configuration File Path')

def get_engine_param_file_name(stage_config):
    file_path = get_engine_param_file_path(stage_config)
    return get_file_name_from_path(file_path)

def get_engine_param(stage_config):
    engine_param_file_path = get_engine_param_file_path(stage_config)
    engine_param = json.load(open(engine_param_file_path))
    return engine_param


## SOE ####################################
def get_soe_file_path(stage_config):
    return stage_config.get('Sequence of Event File Path')

def get_soe_file_name(stage_config):
    file_path = get_soe_file_path(stage_config)
    return get_file_name_from_path(file_path)

def get_soe(stage_config):
    soe_file_path = get_soe_file_path(stage_config)
    soe = json.load(open(soe_file_path))
    return soe


def is_enable_parachute(soe):
    return soe.get('Enable Parachute Open')

def is_enable_secondary_parachute(soe):
    return soe.get('Enable Secondary Parachute Open')


## Json's Copy #############################
def _file_copy_by_param(param, enable_item, file_block_item, path_item, dst_dir):
    if param.get(enable_item):
        path = param.get(file_block_item).get(path_item)
        name = get_file_name_from_path(path)
        shutil.copy2(path, dst_dir+'/'+name)

def copy_config_files(solver_config, dst_dir):
    if not os.path.exists(dst_dir):
        print('Error! Not found destination directory')
        exit()

    for i in range(get_stage_count(solver_config)):
        #  from solver config
        stage_config_file_name = get_stage_config_file_name(solver_config, i+1)
        stage_config_file_path = get_stage_config_file_path(solver_config, i+1)
        shutil.copy2(stage_config_file_path, dst_dir+'/'+stage_config_file_name)

        # from stage_config
        stage_config = get_stage_config(solver_config, i+1)
        rocket_param_file_name = get_rocket_param_file_name(stage_config)
        rocket_param_file_path = get_rocket_param_file_path(stage_config)
        shutil.copy2(rocket_param_file_path, dst_dir+'/'+rocket_param_file_name)
        engine_param_file_name = get_engine_param_file_name(stage_config)
        engine_param_file_path = get_engine_param_file_path(stage_config)
        shutil.copy2(engine_param_file_path, dst_dir+'/'+engine_param_file_name)
        soe_file_name = get_soe_file_name(stage_config)
        soe_file_path = get_soe_file_path(stage_config)
        shutil.copy2(soe_file_path, dst_dir+'/'+soe_file_name)

        # from rocket config
        rocket_param = get_rocket_param(stage_config)
        _file_copy_by_param(rocket_param, 'Enable Program Attitude', 'Program Attitude File', 'Program Attitude File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable X-C.G. File', 'X-C.G. File', 'X-C.G. File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable M.I. File', 'M.I. File', 'M.I. File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable X-C.P. File', 'X-C.P. File', 'X-C.P. File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable CA File', 'CA File', 'CA File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable CA File', 'CA File', 'BurnOut CA File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable CNa File', 'CNa File', 'CNa File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable Cld File', 'Cld File', 'CldFile Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable Clp File', 'Clp File', 'Clp File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable Cmq File', 'Cmq File', 'Cmq File Path', dst_dir)
        _file_copy_by_param(rocket_param, 'Enable Cnr File', 'Cnr File', 'Cnr File Path', dst_dir)

        # from engine config
        engine_param = get_engine_param(stage_config)
        _file_copy_by_param(engine_param, 'Enable Thrust File', 'Thrust File', 'Thrust at vacuum File Path', dst_dir)

       
    
