


def generate_ballistic_config_json_file_path(json_file_path):
    """
    '****.json'を'****_ballistic.json'にする
    """
    sep = json_file_path.rsplit('.json', 1)
    return sep[0] + '_ballistic' + '.json'

