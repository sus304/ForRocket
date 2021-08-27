import os

from post_tool.post_df import csv2df
from post_tool.post_summary import post_summary
from post_tool.post_graph import plot_graph
from post_tool.post_kml import dump_trajectory_kml

def post_trajectory(result_csv_file_path, stdout_enable=False):
    # ポスト処理ファイルを出力するディレクトリを作成
    model_name = os.path.basename(result_csv_file_path).rsplit('_flight_log.csv', 1)[0]
    result_dir = 'result_' + model_name
    if os.path.exists(result_dir):
        resultdir_org = result_dir
        i = 1
        while os.path.exists(result_dir):
            result_dir = resultdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(result_dir)

    # Post
    df_all, df_burning, df_coasting = csv2df(result_csv_file_path)
    plot_graph(df_all, df_burning, df_coasting, result_dir+'/')
    dump_trajectory_kml(df_all, result_dir+'/')
    summary_txt, _ = post_summary(df_all, result_dir+'/')

    if stdout_enable:
        print(summary_txt)