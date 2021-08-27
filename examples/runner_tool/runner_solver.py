import shutil
import subprocess

solver_binaly_name = "ForRocket"

def copy_solver_binary(dst_dir):
    shutil.copy2(solver_binaly_name, dst_dir+'/'+solver_binaly_name)

def run_solver(solver_config_json_file_path):
    cmd = './' + solver_binaly_name + ' ' + solver_config_json_file_path
    res = subprocess.run(cmd, shell=True, text=True, stdout=subprocess.PIPE)
    # print(res.stdout) 
    
    # p = subprocess.Popen(cmd, shell=False)  # 標準出力なし
    # p.wait()
