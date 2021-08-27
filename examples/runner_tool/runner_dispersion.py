import os

def run_dispersion():
    # 分散計算ファイルを出力するディレクトリを作成
    work_dir = './work_dispersion'
    if os.path.exists(work_dir):
        workdir_org = work_dir
        i = 1
        while os.path.exists(work_dir):
            work_dir = workdir_org + '_%02d' % (i)
            i = i + 1
    os.mkdir(work_dir)