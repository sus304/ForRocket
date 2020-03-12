# # coding: utf-8
from pathlib import Path


def MakeSrcList():
    src_path = "../src"
    p = Path(src_path)

    # 直下のファイルとディレクトリを取得
    file_list = list(p.glob("**/*"))
    for i in range(len(file_list)):
        file_list[i] = file_list[i].as_posix().replace("../src", "")
    print(file_list)
    f = open("file_list.txt", "w")
    f.write("\n".join(file_list))
    f.close()



if __name__ == "__main__":
    MakeSrcList()

