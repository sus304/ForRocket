import sys
import datetime


def GenerateNewFile(file_name):
    source_full_file_name = file_name + ".cpp"
    header_full_file_name = file_name + ".hpp"
    date = datetime.datetime.now()

    source_file = open(file_name + ".cpp", mode='w')
    header_file = open(file_name + ".hpp", mode='w')

    source_file.write("// ******************************************************\n")
    source_file.write("// Project Name    : ForRocket\n")
    source_file.write("// File Name       : " + source_full_file_name + "\n")
    source_file.write("// Creation Date   : " + date.strftime("%Y/%m/%d") + "\n")
    source_file.write("\n")
    source_file.write("// Copyright © " + date.strftime("%Y") + " Susumu Tanaka. All rights reserved.\n")
    source_file.write("// ******************************************************\n")
    source_file.write("\n")
    source_file.write('#include "' + header_full_file_name + '"\n')
    source_file.close()

    header_include_guard = ""
    for st in file_name.split("_"):
        header_include_guard += st.upper()
    header_include_guard += "_HPP_"

    header_file.write("// ******************************************************\n")
    header_file.write("// Project Name    : ForRocket\n")
    header_file.write("// File Name       : " + header_full_file_name + "\n")
    header_file.write("// Creation Date   : " + date.strftime("%Y/%m/%d") + "\n")
    header_file.write("\n")
    header_file.write("// Copyright © " + date.strftime("%Y") + " Susumu Tanaka. All rights reserved.\n")
    header_file.write("// ******************************************************\n")
    header_file.write("\n")
    header_file.write("#ifndef " + header_include_guard + "\n")
    header_file.write("#define " + header_include_guard + "\n")
    header_file.write("\n")
    header_file.write("namespace forrocket {\n")
    header_file.write("\n")
    header_file.write("}\n")
    header_file.write("\n")
    header_file.write("#endif\n")
    header_file.close()



if __name__ == "__main__":
    GenerateNewFile(sys.argv[1])


