import pandas as pd

def csv2df(csv_file_path):
    df = pd.read_csv(csv_file_path)
    df = df.dropna(how='any', axis=1)  # NaN落とし
    df_burning = df[df["Burning [0/1]"] == 1]
    df_coasting = df[df["Burning [0/1]"] == 0]
    return df, df_burning, df_coasting