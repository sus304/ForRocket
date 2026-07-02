# Installation

## 動作環境

| 環境 | 用途 |
|---|---|
| WSL2 (Ubuntu 20.04 以降) | 開発・デバッグ・Linux バイナリのビルド |
| WSL2 + MinGW-w64 | Windows 用バイナリのクロスコンパイル |
| Windows (実行のみ) | 生成した `ForRocket.exe` をそのまま実行可能 |

Windows ネイティブでのコンパイルは検証していない。

## 必要なツール・ライブラリ

### ビルドツール

| ツール | 最低バージョン | 備考 |
|---|---|---|
| CMake | 3.13 | ビルドシステム |
| g++ / GCC | 7 以降 | C++11 対応必須 |
| MinGW-w64 (オプション) | — | Windows 向けクロスコンパイル用 |

### 外部ライブラリ（要インストール）

| ライブラリ | バージョン | 用途 | ライセンス |
|---|---|---|---|
| boost (boost::odeint) | 1.70 以降 | 常微分方程式ソルバ | Boost Software License |

boost は `apt` 等でインストールする:
```sh
sudo apt install libboost-dev
```

### 同梱ライブラリ（`lib/` ディレクトリに含まれる）

| ライブラリ | バージョン | 用途 | ライセンス |
|---|---|---|---|
| Eigen | 3.3.7 | ベクトル・行列計算 | Mozilla Public License v2.0 |
| nlohmann/json | 3.7.3 | JSON パース | MIT License |

これらはリポジトリに含まれているため、別途インストール不要。

## ForRocket のビルド

### ソースの取得

```sh
git clone https://github.com/sus304/ForRocket.git
cd ForRocket
```

### Linux バイナリのビルド

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
# 生成物: build/ForRocket
```

### Windows バイナリのクロスコンパイル（WSL2 上）

MinGW-w64 のインストール:
```sh
sudo apt install mingw-w64
```

ビルド:
```sh
cmake -B build-win -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64-x86_64.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-win
# 生成物: build-win/ForRocket.exe（スタティックリンク済み、Windows 側でランタイム DLL 不要）
```

### Windows 用パッケージの作成

```sh
bash build_package.sh
# カレントディレクトリに ForRocket_v{ver}_{datetime}.zip が生成される
# 内容: ForRocket.exe + examples/ の各サンプルファイル
```

## サンプル計算

```sh
cd examples
../build/ForRocket sample_config_solver.json
```

正常に終了すると以下のように表示され、`sample_stage1_flight_log.csv` が生成される:

```
ForRocket v4.4.2 Contact.
Solver Start.
Solver Terminate.
Export Result ...
Export Complete.

Running Time: 123 msec
Good Day.
```

計算は実行バイナリと入力 JSON ファイルのみに依存する。任意のディレクトリから実行可能（出力は実行ディレクトリに生成される）。
