# Installation
## 動作環境
ForRocketの開発はWindows10Pro(64bit)+Cygwin(v3.1.4 64bit)で行っている。
上記環境に関わらず、Linuxライクな環境であれば動作するはずである。
VisualStudioなど純Windows上でのコンパイルは可能なはずだが、検証はしていない。

以下、()内に開発時点で使用しているver.を示している。

## 必要ライブラリ
### boost(v1.70.0)
常微分方程式のソルバとしてboost::odeint(v2.2.0)を使用している。

boostのインストール手順は[公式サイト]("https://www.boost.org/")や[こちら]("https://boostjp.github.io/howtobuild.html")などを参照すること。なお、odeintはヘッダオンリーライブラリのためコンパイルは不要である。

License:Boost Software License

### Eigen(v3.3.7)
ベクトル・行列計算ライブラリとしてEigenを使用している。

[公式サイト]("http://eigen.tuxfamily.org/")より入手できる。
Eigenはヘッダオンリーライブラリのためコンパイル不要である。

License:Mozilla Public License v2.0

### json(v3.7.3)
jsonファイルのパースに使用している。

[githubリポジトリ]("https://github.com/nlohmann/json")より取得する。

License:MIT License

### gcc(v7.4.0)
プログラムはC++11で記述されているため、これに対応したコンパイラを用意すること。
gcc以外のコンパイルは検証していない。

### make(v4.2.1)
makefileによる自動コンパイルに対応している。


## ForRocketインストール
githubよりソースファイルを取得しコンパイルする。

デフォルトでは必要ライブラリがlibフォルダに格納されていることとしているので、ライブラリインストール場所に応じてmakefileのINCLUDESを変更すること。

```sh
$ git clone https://github.com/sus304/ForRocket.git
$ cd ForRocket
$ make release
```

## サンプル計算
デフォルトではプロジェクト内binフォルダに実行ファイルが生成される。
生成された実行ファイルに引数でsolver_config.jsonを与えることで軌道計算が行われる。

ここではCygwinでの実行例を示す。環境に応じて拡張子など変更すること。
```sh
$ cd bin
$ ./ForRocket.exe sample_solver_config.json
```

実行した後、以下の表示とともにsample_stage1_flight_logが生成されればコンパイルが正常に終了している。

```sh
$ ./ForRocket.exe sample_solver_config.json
ForRocket v4.1.0 Contact.
Solver Start.
Solver Terminate.
Export Result ...
Export Compleate.

Running Time: 2904 msec
Good Day.
```

計算は実行ファイルと入力jsonファイルのみに依存する。binフォルダから任意のディレクトリに移動しても構わない。