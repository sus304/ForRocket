# Theory
## 地球モデル
地球モデルとしてWGS84を使用している。

[パラメータファイル](../../src/environment/wgs84.hpp)

## 大気モデル
地上から高度1000 kmまでの大気モデルとしてU.S. Standard Atmosphere 1976を使用している。

[アルゴリズム](../../src/environment/satmo1976.cpp)

## 2点間距離の計算
Vincentyの方法を用いて楕円体上の2点間の距離となる測地線を計算する。
