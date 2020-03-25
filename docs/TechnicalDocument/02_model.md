# Model
## 座標系

| 座標系 | 記号 | 原点 |  |
|----|----|----|----|
| 地球中心慣性座標系 <br> ECI <br> (Earth Center, Inertia) | I <br> ($X^I$,$Y^I$,$Z^I$) | 地球中心 | $X^I$:$t=0$で緯度経度0 deg方向 <br> $Y^I$:右手直交座標となる方向 <br> $Z^I$:地球自転軸で北極方向 |
| 地球中心地球固定座標系 <br> ECEF <br> (Earth Center, Earth Fixed) | E <br> ($X^E$,$Y^E$,$Z^E$) | 地球中心 | $X^E$:緯度経度0 deg方向 <br> $Y^E$:右手直交座標となる方向 <br> $Z^E$:地球自転軸で北極方向 |
| 局所水平面座標系 <br> NED <br> (North, East, Down) | L <br> ($X^L$,$Y^L$,$Z^L$) | 機体重心 | $X^L$:局所水平面内で北方向 <br> $Y^L$:局所水平面内で東方向 <br> $Z^L$:水平面垂直に地球中心方向 |
| 機体座標系 <br> BODY | B <br> ($X^B$,$Y^B$,$Z^B$) | 機体重心 | $X^B$:機軸方向(ロール軸) <br> $Y^B$:頭上げ方向(ピッチ軸) <br> $Z^B$:右首振り方向(ヨー軸) |
| 測地座標系 <br> LLH <br> (Latitude, Longitude, Height) | G <br> ($X^G$,$Y^G$,$Z^G$) | - | $X^G$:地理緯度 [deg] <br> $Y^G$:経度 [deg] <br> $Z^G$:海抜高度 [m] |

## 座標変換
異なる座標系で表現されているベクトルの座標系変換は座標変換行列を用いて表現する。
座標変換行列は方向余弦行列 (Direct Cosine Matrix, DCM) で定義され、
J座標系をK座標系に変換するDCMを $C^{KJ}$ と表す。

例えば、I系の速度ベクトル $V^I$ をE系に変換するには
```math
V^E = C^{EI} \cdot V^I
```
となる。


## 並進運動方程式
並進運動では慣性座標系で定義した運動方程式を解く。

```math
\frac{d\boldsymbol{r^I}}{dt} = \boldsymbol{v^I}
```

```math
\begin{align}

\frac{d\boldsymbol{v^I}}{dt} &= \frac{\boldsymbol{F^I}}{m} \\
&= \frac{C^{IB} \cdot \boldsymbol{F^B}}{m}

\end{align}
```
<!-- \mathbf -->

## 外力
運動方程式中に表れる機体座標系の外力 $F^B$ の詳細について記す。

```math
\boldsymbol{F}^B = \boldsymbol{F}^B_T + \boldsymbol{F}^B_A + \boldsymbol{F}^B_g
```

### ロケットエンジン/モータ推力
ロケットエンジンによる推力 $T$ を機体座標系で定義すれば

```math
\boldsymbol{F}^B_T = [T, 0, 0]^T
```

ForRocketにおける推力入力は真空中推力[N]であるため、大気圧環境下では圧力推力補正をかける。

```math
T = T_{vac} - P_aA_e
```

さらに、機体軸に対する推力軸のミスアライメントを$\varepsilon_y$、$\varepsilon_z$で定義すると

```math
\boldsymbol{T}^B = T \cdot [\cos{\varepsilon_y} \cos{\varepsilon_z}, \sin{\varepsilon_y}, -\sin{\varepsilon_y}]^T
```
したがって、機体座標系の推力 $F^B_T$ は以下となる。

```math
\boldsymbol{F}^B_T = (T_{vac} - P_aA_e) \cdot [\cos{\varepsilon_y} \cos{\varepsilon_z}, \sin{\varepsilon_y}, -\sin{\varepsilon_y}]^T
```

### 空気力
空気力は機体座標系で定義し、軸力 $F_A$ と法線力 $F_N$ で扱う。航空機でよく用いられる抗力 $F_D$ と揚力 $F_L$ とは軸が異なるため注意。

風の影響を含めるため対気速度 $\boldsymbol{v}_{air}$ から迎角 $\alpha$ 、横滑り角 $\beta$ と動圧 $Q$ を算出する。

```math
\boldsymbol{v}_{air}^B = C^{BL} \cdot (\boldsymbol{v}^L - \boldsymbol{v}_{wind}^L)
```
```math
\alpha = \arcsin{\frac{\boldsymbol{v}_{air,z}}{\begin{vmatrix}\boldsymbol{v}_{air}\end{vmatrix}}}
```
```math
\beta = \arcsin{\frac{\boldsymbol{v}_{air,z}}{\begin{vmatrix}\boldsymbol{v}_{air}\end{vmatrix}}}
```
```math
Q = \frac{1}{2} \rho \begin{vmatrix}\boldsymbol{v}_{air}\end{vmatrix}^2
```

yおよびz軸の法線力と軸力で機体座標系の空力を表す。

```math
\boldsymbol{F}_A^B = [-F_A, F_{N,y}, -F_{N,z}]^T
```
```math
F_A = Q \cdot C_{A} \cdot S
```
```math
F_{N,z} = Q \cdot C_{N\alpha} \cdot S \cdot \alpha
```
```math
F_{N,y} = Q \cdot C_{N\alpha} \cdot S \cdot \beta
```

なお、軸力係数 $C_A$ および法線力傾斜 $C_{N\alpha}$ はMach数の関数として入力される。
本来は迎角の関数でもあるが、ロケットは大きな迎角を持って飛行することが少なく、影響が限定的であるため無視している。一応、ForRocketの機能として、入力処理はないものの、Mach数と迎角の2変数関数として扱う機能は実装されている。


### 重力
重力はL系で定義し、地球を球とした簡易計算を行う。

```math
g = \frac{GM}{h^2}
```
```math
\boldsymbol{g}^L = [0, 0, g]^T
```

外力としての重力は
```math
\boldsymbol{F}_g^B = m \cdot C^{BL} \cdot \boldsymbol{g}^L
```
で計算する。


## オイラー角
姿勢表示にはL系とB系間のオイラー角を用いる。

| 姿勢角 |定義|対応するB系回転軸|
|----|----|----|----|
| 方位角 <br> Azimuth $\psi$ | $X^L-Y^L$ 面への $X^B$ 軸の射影と $X^L$ がなす角 | $Z^B$ 軸回転 <br> ヨー回転 |
| 上下角 <br> Elevation $\theta$ | $X^B$ 軸と $X^L-Y^L$ 面がなす角 | $Y^B$ 軸回転 <br> ピッチ回転 |
| ロール角 <br> Roll $\phi$ | $X^B$ 軸まわりの回転角 | $X^B$ 軸回転 <br> ロール回転 |

## クォータニオン
オイラー角によるL系からB系への座標変換行列を表すと

```math
C^{BL} = 
\begin{bmatrix}
 \cos\psi \cos\theta & sin\psi \cos\theta & -sin\theta \\
 -sin\psi \cos\phi + \cos\psi sin\theta sin\phi & \cos\psi \cos\phi + sin\psi sin\theta sin\phi & \cos\theta sin\phi \\
 sin\psi sin\phi + \cos\psi sin\theta \cos\phi & -\cos\psi sin\phi + sin\psi sin\theta \cos\phi & \cos\theta \cos\phi
\end{bmatrix}
```

となるが、$\theta=\pm{\frac{\pi}{2}}$ の時に

```math
C^{BL} = 
\begin{bmatrix}
 0 & 0 & -1 \\
 -sin\psi \cos\phi + \cos\psi sin\phi & \cos\psi \cos\phi + sin\psi sin\phi & 0 \\
 sin\psi sin\phi + \cos\psi \cos\phi & -\cos\psi sin\phi + sin\psi \cos\phi & 0
\end{bmatrix}
```

となるため、1軸 (方位角) の回転しか表せない。
このようなオイラー角の特異点を回避するため、計算ではクォータニオンを用いる。

クォータニオンを用いたL系からB系への座標変換行列は

```math
C^{BL} = 
\begin{bmatrix}
 q_0^2 - q_1^2 - q_2^2 + q_3^2 & 2 (q_0  q_1 + q_2  q_3) & 2 (q_0  q_2 - q_1  q_3) \\
 2 (q_0  q_1 - q_2  q_3) & q_1^2 - q_0^2 - q_2^2 + q_3^2 & 2 (q_1  q_2 + q_0  q_3) \\
 2 (q_0  q_2 + q_1  q_3) & 2 (q_1  q_2 - q_0  q_3) & q_2^2 - q_0^2 - q_1^2 + q_3^2
\end{bmatrix}
```

となり、クォータニオンの時間微分は

```math
\boldsymbol{\omega}^B = [p, q, r]^T
```
```math
\frac{d\boldsymbol{q}}{dt} = \frac{1}{2}
\begin{bmatrix}
 0 & r & -q & p \\
 -r & 0 & p & q \\
 q & -p & 0 & r \\
 -p & -q & -r & 0
\end{bmatrix}
\boldsymbol{q}
```
となる。


## 回転運動方程式
回転運動では機体座標系で定義した運動方程式を解く。機体座標系は剛体に固定された座標系であるため、見かけの力としてジャイロモーメントを加える。

```math
\boldsymbol{M}^B = \frac{d\boldsymbol{L}}{dt} + \boldsymbol{\omega^B} \times \boldsymbol{L}
```

$L = I \omega$を用いれば

```math
\boldsymbol{M}^B = 
\dot{\boldsymbol{I}} \boldsymbol{\omega^B}
+ \boldsymbol{I} \dot{\boldsymbol{\omega}}^B
+ \boldsymbol{\omega^B} \times \boldsymbol{I} \boldsymbol{\omega^B}
```
となり、整理すれば
```math
\boldsymbol{I} \dot{\boldsymbol{\omega}}^B = 
\boldsymbol{M}^B
- \dot{\boldsymbol{I}} \boldsymbol{\omega^B}
- \boldsymbol{\omega^B} \times \boldsymbol{I} \boldsymbol{\omega^B}
```

が解くべき回転の運動方程式となる。
ただし、慣性テンソルが一定であると仮定すれば第2項を省略でき

```math
\boldsymbol{I} \dot{\boldsymbol{\omega}}^B = 
\boldsymbol{M}^B
- \boldsymbol{\omega^B} \times \boldsymbol{I} \boldsymbol{\omega^B}
```
として扱うことができる。
慣性モーメントの変化が小さいロケットであれば、この仮定でも大きな影響はないと思われる。

一般的に回転の運動方程式で目にするのはこれを展開した式となる。
```math
\boldsymbol{\omega}^B = [p, q, r]^T
```
```math
\begin{align}
I_{xx} \dot{p} &= (I_{yy}-I_{zz})qr+M_p \\
I_{yy} \dot{q} &= (I_{zz}-I_{xx})rp+M_q \\
I_{zz} \dot{r} &= (I_{xx}-I_{yy})pq+M_r
\end{align}
```

## モーメント
運動方程式中に表れる機体座標系のモーメント $M^B$ の詳細について記す。

```math
\boldsymbol{M}^B = \boldsymbol{M}^B_T + \boldsymbol{M}^B_A + \boldsymbol{M}^B_{AD} + \boldsymbol{M}^B_{JD}
```

### 推力モーメント
推力軸のミスアライメントなどによって発生する。

外力として計算されている機体座標系での推力ベクトルと、回転中心となる重心 $X_{CG}$ と推力入力点 $X_T$ の距離をモーメントアームとして計算する。

```math
\boldsymbol{r}_T = [X_{CG} - X_T, 0, 0]^T
```
```math
\boldsymbol{M}^B_T = \boldsymbol{F}^B_T \times \boldsymbol{r}_T
```

### 空気力モーメント
外力として計算されている機体座標系での空気力ベクトルと、重心と圧力中心位置 $X_{CP}$ の距離をモーメントアームとして計算する。

```math
\boldsymbol{r}_A = [X_{CG} - X_{CP}, 0, 0]^T
```
```math
\boldsymbol{M}^B_A = \boldsymbol{F}^B_A \times \boldsymbol{r}_A
```

ロール軸の空気力モーメントについては尾翼のカント角 $\delta$ によるモーメントのみを考慮する。
カント角によるロールモーメント係数 $C_{l\delta}$ を用いて

```math
M^B_{A,x} = Q \cdot C_{l\delta} \cdot S \cdot L \cdot \delta
```
で計算する。
ここで $L$ は機体全長、$\delta$ は尾翼1枚あたりのカント角である。
入力する $C_{l\delta}$ の次元に留意すること。

### 空気力減衰モーメント
空気力減衰モーメントは各軸の減衰モーメント係数を用いて計算する。

```math
C_{ad} = [C_{lp}, C_{mq}, C_{nr}]^T
```
```math
\boldsymbol{M}^B_{AD} = Q \cdot C_{ad} \cdot S \cdot \frac{L^2}{2 \boldsymbol{v^B}} \cdot \omega^B
```

軸対称のロケットであれば $C_{mq}$ と $C_{nr}$ は同等である。

### ジェット減衰モーメント
ロケットエンジンの排気による慣性モーメントの変化や角運動量の減少、ロケットモータのグレイン形状によって計算されるモーメントであるが、計算中ではゼロにしている。

```math
\boldsymbol{M}^B_{JD} = [0, 0, 0]^T
```


## 離散化
並進、回転ともにDormand-Princeの方法で離散化を行い、微分方程式を解く。

Runge-Kutta-Dormand-Prince (RKDP) は陽的ルンゲクッタ法の1種である。7段5次の精度だが、FSAL(First Same As Last)という性質を持つため実質的に6段5次の精度を有する。また、埋込み型公式のため、刻み幅自動調整が可能である。

RKDPでの1ステップの計算が
```math
\begin{align}
k_1 &= hf(t_k, y_k) \\
k_2 &= hf(t_k + \frac{1}{5}h, y_k + \frac{1}{5}k_1) \\
k_3 &= hf(t_k + \frac{3}{10}h, y_k + \frac{3}{40}k_1 + \frac{9}{40}k_2) \\
k_4 &= hf(t_k + \frac{4}{5}h, y_k + \frac{44}{45}k_1 - \frac{56}{15}k_2 + \frac{32}{9}k_3) \\
k_5 &= hf(t_k + \frac{8}{9}h, y_k + \frac{19372}{6561}k_1 - \frac{25360}{2187}k_2 + \frac{64448}{6561}k_3 - \frac{212}{729}k_4) \\
k_6 &= hf(t_k + h, y_k + \frac{9017}{3168}k_1 - \frac{355}{33}k_2 - \frac{46732}{5247}k_3 + \frac{49}{176}k_4 - \frac{5103}{18656}k_5) \\
k_7 &= hf(t_k + h, y_k + \frac{35}{384}k_1 + \frac{500}{1113}k_3 + \frac{125}{192}k_4 - \frac{2187}{6784}k_5 + \frac{11}{84}k_6) 
\end{align}
```
となり、次ステップの値が
```math
y_{k+1} = y_k + \frac{35}{384}k_1 + \frac{500}{1113}k_3 + \frac{125}{192}k_4 - \frac{2187}{6784}k_5 + \frac{11}{84}k_6
```
となる。
さらに誤差評価用の計算を行う。
```math
t_{k+1} = y_k + \frac{5179}{57600}k_1 + \frac{7571}{16695}k_3 + \frac{393}{640}k_4 - \frac{92097}{339200}k_5 + \frac{187}{2100}k_6 + \frac{1}{40}k_7
```
$y_{k+1}$ の誤差として差分をとれば
```math
\begin{vmatrix}t_{k+1} - y_{k+1}\end{vmatrix} = 
\begin{vmatrix}
\frac{71}{57600}k_1
- \frac{71}{16695}k_3
+ \frac{71}{1920}k_4
- \frac{17253}{339200}k_5
+ \frac{22}{525}k_6
- \frac{1}{40}k_7
\end{vmatrix}
```
この誤差から求める刻み修正係数 $s$ に安全率0.9をかけて新たな刻み幅 $h_o$ を決定する。
```math
s = 0.9 \biggl(\frac{\varepsilon h}{\begin{vmatrix}t_{k+1} - y_{k+1}\end{vmatrix}}\biggr)^\frac{1}{5}
```
```math
h_o = s \cdot h
```
なお、$\varepsilon$ は許容誤差限界である。




<!-- ||$k_1$|$k_2$|$k_3$|$k_4$|$k_5$|$k_6$|$k_7$|
|----|----|----|----|----|----|----|----|
|0||||||||
|1/5|1/5|||||||
|3/10|3/40|9/40||||||
|4/5|44/45|-56/15|32/9|||||
|8/9|19372/6561|−25360/2187|64448/6561|−212/729||||
|1|9017/3168|−355/33|46732/5247|49/176|−5103/18656|||
|1|35/384|0|500/1113|125/192|−2187/6784|11/84||
||35/384|0|500/1113|125/192|−2187/6784|11/84|0|
||5179/57600|0|7571/16695|393/640|−92097/339200|187/2100|1/40| -->

 

