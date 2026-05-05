# Output Style

## 出力ファイル

出力ファイル名: `{Model ID}_stage{N}_flight_log.csv`

- `Model ID` は `solver_config.json` の `"Model ID"` フィールド
- `N` はステージ番号（1, 2, 3, …）
- 出力先は ForRocket を**実行したカレントディレクトリ**（バイナリのディレクトリではない）

例: `"Model ID": "sample"` で 1 段ロケット → `sample_stage1_flight_log.csv`

## 出力列

デフォルト出力（フルダンプ）と最小出力（`-m` オプション）の 2 種類がある。

### 最小出力（`-m` オプション）

`-m` オプション指定時は以下の列のみ出力される:

| 列名 | 単位 | 説明 |
|---|---|---|
| `Time [s]` | s | ソルバ時刻 |
| `Burn Time [s]` | s | 点火後燃焼経過時間 |
| `Latitude [deg]` | deg | 地理緯度 |
| `Longitude [deg]` | deg | 経度 |
| `Altitude [m]` | m | 海抜高度 |
| `Downrange [m]` | m | 射点からの水平距離 |

### フルダンプ（デフォルト）

以下の順番で出力される。総列数は約 106 列。

#### 基本（全出力共通）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Time [s]` | s | ソルバ時刻 |
| `Burn Time [s]` | s | 点火後燃焼経過時間 |

#### 大気・質量・空力係数（フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `AirDensity [kg/m3]` | kg/m³ | 大気密度 |
| `AirPressure [kPa]` | kPa | 大気圧 |
| `AirTemprature [K]` | K | 大気温度 |
| `SpeedOfSound [m/s]` | m/s | 音速 |
| `Propellant Mass [kg]` | kg | 残推進剤質量 |
| `Mass [kg]` | kg | 全機質量 |
| `X-C.G. [%]` | % | 重心位置（全長比） |
| `X-C.P. [%]` | % | 圧力中心位置（全長比） |
| `StaticMargin [%]` | % | スタティックマージン（(Xcg−Xcp)/L × 100） |
| `xx_InertiaTensor [kg-m2]` | kg·m² | 慣性テンソル成分（3×3、行優先で 9 列） |
| `xy_InertiaTensor [kg-m2]` | kg·m² | |
| … (9 列) | | |
| `Thrust [N]` | N | エンジン推力 |
| `Mdot [kg/s]` | kg/s | 推進剤質量流量 |
| `Burning [0/1]` | — | 燃焼中フラグ（1=燃焼中） |
| `y-Gimbal [deg]` | deg | ギンバル角（ピッチ方向） |
| `z-Gimbal [deg]` | deg | ギンバル角（ヨー方向） |
| `CA [-]` | — | 軸力係数（現在値） |
| `CNa [-]` | — | 法線力傾斜（現在値） |
| `Cld [-]` | — | フィンカントロールモーメント係数（現在値） |
| `Clp [-]` | — | ロール減衰モーメント係数（現在値） |
| `Cmq [-]` | — | ピッチ減衰モーメント係数（現在値） |
| `Cma [-]` | — | ピッチモーメント係数 (= CNa × (Xcg−Xcp)/L) |
| `AoA [deg]` | deg | 迎角 |
| `AoS [deg]` | deg | 横滑り角 |

#### 力（機体座標系・フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Fx-aero [N]` | N | 空気力 X 成分（機体系） |
| `Fy-aero [N]` | N | 空気力 Y 成分 |
| `Fz-aero [N]` | N | 空気力 Z 成分 |
| `Fx-thrust [N]` | N | 推力 X 成分（機体系） |
| `Fy-thrust [N]` | N | 推力 Y 成分 |
| `Fz-thrust [N]` | N | 推力 Z 成分 |
| `Fx-gravity [N]` | N | 重力 X 成分（機体系） |
| `Fy-gravity [N]` | N | 重力 Y 成分 |
| `Fz-gravity [N]` | N | 重力 Z 成分 |

#### 加速度（フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Accx-body [m/s2]` | m/s² | 機体系加速度 X |
| `Accy-body [m/s2]` | m/s² | 機体系加速度 Y |
| `Accz-body [m/s2]` | m/s² | 機体系加速度 Z |
| `Gccx-body [G]` | G | 機体系加速度 X [G] |
| `Gccy-body [G]` | G | 機体系加速度 Y [G] |
| `Gccz-body [G]` | G | 機体系加速度 Z [G] |
| `Accx-ECI [m/s2]` | m/s² | ECI 系加速度 X |
| `Accy-ECI [m/s2]` | m/s² | ECI 系加速度 Y |
| `Accz-ECI [m/s2]` | m/s² | ECI 系加速度 Z |

#### 速度（フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Vx-body [m/s]` | m/s | 機体系対気速度 X |
| `Vy-body [m/s]` | m/s | 機体系対気速度 Y |
| `Vz-body [m/s]` | m/s | 機体系対気速度 Z |
| `MachNumber [-]` | — | マッハ数 |
| `DynamicPressure [kPa]` | kPa | 動圧 |
| `Vx-NED [m/s]` | m/s | NED 系速度 X（北） |
| `Vy-NED [m/s]` | m/s | NED 系速度 Y（東） |
| `Vz-NED [m/s]` | m/s | NED 系速度 Z（下） |
| `Vx-ECEF [m/s]` | m/s | ECEF 系速度 X |
| `Vy-ECEF [m/s]` | m/s | ECEF 系速度 Y |
| `Vz-ECEF [m/s]` | m/s | ECEF 系速度 Z |
| `Vx-ECI [m/s]` | m/s | ECI 系速度 X |
| `Vy-ECI [m/s]` | m/s | ECI 系速度 Y |
| `Vz-ECI [m/s]` | m/s | ECI 系速度 Z |

#### 位置（フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `X-ECEF [km]` | km | ECEF 系位置 X |
| `Y-ECEF [km]` | km | ECEF 系位置 Y |
| `Z-ECEF [km]` | km | ECEF 系位置 Z |
| `X-ECI [km]` | km | ECI 系位置 X |
| `Y-ECI [km]` | km | ECI 系位置 Y |
| `Z-ECI [km]` | km | ECI 系位置 Z |

#### 位置・射程（全出力共通）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Latitude [deg]` | deg | 地理緯度 |
| `Longitude [deg]` | deg | 経度 |
| `Altitude [m]` | m | 海抜高度 |
| `Downrange [m]` | m | 射点からの水平距離（Vincenty 測地線距離） |

#### モーメント・姿勢（フルダンプのみ）

| 列名 | 単位 | 説明 |
|---|---|---|
| `Mx-thrust [Nm]` | N·m | 推力モーメント X（機体系） |
| `My-thrust [Nm]` | N·m | 推力モーメント Y |
| `Mz-thrust [Nm]` | N·m | 推力モーメント Z |
| `Mx-aero [Nm]` | N·m | 空力モーメント X |
| `My-aero [Nm]` | N·m | 空力モーメント Y |
| `Mz-aero [Nm]` | N·m | 空力モーメント Z |
| `Mx-aerodump [Nm]` | N·m | 空力減衰モーメント X |
| `My-aerodump [Nm]` | N·m | 空力減衰モーメント Y |
| `Mz-aerodump [Nm]` | N·m | 空力減衰モーメント Z |
| `Mx-jetdump [Nm]` | N·m | ジェット減衰モーメント X（現在は常に 0） |
| `My-jetdump [Nm]` | N·m | ジェット減衰モーメント Y |
| `Mz-jetdump [Nm]` | N·m | ジェット減衰モーメント Z |
| `Mx-gasjet [Nm]` | N·m | ガスジェットモーメント X（ロール） |
| `My-gasjet [Nm]` | N·m | ガスジェットモーメント Y（通常 0） |
| `Mz-gasjet [Nm]` | N·m | ガスジェットモーメント Z（通常 0） |
| `Mx-gyroeffect [Nm]` | N·m | ジャイロ効果モーメント X |
| `My-gyroeffect [Nm]` | N·m | ジャイロ効果モーメント Y |
| `Mz-gyroeffect [Nm]` | N·m | ジャイロ効果モーメント Z |
| `Mx [Nm]` | N·m | 合計モーメント X |
| `My [Nm]` | N·m | 合計モーメント Y |
| `Mz [Nm]` | N·m | 合計モーメント Z |
| `AngleAccx [rad/s2]` | rad/s² | 角加速度 X（機体系） |
| `AngleAccy [rad/s2]` | rad/s² | 角加速度 Y |
| `AngleAccz [rad/s2]` | rad/s² | 角加速度 Z |
| `AngleVelx [deg/s]` | deg/s | 角速度 X（ロール、機体系） |
| `AngleVely [deg/s]` | deg/s | 角速度 Y（ピッチ） |
| `AngleVelz [deg/s]` | deg/s | 角速度 Z（ヨー） |
| `q1 [-]` | — | 姿勢クォータニオン q0（スカラー部） |
| `q2 [-]` | — | 姿勢クォータニオン q1（ベクトル部 x） |
| `q3 [-]` | — | 姿勢クォータニオン q2（ベクトル部 y） |
| `q4 [-]` | — | 姿勢クォータニオン q3（ベクトル部 z） |
| `Azimuth [deg]` | deg | 方位角 [0, 360) |
| `Elvation [deg]` | deg | 上下角（**注: ソース上の typo。Elevation の誤記**） |
| `Roll [deg]` | deg | ロール角 |

## 注意事項

- **`Elvation [deg]`**: ソースコード中の typo（`Elevation` の誤り）。スクリプトでこの列名を参照する際は `"Elvation [deg]"` を使用すること。
- **`Azimuth [deg]`**: `[0, 360)` の範囲に変換済みで出力される。
- 高度がゼロ以上の行のみ記録される（地面着地後の行は出力されない）。
