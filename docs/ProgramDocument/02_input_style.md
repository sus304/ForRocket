# Input Style

```sh
./ForRocket [option] solver_config.json
```

## コマンドラインオプション

| オプション | 内容 |
|---|---|
| `-m` | 出力 CSV のカラム数を最小限にする（位置・高度・射程のみ） |
| `-q` / `--quiet` | 進行メッセージを非表示にする |
| `-h` / `--help` | ヘルプを表示して終了 |
| `-v` / `--version` | バージョンを表示して終了 |

## 入力ファイル一覧

| ファイル | 内容 |
|---|---|
| `solver_config.json` | 打上条件・風・ステージ構成（エントリーポイント） |
| `stage_config_list.json` | 各ステージの 3 ファイルへのパス |
| `sequence_of_event.json` | フライトシーケンスイベントの有無とタイミング |
| `rocket_config.json` | 機体の構造・空力・姿勢制御パラメータ |
| `engine_config.json` | エンジンのパラメータ |
| `wind.csv` | 風の高度分布（Enable Wind: true の場合） |
| `thrust.csv` | 推力・質量流量の時間履歴（Enable Thrust File: true の場合） |
| `attitude.csv` | 姿勢プログラムの時間履歴（Enable Program Attitude: true の場合） |
| `Xcg.csv` | 全機重心位置の時間履歴（Enable X-C.G. File: true の場合） |
| `Xcp.csv` | マッハ数と圧力中心位置の関係（Enable X-C.P. File: true の場合） |
| `MOI.csv` | 全機慣性モーメントの時間履歴（Enable M.I. File: true の場合） |
| `Ixy.csv` / `Ixz.csv` / `Iyz.csv` | 慣性乗積の時間履歴（Enable Product of Inertia File: true の場合） |
| `CA.csv` / `CAbo.csv` | マッハ数と軸力係数（燃焼中/燃焼後）の関係（Enable CA File: true の場合） |
| `CNa.csv` | マッハ数と法線力傾斜の関係 |
| `Cld.csv` | マッハ数とフィンカントロールモーメント係数の関係 |
| `Clp.csv` | マッハ数とロール減衰モーメント係数の関係 |
| `Cmq.csv` | マッハ数とピッチ減衰モーメント係数の関係 |
| `Cnr.csv` | マッハ数とヨー減衰モーメント係数の関係 |

すべてのファイルはファイル名に制約はなく、JSON 内のパスと一致したファイルを配置すればよい。

---

## solver_config.json

```json
{
    "Model ID": "sample",

    "Launch DateTime": "2020/08/23 9:00:00.0",
    "Launch Condition": {
        "Latitude [deg]": 40.242865,
        "Longitude [deg]": 140.01045,
        "Height for WGS84 [m]": 20.0,

        "Azimuth [deg]": 270.0,
        "Elevation [deg]": 85.0,

        "North Velocity [m/s]": 0.0,
        "East Velocity [m/s]": 0.0,
        "Down Velocity [m/s]": 0.0,

        "Yaw Angular Velocity [deg/s]": 0.0,
        "Pitch Angular Velocity [deg/s]": 0.0,
        "Roll Angular Velocity [deg/s]": 0.0
    },

    "Wind Condition": {
        "Enable Wind": true,
        "Wind File Path": "sample_wind.csv"
    },

    "Gravity Model": "legacy",

    "Number of Stage": 1,
    "Stage1 Config File List": "sample_config_list_stage1.json",
    "Stage2 Config File List": "stage_config_list.json",
    "Stage3 Config File List": "stage_config_list.json"
}
```

| フィールド | 説明 |
|---|---|
| `Model ID` | 出力 CSV のファイル名プレフィックス |
| `Launch DateTime` | 打上日時（UTC）。`"YYYY/MM/DD HH:MM:SS.s"` 形式 |
| `Height for WGS84 [m]` | WGS84 楕円体基準の射点海抜高度 |
| `Azimuth [deg]` | 打上方位角（真北から時計回り、deg） |
| `Elevation [deg]` | 打上上下角（水平が 0°、直上が 90°、deg） |
| `*Velocity [m/s]` | NED 系での初期速度（通常は 0） |
| `*Angular Velocity [deg/s]` | 初期機体角速度（ヨー・ピッチ・ロール軸、deg/s） |
| `Gravity Model` | （任意）重力モデル。`"legacy"`（既定）: GM/(a+h)² を鉛直下向きに与える従来モデル / `"pointmass-j2"`: 質点＋J2 帯状調和項（地心半径・緯度依存・扁平性を考慮）。legacy は緯度 45° で重力の大きさを最大 ~0.35% 過小評価し方向誤差最大 0.19° を持つため、長時間・高高度飛行（ロックーン、軌道投入検討）では `"pointmass-j2"` を推奨（参考: 高度 19 km サンプルでもアポジ −37 m / 着地点 156 m の差） |
| `Number of Stage` | ステージ数（1〜3） |
| `Stage{N} Config File List` | 各ステージの設定ファイルリストへのパス |

---

## stage_config_list.json

```json
{
    "Rocket Configuration File Path": "sample_param_rocket.json",
    "Engine Configuration File Path": "sample_param_engine.json",
    "Sequence of Event File Path": "sample_sequence_of_event.json"
}
```

---

## sequence_of_event.json

時刻はすべて **打上時刻（X+0s）を基準とした X+n 秒**。

```json
{
    "Flight Start Time [s]": 0.0,

    "Engine Ignittion Time [s]": 0.0,

    "Enable Rail-Launcher Launch": true,
    "Rail Launcher": {
        "Length [m]": 5.0,
        "Friction Coefficient [-]": 0.2
    },

    "Enable Engine Cutoff": false,
    "Cutoff": {
        "Cutoff Time [s]": 0.0
    },

    "Enable Stage Separation": false,
    "Upper Stage": {
        "Stage Separation Time [s]": 0.0,
        "Upper Stage Mass [kg]": 100.0
    },

    "Enable Despin Control": false,
    "Despin": {
        "Time [s]": 20.0
    },

    "Enable Fairing Jettson": false,
    "Fairing": {
        "Jettson Time [s]": 0.0,
        "Mass [kg]": 1.0
    },

    "Enable Parachute Open": false,
    "Parachute": {
        "Open Time [s]": 30.0,
        "Drag Factor Cd*S [m2]": 0.3,
        "Enable Forced Apogee Open": false
    },

    "Enable Secondary Parachute Open": false,
    "Secondary Parachute": {
        "Open Time [s]": 60.0,
        "Drag Factor Cd*S [m2]": 1.3
    },

    "Flight End Time [s]": 100.0,
    "Time Step [s]": 0.1,
    "Solver Tolerance Abs": 1.0e-6,
    "Solver Tolerance Rel": 1.0e-6,
    "Enable Auto Terminate SubOrbital Flight": true
}
```

| フィールド | 説明 |
|---|---|
| `Flight Start Time [s]` | このステージの飛行開始時刻（2 段目以降は前段分離時刻に自動上書き） |
| `Engine Ignittion Time [s]` | エンジン点火時刻（Flight Start Time より後なら惰性飛行が続く） |
| `Rail Launcher.Length [m]` | ランチャの有効レール長（レールクリア距離の判定に使用） |
| `Rail Launcher.Friction Coefficient [-]` | （任意）ランチャ・ラグ間の摩擦係数。未指定時は 0.2（従来のハードコード値と同じ） |
| `Cutoff Time [s]` | エンジン強制カットオフ時刻 |
| `Stage Separation Time [s]` | 段間分離時刻 |
| `Upper Stage Mass [kg]` | 上段質量（この質量を下段から減じて計算継続） |
| `Despin.Time [s]` | デスピン時刻（瞬時にロール角速度をゼロにする） |
| `Fairing.Mass [kg]` | フェアリング質量（投棄時にこの質量を減じる） |
| `Parachute.Drag Factor Cd*S [m2]` | 第 1 パラシュートの CdS 値 |
| `Enable Forced Apogee Open` | true のとき頂点（鉛直速度符号反転）で自動開傘 |
| `Secondary Parachute.Drag Factor Cd*S [m2]` | 第 2 パラシュートの CdS 値（ドローグ→メイン想定） |
| `Time Step [s]` | 出力 CSV の時間刻み。積分器（適応ステップ dopri5）の初期刻みヒントにもなるが、実際の刻みは下の許容誤差で自動調整される |
| `Solver Tolerance Abs` | （任意）適応ステップ積分器の絶対許容誤差。未指定時は内部既定値（1.0e-6）を使用 |
| `Solver Tolerance Rel` | （任意）適応ステップ積分器の相対許容誤差。未指定時は内部既定値（1.0e-6）。緩めるほど刻みが大きくなり高速・低精度（位置誤差 ≈ Rel × 6.4e6 [m]、ECI 基準）。計算精度の実質的な制御パラメータ |

**許容誤差の推奨設定**: 既定の `1.0e-6` は ECI 位置（~6.4e6 m）換算で数 m 程度の許容誤差に相当し、サブオービタルの検討には十分。姿勢履歴の精密評価や軌道投入・長距離飛行の検討では `"Solver Tolerance Rel": 1.0e-7 〜 1.0e-9`、`"Solver Tolerance Abs": 1.0e-9` 程度まで締めること（刻み数が増えるため実行時間は数倍になる）。参考: 既定 1e-6 ではロール角 ~0.8°/60s 程度のドリフトが観測されている。
| `Enable Auto Terminate SubOrbital Flight` | true のとき全力積から着地予想時刻を自動計算して Flight End Time を上書き |

---

## rocket_config.json

```json
{
    "Diameter [mm]": 180.0,
    "Length [mm]": 3900.0,
    "Mass": {
        "Inert [kg]": 52.5,
        "Propellant [kg]": 41.0
    },

    "Enable Gas Jet": false,
    "Gas Jet": {
        "Rolling Moment [N.m]": 5.0,
        "Duration [s]": 2.5
    },

    "Enable Program Attitude": false,
    "Program Attitude": {
        "Mode": "Angle",
        "Enable Yaw":   true,
        "Enable Pitch": true,
        "Enable Roll":  true,
        "File Path": "attitude.csv"
    },

    "Enable X-C.G. File": false,
    "X-C.G. File": {
        "X-C.G. File Path": "Xcg.csv"
    },
    "Constant X-C.G.": {
        "Constant X-C.G. from BodyTail [mm]": 1100.0
    },
    "C.G. Offset": {
        "y-C.G. Offset [mm]": 0.0,
        "z-C.G. Offset [mm]": 0.0
    },

    "Comment M.I.": "Moment of Inertia",
    "Enable M.I. File": false,
    "M.I. File": {
        "M.I. File Path": "MOI.csv"
    },
    "Constant M.I.": {
        "Yaw Axis [kg-m2]": 45.0,
        "Pitch Axis [kg-m2]": 45.0,
        "Roll Axis [kg-m2]": 0.5
    },

    "Comment P.O.I.": "Product of Inertia (for spin stability analysis)",
    "Enable Product of Inertia": false,
    "Constant Product of Inertia": {
        "Ixy [kg-m2]": 0.0,
        "Ixz [kg-m2]": 0.0,
        "Iyz [kg-m2]": 0.0
    },
    "Enable Product of Inertia File": false,
    "Product of Inertia File": {
        "Ixy File Path": "Ixy.csv",
        "Ixz File Path": "Ixz.csv",
        "Iyz File Path": "Iyz.csv"
    },

    "Enable X-C.P. File": false,
    "X-C.P. File": {
        "X-C.P. File Path": "Xcp.csv"
    },
    "Constant X-C.P.": {
        "Constant X-C.P. from BodyTail [mm]": 835.0
    },

    "X-ThrustLoadingPoint from BodyTail [mm]": 300.0,
    "y-ThrustLoadingPoint Offset [mm]": 0.0,
    "z-ThrustLoadingPoint Offset [mm]": 0.0,

    "Comment CA": "Axial Force Coefficient",
    "Enable CA File": true,
    "CA File": {
        "CA File Path": "sample_CA.csv",
        "BurnOut CA File Path": "sample_CA.csv"
    },
    "Constant CA": {
        "Constant CA [-]": 0.4,
        "Constant BurnOut CA [-]": 0.5
    },

    "Comment CNa": "Normal Force div AoA Coefficient",
    "Enable CNa File": false,
    "CNa File": {
        "CNa File Path": "CNa.csv"
    },
    "Constant CNa": {
        "Constant CNa [1/rad]": 10.0
    },

    "Comment Cld": "Roll Force div FinCantAngle Coefficient",
    "Fin Cant Angle [deg]": 0.0,
    "Enable Cld File": false,
    "Cld File": {
        "Cld File Path": "Cld.csv"
    },
    "Constant Cld": {
        "Constant Cld [1/rad]": 0.0
    },

    "Comment Clp": "Roll Damping Moment Coefficient",
    "Enable Clp File": false,
    "Clp File": {
        "Clp File Path": "Clp.csv"
    },
    "Constant Clp": {
        "Constant Clp [-]": 0.03
    },

    "Comment Cmq": "Pitch Damping Moment Coefficient",
    "Enable Cmq File": false,
    "Cmq File": {
        "Cmq File Path": "Cmq.csv"
    },
    "Constant Cmq": {
        "Constant Cmq [-]": 7.0
    },

    "Comment Cnr": "Yaw Damping Moment Coefficient",
    "Enable Cnr File": false,
    "Cnr File": {
        "Cnr File Path": "Cnr.csv"
    },
    "Constant Cnr": {
        "Constant Cnr [-]": 7.0
    }
}
```

### スピン安定解析用パラメータ

スピン中のロケットでは、推力作用点・重心の機軸からの横方向ずれ（CG/推力点オフセット）、推力ベクトルの角度ずれ（ミスアライメント角）、および主軸と機軸の不一致（慣性乗積）が動的不安定（コーニング、動的アンバランス）を引き起こす。これらを再現するため以下のパラメータが用意されている。すべて省略時は 0（従来挙動と一致）。

| フィールド | 説明 |
|---|---|
| `C.G. Offset` → `y-C.G. Offset [mm]` / `z-C.G. Offset [mm]` | **乾燥構造（インート）重心**の機軸（body X 軸）からの横方向オフセット（定数のみ。X 軸のファイル入力モードでも併用される）。**v4.4.2 以降**、実効的な横方向重心は推進薬（機軸上にあると仮定）との質量重み付け `y_CG = y_CG_inert · m_inert / (m_inert + m_prop)` で算出され、推進薬の消費に伴いインート値へ漸近する |
| `y-ThrustLoadingPoint Offset [mm]` / `z-ThrustLoadingPoint Offset [mm]` | 推力作用点の機軸からの横方向オフセット |
| `Engine Miss-Alignment` の `y-Axis Angle` / `z-Axis Angle` | 推力ベクトルの機軸に対する角度ずれ（engine_config.json 側、既存） |
| `Enable Product of Inertia` | true: 定数の慣性乗積 Ixy/Ixz/Iyz を使用 |
| `Constant Product of Inertia` | Ixy, Ixz, Iyz [kg·m²]（慣性テンソル off-diagonal、主軸不一致を表現） |
| `Enable Product of Inertia File` | true: 時間履歴 CSV から慣性乗積を読み込む |
| `Product of Inertia File` | Ixy/Ixz/Iyz の CSV パス（1列目: 時刻 [s], 2列目: 値 [kg·m²]） |

CG・推力点オフセットは定数のみ対応。慣性乗積は定数/時系列の両方に対応する。

スピン付与の手段は以下を任意に組み合わせ可能：
- フィンカント（既存）: `Fin Cant Angle [deg]` + `Cld`
- ガスジェット（既存、後述）: ランチクリア後にロールトルクを印加
- 初期スピンレート: Program Attitude (Rate mode) でロールレートを与える

### Gas Jet（ガスジェット）

スピン安定用コールドガスジェットの設定。ランチクリア直後から `Duration [s]` 秒間、機体 +X 軸方向のロールモーメントを印加する。

| フィールド | 説明 |
|---|---|
| `Rolling Moment [N.m]` | ロールモーメント [N·m]。正値でスピンアップ方向 |
| `Duration [s]` | ランチクリアからの作動時間 [s] |

### Program Attitude（姿勢制御プログラム）

CSV ファイルで与えた姿勢（または角速度）に機体を追従させる。軸ごとに制御の有効/無効を設定可能。

| フィールド | 説明 |
|---|---|
| `Mode` | `"Angle"`: 角度追従 / `"Rate"`: 角速度追従 |
| `Enable Yaw` | ヨー軸制御の有効/無効（false の場合、ヨーは空力に従い自由に変化） |
| `Enable Pitch` | ピッチ軸制御の有効/無効 |
| `Enable Roll` | ロール軸制御の有効/無効 |
| `File Path` | 姿勢プログラム CSV ファイルのパス |

> **例**: ロール制御システムのみ搭載し、ピッチ・ヨーを無制御とする場合は `Enable Roll: true`、`Enable Yaw: false`、`Enable Pitch: false` と設定する。これは小型ロケットでよくある構成。

### 空力係数の入力パターン

各係数は「定数」または「マッハ数の関数（CSV）」として与えられる。CSVの場合、範囲外は先頭/末尾の値を外挿する（step-hold）。

| 係数 | 説明 |
|---|---|
| `CA` | 軸力係数（燃焼中・燃焼後で別指定可） |
| `CNa` | 法線力傾斜 [1/rad] |
| `Cld` | フィンカント角によるロールモーメント係数 [1/rad]。**v4.2.2 以降、フィン全体（クロスフィン）合計の値を与える**（旧版の「1 枚あたり係数 ×4」ではない）。ロールモーメント `= Q·Cld·S·d·δ`（`d`=機体直径、`δ`=カント角） |
| `Fin Cant Angle [deg]` | フィン 1 枚あたりのカント角（ロール正方向が正） |
| `Clp` | ロール減衰モーメント係数 [-] |
| `Cmq` | ピッチ減衰モーメント係数 [-] |
| `Cnr` | ヨー減衰モーメント係数 [-] |

> **基準長の変更（v4.2.2 以降）**: 空力モーメントの基準長は機体全長 `length` から**機体直径 `d`** に変更された。減衰モーメントは `M = Q · (Clp,Cmq,Cnr) · S · d² / (2V) · ω` で計算されるため、`Clp`/`Cmq`/`Cnr` は**直径 `d` で無次元化**した値を与えること（旧版の機体全長基準の値とは一致しない）。

重心位置・圧力中心位置・慣性モーメントも同様に定数/時間変化 CSV で指定。

---

## engine_config.json

```json
{
    "Nozzle Exit Diameter [mm]": 100.0,

    "Enable Thrust File": true,
    "Thrust File": {
        "Thrust at vacuum File Path": "sample_thrust.csv"
    },
    "Constant Thrust": {
        "Thrust at vacuum [N]": 5780.0,
        "Propellant Mass Flow Rate [kg/s]": 3.0,
        "Burn Duration [sec]": 13.7
    },

    "Enable Engine Miss Alignment": false,
    "Engine Miss-Alignment": {
        "y-Axis Angle [deg]": 0.0,
        "z-Axis Angle [deg]": 0.0
    }
}
```

| フィールド | 説明 |
|---|---|
| `Nozzle Exit Diameter [mm]` | ノズル出口直径。圧力推力補正に加え、**v4.4.2 以降はジェットダンピングモーメント**（排気がノズル出口で持ち去る角運動量による減衰）のロール軸成分 `k² = A_exit/(2π)` の算出にも使用 |
| `Enable Thrust File` | true: thrust.csv を使用 / false: 矩形推力（Constant Thrust を使用） |
| `Thrust at vacuum` | 真空中推力 [N]（実効推力 = 真空推力 − 大気圧 × 出口面積） |
| `Propellant Mass Flow Rate [kg/s]` | 定常質量流量（Constant Thrust モード時） |
| `Burn Duration [sec]` | 燃焼時間（Constant Thrust モード時） |
| `Engine Miss-Alignment` | 推力軸ミスアライメント角（ピッチ・ヨー方向、deg） |

---

## CSV ファイル仕様

すべての CSV ファイルは**1行目をヘッダとして読み飛ばす**。データ点間は線形補間。

### wind.csv

| 列 1 | 列 2 | 列 3 |
|---|---|---|
| 海面高度 [m] | 東西方向風速（東が正）[m/s] | 南北方向風速（北が正）[m/s] |

ファイル範囲外は無風（0）として扱う。

### thrust.csv（推力ファイル）

| 列 1 | 列 2 | 列 3 |
|---|---|---|
| 点火からの時間 [s] | 真空中推力 [N] | 推進剤質量流量 [kg/s] |

ファイル範囲外は推力・質量流量ともにゼロ。

### attitude.csv（姿勢プログラム）

**角度モード** (`"Mode": "Angle"`):

| 列 1 | 列 2 | 列 3 | 列 4 |
|---|---|---|---|
| 打上からの時刻 [s] | 方位角（北から時計回り）[deg] | 上下角 [deg] | ロール角 [deg] |

**レートモード** (`"Mode": "Rate"`):

| 列 1 | 列 2 | 列 3 | 列 4 |
|---|---|---|---|
| 打上からの時刻 [s] | ヨー角速度 [deg/s] | ピッチ角速度 [deg/s] | ロール角速度 [deg/s] |

ファイル範囲外は先頭/末尾の値を外挿（step-hold）。

### Xcg.csv（重心位置）

| 列 1 | 列 2 |
|---|---|
| 打上からの時刻 [s] | 重心位置（機体後端基準）[m] |

### Xcp.csv（圧力中心位置）

| 列 1 | 列 2 |
|---|---|
| マッハ数 [-] | 圧力中心位置（機体後端基準）[m] |

### MOI.csv（慣性モーメント）

| 列 1 | 列 2 | 列 3 | 列 4 |
|---|---|---|---|
| 打上からの時刻 [s] | ヨー軸慣性モーメント [kg·m²] | ピッチ軸慣性モーメント [kg·m²] | ロール軸慣性モーメント [kg·m²] |

### Ixy.csv / Ixz.csv / Iyz.csv（慣性乗積）

| 列 1 | 列 2 |
|---|---|
| 打上からの時刻 [s] | 慣性乗積 [kg·m²] |

`Enable Product of Inertia File: true` の場合に使用。Ixy / Ixz / Iyz をそれぞれ別ファイルで指定する（`Product of Inertia File` 内の `Ixy File Path` / `Ixz File Path` / `Iyz File Path` で個別にパス指定）。ファイル範囲外は先頭/末尾の値を外挿（step-hold）。サンプル: [`sample_Ixy.csv`](../../examples/sample_Ixy.csv) / [`sample_Ixz.csv`](../../examples/sample_Ixz.csv) / [`sample_Iyz.csv`](../../examples/sample_Iyz.csv)。

### CA.csv / CAbo.csv（軸力係数）

| 列 1 | 列 2 |
|---|---|
| マッハ数 [-] | 軸力係数 [-] |

燃焼中と燃焼後で別ファイルを指定可能。

### CNa / Cld / Clp / Cmq / Cnr .csv（各空力係数）

| 列 1 | 列 2 |
|---|---|
| マッハ数 [-] | 係数値 |
