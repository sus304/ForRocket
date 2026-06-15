# ForRocket Integration Test Suite

`tests/simulation_test.py` は ForRocket バイナリを実際に実行し、出力 CSV を検証する統合テストスクリプトです。

---

## 前提

```bash
# ビルドが済んでいること
cmake --build build
```

Python 3.8 以上。外部パッケージは不要（標準ライブラリのみ）。

---

## 実行方法

```bash
# リポジトリルートから実行
python3 tests/simulation_test.py              # 全テストグループを実行
python3 tests/simulation_test.py attitude     # 名前に "attitude" を含むグループのみ
python3 tests/simulation_test.py all_axes     # 名前に "all_axes" を含むグループのみ
```

終了コード: `0` = 全パス、`1` = 1件以上失敗

---

## 出力例

```
Running 2 test group(s)...

======================================================================
  TEST GROUP: attitude_free_axes
======================================================================

[Baseline (no program attitude)]
  Baseline: 1727 rows, max_t=135.9s

[T1: Roll angle=0° control, pitch/yaw free (angle mode)]
  T1: 1734 rows, max_t=136.0s
  [OK] Roll at t=10s ≈ 0°: 0.0000°  (expected 0.0000±0.5000)
  [OK] Elevation changes (not frozen): Δ=19.51°
  ...

======================================================================
TOTAL: 29 passed, 0 failed
======================================================================
```

---

## テストケースの追加方法

### 1. テスト関数を定義する

```python
def test_myfeature(suite: TestSuite, runner: ScenarioRunner):
    """機能の説明をここに書く。"""

    # ── ベースライン（比較用） ────────────────────────────────────────
    suite.section("Baseline")
    baseline = runner.run("Baseline", make_rocket())
    base_elv_30 = baseline.at(Col.ELEVATION, 30)

    # ── 機能を有効にしたシナリオ ────────────────────────────────────
    suite.section("My feature ON")
    log = runner.run(
        "Feature scenario",
        make_rocket(...),          # ロケット設定
        attitude_csv="time,...\n", # 姿勢 CSV テキスト（必要なら）
        attitude_csv_name="att.csv",
    )

    # ── アサーション ────────────────────────────────────────────────
    suite.check("Elevation at t=30s", log.at(Col.ELEVATION, 30), 55.0, 0.5, "°")
    suite.check_near("Elevation matches baseline", log.at(Col.ELEVATION, 30), base_elv_30, 0.5, "°")
    suite.check_angle("Azimuth matches baseline", log.at(Col.AZIMUTH, 30), baseline.at(Col.AZIMUTH, 30), 1.0)
    suite.info("Roll at t=30s", log.at(Col.ROLL, 30), "°")  # 参考表示のみ
```

### 2. `ALL_TESTS` に登録する

ファイル末尾の辞書に追加するだけ：

```python
ALL_TESTS = {
    "attitude_free_axes":       test_attitude_free_axes,
    "attitude_angle_all_axes":  test_attitude_angle_all_axes,
    "myfeature":                test_myfeature,   # ← 追加
}
```

---

## API リファレンス

### `runner.run(label, rocket, ...)` → `FlightLog`

一時ディレクトリを作成し ForRocket を実行、結果の `FlightLog` を返す。

| 引数 | 型 | 説明 |
|---|---|---|
| `label` | `str` | ログ表示用のラベル |
| `rocket` | `dict` | ロケット設定（`make_rocket()` をベースに） |
| `seq` | `dict` | シーケンス設定（省略時は `make_seq()` のデフォルト） |
| `solver` | `dict` | ソルバー設定（省略時は `make_solver()` のデフォルト） |
| `attitude_csv` | `str` | 姿勢 CSV のテキスト内容（省略可） |
| `attitude_csv_name` | `str` | 姿勢 CSV のファイル名（デフォルト: `"attitude.csv"`） |
| `extra_files` | `dict` | `{ファイル名: テキスト}` の辞書（追加ファイルが必要な場合） |

### `FlightLog`

| メソッド | 説明 |
|---|---|
| `.at(col, t)` | 時刻 `t` に最も近い行の `col` 列の値（`float`） |
| `.col(col)` | `col` 列の全値リスト（`list[float]`） |
| `.time()` | 時刻列の全値リスト |
| `.max_time()` | 最終時刻 |
| `len(log)` | 行数 |

### `TestSuite` アサーションメソッド

| メソッド | 条件 | 使い所 |
|---|---|---|
| `suite.check(label, val, expected, tol, unit="")` | `\|val - expected\| ≤ tol` | 絶対値で期待値が決まる場合 |
| `suite.check_near(label, val, ref, tol, unit="")` | `\|val - ref\| ≤ tol` | ベースラインとの比較 |
| `suite.check_angle(label, val, ref, tol)` | 角度差 `≤ tol`（±360°ラップ対応） | 方位角・姿勢角の比較 |
| `suite.check_cond(label, cond, detail="")` | `cond == True` | 任意の条件 |
| `suite.info(label, val, unit="")` | — | pass/fail なし、参考情報の表示のみ |

### `Col` 列名定数

| 定数 | CSV カラム名 | 意味 |
|---|---|---|
| `Col.TIME` | `Time [s]` | 時刻 |
| `Col.ELEVATION` | `Elvation [deg]` | ピッチ角（※ typo: Elvation） |
| `Col.AZIMUTH` | `Azimuth [deg]` | ヨー角 |
| `Col.ROLL` | `Roll [deg]` | ロール角 |
| `Col.VEL_ROLL` | `AngleVelx [deg/s]` | 機体ロール角速度 p |
| `Col.VEL_PITCH` | `AngleVely [deg/s]` | 機体ピッチ角速度 q |
| `Col.VEL_YAW` | `AngleVelz [deg/s]` | 機体ヨー角速度 r |
| `Col.ALTITUDE` | `Altitude [m]` | 高度 |
| `Col.DOWNRANGE` | `Downrange [m]` | 射程 |
| `Col.MACH` | `MachNumber [-]` | マッハ数 |
| `Col.DYNAMIC_PRESS` | `DynamicPressure [kPa]` | 動圧 |
| `Col.AOA` | `AoA [deg]` | 迎角 |
| `Col.THRUST` | `Thrust [N]` | 推力 |
| `Col.MASS` | `Mass [kg]` | 全質量 |
| `Col.BURNING` | `Burning [0/1]` | 燃焼中フラグ |

### 設定ファクトリ関数

```python
make_solver(
    azimuth=270.0,      # 発射方位角 [deg]
    elevation=85.0,     # 発射仰角 [deg]
    wind=False,         # 風の有効/無効
)

make_seq(
    end_time=60.0,      # 飛行終了時刻 [s]
    timestep=0.1,       # 積分ステップ [s]
    rail_len=5.0,       # ランチャ長 [m]
)

make_rocket(
    program_attitude=None   # None: 姿勢制御なし
    # または:
    program_attitude={
        "mode":          "Angle" | "Rate",
        "enable_yaw":    True | False,
        "enable_pitch":  True | False,
        "enable_roll":   True | False,
        "file_path":     "attitude.csv",  # attitude_csv_name と合わせる
    }
)
```

---

## 姿勢制御 CSV のフォーマット

`attitude_csv` に渡す文字列の書式。**1行目はヘッダー必須**（`LoadCsvLog` がデフォルトで1行スキップ）。

**角度モード (`"Mode": "Angle"`)**
```
time,yaw,pitch,roll
0.0,270,85,0
60.0,270,55,0
```
単位: 時刻 [s]、角度 [deg]

**レートモード (`"Mode": "Rate"`)**
```
time,yaw_rate,pitch_rate,roll_rate
0.0,0,0,10
60.0,0,0,10
```
単位: 時刻 [s]、角速度 [deg/s]

---

## 現在のテストグループ一覧

| グループ名 | 内容 |
|---|---|
| `attitude_free_axes` | 軸別姿勢制御: 非制御軸が空力で自由に動くことを確認 |
| `attitude_angle_all_axes` | 全軸角度制御: CSV 値に正確に追従することを確認 |
