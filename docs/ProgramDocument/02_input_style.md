# Input Style

```sh
$ ./ForRocket.exe [option] [solver_config.json]
```

## コマンドラインオプション

|  option  |  内容  |
| ---- | ---- |
|  -m  |  minimum output. 出力csvファイルのパラメータ数を最小限に減らす。  |
|  -q  |  進行中の案内メッセージを非表示。  |
|  --quite  |  同上  |
|  -h  |  display help. オプションを表示。  |
|  --help  |  同上  |
|  -v  |  display version. バージョンを表示。  |
|  --version  |  同上  |

## 入力ファイル

| file | 内容 |
| ---- | ---- |
| solver_config.json | ロケット全体に関わるパラメータとステージの指定 |
|  | 以降のファイルはステージ毎に用意される |
| stage_config_list.json | 下記の3つのjsonのパス |
| sequence_of_event.json | フライトシーケンスイベントの有無およびタイミング |
| rocket_config.json | ステージの構造と空力のパラメータ |
| engine_config.json | ロケットエンジンに関するパラメータ |
| | 以降のファイルは上記jsonでそれぞれの"File": true とした時に読まれるファイル|
| wind.csv | 風の高度分布 |
| thrust_mdotp.csv | 推力と推進剤質量流量の時間履歴 |
| attitude.csv | 機体姿勢の時間履歴 |
| Xcg.csv | 全機重心位置の時間履歴 |
| Xcp.csv | Mach数と圧力中心位置の関係 |
| MOI.csv | 全機慣性モーメントの時間履歴 |
| CA.csv | Mach数と軸力係数(燃焼中)の関係 |
| CAbo.csv | Mach数と軸力係数(燃焼終了後)の関係 |
| CNa.csv | Mach数と法線力傾斜の関係 |
| Cld.csv | Mach数とフィンカント角によるロールモーメント係数の関係 |
| Clp.csv | Mach数とロールダンピングモーメント係数の関係 |
| Cmq.csv | Mach数とピッチダンピングモーメント係数の関係 |
| Cnr.csv | Mach数とヨーダンピングモーメント係数の関係 |

なお、すべてのファイルにおいてファイル名に指定はなく、コマンドライン引数またはjsonファイル内のパスに内容が一致したファイルを配置すればよい。

## solver_config.json

```json
{
	"Model ID": "sample",  // モデル名。出力csvのプレフィックスになる

	"Launch DateTime": "2020/08/23 9:00:00.0",  // 打上日時
	"Launch Condition": {
		"Latitude [deg]": 40.242865,  // 打上射点の緯度
		"Longitude [deg]": 140.01045,  //経度
		"Height for WGS84 [deg]": 20.0,  // WGS84での海面高度
		
		"Azimuth [deg]": 270.0,  // 打上方位角（北から時計まわり）
		"Elevation [deg]": 85.0,  // 打上上下角
		
		"North Velocity [m/s]": 0.0,  // 北方向初期速度
		"East Velocity [m/s]": 0.0,  // 東方向初期速度
		"Down Velocity [m/s]": 0.0  // 地球中心方向初期速度
	},

	"Wind Condition": {
		"Enable Wind": true,  // 風ファイルの有無
		"Wind File Path": "sample_wind.csv"  // 風ファイルパス
	},

	"Number of Stage": 1,  // ステージ数
	"Stage1 Config File List": "sample_stage_config_list.json",  // 1段目のstage_config_list.json
	"Stage2 Config File List": "stage_config_list.json",  // 2段目
	"Stage3 Config File List": "stage_config_list.json"  // 3段目
}
```

## stage_config_list.json

```json
{
	"Rocket Configuration File Path": "sample_rocket_config.json",
	"Engine Configuration File Path": "sample_engine_config.json",
	"Sequence of Event File Path": "sample_sequence_of_event.json"
}
```

## sequence_of_event.json

```json
// 時刻は打上げ時をX+0secとした時のX+nの"打上"相対時刻
{
  "Flight Start Time [s]": 0.0,  // ステージの飛行開始時刻
  // 2段目以降は下段の分離時刻に自動修正される

  "Engine Ignittion Time [s]": 0.0,  // エンジンスタート時刻
  // Flight Start Timeよりあとの場合は慣性飛行が続く
	
	"Enable Rail-Launcher Launch": true,  // レールランチャからの打上げON/OFF
	"Rail Launcher": {
		"Length [m]": 5.0  // ランチャの有効レール長さ
	},

	"Enable Engine Cutoff": false,  // エンジンカットオフのON/OFF
	"Cutoff": {
    "Cutoff Time [s]": 0.0  //　カットオフ時刻
    // この時刻で推力ゼロおよび質量減少が停止
	},

	"Enable Program Attitude": false,  // 姿勢制御飛行ON/OFF
	"Attitude Control": {
		"Start Time [s]": 0.0,  // 姿勢制御の開始時刻
		"End Time [s]": 100.0  // 終了時刻
	},

	"Enable Stage Separation": false,  // 段間分離ON/OFF
	"Upper Stage": {
		"Stage Separation Time [s]": 0.0,  // 分離時刻
    "Upper Stage Mass [kg]": 100.0  // 上段質量
    // この質量を下段から減じて下段の落下までを計算する
	},

	"Enable Despin Control": false,  // デスピン制御ON/OFF
	"Despin": {
    "Time [s]": 20.0  // デスピン時刻
    // この時刻で瞬間的にスピンゼロおよびスピンレートゼロ
    // ヨーヨーデスピナを想定
	},

	"Enable Fairing Jettson": false,  // フェアリング投棄ON/OFF
	"Fairing": {
		"Jettson Time [s]": 0.0,  // 投棄時刻
    "Mass [s]": 1.0  // フェアリング質量
    // この質量を減じて本体の計算を続行する
	},

	"Enable Parachute Open": false,  // パラシュート開傘ON/OFF
	"Parachute": {
		"Open Time [s]": 30.0,  // 開傘時刻
		"Enable Forced Apogee Open": false  // 飛行頂点開傘ON/OFF
	},

	"Enable Secondary Parachute Open": false,  // 第2パラシュート開傘ON/OFF
	"Secondary Parachute": {
    "Open Time [s]": 60.0  // 開傘時刻
    // ドローグシュートに対するメインシュートを想定
	},

  "Flight End Time [s]": 100.0,  // 飛行終了時刻
  // この時刻まで飛行計算が行われる
	"Time Step [s]": 0.1,  // 出力csvの時間刻み
  "Enable Auto Terminate SubOrbital Flight": true  // 飛行終了時刻の自動決定ON/OFF
  // 全力積から着地までのおおよその時間を決定する
  // ONの場合はFlight End Time [s]は上書きされる
}
```

## rocket_config.json

```json
{
    "Diameter [mm]": 180.0,  // 機体代表直径
    "Length [mm]": 3900.0,  // 機体代表長さ
    "Mass": {
        "Inert [kg]": 52.5,  // イナート質量
        "Propellant [kg]": 41.0  // 推進剤質量
    },

	"Enable Program Attitude": false,  // 姿勢制御ON/OFF
	// sequence_of_event.jsonでもONである必要がある
    "Program Attitude File": {
        "Program Attitude File Path": "attitude.csv"  // 姿勢履歴ファイルパス
    },

    "Enable X-C.G. File": false,  // 全機重心位置履歴ファイル有無
    "X-C.G. File": {
        "X-C.G. File Path": "Xcg.csv"  // 重心位置履歴ファイルパス
    },
    "Constant X-C.G.": {
		"Constant X-C.G. from BodyTail [mm]": 1100.0  // 飛行中一定の全機重心位置(機体後端基準)
		// ファイル無しの時のみこちらが使われる
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

    "Enable X-C.P. File": false,
    "X-C.P. File": {
        "X-C.P. File Path": "Xcp.csv"
    },
    "Constant X-C.P.": {
		"Constant X-C.P. from BodyTail [mm]": 835.0  // 飛行中一定の圧力中心位置(機体後端基準)
    },

	"X-ThrustLoadingPoint from BodyTail [mm]": 300.0,  // 推力印加位置(機体後端基準)
	// 推力ミスアライメントの計算に使用する

    "Comment CA": "Axial Force Coefficient",
    "Enable CA File": true,
    "CA File": {
        "CA File Path": "sample_CA.csv",
		"BurnOut CA File Path": "sample_CA.csv"
		// 燃焼終了前後で軸力係数を変える場合は別ファイルを指定する
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
	"Fin Cant Angle [deg]": 0.0,  // フィン1枚あたりのフィンカント角
	// ロール回転が正となる角度が正のカント角
    "Enable Cld File": false,
    "Cld File": {
        "CldFile Path": "Cld.csv"
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

## engine_config.json

```json
{
    "Nozzle Exit Diameter [mm]": 100.0,  // ノズル出口直径

    "Enable Thrust File": true,  // 推力履歴ファイル有無
    "Thrust File": {
		"Thrust at vacuum File Path": "sample_thrust.csv"  // 推力履歴ファイルパス
		// 推力は真空中推力を[N]の単位
    },
    "Constant Thrust": {
		// ファイル無しの時のみ使用する矩形推力パラメータ
        "Thrust at vacuum [N]": 5780.0,  // 真空中推力
        "Propellant Mass Flow Rate [kg/s]": 3.0,  // 推進剤質量流量
        "Burn Duration [sec]": 13.7  // 燃焼時間
    },

    "Enable Engine Miss Alignment": false,  // 推力ミスアライメントON/OFF
    "Engine Miss-Alignment": {
		// 機体座標での角度と同じ定義で推力軸を傾ける
        "y-Axis Angle [deg]": 0.0,  // ピッチ方向に推力軸を傾ける(ピッチモーメント)
        "z-Axis Angle [deg]": 0.0  // ヨー方向に推力軸を傾ける(ヨーモーメント)
    }
}
```

## wind.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は無風として扱われる。
データ点間は線形で補間される。

|  海面高度 [m]  |  東西方向風速(東への風が正) [m/s]  |  南北方向風速(北への風が正) [m/s]  |
| ---- | ---- | ---- |
|  0.0  |  2.0  | -3.0 |
|  300.0  | 3.2 | -4.1 |
| ... | ... | ... |
| 12000 | 30.0 | -50.0 |

## thrust_mdotp.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は推力および質量流量ゼロとして扱われる。
データ点間は線形で補間される。

|  点火からの時間 [s]  |  真空中推力 [N]  |  推進剤質量流量 [kg/s]  |
| ---- | ---- | ---- |
|  0.0  |  0.0  | 0.0 |
|  0.1  | 3000.0 | 1.5 |
|  0.2  | 6000.0 | 3.0 |
| ... | ... | ... |
| 30.0 | 0.0 | 0.0 |

## attitude.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  打ち上げからの時刻 X+ [s]  |  方位角(北から時計回り) [deg]  | 上下角 [deg] | ロール角 [deg] |
| ---- | ---- | ---- | ---- |
|  0.0  | 172.0 | 88.0 | 0.0 |
|  1.0  | 175.5 | 87.0 | 0.0 |
| ... | ... | ... |
| 30.0 | 186.2 | 62.9 | 0.0 |

## Xcg.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  打ち上げからの時刻 X+ [s]  |  全機重心位置(機体後端基準) [m]  |
| ---- | ---- |
|  0.0  | 1.53 |
|  1.0  | 1.62 |
| ... | ... | ... |
| 30.0 | 2.19 |


## Xcp.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  圧力中心位置(機体後端基準) [m]  |
| ---- | ---- |
|  0.0  | 1.3 |
|  0.4  | 1.15 |
| ... | ... | ... |
| 3.0 | 2.02 |


## MOI.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  打ち上げからの時刻 X+ [s]  |  ヨー回転慣性モーメント [kg-m2]  | ピッチ回転慣性モーメント [kg-m2] | ロール回転慣性モーメント [kg-m2] |
| ---- | ---- | ---- | ---- |
|  0.0  | 300.0 | 300.0 | 15.0 |
|  1.0  | 299.3 | 299.3 | 14.3 |
| ... | ... | ... |
| 30.0 | 186.2 | 186.2 | 9.8 |

## CA.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  軸力係数(燃焼中) [-]  |
| ---- | ---- |
|  0.0  | 0.4 |
|  0.4  | 0.41 |
| ... | ... | ... |
| 3.0 | 0.5 |

## CAbo.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  軸力係数(燃焼終了後) [-]  |
| ---- | ---- |
|  0.0  | 0.42 |
|  0.4  | 0.45 |
| ... | ... | ... |
| 3.0 | 0.59 |

## CNa.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  法線力傾斜 [1/rad]  |
| ---- | ---- |
|  0.0  | 8.2 |
|  0.4  | 8.2 |
| ... | ... | ... |
| 3.0 | 6.2 |

## Cld.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  ロールモーメント係数 [1/rad]  |
| ---- | ---- |
|  0.0  | 0.5 |
|  0.4  | 0.51 |
| ... | ... | ... |
| 3.0 | 0.43 |

## Clp.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  ロール減衰モーメント係数 [-]  |
| ---- | ---- |
|  0.0  | 0.03 |
|  0.4  | 0.033 |
| ... | ... | ... |
| 3.0 | 0.027 |

## Cmq.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  ピッチ減衰モーメント係数 [-]  |
| ---- | ---- |
|  0.0  | 0.6 |
|  0.4  | 0.61 |
| ... | ... | ... |
| 3.0 | 0.58 |

## Cnr.csv
1行目はヘッダ行として読み込まれない。
ファイル範囲外は先頭および後尾の値が適用される。
データ点間は線形で補間される。

|  Mach数 [-]  |  ヨー減衰モーメント係数 [-]  |
| ---- | ---- |
|  0.0  | 0.6 |
|  0.4  | 0.61 |
| ... | ... | ... |
| 3.0 | 0.58 |

