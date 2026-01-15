# setup_ELRS_RLS.py パラメータ設定

## 概要
ArduCopter用のMAVLinkパラメータ設定スクリプト（ELRS受信機 + RLS Observer）

---

## 📡 接続設定
- **USB接続**: `/dev/ttyACM0` (115200 baud)
- **UART接続**: `/dev/ttyAMA0` (1000000 baud, フロー制御有効)

---

## ⚙️ パラメータ一覧

### 1. EKF3基本設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `AHRS_EKF_TYPE` | 3.0 | EKF3を使用 |
| `EK3_ENABLE` | 1.0 | EKF3有効化 |
| `EK3_IMU_MASK` | 3 | IMUマスク |

### 2. ハイブリッド設定（GPS + コンパス）
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `EK3_SRC1_POSXY` | 3 | GPS（水平位置） |
| `EK3_SRC1_VELXY` | 3 | GPS（水平速度） |
| `EK3_SRC1_POSZ` | 3 | GPS（垂直位置） |
| `EK3_SRC1_VELZ` | 3 | GPS（垂直速度） |
| `EK3_SRC1_YAW` | 3 | GPS with compass fallback |

### 3. EKF3精度設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `EK3_GPS_CHECK` | 1 | GPS健全性チェック |
| `EK3_POS_I_GATE` | 8.0 | 位置ゲート |
| `EK3_VEL_I_GATE` | 8.0 | 速度ゲート |
| `EK3_HGT_I_GATE` | 10.0 | 高度ゲート |

### 4. ノイズパラメータ
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `EK3_POSNE_M_NSE` | 0.2 | 水平位置ノイズ |
| `EK3_VELNE_M_NSE` | 0.3 | 水平速度ノイズ |
| `EK3_VELD_M_NSE` | 0.5 | 垂直速度ノイズ |
| `EK3_YAW_M_NSE` | 0.2 | ヨー角ノイズ |
| `EK3_ALT_M_NSE` | 10.0 | 気圧センサーノイズ |
| `EK3_GYRO_P_NSE` | 0.02 | ジャイロプロセスノイズ |

### 5. コンパス設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `COMPASS_ENABLE` | 1 | コンパス有効化 |
| `COMPASS_USE` | 1.0 | 内蔵コンパス使用 |
| `COMPASS_USE2` | 0.0 | 外付コンパス2無効 |
| `COMPASS_USE3` | 0.0 | 外付コンパス3無効 |
| `COMPASS_AUTODEC` | 1 | 自動磁気偏角有効 |
| `COMPASS_LEARN` | 1 | コンパス学習有効 |

### 6. EKF追加設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `EK3_MAG_CAL` | 3 | 地上でheading fusion、空中で3-axis fusion |
| `EK3_SRC_OPTIONS` | 1 | Fuse all velocity sources |
| `EK3_GLITCH_RAD` | 5 | GPS Glitch検出半径緩和 |
| `EK3_CHECK_SCALE` | 100 | EKFチェックスケール |
| `EK3_PRIMARY` | -1 | 自動切り替え無効 |

### 7. GPS設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `GPS1_TYPE` | 14 | MAVLink GPS Input |
| `GPS_AUTO_CONFIG` | 0 | 自動設定無効 |
| `GPS_PRIMARY` | 0 | プライマリGPS |

### 8. Guidedモード設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `WPNAV_SPEED_UP` | 40 | 上昇速度 [cm/s] |
| `WPNAV_SPEED_DN` | 30 | 下降速度 [cm/s] |
| `WPNAV_ACCEL_Z` | 70 | 垂直加速度 [cm/s²] |
| `WPNAV_SPEED` | 500 | 水平速度 [cm/s] |
| `WPNAV_ACCEL` | 500 | 水平加速度 [cm/s²] |
| `WPNAV_RADIUS` | 5 | 到達半径 [cm] |
| `GUID_TIMEOUT` | 3 | タイムアウト [秒] |
| `GUID_OPTIONS` | 0 | オプション |

### 9. Loiterモード設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `LOIT_SPEED` | 50 | 最大速度 [cm/s] |
| `LOIT_ACC_MAX` | 50 | 最大加速度 [cm/s²] |
| `LOIT_BRK_ACCEL` | 50 | ブレーキ加速度 [cm/s²] |
| `LOIT_BRK_DELAY` | 0.3 | ブレーキ遅延 [秒] |
| `LOIT_BRK_JERK` | 300 | ブレーキジャーク [cm/s³] |
| `LOIT_ANG_MAX` | 10 | 最大傾斜角 [度] |

### 10. パイロット制御速度
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `PILOT_SPEED_UP` | 250 | パイロット上昇速度 [cm/s] |
| `PILOT_SPEED_DN` | 150 | パイロット下降速度 [cm/s] |
| `PILOT_ACCEL_Z` | 250 | パイロット加速度 [cm/s²] |

### 11. 垂直制御PID（カスケード制御構造）
ArduCopterの垂直制御は3層のカスケード構造を持ちます：
1. **位置制御層** → 2. **速度制御層** → 3. **加速度制御層**

#### 位置制御層（Position Controller）
| パラメータ | 値 | 範囲 | 説明 |
|-----------|-----|------|------|
| `PSC_POSZ_P` | 1.0 | 0.50-4.00 | 高度位置制御Pゲイン。目標高度と実高度の差を上昇/下降速度に変換 |

- **動作**: 高度誤差を検出し、目標垂直速度を生成して速度制御層に渡す
- **調整**: 値を大きくすると高度への追従が速くなるが、オーバーシュートしやすくなる

#### 速度制御層（Velocity Controller）
| パラメータ | 値 | 範囲 | 説明 |
|-----------|-----|------|------|
| `PSC_VELZ_P` | 4.0 | 1.0-10.0 | 垂直速度Pゲイン。速度誤差を加速度指令に変換 |
| `PSC_VELZ_I` | 8.0 | 0.00-10.0 | 垂直速度Iゲイン。長期的な速度誤差を補正 |
| `PSC_VELZ_D` | 0.01 | 0.00-2.00 | 垂直速度Dゲイン。速度の短期的変化に応答 |

- **Pゲイン**: 速度誤差に比例した加速度を生成。値が大きいほど応答が速い
- **Iゲイン**: ホバリング時のドリフトや定常偏差を補正。大きすぎると振動の原因に
- **Dゲイン**: 速度変化を先読みして制御を滑らかにする。通常は小さい値

#### 加速度制御層（Acceleration Controller）
| パラメータ | 値 | 範囲 | 説明 |
|-----------|-----|------|------|
| `PSC_ACCZ_P` | 0.3 | 0.010-0.250 | 垂直加速度Pゲイン。加速度誤差をスロットル出力に変換 |
| `PSC_ACCZ_I` | 1.0 | 0.000-0.500 | 垂直加速度Iゲイン。長期的な加速度誤差を補正 |

- **Pゲイン**: 目標加速度と実加速度の差をモーター出力に変換。機体の推力/重量比に依存
- **Iゲイン**: バッテリー電圧低下や重量変化による推力変動を補正

#### 制御フロー
```
目標高度 → [位置P] → 目標速度 → [速度PID] → 目標加速度 → [加速度PI] → スロットル出力
```

#### チューニングのポイント
1. **加速度制御から調整**: `PSC_ACCZ_P`を機体に合わせる（軽量機は小さく、重量機は大きく）
2. **速度制御の調整**: `PSC_VELZ_P`で応答性を調整、`PSC_VELZ_I`でドリフトを補正
3. **位置制御の調整**: 最後に`PSC_POSZ_P`で高度への追従速度を調整
4. **Iゲインの注意**: 積分飽和（Integrator Windup）を避けるため、過度に大きくしない

### 12. 水平制御PID
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `PSC_POSXY_P` | 5.0 | 水平位置制御P |
| `PSC_VELXY_P` | 3 | 水平速度制御P |
| `PSC_VELXY_I` | 2.5 | 水平速度制御I |
| `PSC_VELXY_D` | 0.5 | 水平速度制御D |

### 13. 姿勢制御PID
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `ATC_RAT_RLL_P` | 0.04 | Roll P |
| `ATC_RAT_RLL_I` | 0.05 | Roll I |
| `ATC_RAT_RLL_D` | 0.0012 | Roll D |
| `ATC_RAT_PIT_P` | 0.05 | Pitch P |
| `ATC_RAT_PIT_I` | 0.05 | Pitch I |
| `ATC_RAT_PIT_D` | 0.0012 | Pitch D |
| `ATC_RAT_YAW_P` | 0.2 | Yaw P |
| `ATC_RAT_YAW_I` | 0.02 | Yaw I |

### 14. 🔴 Observer設定（吊荷制御）
| パラメータ | 値 | 範囲 | 説明 |
|-----------|-----|------|------|
| `OBS_CORR_GAIN` | **0.004** | 0.0-1.0 | Observer補正ゲイン |
| `OBS_FILT_CUTOFF` | 20.0 | 1.0-100.0 | Observerフィルタカットオフ周波数 [Hz] |
| `OBS_RLS_LAMBDA` | 0.988 | 0.9-0.9999 | RLS忘却係数 |
| `OBS_RLS_COV_INIT` | 100.0 | 0.001-1000.0 | RLS初期共分散 |
| `OBS_DIST_FREQ` | 0.6 | 0.1-10.0 | 外乱周波数 [Hz] |
| `OBS_PRED_TIME` | 0.01 | 0.0-0.5 | 予測時間 [秒] |

### 15. IMUフィルタ
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `INS_GYRO_FILTER` | 20 | ジャイロフィルタ（応答性向上） |
| `INS_ACCEL_FILTER` | 20 | 加速度フィルタ（応答性向上） |

### 16. フェイルセーフ設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `FS_EKF_ACTION` | 2 | EKF失敗時Stabilizeモード |
| `FS_EKF_THRESH` | 0.8 | EKF信頼度閾値 |
| `FS_THR_ENABLE` | 3 | 送信機喪失時着陸 |
| `FS_THR_VALUE` | 975 | 失効検出値 |
| `FS_OPTIONS` | 0 | フェイルセーフオプション |
| `FS_CRASH_CHECK` | 0 | クラッシュ検出無効化 |
| `FS_VIBE_ENABLE` | 0 | 振動検出無効化 |
| `FS_DR_ENABLE` | 0 | Dead Reckoning無効化 |
| `RTL_ALT` | 50 | RTL高度 [cm] |

### 17. シリアル設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `SERIAL1_PROTOCOL` | 2 | MAVLink v2 |
| `SERIAL1_BAUD` | 1000000 | ボーレート |
| `BRD_SER1_RTSCTS` | 2 | ハードウェアフロー制御有効 |
| `SERIAL2_PROTOCOL` | 23 | ELRSレシーバー |

### 18. RC設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `RC10_OPTION` | 56 | チャンネル10オプション |
| `RC11_OPTION` | 55 | チャンネル11オプション |
| `THR_DZ` | 200 | スロットルデッドゾーン |
| `RC_OPTIONS` | 10336 | RCオプション |
| `RSSI_TYPE` | 3 | ELRSレシーバー |
| `RC9_OPTION` | 153 | チャンネル9オプション |

### 19. ログ制御設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `LOG_DISARMED` | 0 | 非アーム時はログ記録しない |
| `LOG_FILE_DSRMROT` | 1 | ディスアーム時ファイルローテーション |
| `LOG_FILE_TIMEOUT` | 5 | ログファイルタイムアウト [秒] |
| `LOG_BACKEND_TYPE` | 1 | ログバックエンド（ファイル） |

### 20. 推力設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `MOT_THST_HOVER` | 0.223 | ホバリングスロットル比 |
| `MOT_THST_EXPO` | 0 | 推力曲線指数 |
| `MOT_HOVER_LEARN` | 0 | ホバリング学習（無効） |

### 21. モーター・サーボ設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `SERVO1_FUNCTION` | 0 | サーボ1無効 |
| `SERVO2_FUNCTION` | 0 | サーボ2無効 |
| `SERVO3_FUNCTION` | 0 | サーボ3無効 |
| `SERVO4_FUNCTION` | 0 | サーボ4無効 |
| `SERVO9_FUNCTION` | 33 | モーター1 |
| `SERVO10_FUNCTION` | 34 | モーター2 |
| `SERVO11_FUNCTION` | 35 | モーター3 |
| `SERVO12_FUNCTION` | 36 | モーター4 |
| `MOT_PWM_TYPE` | 4 | DShot150 |
| `SERVO_DSHOT_ESC` | 2 | BLHeli32 ESC |
| `SERVO_BLH_MASK` | 3840 | BLHeliマスク |
| `SERVO_BLH_AUTO` | 1 | 自動BLHeli設定 |

### 22. 安全設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `BRD_SAFETY_DEFLT` | 0 | セーフティスイッチ無効 |
| `BRD_SAFETYOPTION` | 0 | セーフティオプション |
| `ARMING_CHECK` | 80 | アーミングチェック |
| `ARMING_RUDDER` | 0 | ラダーアーミング無効 |
| `DISARM_DELAY` | 0 | ディスアーム遅延 |
| `MOT_SPIN_ARM` | 0.02 | アーム時モーター回転 |
| `MOT_SPIN_MIN` | 0.02 | 最小モーター回転 |

### 23. バッテリー設定
| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `BATT_MONITOR` | 3 | バッテリーモニター |
| `BATT_ARM_VOLT` | 16.0 | アーミング最低電圧 [V] |
| `BATT_CRT_VOLT` | 14.0 | クリティカル電圧 [V] |
| `BATT_LOW_VOLT` | 15.5 | ロー電圧 [V] |
| `BATT_CAPACITY` | 0 | バッテリー容量 [mAh] |
| `BATT_ARM_MAH` | 0 | アーミング最小容量 [mAh] |
| `BATT_CRT_MAH` | 0 | クリティカル容量 [mAh] |
| `BATT_LOW_MAH` | 0 | ロー容量 [mAh] |
| `MOT_BAT_VOLT_MAX` | 21.0 | モーター最大電圧 [V] |
| `MOT_BAT_VOLT_MIN` | 13.5 | モーター最小電圧 [V] |

---

## 📝 注意事項
- パラメータは一度RAMに設定された後、`MAV_CMD_PREFLIGHT_STORAGE`コマンドでEEPROMに永続保存されます
- 設定後、全パラメータの再読み込みによる確認が実行されます
- **OBS_CORR_GAIN**（吊荷補正ゲイン）は重要パラメータとして赤色表示されます

---

## 🔗 関連ファイル
- スクリプト本体: [setup_ELRS_RLS.py](setup_ELRS_RLS.py)
