# MAVLink モーター制御 - ユーザーガイド

STM32H753でMAVLink(UDP/Ethernet経由)を使ってモーターを制御するためのシンプルなガイドです。

---

## これは何?

STM32マイコン用の**YAML駆動型MAVLinkモーター制御システム**です。YAMLファイルでモーターを設定し、Cコードを自動生成して、Ethernet経由のMAVLinkコマンドですべてを制御できます。

**対応モーター:**
- サーボモーター(PWM)
- DCモーター(エンコーダ付き)
- DJI RoboMasterモーター(CAN)
- RS485モーター(池谷MD)

---

## クイックスタート

### 1. モーターを設定する(YAML)

設定ファイルを編集: **`Lib/mavlink_hal/config/examples/h753_full.yaml`**

```yaml
# ネットワーク設定
mavlink:
  system_id: 1
  component_id: 1
  transport:
    type: udp
    udp:
      local_ip: "192.168.11.4"    # ボードのIP
      local_port: 14550
      remote_ip: "192.168.11.2"   # PCのIP
      remote_port: 14550

# システム設定
system:
  loop_frequency_hz: 100          # 制御ループ速度
  heartbeat_rate_hz: 1

# モーターを定義
devices:
  # 例: サーボモーター
  - id: 1
    type: servo
    name: servo_gripper
    hardware:
      timer: "15"
      channel: 1
      pins:
        pwm: PE5
    config:
      servo:
        min_pulse_us: 500
        max_pulse_us: 2500
    limits:
      min_value: -90.0
      max_value: 90.0

  # 例: DCモーター
  - id: 10
    type: dc_motor
    name: dc_left_wheel
    hardware:
      timer: "3"
      channel: 1
      pins:
        pwm: PA6
        dir: PF3
    config:
      dc_motor:
        pid_speed:
          kp: 0.1
          ki: 0.05
          kd: 0.01

  # 例: RoboMasterモーター
  - id: 20
    type: robomaster_motor
    name: rm_gimbal_yaw
    hardware:
      can_bus: 1
      can_id: 0x201
    config:
      robomaster:
        motor_type: GM6020
        pid_speed:
          kp: 50.0
          ki: 0.1
          kd: 5.0

  # 例: RS485モーター
  - id: 30
    type: rs485_motor
    name: rs485_conveyor
    hardware:
      uart: 1
    config:
      rs485:
        device_id: 1
        motor_index: 0
        control_mode: velocity
        max_rps: 100.0
```

**モーターIDの範囲:**
- サーボ: 1-9
- DCモーター: 10-15
- RoboMaster: 20-29
- RS485: 30-49

### 2. Cコードを生成

```bash
cd Lib/mavlink_hal/config

# YAMLを検証
python generator/cli.py validate examples/h753_full.yaml

# コード生成
python generator/cli.py generate examples/h753_full.yaml \
    --platform stm32 \
    --output generated/h753_full \
    --force
```

これで以下が生成されます:
- `mavlink_generated_config.h` - 設定定数
- `mavlink_generated_devices.c` - デバイス初期化
- `mavlink_generated_handlers.c` - MAVLinkメッセージハンドラ
- `mavlink_generated_params.h` - PIDパラメータ

### 3. ハードウェアを設定(STM32CubeMX)

**`H753UDP.ioc`** をSTM32CubeMXで開いて設定:

- **タイマー**: モーターが使用するタイマーを有効化(例: サーボ用にTIM15チャンネル1)
- **CAN**: RoboMasterモーター用にFDCAN1/2を有効化
- **UART**: RS485モーター用にUSARTを有効化(ボーレート500 kbps)
- **GPIO**: DCモーター用の方向制御ピンを設定
- **Ethernet**: すでに設定済み(必要な場合のみ変更)

コード生成: **Project → Generate Code**

### 4. ビルドと書き込み

```bash
cd h753

# ファームウェアをビルド
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .

# ボードに書き込み
STM32_Programmer_CLI --connect port=swd --download H753UDP.elf --hardRst --rst --start
```

### 5. 接続テスト

Pythonとpymavlinkを使用:

```python
from pymavlink import mavutil

# ボードに接続
master = mavutil.mavlink_connection('udp:192.168.11.4:14550')

# ハートビートを待つ
print("ハートビートを待っています...")
master.wait_heartbeat()
print(f"接続成功! System {master.target_system}, Component {master.target_component}")
```

---

## モーターの制御

### 方法1: RCチャンネル(モーター1-8)

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# モーターID 1(サーボ)をRCチャンネル1で制御
# PWM範囲: 1000-2000 (中立 = 1500)
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500,  # チャンネル1 → モーターID 1
    1500,  # チャンネル2 → モーターID 2
    1500,  # チャンネル3 → モーターID 3
    1500,  # チャンネル4 → モーターID 4
    1500,  # チャンネル5 → モーターID 5
    1500,  # チャンネル6 → モーターID 6
    1500,  # チャンネル7 → モーターID 7
    1500,  # チャンネル8 → モーターID 8
)
```

### 方法2: モーターコマンド(モーター1-255)

```python
# 任意のモーター(ID 1-255)を特定のモードで制御

# 例1: サーボを制御(位置モード)
master.mav.motor_command_send(
    motor_id=1,
    control_mode=0,        # 0 = 位置
    target_value=45.0,     # 45度
    enable=1
)

# 例2: DCモーターを制御(速度モード)
master.mav.motor_command_send(
    motor_id=10,
    control_mode=1,        # 1 = 速度
    target_value=50.0,     # 50 RPS
    enable=1
)

# 例3: RS485モーターを制御(速度モード)
master.mav.motor_command_send(
    motor_id=30,
    control_mode=1,        # 1 = 速度
    target_value=80.0,     # 80 RPS
    enable=1
)

# モーターを停止
master.mav.motor_command_send(
    motor_id=10,
    control_mode=1,
    target_value=0.0,
    enable=0               # モーターを無効化
)
```

**制御モード:**
- `0` = 位置(度またはラジアン)
- `1` = 速度(RPSまたはrad/s)
- `2` = 電流(アンペア)
- `3` = デューティサイクル(-1.0から+1.0)

---

## リアルタイムPIDチューニング

再コンパイル不要でPIDゲインを調整:

### すべてのパラメータをリスト表示

```python
# すべてのパラメータをリクエスト
master.mav.param_request_list_send(
    master.target_system,
    master.target_component
)

# パラメータを受信
while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg:
        print(f"{msg.param_id} = {msg.param_value}")
```

### PIDパラメータを設定

```python
# 例: RoboMasterモーター20の速度Kpを設定
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'RM_20_SPD_KP',        # パラメータ名
    55.0,                   # 新しい値
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# 確認を待つ
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
print(f"設定完了: {msg.param_id} = {msg.param_value}")
```

**パラメータの命名規則:**
- **RoboMaster**: `RM_<ID>_SPD_KP`, `RM_<ID>_SPD_KI`, `RM_<ID>_SPD_KD`
- **DCモーター**: `DC_<ID>_SPD_KP`, `DC_<ID>_POS_KP`, など

`<ID>`はモーターID(RoboMasterは20-29、DCモーターは10-15)

---

## よくあるタスク

### 新しいモーターを追加

1. **YAMLを編集** (`h753_full.yaml`):
```yaml
devices:
  - id: 2                 # 新しいサーボ
    type: servo
    name: servo_arm
    hardware:
      timer: "16"
      channel: 1
      pins:
        pwm: PF6
    config:
      servo:
        min_pulse_us: 500
        max_pulse_us: 2500
    limits:
      min_value: -90.0
      max_value: 90.0
```

2. **コード再生成**:
```bash
cd Lib/mavlink_hal/config
python generator/cli.py generate examples/h753_full.yaml --force
```

3. **STM32CubeMXで設定**:
   - TIM16 CH1をピンPF6で有効化
   - コード生成

4. **リビルド**:
```bash
cd h753
cmake --build build/
```

### ネットワーク設定を変更

YAMLで編集:
```yaml
mavlink:
  transport:
    udp:
      local_ip: "192.168.1.100"   # 新しいボードIP
      remote_ip: "192.168.1.50"   # 新しいPC IP
```

その後、再生成してリビルド。

---

## トラブルシューティング

### ハートビートが受信できない

**確認項目:**
- ボードIP: `192.168.11.4`(または設定したIP)
- PC IP: `192.168.11.2`(または設定したIP)
- 同じサブネット(例: 両方とも`192.168.11.x`)
- イーサネットケーブル接続
- ファイアウォールがUDPポート14550をブロックしていない

**テスト:**
```bash
# ボードにping
ping 192.168.11.4

# MAVLinkポートが開いているか確認
nc -u 192.168.11.4 14550
```

### モーターが動かない

**確認項目:**
1. **モーターIDは正しいか?** (サーボ: 1-9、DC: 10-15、RoboMaster: 20-29、RS485: 30-49)
2. **CubeMXでハードウェア設定したか?** (タイマー、CAN、UART)
3. **電源は接続されているか?**
4. **モーターは有効化されているか?** (コマンドで`enable=1`)
5. **値は有効範囲内か?** (YAMLの`limits`を確認)

**デバッグ:**
```python
# モーターが存在するか確認
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'RM_20_SPD_KP',  # そのモーターのパラメータを読んでみる
    -1
)
# タイムアウトした場合 → モーターが設定されていない
```

### コード生成に失敗する

**確認項目:**
1. **YAMLの構文は正しいか?**
   ```bash
   python generator/cli.py validate examples/h753_full.yaml
   ```
2. **モーターIDが重複していないか?** 各モーターは一意のIDが必要
3. **ハードウェア設定は有効か?** (例: 存在しないタイマー)

### ビルドエラー

**よくある問題:**
- `freertos.c`に`hw_timer_register()`の呼び出しがない
- ハードウェア変更後にCubeMXコードを再生成していない
- ペリフェラル初期化が不足(UART、CAN、タイマー)

**修正方法:**
1. `H753UDP.ioc`を開く
2. 必要なペリフェラルを有効化
3. コード生成
4. `freertos.c`でハードウェアを登録:
```c
extern TIM_HandleTypeDef htim15;
hw_timer_register(15, &htim15);
```

---

## パフォーマンス仕様

- **制御ループ**: 100 Hz(10ms更新レート)
- **ハートビート**: 1 Hz
- **レイテンシ**: <20ms(コマンドからモーター応答まで)
- **最大モーター数**: 合計45個(サーボ9 + DC 6 + RoboMaster 10 + RS485 20)
- **ネットワーク**: UDPポート14550
- **対応MAVLink**: v2.0

---

## 例: 完全なロボット設定

```yaml
# 2輪ロボット + グリッパー + カメラジンバル
devices:
  # 車輪(DCモーター)
  - id: 10
    type: dc_motor
    name: wheel_left
    hardware:
      timer: "3"
      channel: 1
      pins: {pwm: PA6, dir: PF3}
    config:
      dc_motor:
        pid_speed: {kp: 0.1, ki: 0.05, kd: 0.01}
    limits: {max_velocity: 100.0}

  - id: 11
    type: dc_motor
    name: wheel_right
    hardware:
      timer: "3"
      channel: 2
      pins: {pwm: PA7, dir: PF4}
    config:
      dc_motor:
        pid_speed: {kp: 0.1, ki: 0.05, kd: 0.01}
    limits: {max_velocity: 100.0}

  # グリッパー(サーボ)
  - id: 1
    type: servo
    name: gripper
    hardware:
      timer: "15"
      channel: 1
      pins: {pwm: PE5}
    config:
      servo: {min_pulse_us: 500, max_pulse_us: 2500}
    limits: {min_value: 0.0, max_value: 180.0}

  # カメラジンバル(RoboMasterモーター)
  - id: 20
    type: robomaster_motor
    name: gimbal_pitch
    hardware: {can_bus: 1, can_id: 0x201}
    config:
      robomaster:
        motor_type: GM6020
        pid_speed: {kp: 50.0, ki: 0.1, kd: 5.0}

  - id: 21
    type: robomaster_motor
    name: gimbal_yaw
    hardware: {can_bus: 1, can_id: 0x202}
    config:
      robomaster:
        motor_type: GM6020
        pid_speed: {kp: 50.0, ki: 0.1, kd: 5.0}
```

**制御スクリプト:**
```python
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:192.168.11.4:14550')
master.wait_heartbeat()

# 前進
master.mav.motor_command_send(10, 1, 50.0, 1)  # 左車輪 50 RPS
master.mav.motor_command_send(11, 1, 50.0, 1)  # 右車輪 50 RPS
time.sleep(2)

# 停止
master.mav.motor_command_send(10, 1, 0.0, 0)
master.mav.motor_command_send(11, 1, 0.0, 0)

# グリッパーを開く
master.mav.motor_command_send(1, 0, 180.0, 1)  # 位置モード、180度
time.sleep(1)

# グリッパーを閉じる
master.mav.motor_command_send(1, 0, 0.0, 1)
```

---

## QGroundControlで使用

1. **接続**: QGroundControlを開く → Comm Links → Add
   - Type: UDP
   - Port: 14550
   - Server Address: `192.168.11.4`

2. **パラメータ表示**: Vehicle Setup → Parameters
   - `RM_`(RoboMaster)または`DC_`(DCモーター)で検索
   - リアルタイムでPID値を編集

3. **モニタリング**: Flight View → MAVLink Inspector
   - すべてのメッセージを表示
   - 通信をデバッグ

---

## さらにヘルプが必要な場合

- **完全なドキュメント**: リポジトリルートの`CLAUDE.md`を参照
- **RS485固有**: `RS485_QUICKSTART.md`を参照
- **ネットワーク設定**: `MAVLINK_UDP_README.md`を参照
- **カスタムメッセージ**: `MAVLINK_STANDARDIZATION.md`を参照

---

**楽しく開発してください！🚀**