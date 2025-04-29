<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" class="logo" width="120"/>

# Pixhawk 6cとRaspberry pi 5をusb接続し、Pixhawk 6cから送られてくるMavlinkのデータをRaspberrypiで受信したい。

Pixhawk 6CとRaspberry Pi 5をUSBケーブルで接続し、Pixhawkから送られてくるMAVLinkデータをRaspberry Piで受信するには、以下の手順で実現できます。

## 1. ハードウェア接続

- **Pixhawk 6CのUSB Type-Cポート**と**Raspberry Pi 5のUSB-Aポート**を、データ通信対応のUSBケーブルで接続します。
- Pixhawk 6CはUSB接続時、自動的にシリアルデバイス（例：`/dev/ttyACM0`）としてRaspberry Pi側に認識されます[^4]。


## 2. Raspberry Pi側の準備

- **シリアルデバイスの権限設定**
MAVLink通信に必要な権限を付与します。

```bash
sudo chmod a+rw /dev/ttyACM0
```

※ デバイス名は環境によって`/dev/ttyACM1`などになる場合もあるので、`ls /dev/ttyACM*`で確認してください。
- **MAVProxyまたはpymavlinkのインストール**
MAVLinkメッセージを受信するには、MAVProxyやpymavlinkなどのツールを利用します。

```bash
sudo apt update
sudo apt install python3-pip
pip3 install MAVProxy pymavlink
```


## 3. Pixhawk側の設定

- **Pixhawkのファームウェア設定**
USB経由のMAVLink通信は通常自動で有効ですが、QGroundControlやMission Plannerで「SERIAL0_PROTOCOL」を「2（MAVLink 2）」に、「SERIAL0_BAUD」を「115200」または「921600」に設定しておくと確実です[^4][^5]。


## 4. Raspberry PiでMAVLinkデータを受信

### MAVProxyを使う場合

以下のコマンドでPixhawkからのMAVLinkデータを受信し、コンソールに表示できます。

```bash
sudo mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

- 「Waiting for heartbeat from /dev/ttyACM0」と表示された後、Pixhawkからのハートビートを受信するとMAVLinkメッセージが表示されます[^4][^6]。


### Python（pymavlink）を使う場合

以下のサンプルスクリプトでMAVLinkメッセージを受信し、コンソールに表示できます。

```python
from pymavlink import mavutil

# USB接続の場合、/dev/ttyACM0を指定
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received")

while True:
    msg = connection.recv_match(blocking=True)
    if msg:
        print(msg)
```

- `/dev/ttyACM0`やボーレートは環境に合わせて変更してください[^4]。


## 5. トラブルシューティング

- **デバイスが認識されない場合**
    - ケーブルを交換する
    - `lsusb`や`dmesg`で認識状況を確認
    - 権限設定やユーザーグループ（`dialout`）の確認
- **ハートビートが受信できない場合**
    - PixhawkのUSBポートが有効か、ファームウェア設定を見直す
    - 別のUSBポートやケーブルを試す
    - ボーレート設定をPixhawkと一致させる[^5]

---

この手順でPixhawk 6Cから送られてくるMAVLinkデータを、Raspberry Pi 5上のPythonやMAVProxyで受信・表示できます[^4][^5][^6]。

<div style="text-align: center">⁂</div>

[^1]: https://qiita.com/takurot/items/e9f1c9dfa05336f180cd

[^2]: https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html

[^3]: https://bellergy.com/5-wring-and-connect-pixhawk-to-raspberry-pi/

[^4]: https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html

[^5]: https://discuss.px4.io/t/problems-connecting-a-pixhawk-6c-to-a-raspberry-pi/32318

[^6]: https://www.youtube.com/watch?v=nIuoCYauW3s

[^7]: https://discuss.px4.io/t/talking-to-a-px4-fmu-with-a-rpi-via-serial-nousb/14119

[^8]: https://discuss.ardupilot.org/t/pixhawk-6c-to-rpi-4-connection/104768

[^9]: https://www.youtube.com/watch?v=cZVNndOaYCE

[^10]: http://community.dojofordrones.com/t/communicating-with-raspberry-pi-via-mavlink/285

[^11]: https://discuss.ardupilot.org/t/mavlink-connection-on-raspberry-pi/127360

[^12]: https://px4.gitbook.io/px4-user-guide/drone_parts/companion_computer/holybro_pixhawk_rpi_cm4_baseboard

[^13]: https://www.youtube.com/watch?v=u-NOD0PegwA

[^14]: https://discuss.bluerobotics.com/t/raspberry-pi-to-pixhawk-6x-usb-question/13272

[^15]: https://sites.google.com/site/satakegiken/ドローン情報/ardupilot-pixhawk

[^16]: https://www.youtube.com/watch?v=DGAB34fJQFc

