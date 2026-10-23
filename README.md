# sample

このブランチはteleopで地図を調べる構成。Governor・WFDはビルド対象から外し、Nav2は起動しない。
自動探索の変更と試走記録は、別のローカルブランチ`codex/exploration-checkpoint`に保存してある。

## 実機と可視化の分担

| マシン | パッケージ | 起動するもの |
| --- | --- | --- |
| 実機PC | `my_node`（`make run`） | シリアル入出力、センサー配信、Commander |
| 実機PC | `my_node`（`make model`） | モデル・TF配信 |
| 実機PC | `my_node`（`make stack`） | EKF、SLAM |
| 可視化PC | `my_visualization` | RVizによるモデル・地図・LiDAR・推定軌跡の表示 |

実機PCが`robot_description`、`tf`、`tf_static`、`map`、`scan`、`odometry/filtered`を配信する。
可視化PCはそれらを購読する。モデルの読み込みとTF配信も実機側で行うので、RVizを止めても実機側の処理は継続する。
可視化パッケージは実機パッケージ、sensor-flow、C++コンパイラに依存しない。

両PCでROS Jazzyを使い、同じLANから通信できるようにする。時計も同期する。
ROSを実行する端末では両PCのドメインを0にそろえる。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

実機側のLinux Dockerは`network_mode: host`を使う。
Makefileは未指定の場合に`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`と、
リポジトリ内の`src/my_node/config/fastdds.xml`を設定する。
`make run`・`make model`・`make stack`・`make teleop`に共通で適用する。
既存の環境変数によるRMWやプロファイルの指定は優先するため、可視化側のCycloneDDSも維持できる。

付属のFast DDS設定はUDPの最大サイズを1400バイトに制限し、大きなメッセージをDDS側で分割する。
送信側と受信側の設定を揃える。受信側だけを1400バイトにすると、大きな`LaserScan`を受信できない場合がある。
ROSのドメインは両PCで0にそろえ、実機側の探索範囲は`SUBNET`を使う。
設定を変えた場合は関係するROSプロセスを起動し直す。OSやDockerの再起動は不要。

可視化側のmacOS＋OrbStackでは、Linux VMとLANの間にNATがある。
`Lidar-SLAM/env/docker-compose.yml`はbridge接続でDDSのUDPポート`7410–7511`を同じ番号で公開する。
これは`ROS_DOMAIN_ID=0`、CycloneDDSの参加者インデックス0〜50に対応する。ドメインを変える場合は公開ポートも変更する。
`env/cyclonedds.xml`で探索先と、返答先として通知するMacのLAN側IPを指定する。
こちらもUDPを1400バイト以下に分割する。

`Lidar-SLAM/env/.env`にはMac本体とRasPi本体のLAN側IPを設定する。コンテナのIPは使わない。

```dotenv
ROS_LOCAL_IP=192.168.0.10
ROS_PEER_IP=192.168.0.6
```

可視化側はcomposeの`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`と`ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT`を使い、
`CYCLONEDDS_URI=file:///etc/cyclonedds.xml`でXMLのユニキャスト設定を読み込む。
OrbStackの「Expose ports to LAN」を有効にする。端末から探索範囲を`LOCALHOST`や`SUBNET`へ上書きしない。
IPが変わった場合は`.env`も更新する。各PCの設定はlaunchから書き換えない。
noVNCは`6080:6080`で公開し、`http://localhost:6080/vnc.html`で開く。
通信設定はノードの起動時に読み込まれるため、変更後はROSノードを起動し直す。OSの再起動は不要。

## Makefileからの起動・疎通確認

ROS環境とビルド済みワークスペースの`install/setup.bash`を読み込んだ端末で実行する。
`ROS_DOMAIN_ID`は両PCで0にそろえ、探索設定は上記の各環境に合わせる。
`make nodes`と`make topics`は探索を10秒待ってから一覧を表示する。

| コマンド | 内容 |
| --- | --- |
| `make build` | `my_node`だけビルド |
| `make visual` | `my_visualization`だけビルド |
| `make init` | `my_node`だけsymlink-installでビルド |
| `make run` | `my_node`本体を起動 |
| `make stack` | EKF・SLAMを起動 |
| `make teleop` | キーボードで前進・旋回・停止を指示 |
| `make display` | 可視化PCでRVizを起動 |
| `make preview` | 可視化PCでモデル確認用のRVizを起動 |
| `make model` | モデル・TF配信だけ起動 |
| `make nodes` | 見えているノードを一覧表示 |
| `make topics` | 見えているトピックと型を一覧表示 |
| `make imu` / `make scan` / `make odom` | IMU・LiDAR・エンコーダの生Odometryを1件受信 |
| `make ekf` | EKFのOdometryを1件受信 |
| `make map` | 地図の寸法・解像度などを1件受信 |
| `make tf` | `map -> base_link`の現在位置を継続表示 |

センサー・地図の確認は受信すると終了する。受信待ちやTF表示を止める場合はCtrl+C。
モデルだけを見る場合は実機側で`make model`、可視化側で`make preview`を使う。

## 実機PCでの起動

既存のsensor-flowライブラリをビルド・インストールした状態で、実機パッケージだけをビルドする。

```bash
rosdep install --from-paths src/my_node --ignore-src -r -y
make build
source install/setup.bash
make model
```

`make model`でモデルとTFを配信する。別の端末で同じROS環境とワークスペースを読み込み、地図生成を起動する。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
make stack
```

`make stack`はEKFとSLAMだけを起動する。さらに別の端末でC++アプリを起動する。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
make run
```

`make run`は`my_node.launch.py`でC++アプリだけを起動し、`make stack`は`stack.launch.py`を起動する。
起動した端末ごとにCtrl+Cで停止する。`my_node`を終了・再起動しても、別端末のモデル・EKF・SLAMは継続する。
モデルとTFは`make model`だけが配信するため、地図生成中もこの端末を起動しておく。

シリアル接続先の既定値は`/dev/ttyACM0`。変更する場合は次のように直接launchする。

```bash
ros2 launch my_node my_node.launch.py serial_device:=/dev/ttyACM1
```

既存プロトコルのENTER待ちには`my_node.launch.py`が改行を送り、ハンドシェイクと時刻同期の後に自動でスキャンを開始する。
CommanderのスレッドはC++アプリ内で管理する。GovernorとWFDのソースは残すが、ビルドには含めない。

さらに別の端末で同じROS環境とワークスペースを読み込み、キーボード操作を起動する。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
make teleop
```

`i`で前進、`u`で左へ曲がりながら前進、`o`で右へ曲がりながら前進、`k`で停止する。
初期値は前進0.15m/s、旋回0.8rad/s。停止して終了する場合はCtrl+C。
車輪は両方とも正転のみで、後退・その場旋回はできない。

速度指令は`teleop_twist_keyboard -> cmd_vel -> Commander`の順で届く。
型は`geometry_msgs/msg/Twist`。このブランチではNav2の速度平滑化・衝突監視を通さない。
Commanderはnumericの`toVelocityRatio`で前進・旋回速度から左右の速度比を求める。
左右それぞれの負値は0、上限超えは100%へ収め、NaN・無限大の場合は両輪0%を返す。

EKFは`odom`と`imu`を購読し、`odometry/filtered`と`odom -> base_link`を配信する。
エンコーダの前進速度・横速度0・旋回角速度に加え、IMUの加速度XYと角速度Zを融合する。
`two_d_mode: true`を維持し、IMUの姿勢と加速度Zは融合しない。
IMUは水平設置を前提にし、姿勢未提供のため重力除去は無効とする。傾くとXYへ重力が混ざる。
入力の共分散は`device.hpp`の暫定値を使う。静止時の偏りと分散、走行時の横滑りに合わせた調整は未実施。

SLAMは`scan`を購読し、`map`と`map -> odom`を配信する。
`config/slam.yaml`はJazzyのSLAM標準設定を基に、試走時と同じ
`restamp_tf: true`、`minimum_travel_distance: 0.1`、`minimum_travel_heading: 0.1`、
`check_min_dist_and_heading_precisely: true`を設定する。
地図の配信間隔5秒、スキャン処理の最小間隔0.5秒は維持する。

Commanderの片輪速度の換算範囲は`device.hpp`の0〜0.3665m/s。
sensor-echoの測定結果は次のとおり。距離を経過時間で割って平均速度を求めた。

| PWM duty | 移動距離 | 経過時間 | 平均速度 |
| --- | --- | --- | --- |
| 25% | 0.215 m | 3.001 s | 0.0716 m/s |
| 50% | 0.51 m | 3.001 s | 0.1699 m/s |
| 75% | 0.75 m | 3.001 s | 0.2499 m/s |
| 100% | 1.10 m | 3.001 s | 0.3665 m/s |

最大速度は`1.10 / 3.001 ≈ 0.3665m/s`を左右輪共通の換算基準として使う近似。
測定点の補間は行わない。25%の値は始動速度の下限を測ったものではなく、停止指令には0を使う。

`make run`の代わりに、C++アプリを直接起動して調査することもできる。
この方法では、プロトコルのスキャン開始待ちにENTERを押す。

```bash
ros2 run my_node my_node /dev/ttyACM0
```

## 可視化PCでの起動

可視化パッケージだけ依存を導入してビルドする。実機用のコンパイラ指定とsensor-flowのビルドは不要。

```bash
rosdep install --from-paths src/my_visualization --ignore-src -r -y
make visual
source install/setup.bash
ros2 launch my_visualization visualization.launch.py
```

RVizの基準座標は`map`で、地図・モデル・TF・LiDAR・EKFのOdometryを表示する。
`robot_description`と`map`はTransient Local、`scan`はBest Effortで購読する。
カスタムRViz設定は`rviz_config:=/path/to/config.rviz`で指定できる。

地図を作らずモデルだけ確認する場合は、実機PCで次を起動する。

```bash
ros2 launch my_node description.launch.py
```

可視化PCでは地図を必要としない座標を指定する。

```bash
ros2 launch my_visualization visualization.launch.py fixed_frame:=base_footprint
```

可視化PCで`ros2 topic list`に実機のトピックが出ない場合は、ドメイン・探索範囲・ネットワークを確認する。
ROS CLIのdaemonを起動済みの端末で通信設定を変えた場合は、`ros2 daemon stop`の後で再確認する。

## ロボットモデル

Nav2公式チュートリアルの[sam_bot_description](https://github.com/ros-navigation/navigation2_tutorials/blob/f3bd892a60d3fb91560ecb726bf922eb9bcbc858/sam_bot_description/src/description/sam_bot_description.urdf)を元に、
[DEVICE.md](DEVICE.md)の寸法へ変更したURDF/Xacroを`src/my_node/urdf/sample.urdf.xacro`に置く。
出典・変更内容・Apache 2.0のライセンスは同じディレクトリの`NOTICE.md`と`LICENSE`に記載している。

- 全長200mm、全幅180mm、全高220mm。`DEVICE.md`の`22ch`は22cmとして扱う。
- 車輪は直径42mm・幅19mm、左右の中心間距離は130mm。
- `base_link`は左右車軸の中点。前方が+x、左が+y、上が+z。
- 車軸は後端から50mm、キャスターは後端から150mm。
- LiDARは左右中央・後端から140mm・地面から200mm。
- IMUは左右中央・後端から100mm・地面から165mm。

モデルのTFは次の配置になる。単位はm。

| 親 | 子 | x | y | z |
| --- | --- | --- | --- | --- |
| base_link | base_footprint | 0 | 0 | -0.021 |
| base_link | drivewhl_l_link | 0 | 0.065 | 0 |
| base_link | drivewhl_r_link | 0 | -0.065 | 0 |
| base_link | front_caster | 0.10 | 0 | -0.011 |
| base_link | imu_link | 0.05 | 0 | 0.144 |
| base_link | lidar_link | 0.09 | 0 | 0.179 |

IMUの地面からの高さは`src/my_node/urdf/sample.urdf.xacro`の`imu_height`で設定する。既定値は0.165m。
`base_link -> imu_link`のzは`imu_height - 0.021`になる。
車輪角度の配信は未実装のため、joint_state_publisherが角度0を配信し、モデル上の車輪は回転しない。
筐体は車輪の上から全高までの箱で近似し、キャスター径とセンサーの外観寸法も表示用の仮値とする。
IMU・LiDARの取り付け軸は`base_link`と同じ向きを仮定している。取り付け向きが異なる場合は実機に合わせて調整すること。
質量・慣性とGazeboの制御プラグインは含めていないため、現状は表示とTF用のモデルになる。

## reference

- https://arxiv.org/pdf/1806.03581
- https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst
