# sample

## 実機と可視化の分担

| マシン | パッケージ | 起動するもの |
| --- | --- | --- |
| 実機PC | `my_node` | シリアル入出力、センサー配信、Commander、Governor、モデル・TF配信、EKF、SLAM、Nav2 |
| 可視化PC | `my_visualization` | RVizによるモデル・地図・LiDAR・推定軌跡の表示 |

実機PCが`robot_description`、`tf`、`tf_static`、`map`、`scan`、`odometry/filtered`を配信する。
可視化PCはそれらを購読する。モデルの読み込みとTF配信も実機側で行うので、RVizを止めても実機側の処理は継続する。
可視化パッケージは実機パッケージ、sensor-flow、C++コンパイラに依存しない。

両PCでROS Jazzyを使い、同じLANから通信できるようにする。
各PCでROSを実行するすべての端末に、同じドメインとDDSの探索範囲を設定する。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=18
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_DOMAIN_ID`は両PCで同じ値なら変更してよい。
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`では別PCを自動検出できない。
マルチキャストによる探索が通るネットワークを使い、両PCの時計も同期する。
設定はlaunchから書き換えず、各PCの環境変数で指定する。

## 実機PCでの起動

既存のsensor-flowライブラリをビルド・インストールした状態で、実機パッケージだけをビルドする。

```bash
rosdep install --from-paths src/my_node --ignore-src -r -y
colcon build --packages-select my_node --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-16
source install/setup.bash
ros2 launch my_node my_node.launch.py serial_device:=/dev/ttyACM0
```

このlaunchは、C++アプリのシリアル入出力とセンサー配信、モデル・TF配信、EKF、SLAM、Nav2をまとめて起動する。
シリアル接続先は`serial_device`で指定する。既存プロトコルのENTER待ちにはlaunchが改行を送り、ハンドシェイクと時刻同期の後に自動でスキャンを開始する。
C++アプリが終了すると、同じlaunchのほかのノードも終了する。
CommanderとGovernorのスレッドは引き続きC++アプリ内で管理する。

EKFは`odom`と`imu`を購読し、`odometry/filtered`と`odom -> base_link`を配信する。
SLAMは`scan`を購読し、`map`と`map -> odom`を配信する。
Nav2は`navigation_launch.py`から起動するため、AMCLと地図サーバーは起動せず、SLAMの地図とTFを使う。
Nav2の設定は`config/nav2_params.yaml`を使い、`nav2_params_file:=/path/to/params.yaml`で変更できる。

速度指令の経路は次のようになる。指令の型はCommanderに合わせて`geometry_msgs/msg/Twist`を使う。

```text
Nav2 controller/behavior -> cmd_vel_nav -> velocity_smoother
    -> cmd_vel_smoothed -> collision_monitor -> cmd_vel -> Commander
```

Nav2のOdometry入力は`odometry/filtered`へ合わせる。
前後速度の下限は0にしているが、Commander::transformは引き続き未実装で既定値のMotorCommandを返す。
実機走行用の速度変換と左右モーターの正転制約を満たす制御・回復動作の調整は未完了。

C++アプリだけを端末から起動して調査する場合は、次の従来の起動方法も使える。
統合launchと同時に実行するとシリアルポートとノードが重複するため、どちらか一方を起動する。

```bash
ros2 run my_node my_node /dev/ttyACM0
```

## 可視化PCでの起動

可視化パッケージだけ依存を導入してビルドする。実機用のコンパイラ指定とsensor-flowのビルドは不要。

```bash
rosdep install --from-paths src/my_visualization --ignore-src -r -y
colcon build --packages-select my_visualization
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
- 車軸は後端から50mm、キャスターは後端から150mm、LiDARとIMUは後端から100mm。
- LiDARの中心は地面から200mm。IMUの高さは未計測のため、初期値は車軸と同じ21mmとする。

モデルのTFは次の配置になる。単位はm。

| 親 | 子 | x | y | z |
| --- | --- | --- | --- | --- |
| base_link | base_footprint | 0 | 0 | -0.021 |
| base_link | drivewhl_l_link | 0 | 0.065 | 0 |
| base_link | drivewhl_r_link | 0 | -0.065 | 0 |
| base_link | front_caster | 0.10 | 0 | -0.011 |
| base_link | imu_link | 0.05 | 0 | imu_height - 0.021 |
| base_link | lidar_link | 0.05 | 0 | 0.179 |

IMUの高さは実機PCのlaunchに`imu_height:=0.08`のように地面からの実測値[m]を指定する。
車輪角度の配信は未実装のため、joint_state_publisherが角度0を配信し、モデル上の車輪は回転しない。
筐体は車輪の上から全高までの箱で近似し、キャスター径とセンサーの外観寸法も表示用の仮値とする。
IMU・LiDARの取り付け軸は`base_link`と同じ向きを仮定している。実機に合わせて高さと向きを調整すること。
質量・慣性とGazeboの制御プラグインは含めていないため、現状は表示とTF用のモデルになる。
Nav2のglobal/local costmapの外形は車軸基準で前150mm・後50mm・左右90mmに合わせている。

## reference

- https://arxiv.org/pdf/1806.03581
- https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst
