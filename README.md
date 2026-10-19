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

## Makefileからの起動・疎通確認

ROS環境とビルド済みワークスペースの`install/setup.bash`を読み込んだ端末で実行する。
上記の`ROS_DOMAIN_ID`と探索範囲も両PCで合わせる。

| コマンド | 内容 |
| --- | --- |
| `make run` | 実機側一式を起動 |
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
Nav2 controller -> cmd_vel_nav -> velocity_smoother
    -> cmd_vel_smoothed -> collision_monitor -> cmd_vel -> Commander
```

Nav2のOdometry入力は`odometry/filtered`へ合わせる。
Commander::transformはnumericのtoVelocityRatioで前進・旋回速度から左右の速度比を求め、0〜100%のPWM指令へ変換する。
左右それぞれの負値は0、上限超えは100%へ収め、NaN・無限大の場合は両輪0%を返す。
異常値はERROR、範囲外を収めた場合はWARNで、入力・速度上下限・車輪間隔・左右の比率・実際のPWM出力を記録する。
速度比とPWMの関係は線形と仮定し、最大速度には以下の実測値を使う。

速度に関係する設定値は次のとおり。Commanderはdevice.hppを参照し、Nav2はnav2_params.yamlを参照する。

| 設定箇所 | 対象 | 下限 | 上限 | 単位 |
| --- | --- | --- | --- | --- |
| device.hppのMIN/MAX_VELOCITY | Commanderが換算する片輪の速度 | 0.0 | 0.3665 | m/s |
| FollowPath.desired_linear_vel | RPPの前進速度 | 0.0 | 0.3665 | m/s |
| velocity_smoother.min/max_velocityのx | 平滑化後の前後速度 | 0.0 | 0.3665 | m/s |
| velocity_smoother.min/max_velocityのy | 平滑化後の横速度 | 0.0 | 0.0 | m/s |
| velocity_smoother.min/max_velocityのz | 平滑化後の旋回角速度 | -2.0 | 2.0 | rad/s |
| docking_server.controller.v_linear_min / v_linear_max | ドッキングの前後速度 | 0.15 | 0.15 | m/s |

sensor-echoのローカルREADMEにある走行測定から、距離を経過時間で割って速度を求めた。

| PWM duty | 移動距離 | 経過時間 | 平均速度 |
| --- | --- | --- | --- |
| 25% | 0.215 m | 3.001 s | 0.0716 m/s |
| 50% | 0.51 m | 3.001 s | 0.1699 m/s |
| 75% | 0.75 m | 3.001 s | 0.2499 m/s |
| 100% | 1.10 m | 3.001 s | 0.3665 m/s |

CommanderとNav2の前後速度上限は、100%時の`1.10 / 3.001 ≈ 0.3665 m/s`にそろえる。
これは直進時の車体速度を、左右輪共通の換算基準として使う近似。測定点の補間は行わない。
25%時の0.0716 m/sは測った最小の点で、動き出す最低速度を測ったものではない。停止指令のため下限は0のままにする。
角速度は今回測定していないため、既存の設定値を使う。

車輪は両方とも正転のみで、後退・その場旋回はできない。
経路追従にはRegulated Pure Pursuit（RPP）を使い、`allow_reversing: false`と`use_rotate_to_heading: false`を指定する。
速度上限は実測値の0.3665 m/s。カーブ・障害物付近・ゴール手前では減速する。
減速時の走行速度下限0.05 m/sは調整用の初期値で、実測の始動速度ではない。停止指令は0 m/sを使う。

先読み距離は20cmで固定し、曲率計算には`use_fixed_curvature_lookahead: true`を使う。
`interpolate_curvature_after_goal: true`で、ゴール付近でも曲率用の点を20cm先まで補う。
Pure Pursuitの曲率は`2 * y / L²`なので、先読み距離`L = 0.2m`では旋回半径が0.1m以上になる。
これは両輪を正転・停止に保つための幾何上の下限、車輪間隔0.13mの半分の0.065mより大きい。
`regulated_linear_scaling_min_radius: 0.3`は半径30cm未満で減速を始める値で、旋回半径そのものの下限ではない。
velocity_smootherは`scale_velocities: true`で前進と旋回の速度変化を同じ比率に抑える。
角速度の負値は右旋回、加速度・減速度の負値は減速にも使うため残す。

Governorが使う`NavigateToPose`だけを有効にし、Nav2同梱の`navigate_w_replanning_time.xml`を使う。
1秒ごとに経路を再計算してRPPで追従する。独自のBT XMLは置かない。
後退・その場旋回・costmapクリア・待機による回復処理は行わず、経路計算や追従に失敗したらゴールを失敗として返す。
ゴールは位置の許容範囲0.25mで判定し、その場旋回による向き合わせは行わない。
`dock_backwards: false`は維持するが、後退が必要な`UndockRobot`は使わない。
Commanderの各車輪0〜100%制限も最終段に残す。
経路探索は既存のNavfnを使う。旋回半径を考慮した経路生成や、真後ろの経路からの切り返しは扱わない。実機での走行は未確認。

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
Nav2のglobal/local costmapの外形は車軸基準で前150mm・後50mm・左右90mmに合わせている。

## reference

- https://arxiv.org/pdf/1806.03581
- https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Advanced/Improved-Dynamic-Discovery.rst
