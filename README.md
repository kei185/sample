# sample

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

筐体は車輪の上から全高までの箱で近似し、キャスター径とセンサーの外観寸法も表示用の仮値とする。
IMU・LiDARの取り付け軸は`base_link`と同じ向きを仮定している。実機に合わせて高さと向きを調整すること。
質量・慣性とGazeboの制御プラグインは含めていないため、現状は表示とTF用のモデルになる。

## 起動

ROS Jazzyで、既存のsensor-flowライブラリをビルド・インストールした状態で実行する。
追加の実行時依存は`rosdep install --from-paths src/my_node --ignore-src -r -y`で導入できる。

```bash
colcon build --packages-select my_node --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-16
source install/setup.bash
ros2 launch my_node display.launch.py
```

RVizを使わずにモデルとTFだけを配信する場合は`use_rviz:=false`を指定する。
IMUの高さは`imu_height:=0.08`のように地面からの実測値[m]を指定する。
車輪角度の配信は未実装のため、joint_state_publisherが角度0を配信し、モデル上の車輪は回転しない。

実機のセンサー入力は別の端末から起動する。

```bash
ros2 run my_node my_node /dev/ttyACM0
```

モデル・EKF・SLAMをまとめて起動する場合は次を使う。

```bash
ros2 launch my_node my_node.launch.py
```

EKFは`odom`と`imu`を購読し、`odometry/filtered`と`odom -> base_link`を配信する。
SLAMは`scan`を購読し、`map -> odom`を配信する。Nav2自体の起動は含まない。
Nav2のglobal/local costmapの外形は車軸基準で前150mm・後50mm・左右90mmに合わせている。

## reference

- https://arxiv.org/pdf/1806.03581
