# sample

速度に関係する設定値は次のとおり。Commanderはdevice.hppを参照し、Nav2はnav2_params.yamlを参照する。

| 設定箇所                                              | 対象                          | 下限 | 上限   | 単位  |
| ----------------------------------------------------- | ----------------------------- | ---- | ------ | ----- |
| device.hppのMIN/MAX_VELOCITY                          | Commanderが換算する片輪の速度 | 0.0  | 0.3665 | m/s   |
| FollowPath.desired_linear_vel                         | RPPの前進速度                 | 0.0  | 0.3665 | m/s   |
| velocity_smoother.min/max_velocityのx                 | 平滑化後の前後速度            | 0.0  | 0.3665 | m/s   |
| velocity_smoother.min/max_velocityのy                 | 平滑化後の横速度              | 0.0  | 0.0    | m/s   |
| velocity_smoother.min/max_velocityのz                 | 平滑化後の旋回角速度          | -2.0 | 2.0    | rad/s |
| docking_server.controller.v_linear_min / v_linear_max | ドッキングの前後速度          | 0.15 | 0.15   | m/s   |

sensor-echoのローカルREADMEにある走行測定から、距離を経過時間で割って速度を求めた。

| PWM duty | 移動距離 | 経過時間 | 平均速度   |
| -------- | -------- | -------- | ---------- |
| 25%      | 0.215 m  | 3.001 s  | 0.0716 m/s |
| 50%      | 0.51 m   | 3.001 s  | 0.1699 m/s |
| 75%      | 0.75 m   | 3.001 s  | 0.2499 m/s |
| 100%     | 1.10 m   | 3.001 s  | 0.3665 m/s |

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

## ロボットモデル

モデルのTFは次の配置になる。単位はm。

| 親        | 子              | x    | y      | z      |
| --------- | --------------- | ---- | ------ | ------ |
| base_link | base_footprint  | 0    | 0      | -0.021 |
| base_link | drivewhl_l_link | 0    | 0.065  | 0      |
| base_link | drivewhl_r_link | 0    | -0.065 | 0      |
| base_link | front_caster    | 0.10 | 0      | -0.011 |
| base_link | imu_link        | 0.05 | 0      | 0.144  |
| base_link | lidar_link      | 0.09 | 0      | 0.179  |

IMUの地面からの高さは`src/my_node/urdf/sample.urdf.xacro`の`imu_height`で設定する。既定値は0.165m。
`base_link -> imu_link`のzは`imu_height - 0.021`になる。
車輪角度の配信は未実装のため、joint_state_publisherが角度0を配信し、モデル上の車輪は回転しない。
筐体は車輪の上から全高までの箱で近似し、キャスター径とセンサーの外観寸法も表示用の仮値とする。
IMU・LiDARの取り付け軸は`base_link`と同じ向きを仮定している。取り付け向きが異なる場合は実機に合わせて調整すること。
質量・慣性とGazeboの制御プラグインは含めていないため、現状は表示とTF用のモデルになる。
Nav2のglobal/local costmapの外形は車軸基準で前150mm・後50mm・左右90mmに合わせている。
