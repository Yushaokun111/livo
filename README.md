# LIVO 地铁隧道 Gazebo 场景

**总长 2200 m，内径 5 m，包含直线、左弯、右弯和三次抛物线缓和段。** 总长按线路中心线里程计算，保持水泥内壁、双轨、重复轨枕、灯具、电缆槽和检修步道。另有 MID-360、FAST-LIO2 和 APM 4.6.3 SITL 四旋翼飞行入口，见下方“MID-360 激光定位”一节；尚未接入视觉 LIVO。

## 启动

```bash
cd /home/qwer/livo
./scripts/run.sh
```

默认视角位于 **K0+520 左弯内部**。显卡兼容性有问题时可用 `./scripts/run.sh --software`；只运行服务端用 `./scripts/run.sh --server`。全部资源在本地，不依赖 Fuel 下载或 ROS 编译。关闭窗口或在终端按 Ctrl+C 停止。

场景针对 Gazebo Harmonic 8 / Ogre2 / DART。2200 m 资源约 315 MB，按 50 m 分成 44 块以支持视锥剔除，首次加载比原 120 m 场景更慢。完整场景包含 734 盏点光源，软件渲染适合检查画面，不应据此假定正式相机与点云能实时运行。

## 线路布置

![线路平面及曲率](output/alignment_plan.png)

| 里程范围 | 线形 | 长度 | 圆曲线半径 / 方向 |
| --- | --- | ---: | --- |
| K0+000～K0+400 | 直线 | 400 m | — |
| K0+400～K0+480 | 入口缓和段 | 80 m | 进入 R600 左弯 |
| K0+480～K0+880 | 圆曲线 | 400 m | R600 m，左弯 |
| K0+880～K0+960 | 出口缓和段 | 80 m | 回到直线 |
| K0+960～K1+360 | 直线 | 400 m | — |
| K1+360～K1+440 | 入口缓和段 | 80 m | 进入 R800 右弯 |
| K1+440～K1+840 | 圆曲线 | 400 m | R800 m，右弯 |
| K1+840～K1+920 | 出口缓和段 | 80 m | 回到直线 |
| K1+920～K2+200 | 直线 | 280 m | — |
| **合计** | **直线 1080 m + 圆曲线 800 m + 缓和段 320 m** | **2200 m** | |

起点为世界坐标 `(0,0)`，初始朝向 `+X`；左侧为 `+Y`，向上为 `+Z`。终点约为 `(1933.461,817.469)` m，所以 2200 m 是沿线长度，不是世界 X 轴跨度或端点直线距离。线路纵坡为零。

`config/centerline.csv` 提供每 1 m 的里程、世界 X/Y、航向、曲率和轨道横滚角，便于后续设置传感器轨迹。`scripts/alignment.py` 可以查询任意里程的连续位置。

## 国标选取依据与假设

本场景按 **普通钢轮钢轨地铁区间正线、设计速度 80 km/h** 选取平面参数；不是车站、车辆段或极限小半径工况。

- GB 55033—2022 第 5.2.6 条要求线形与车辆及设计速度相适应；该规范自 2023-03-01 实施，相关旧强条按新规范执行。[条文](https://gf.cabr-fire.com/article-41855.htm) / [发布说明](https://gf.cabr-fire.com/article-41838.htm)
- GB 50157—2013 表 6.2.1-1 的普通正线最小半径为 A 型车 350 m、B 型车 300 m；这里选 600 m 和 800 m。[原规范第 6.2 节](https://gf.cabr-fire.com/article-11359.htm)
- 按该规范表 6.2.2，80 km/h 时 R600 对应 `h=120 mm、L=70 m`，R800 对应 `h=95 mm、L=55 m`。这里统一采用 **80 m 缓和段**，使用实际的 `y=a·x³` 三次抛物线，通过求解使弧长及末端曲率同时匹配圆曲线；出口作反向构造，连接处位置、切向和曲率连续。[表格原图](https://gf.cabr-fire.com/Web/uploads/allimg/221122/10-221122091621607.jpg) / [续表](https://gf.cabr-fire.com/Web/uploads/allimg/221122/10-221122091F14Z.jpg)
- 曲线中间夹直线取 400 m；80 km/h 的一般条件下最小值为 40 m。两段圆曲线各 400 m，也大于 A 型车 25 m 的最小长度要求。[第 6.2 节](https://gf.cabr-fire.com/article-11359.htm)
- 轨道采用外轨升高一半、内轨降低一半的超高方式，并在缓和段内线性递变。最大顺坡率为 **1.5‰**，低于 2‰；R600/R800 不触发小半径轨距加宽。[第 7.2 节](https://gf.cabr-fire.com/article-11364.htm)

上述核对范围是本仿真的平面线形、缓和段及超高参数。保留的约 5 m 内径及简化轨道、步道不是完整车辆限界或土建工程设计，不将此仿真模型标记为全专业国标验收合格。

## 几何和材质

| 项目 | 设置 |
| --- | --- |
| 圆形内径 / 壁厚 | 5.0 m / 0.20 m |
| 圆心 / 拱顶 | Z=2.10 m / Z=4.60 m |
| 轨面中心基准高度 | Z=0.25 m，弯道内外轨随超高分别降低/升高 |
| 轨距 | 1.435 m，按轨道横断面内两轨头内侧面距离定义 |
| 管片环缝 | 沿中心线每 1.5 m；另有 6 条纵缝 |
| 轨枕 | 每 0.6 m，共 3666 根 |
| 灯具 | 每 6 m 左右各一盏，共 734 盏 |
| 采样 | 环向 128 段；纵向网格最长 2 m |
| 碰撞 | 每块采用静态三角网格，覆盖衬砌、道床、轨枕、铁轨和步道 |

内壁、轨道、道床、电缆槽和灯具都沿同一中心线弯曲。轨道、轨枕、扣件与道床同步横向倾斜；隧道壳、步道、灯具保持竖直横断面。相邻块共用精确里程截面，避免弯道拼接缝隙。纵向 2 m 离散在 R600 段的中心线弦高误差约 0.83 mm；视觉和碰撞网格均为离散近似，连续曲率指设计中心线。

水泥纹理保持固定种子、低对比度并沿里程重复；不加入独有标识、车辆或杂物。点光源产生重复明暗变化，默认关闭阴影。灯具、电缆、扣件和毫米级接缝保留视觉几何，未单独加入碰撞；GPU LiDAR 能看到这些视觉表面。

## 修改参数

编辑 `config/tunnel.json` 后重新生成，再重启 Gazebo：

```bash
python3 scripts/generate_tunnel.py
./scripts/run.sh
```

`length` 是总里程校验值，必须与 `alignment` 中各段 `length` 的总和一致。改长短时编辑分段长度；生成器不会缩放整条线路，避免悄悄改变曲线半径。当前验证配置限定为 80 km/h、R600/R800 及对应超高，改变速度/半径需要先重新核对规范表。

照明和纹理可以临时覆盖：

```bash
python3 scripts/generate_tunnel.py --texture-contrast 0.03 --ambient 0.08 --light-intensity 0.6
```

其他参数包括 `inner_diameter`（4.5～6 m）、`light_spacing`、`chunk_length`（10～100 m）、`mesh_step`（0.25～3 m）和 `seed`。默认主要结构按里程以 6 m 为共同重复周期。实际输出参数保存到 `config/generated.json`。每次生成会覆盖生成文件，并清理上一版生成器的网格。

## 验证和预览

```bash
python3 scripts/check_alignment.py
SDF_PATH="$PWD/models" gz sdf -k worlds/metro_tunnel.sdf
python3 scripts/verify_scene.py --software
```

生成器只依赖 Python 标准库；平面图使用 Matplotlib；Gazebo 验收使用 `python3-gz-transport13` 和 `python3-gz-msgs10`。本机均已安装。

- `output/alignment_checks.json`：总长、段间连续性、数值曲率、轨距、超高及无效配置拒绝检查。
- `output/verification.json`：完整场景中直段、左右圆曲线和进出缓和段的激光测距，以及四个位置上的道床、双轨、内壁碰撞检查。
- `output/tunnel_preview.png` / `tunnel_right.png` / `tunnel_straight.png`：真实 Gazebo 相机渲染。
- `output/gazebo.log`：完整场景验收日志。

弯道半径检查使用前后反向射线长度乘积推算半径，可消除掠射方向上 GPU 角度离散的一阶影响；验收容差为 1 m。报告中的激光距离 `null` 表示 150 m 量程内无回波。设计半径的精确性另由中心线曲率数值检查验证。

验收脚本针对本预设线形创建临时相机、LiDAR 和竖直导向落球，避免超高轨面上的球滚走干扰接触高度测量。它使用独立通信分区，结束后关闭测试进程，不向正式世界添加传感器。

## 后续 LIVO 测试

建议先在 K0+050～K0+350 或 K1+010～K1+310 的直段测试纵向退化，再穿过缓和段与弯道比较结果。端部封闭墙和弯道可提供额外约束，不能假定它们与无限直隧道等价。

基础场景 `worlds/metro_tunnel.sdf` 保留空隧道版本。下节的飞行世界已接入无人机、MID-360、IMU 和 FAST-LIO2；视觉相机及 LIVO 融合留待后续接入。后续跨弯道试验应使用里程坐标生成运动轨迹，并明确传感器是否随轨道超高倾斜。
# MID-360 激光定位与 APM 4.6.3 SITL 飞行

已增加 `scripts/run_flight.sh`：在隧道起点后 100.5 m 处启动 Iris 四旋翼，机顶安装 MID-360，激光定位稳定后进入 GUIDED，起飞到相对高度 1.5 m，沿直段前进 100 m，悬停后 LAND。轨迹控制使用飞控融合后的 ExternalNav；Gazebo 真值单独记录用于误差评估。

```bash
cd /home/qwer/livo
./scripts/run_flight.sh --distance 100 --name my-mid360-100m
python3 scripts/evaluate_flight.py runs/my-mid360-100m
python3 scripts/export_map.py runs/my-mid360-100m
```

每次 `--name` 必须是新名称，防止覆盖记录。默认仅使用本机回环地址：MAVLink UDP 15760、Gazebo/APM JSON UDP 19002；同时只运行一个该测试。Gazebo 使用独立通信分区。默认无 GUI，使用软件渲染，目标实时因子 0.25，墙钟时间约为仿真时间的四倍。队列溢出或定位滞后超过 1 s 时终止试验，不静默丢扫描帧；曾尝试的 Gazebo 暂停服务同步方式未通过联调，已移除。

定位链路：Gazebo MID-360 距离采样 → Livox 官方非重复扫描轨迹及原始 IMU → [FAST-LIO2 的 ROS 2 实现](https://github.com/Ericsii/FAST_LIO_ROS2) → 10 Hz MAVLink `ODOMETRY` → APM EKF3 → GUIDED 控制 → Gazebo 四旋翼动力学。当前为激光惯性里程计与建图，不包含回环检测或视觉融合。该链路不向定位算法提供 Gazebo 位姿、IMU 姿态真值、GPS 或真值速度。独立评估器可在误差超过 5 m、离开飞行走廊或意外上锁时终止试验，但不修正定位或航点。

默认雷达**倒装**，安装高度为机体原点上方 0.18 m，使 MID-360 的宽垂直视场朝向轨道及轨枕。激光点云保留倒装雷达坐标，原始 IMU 旋转到机体同向的虚拟 IMU 坐标；FAST-LIO 的雷达到 IMU 旋转外参为 `diag(1,-1,-1)`。之前朝上安装的纯激光 KISS-ICP 和 FAST-LIO 都在单一衬砌环境出现明显轴向退化，失败记录分别保留在 `runs/mid360-flight-04`、`runs/mid360-fastlio-05`；不能把倒装条件下的结果推广为朝上安装也可可靠定位。

MID-360 仿真配置依据 [Livox 官方规格](https://www.livoxtech.com/mid-360/specs)：水平 360°、垂直 −7°～52°、20 万发射点/秒、0.1 m 盲区。本次选择 40 m 测距上限，模拟距离噪声标准差 2 cm。噪声和反射率未按实物标定；强度为固定合成值。为去除机体遮挡，定位输入过滤距离小于 0.4 m 的回波，故有效点数略少于每帧 20000 点。

非重复扫描采用 [Livox 官方 CSV](https://github.com/Livox-SDK/livox_laser_simulation/blob/main/scan_mode/mid360.csv)，800000 个角度样本，按 20 万点/秒播放，每 4 s 回绕；这不是无限长实机扫描序列。Harmonic 的 GPU 距离图分辨率为水平 0.2°、垂直约 0.496°，50 Hz 更新，每次按 CSV 选 4000 条射线，5 个子帧组成 10 Hz 点云。使用实际离散射线方向重建点坐标，避免把一条射线的距离配给另一角度。CSV 中超出标称视场的少量边界角度裁剪到 −7°～52°。

运动按 20 ms 子帧离散。原始 NPZ 的 `time` 是实际采样快照时刻，`nominal_time` 是按 5 µs 发射间隔分配的名义时刻，两者分开保存；算法去畸变使用 `time`。这是一套 MID-360 扫描近似模型，不能用于证明实机角度精度、反射率响应、雾尘穿透能力或微秒级运动畸变性能。IMU 位于雷达安装点，发布 200 Hz 数据；当前 IMU 使用理想仿真读数，尚未注入实物标定的偏置、温漂和振动。

启动时仅用首帧激光点云拟合圆柱隧道轴线，将它作为局部地图 X 轴，避免以飞机停放时的任意偏航作为直行方向。拟合结果保存在 `lidar_initial_alignment.json`。雷达到机体外参为前左上坐标系 `[0, 0, 0.18] m`，里程计发送前已补偿这一杆臂。Gazebo 世界 +X 对应飞控北向、+Y 对应西向、+Z 向上；MAVLink 位姿为 `LOCAL_FRD(20)`，速度为 `BODY_FRD(12)`，符合当前 4.6.3 源码接收条件。GPS1/2 关闭，EKF3 位置、速度、航向源均为 ExternalNav(6)，保留 `ARMING_CHECK=1`。经纬度只用于设置无 GPS EKF 的参考原点，不作为定位测量。

所有运行产物保存在 `runs/<名称>/`：

- `logs/*.BIN`、`telemetry.tlog`：飞控日志与 MAVLink 遥测；`telemetry.jsonl`、`events.jsonl`：可读遥测及任务事件。
- `clouds/*.npz`：逐帧原始点云、采样时间、名义发射时间；`imu.csv`：倒装雷达原始 IMU。运行地图导出脚本后生成 `estimated_map.npy/.ply`，只用 SLAM 估计位姿及各子帧时间拼接，不使用真值。当前 ROS 2 分支没有启用周期 PCD 保存代码；可选 `/map_save` 服务可保存实时局部地图。
- `slam.csv`、`mavlink_odometry.csv`、`truth.csv`：激光估计、实际发送到飞控的里程计、独立仿真真值。
- `world.sdf`、`run_config.json`、`fastlio_mid360.yaml`：本次世界与配置；`eeprom.bin`：本次 SITL 参数存储。`kiss_icp.json` 仅供可选纯激光基线使用。
- `gazebo/`：状态回放记录、控制台与 Ogre 渲染日志；`cache/`、`tmp/`、`ros/`：本次缓存和临时目录。
- `result.json`：飞行流程是否完成；运行评估脚本后生成 `metrics.json`、`trajectory_comparison.csv`、`flight_evaluation.png`。流程完成与定位误差是否达标是两个独立结论。

运行时的 Gazebo 话题为 `/mid360/points`（`gz.msgs.PointCloudPacked`，字段 x/y/z/intensity/timestamp）和 `/mid360/imu`。ROS 2 话题为 `/livox/lidar`（官方 `livox_ros_driver2/CustomMsg`，纳秒 `offset_time`）和 `/livox/imu`（`sensor_msgs/Imu`），定位输出 `/Odometry`。内部使用隔离的 localhost ROS_DOMAIN_ID=77；不要同时运行另一套相同域的测试。`line=0` 和固定反射强度仅是仿真元数据，不代表实机回波属性；未模拟原生 Livox UDP 协议。

飞控源码 `dependencies/apm4.6.3/` 已于 2026-10-20 恢复为 [ArduPilot 官方 Copter-4.6.3](https://github.com/ArduPilot/ardupilot/releases/tag/Copter-4.6.3)，提交 `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`，包含官方固定版本的全部子模块。该副本中的自定义往返飞行、避障、SmartRTL/SITL 改动及附加测试脚本已移除。它拥有独立 Git 仓库，不再借用原工程的 Git 元数据；原 `/home/qwer/apm4.6.3` 未修改。

SITL 二进制位于 `build/apm/sitl/bin/arducopter`，重建命令为 `./scripts/build_flight.sh`。来源提交和二进制哈希记录在 `config/flight_provenance.json`；官方源码核对结果在 `config/ardupilot-restore-verification.json`，子模块清单在 `config/ardupilot-official-manifest.json`。仿真参数已移除自定义的 `LOIT_LR_EN_CH` 和 `ROBS_ENABLE`。

Python 依赖版本在 `config/flight-requirements.txt`，已隔离安装在 `.runtime/python/`。FAST-LIO 源码及 ROS 2 消息包位于 `lio_ws/src/`；本地补丁仅补齐估计速度输出、修正协方差发布时间，并将 Log/PCD 路径改为本次运行目录，差异保存在 `config/fastlio-local.patch`，未改变滤波方程。

Gazebo Ogre 2 内部固定向用户目录写渲染日志，因此启动脚本仅对 Gazebo 子进程加载 `build/libgazebo_log_redirect.so`，把该单一日志路径重定向到本次 `gazebo/ogre2.log`，不更改用户的 HOME。其源码为 `scripts/gazebo_log_redirect.c`。

## 已完成的 100 m 试验

2026-10-20 的 `runs/mid360-100m-07` 已完成起飞、直行、终点悬停和自动降落，未强制解锁。详情见 [报告](runs/mid360-100m-07/REPORT.txt)、[误差图](runs/mid360-100m-07/flight_evaluation.png) 和 [指标](runs/mid360-100m-07/metrics.json)。

这次历史试验使用恢复前的定制固件 `87ccf0c1c7…`，不能作为恢复后官方固件的飞行验证。原日志、来源记录和运行时源码快照保持不变，旧二进制另存为 `runs/mid360-100m-07/reproducibility/bin/arducopter-custom-used-in-run07`；后续运行将使用新编译的官方版本。

| 指标 | 本次结果 |
| --- | --- |
| 实际终点前进距离 | 99.980 m |
| 世界坐标最大横向偏离（至终点悬停） | 0.105 m |
| 全程三维 ATE RMSE / 最大误差，含降落 | 0.148 m / 0.630 m |
| 起飞至终点悬停 ATE RMSE | 0.123 m |
| 终点 SLAM 三维误差，首帧位姿对齐 | 0.252 m |
| 终点实际相对高度 / 目标高度 | 1.175 m / 1.500 m |
| 飞控 VISP / VISV / 位置观测拒绝 | 1446 / 1446 / 0 |
| 原始扫描帧 / 丢帧 | 1450 / 0 |

ATE 只做首帧位姿对齐，不拟合全程轨迹或尺度。实际高度采用世界竖直方向、不旋转坐标，因此与对齐后的 Z 误差不同。结果显示累积高度漂移，触地阶段轴向定位误差也增大；流程完成不代表精度已满足某项指标。此次仅验证直段、倒装雷达及理想 IMU 条件，未验证全长或弯道飞行。

本次 `.tlog` 为空，原因是 pymavlink 的 UDP 接收实现未调用其通用日志写入；飞控 BIN、JSON 遥测、点云和 IMU 完整。飞行结束后已修复后续运行的 TLOG 写入，并通过 10 条 MAVLink 消息写入/回读检查；本次原始记录未回填。运行时源码快照保留于 `reproducibility/`。导出的 `estimated_map.ply/.npy` 含 304935 个点，仅用估计位姿重建。

---
