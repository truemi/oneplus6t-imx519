# IMX519 纯直驱清理结果

设备已切换为纯原生 IMX519 相机链路，管理器不再启动 android_hal_manager，camerad 启动器没有 HAL 回退代码。固定焦点 DAC 1366、1920×1080 RAW10 传感器模式、IFE 输出 1280×720 NV12、20 FPS、实测 GPU 去畸变及模型/显示内参保持不变。

生产采集不再以 IsOnroad/IsEngaged 为退出或启动阻断条件，状态切换时相机继续由管理器管理。私有标定模式仍只允许停车。原生相机异常退出或心跳超时，由启动器退避重启原生采集，最长间隔 8 秒，不启动其他后端，不自动重启设备。

Python 模型检测 IPC 服务器变更后退出，由原有 manager 重启，以重置模型时序状态，避免一直读取旧缓冲。UI 已有断线重连机制。当前 ModelRunnerTypeCache=2，即 stock 模型，已核对其内参接入路径。

## 已从设备移除

- 相机 HAL 管理器源码及 Python 缓存、manager 启动项。
- hal3_direct、旧 camerad_hal_v2、camera_android_hal 源码/对象文件及旧 HAL 设置脚本。
- hal_camera 共享帧文件、SSC stub、旧 HAL 日志和切换脚本。
- 含旧相机 HAL 的设备备份目录、旧安装包/安装目录及依赖 HAL 的一次性调试脚本。
- 原有自动恢复 HAL 的 launcher 已被纯直驱启动器替换。默认相机 SConscript 现在只构建 IMX519 原生目标。

旧文件删除前已在电脑上保存并校验恢复归档：`firmware/imx519_port/native_only_20261029/imx519_retired_hal_20261029.tar.gz`，SHA256 `af028b0f2c1e98c1db5ebc8712b34c78619ae4f65072bc7708003f866deb39e1`。归档本身也已从设备删除；电脑上的历史交付包不作为设备运行依赖。旧诊断脚本另存 retired_helpers 归档。

Android vendor 目录、binder/显示基础服务和 GPU 库仍被显示、OpenGL/OpenCL 等共用，未删除整个 Android 底层目录。相机原生采集不加载 camera.qcom.so、libcamera_metadata 或 CHI 模块。

## 实机检查

1. 原生启动器真实启动，重启前 100 帧有效；对采集子进程实际发送 SIGKILL 后，启动器拉起新 PID，重启后 100 帧全部有效。两个稳定采集段内均无帧缺口；故障期间存在中断，不能把它称为全程零中断。
2. 两个采集段 SOF 至客户端中位延迟分别 9.193ms、9.196ms。旧客户端检测到了 IPC 服务器变更；新客户端成功连接，12 帧真实模型 OpenCL 预处理通过。
3. 删除旧 HAL 文件后再次从实际 launcher 启动：119/119 帧有效、20.003 FPS、无帧缺口；中位 9.184ms，95 分位 9.243ms。
4. 正常 SCons 构建路径 dry-run 成功，原生目标为最新；Python 启动配置和模型代码语法检查通过。

新原生程序 SHA256：`1697dcdc1ce86601bb7fc2a9170afde35fc259263c6141fa5e93f656d1dd65f4`。

整个检查期间 IsOnroad/IsEngaged 保持真实的 0，没有伪造行车状态，没有启动控制程序或重启设备。生产路径现已允许行车状态持续采集，但尚未实际道路验证。主摄模拟辅助宽视角时约 18% 的输入区域缺少真实视场，该已知限制仍存在；未启用 IMX376K 双摄。

## 当前交付

`imx519_native_only_20261029.zip` 包含当前纯直驱设备文件、相机源码、固定焦点标定、运行库、模型/显示配置及验证记录。设备已完成清理和部署，不需再次安装。

解压到设备后可执行 `python3 install_imx519_native_only_snapshot.py --check` 做只读校验。`sudo python3 install_imx519_native_only_snapshot.py --install` 只用于相同 openpilot 基础版本、已清除旧 HAL、停车且停止应用/相机的设备，恢复本次文件快照；不是其他手机或分支的通用安装器。历史 HAL 回滚脚本已不再适用于本次设备状态。
