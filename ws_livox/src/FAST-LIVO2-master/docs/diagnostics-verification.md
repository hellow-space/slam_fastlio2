# 诊断补丁验证记录（2026-11-12）

范围：只对定位包增加诊断；未启动定位/导航节点、未发送运动指令、未采集新的实机定位数据。

## 已完成

- `python3 -m unittest discover -s tests -v`：3项测试通过。原先缺少记录器/汇总器时观察到失败；新增状态分组/乱序样本用例先观察到失败，修正后通过。
- 记录器测试使用 `-O3 -ffast-math`，覆盖1200行缓冲切换无丢失、缺失值、回调无处理帧归属、处理前后状态时间、提前返回、不可写目录、模拟最终缓冲写盘失败。
- 写盘失败曾复现 `flush()` 关闭文件后 `close()` 再次 `fclose(nullptr)` 的崩溃，已增加检查并通过回归测试。
- 汇总测试覆盖 nearest-rank P95、空/非有限样本、成功与初始化分组，以及嵌套 scope 按结束顺序写出时的启动窗口过滤。
- ON 构建成功：`/home/nvidia/.cache/fastlivo_timing_build/on/`；日志 `/tmp/fastlivo_timing_build_on_verified.log`。
- OFF 构建成功：`/home/nvidia/.cache/fastlivo_timing_build/off/`；日志 `/tmp/fastlivo_timing_build_off_verified.log`。
- 两种构建均有原环境 OpenCV 4.5/4.10 混合链接警告；未修改这些依赖。
- 删除诊断条件块和空诊断宏后，LIVMapper.cpp/.h、voxel_map.cpp/.h 的原始 C++ token 序列与 `ac47d06` 一致（仅归一化单条return的新增花括号）。
- 诊断开关默认 OFF；二进制检查确认 ON 包含诊断入口字符串，OFF 不包含。
- ON 独立安装到 `/home/nvidia/.cache/fastlivo_timing_install/on/`，`ros2 pkg prefix fast_livo` 在该前缀下解析正确，ldd 未发现缺失依赖；安装的 nav_test.yaml/nav_test.py 与原源码一致。
- 原安装程序和两项工作区内动态依赖的 SHA256 与备份完全一致。
- `git diff --check` 通过。

## 独立审查

一次只读独立审查未发现 Critical，提出两项 Important：回调被误关联为上一处理帧、汇总混合初始化与成功周期。均已修复并加入回归用例。回调现在 frame_id=0，使用loop与消息时间关联；IMU处理单列 state_before_s，结束后填写scan_end_s；统计提供by_status分组。

同时修正：启动过滤采用最早事件开始时间、零跳过不过滤嵌套外层；编译定义使用兼容最低CMake版本的写法。没有遗留的本次审查问题。

## 尚需实机验证

ON/OFF 总诊断扰动、室外各阶段实际耗时、队列积压位置及扫描年龄增长。编译和离线测试不构成性能或定位精度验收；不能据此认定具体阶段是故障根因。操作和对照方法见 `timing-diagnostics.md`。
