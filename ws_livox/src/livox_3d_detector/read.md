cd ~/wsnew/ws_livox
source ~/wsnew/ws_livox/install/setup.bash

ros2 launch livox_3d_detector obstacle_avoidance.py


ros2 topic list | grep processed
ros2 topic list | grep obstacle_avoidance
ros2 topic echo /livox/processed_final

# 话题
/livox/processed_final          <-- 最终处理后点云
/obstacle_avoidance/cloud_obstacles  障碍物点云
/obstacle_avoidance/cloud_ground    地面点云
/obstacle_avoidance/cloud_roi       滤波后点云
/obstacle_avoidance/markers         障碍物包围盒
## 点云数据
    header:          # 点云的头信息（时间、坐标系）
    seq: 743         # 帧序号（一直在增加，说明在持续发数据）
    stamp:           # 时间戳
    frame_id: "livox_frame"  # 坐标系（正确）
    height: 1
    width: 6         # ✅ 这一帧有 **6个点**
    data: "<...>"    # 点云的原始二进制数据（不用管）
