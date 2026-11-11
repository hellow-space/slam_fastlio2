用于建图保存pcd文件
1.检查ws_livox/src/FAST-LIVO2-master/config/test.yaml中的pcd_save_en参数是否为true，允许保存
2.ws_livox/src/FAST-LIVO2-master/config/test.yaml中的dense_map_en用于决定发布的点云是否降采样，false为降采样
3.保存路径为ws_livox/src/FAST-LIVO2-master/Log/PCD/all_raw_points.pcd
4.启动
终端一：
    cd ~/wsnew/ws_livox
    source install/setup.sh
    ros2 launch fast_livo test.py use_rviz:=true

终端二：
    cd ~/wsnew/ws_livox
    source install/setup.sh
    ros2 launch livox_ros_driver2 msg_MID360_launch.py

启动完成后会开始建图，是否启动rviz可选。
5.如果参数正确，一般输入ctrl+C后会pcd会保存到对应位置。（有时可能保存失败）