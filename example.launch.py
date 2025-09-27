# ROS2 启动文件 - FAST-LIO2 重定位系统
# 该文件用于启动基于FAST-LIO的激光雷达定位系统，结合ICP重定位算法
# 实现机器人在已知地图中的精确定位

import os

# ROS2 launch 系统相关导入
from ament_index_python.packages import get_package_share_directory  # 获取包的共享目录路径
from launch import LaunchDescription  # 启动描述类
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # 启动动作
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node  # ROS节点启动类
from launch.substitutions import LaunchConfiguration  # 启动配置替换

def generate_launch_description():
  """
  生成ROS2启动描述
  主要功能：
  1. 启动坐标变换发布器
  2. 启动ICP重定位节点
  3. 启动FAST-LIO定位节点
  4. 启动RViz2可视化界面
  """

  # 获取fast_lio包的配置文件路径
  config_path = os.path.join(
      get_package_share_directory('fast_lio'), 'config') 

  # ========== ICP重定位模块 ==========
  
  # 坐标变换发布器节点
  # 功能：发布map坐标系与odom坐标系之间的变换关系
  # 作用：维护全局地图坐标系和里程计坐标系的连接
  map_odom_trans = Node(
      package='icp_relocalization',        # 所属包名
      executable='transform_publisher',    # 可执行文件名
      name='transform_publisher',          # 节点名称
      output='screen'                      # 输出到屏幕
  )

  # ICP重定位核心节点
  # 功能：使用ICP(Iterative Closest Point)算法进行点云匹配，实现机器人在全局地图中的重定位
  # 工作原理：将当前激光雷达点云与预先构建的全局地图进行匹配，估计机器人在地图中的精确位置
  icp_node = Node(
      package='icp_relocalization',        # 所属包名
      executable='icp_node',               # 可执行文件名
      name='icp_node',                     # 节点名称
      output='screen',                     # 输出到屏幕
      parameters=[
          # === 初始位置参数 ===
          {'initial_x':0.0},               # 机器人初始X坐标 (米)
          {'initial_y':0.0},               # 机器人初始Y坐标 (米)
          {'initial_z':0.0},               # 机器人初始Z坐标 (米)
          {'initial_a':0.0},               # 机器人初始航向角 (弧度)

          # === 点云处理参数 ===
          {'map_voxel_leaf_size':0.5},     # 地图点云体素滤波尺寸 (米) - 降采样减少计算量
          {'cloud_voxel_leaf_size':0.3},   # 当前点云体素滤波尺寸 (米) - 降采样减少计算量
          {'map_frame_id':'map'},          # 地图坐标系ID
          
          # === ICP算法参数 ===
          {'solver_max_iter':100},         # ICP最大迭代次数 - 控制算法收敛性能
          {'max_correspondence_distance':0.1},  # 最大对应点距离 (米) - 超过此距离的点不参与匹配
          {'RANSAC_outlier_rejection_threshold':0.5},  # RANSAC外点剔除阈值 - 提高匹配鲁棒性
          
          # === 地图文件路径 ===
          # {'map_path':'/home/sentry_ws/src/sentry_bringup/maps/CC#0.pcd'},  # 备用地图路径
          {'map_path':'/home/getting/fast_lio2_relocation2_ws/test.pcd'},   # 全局地图PCD文件路径
          
          # === 收敛判断参数 ===
          {'fitness_score_thre':0.2},      # 适应度得分阈值 - 最近点距离的平均值，越小匹配越严格
          {'converged_count_thre':40},     # 收敛计数阈值 - 连续收敛次数 (点云发布频率20Hz，约2秒)
          {'pcl_type':'livox'},            # 点云类型 - 适配Livox激光雷达数据格式
      ],
  )
  
  # ========== FAST-LIO定位模块 ==========
  
  # 获取FAST-LIO配置文件路径
  fast_lio_param = os.path.join(
      config_path, 'fast_lio_relocalization_param.yaml')
  
  # FAST-LIO核心节点
  # 功能：基于紧耦合的激光-惯性里程计，提供高频、高精度的状态估计
  # 工作原理：融合IMU和激光雷达数据，通过卡尔曼滤波实现实时定位和建图
  # 特点：计算效率高，适合实时应用，在纹理缺乏环境下表现优异
  fast_lio_node = Node(
      package='fast_lio',                 # 所属包名
      executable='fastlio_mapping',       # 可执行文件名
      parameters=[
          fast_lio_param                   # 从YAML文件加载详细参数配置
      ],
      output='screen',                     # 输出到屏幕
      remappings=[
          ('/Odometry','/state_estimation') # 话题重映射：将里程计话题重命名为状态估计话题
      ]
  )
  
  # ========== 可视化模块 ==========
  
  # 获取RViz配置文件路径
  rviz_config_file = os.path.join(
    get_package_share_directory('icp_relocalization'), 'rviz', 'loam_livox.rviz')
    
  # RViz2可视化节点
  # 功能：提供3D可视化界面，显示点云、轨迹、坐标变换等信息
  # 作用：帮助用户直观观察定位效果和系统运行状态
  start_rviz = Node(
    package='rviz2',                     # RViz2包
    executable='rviz2',                  # RViz2可执行文件
    arguments=[
        '-d', rviz_config_file,          # 加载预设的可视化配置文件
        '--ros-args', '--log-level', 'warn'  # 设置日志级别为警告，减少控制台输出
    ],
    output='screen'                      # 输出到屏幕
  )

  # ========== 启动时序控制 ==========
  
  # 延迟启动定位模块
  # 原因：需要等待坐标变换发布器和RViz初始化完成，确保系统稳定启动
  # 延迟时间：5秒 - 给基础模块充足的初始化时间
  delayed_start_lio = TimerAction(
    period=5.0,                          # 延迟5秒启动
    actions=[
      icp_node,                          # ICP重定位节点
      fast_lio_node                      # FAST-LIO定位节点
    ]
  )

  # ========== 启动描述构建 ==========
  
  # 创建启动描述对象
  ld = LaunchDescription()

  # 按顺序添加启动动作
  # 1. 首先启动坐标变换发布器 - 建立坐标系基础
  ld.add_action(map_odom_trans)
  
  # 2. 启动可视化界面 - 便于观察系统状态
  ld.add_action(start_rviz)
  
  # 3. 延迟启动核心定位模块 - 确保基础设施就绪
  ld.add_action(delayed_start_lio)

  return ld
