# LidarToolkit
 LidarToolkit — 激光点云工具集合（示例：ICP 配准/合并/裁剪）

构建：
```
colcon build
```
运行程序示例：
```
ICP配准可视化：
./build/LidarToolkit/icp_test /path/to/base.ply /path/to/input.ply <迭代次数>

点云融合：
./build/LidarToolkit/merge
待融合点云文件储存在./merge中

点云裁剪：
./build/LidarToolkit/tailor <input_path>
```