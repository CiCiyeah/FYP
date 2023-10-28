import pybullet as p
import pybullet_data
import time
import math
from pybullet import getQuaternionFromEuler

def draw_axis_and_ticks():
    # 绘制坐标轴
    p.addUserDebugLine([1, 0, 0], [0, 0, 0], [0, 0, 0])  # X轴以红色表示
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 0, 0])  # Y轴以绿色表示
    p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 0, 1])  # Z轴以蓝色表示

    tick_length = 0.1
    for i in range(10):
        p.addUserDebugLine([0.1 * (i+1), -tick_length, 0], [0.1 * (i+1), tick_length, 0], [1, 0, 0])
        p.addUserDebugLine([-tick_length, 0.1 * (i+1), 0], [tick_length, 0.1 * (i+1), 0], [0, 1, 0])
        p.addUserDebugLine([-tick_length, 0, 0.1 * (i+1)], [tick_length, 0, 0.1 * (i+1)], [0, 0, 1])

# 初始化PyBullet环境
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
p.changeVisualShape(plane, -1, rgbaColor=[1, 1, 1, 1])

base_position = [0.0, 1, 0.5]
base_orientation = p.getQuaternionFromEuler([0, 0, 0])
base_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.5])
base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.5], rgbaColor=[1, 1, 1, 1])
base_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=base_shape, baseVisualShapeIndex=base_visual, basePosition=base_position)

ur5 = p.loadURDF("urdf/ur5/ur5.urdf", base_position, base_orientation, useFixedBase=True)

num_joints = p.getNumJoints(ur5)
for joint in range(num_joints):
    p.resetJointState(ur5, joint, 0)

def create_conveyor_belt(pos, length=2.0, width=0.5):
    half_length = length / 2.0
    half_width = width / 2.0
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_length, half_width, 0.02])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_length, half_width, 0.02], rgbaColor=[0.6, 0.6, 0.6, 1])
    body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=pos)
    return body

conveyor = create_conveyor_belt([0, 0.5, 0.02])

# 紫色物体源头盒子（修改为只有四个面）
box_extents = [0.25, 0.25, 0.25]
box_thickness = 0.01
positions = [
    [-0.8, 0.5, 0.5], # 顶上
    [-1.025, 0.5, 0.25], # 背面
    [-0.8, 0.25, 0.25], # right
    [-0.8, 0.75, 0.25], # back
]
for pos in positions:
    rotation = None
    if pos[2] == 0.5:  # 顶上
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]], rgbaColor=[0.5, 0, 0.5, 1])
        rotation = getQuaternionFromEuler([math.pi / 2, 0, 0])
    elif pos[1] == 0.5:  # left/right
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_thickness, box_extents[1], box_extents[2]])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_thickness, box_extents[1], box_extents[2]], rgbaColor=[0.5, 0, 0.5, 1])
    else:  # back
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]], rgbaColor=[0.5, 0, 0.5, 1])

    if rotation:
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=pos, baseOrientation=rotation)
    else:
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=pos)

# 粉色缓冲区
buffer_position = [0.5, -0.2, 0.01]
buffer_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.01])
buffer_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.01], rgbaColor=[1, 0.5, 0.5, 1])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=buffer_box, baseVisualShapeIndex=buffer_visual, basePosition=buffer_position)

# 灰色最终盒子（五块板子组成）
positions_gray = [
    [1.5, 0, 0.01], # bottom - 放平
    [1.25, 0, box_extents[2]], # left
    [1.75, 0, box_extents[2]], # right
    [1.5, 0.25, box_extents[2]],  # front
    [1.5, -0.25, box_extents[2]]  # back
]

for pos in positions_gray:
    if pos[2] == 0.01:  # bottom
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_extents[1], box_thickness])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_extents[1], box_thickness], rgbaColor=[0.5, 0.5, 0.5, 1])
    elif pos[0] == 1.25 or pos[0] == 1.75:  # left/right
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_thickness, box_extents[1], box_extents[2]])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_thickness, box_extents[1], box_extents[2]], rgbaColor=[0.5, 0.5, 0.5, 1])
    else:  # front/back
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_extents[0], box_thickness, box_extents[2]], rgbaColor=[0.5, 0.5, 0.5, 1])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=pos)
draw_axis_and_ticks()

# 用于跟踪夹取的物体数量
objects_picked = 0

# 创建物体的函数
def create_object(pos, color=[1, 0, 0, 1]):
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=color)
    body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=shape, baseVisualShapeIndex=visual, basePosition=pos)
    return body

# UR5夹取和放置的简化实现（基于位置，不涉及真正的夹子控制和动画）
def move_ur5_to_pick_and_place(object_id, place_position):
    global objects_picked
    # 提取物体当前位置
    object_pos, _ = p.getBasePositionAndOrientation(object_id)
    # 移动手臂到物体上方
    p.resetBasePositionAndOrientation(ur5, [object_pos[0], object_pos[1], 0.5], base_orientation)
    # "夹取"物体（这里简化为设置其位置）
    p.resetBasePositionAndOrientation(object_id, [object_pos[0], object_pos[1], 0.6], [0, 0, 0, 1])
    # 移动手臂到放置位置
    p.resetBasePositionAndOrientation(ur5, [place_position[0], place_position[1], 0.5], base_orientation)
    # "放下"物体
    p.resetBasePositionAndOrientation(object_id, place_position, [0, 0, 0, 1])
    # 记录夹取的物体数量
    objects_picked += 1

object_position = [-0.8, 0.5, 0.1]
next_place_position = buffer_position

# 主循环
object_ids = []

conveyor_speed = 0.02  # 传送带的速度，根据需要调整
conveyor_moving = False  # 标志传送带是否正在移动

while objects_picked < 50:
    # 生成物体
    if len(object_ids) == 0:
        obj = create_object(object_position)
        object_ids.append(obj)
        conveyor_moving = True  # 启动传送带

    # 如果传送带正在移动并且物体足够远离机械臂底座，则停止传送带
    if conveyor_moving:
        for obj in object_ids:
            obj_pos, _ = p.getBasePositionAndOrientation(obj)
            if obj_pos[0] >= 0.2:
                conveyor_moving = False
                p.resetBaseVelocity(conveyor, [0, 0, 0])  # 停止传送带

    # 检查物体是否到达 UR5 附近，若是，则夹取并放置
    for obj in object_ids:
        obj_pos, _ = p.getBasePositionAndOrientation(obj)
        if 0 <= obj_pos[0] <= 0.1:
            move_ur5_to_pick_and_place(obj, next_place_position)
            object_ids.remove(obj)
            break  # 只夹取一个物体

    p.stepSimulation()
    time.sleep(1. / 240.)


