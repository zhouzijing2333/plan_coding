import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Environment3D
# │
# ├── generate_buildings()   # 随机建筑
# ├── generate_spheres()     # 球形障碍物
# ├── generate_cylinders()   # 圆柱（树木、电线杆）
# ├── save_map()             # 保存地图
# ├── load_map()             # 读取地图
# ├── is_collision()         # 碰撞检测
# ├── is_outside()           # 越界检测
# └── visualize()            # 三维显示




# #  AStar()
# 定义节点---
# 初始化OPEN---初始化CLOSED---OPEN加入Start

# while OPEN非空
#     取f最小节点
#     if Goal
#         回溯路径
#     else
#         for 邻居
#             判断障碍
#             计算g
#             计算h
#             计算f
#             更新父节点
#             放入OPEN
# return Path

#定义节点（Node）
class Node:
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z

        self.g=float('inf') #起点到当前节点的代价
        self.h=0 #当前节点到终点的估计代价
        self.f=float('inf') #总代价
        
        self.parent=None #父节点
    def __eq__(self,other):
        return(
            self.x==other.x and
            self.y==other.y and
            self.z==other.z
        )


def get_neighbors(node):
    neighbors=[]
    #定义26邻域邻居点
    for dx in [-1,0,1]:
        for dy in [-1,0,1]:
            for dz in [-1,0,1]:
                if dx==0 and dy==0 and dz==0:
                    continue
                nx=node.x+dx
                ny=node.y+dy
                nz=node.z+dz
                #检查边界
                if (0<=nx<100 and 0<=ny<100 and 0<=nz<30):
                    neighbors.append(Node(nx,ny,nz))
    return neighbors

#初始化3D环境
grid=np.zeros((30,100,100),dtype=np.uint8)
#设置10个建筑物障碍物
for i in range(10):
    x=np.random.randint(1,89)
    y=np.random.randint(1,89)
    w=np.random.randint(4,10)
    l=np.random.randint(8,10)
    h=np.random.randint(8,20)
    grid[0:h,y:y+w,x:x+l]=1
np.save("data/city_map.npy",grid)

   
# 建立open和closed列表
open_list=[]  #未探索的节点
closed_list=[]#已探索过的节点

#确定起点和终点
start=Node(0,0,0)
goal=Node(99,99,29)
start.g=0
start.h=math.sqrt((goal.x-start.x)**2+(goal.y-start.y)**2+(goal.z-start.z)**2)
start.f=start.h+start.g
start.parent=None
goal.g=0
goal.h=0
goal.f=goal.h+goal.g

#OPEN加入Start
open_list.append(start)

print("开始循环")
print('循环中')
# 开始循环
while len(open_list)>0:
    # 取最小的F值节点
    currtent=min(open_list,key=lambda node:node.f)
    open_list.remove(currtent)
    closed_list.append(currtent)
    if currtent.x==goal.x and currtent.y==goal.y and currtent.z==goal.z:
        print("找到终点")
        path=[]
        while currtent is not None:
            path.append((currtent.x,currtent.y,currtent.z))
            currtent=currtent.parent
        path.reverse()
        path
        break
    else:
        neighbors=get_neighbors(currtent)
        for i in neighbors:
            if grid[i.z,i.y,i.x]==1:
                # neighbors.remove(i)不能直接删除，会破坏列表的循环指针
                continue
            else:
                if grid[i.z,i.y,i.x]==0 : #认为是可行路径
                    #需要判断邻居是否已经访问过
                    if i in closed_list :
                        continue
                    nx=i.x
                    ny=i.y
                    nz=i.z
                    g=currtent.g+math.sqrt((nx-currtent.x)**2+(ny-currtent.y)**2+(nz-currtent.z)**2)
                    h=math.sqrt((goal.x-nx)**2+(goal.y-ny)**2+(goal.z-nz)**2)
                    f=g+h
                    if i in open_list:
                        if g<i.g:
                            i.g=g
                            i.h=h
                            i.f=f
                            i.parent=currtent
                    else:
                        i.parent=currtent 
                        i.g=g
                        i.h=h
                        i.f=f
                        open_list.append(i)

print("循环结束")
#可视化3D环境
obstacle=np.argwhere(grid==1)

z=obstacle[:,0]
y=obstacle[:,1]
x=obstacle[:,2]

fig=plt.figure(figsize=(12,10))
ax=fig.add_subplot(111,projection='3d')
#绘制障碍物
ax.voxels(
    grid.transpose(2,1,0),
    facecolors='burlywood',#表面颜色
    edgecolor='gray', #描边效果
    alpha=0.7
)   

#可视化路线
ax.scatter(
    start.x,start.y,start.z,
    c='green',s=100,label='start'
) #s为点面积
ax.scatter(
    goal.x,goal.y,goal.z,
    c='red',s=100,label='goal'
)
if 'path' in locals(): #防止path不存在
    path=np.array(path) #列表变为矩阵
    ax.plot(
        path[:,0],
        path[:,1],
        path[:,2],
        c='blue',
        linewidth=3,
        label="A*path"
    )
ax.view_init(elev=90, azim=-90)
plt.legend()
plt.show()



    

