import cv2
import numpy as np

# 创建一个空白图像（例如 512x512 大小，3 通道的彩色图像）
image = np.zeros((512, 512, 3), dtype=np.uint8)

# 定义两条直线的起点和终点
x1=74
y1=387
x2=60
y2=414
x3=23
y3=349
x4=42
y4=372
line1_start = (x1, y1)
line1_end = (x2, y2)
line2_start = (x3, y3)
line2_end =  (x4, y4)
# line1_start = (120, 150)
# line1_end = (285, 390)
# line2_start = (150, 50)
# line2_end = (100, 400)

# 颜色 (BGR 格式)
color1 = (0, 250, 50)  # 绿色
color2 = (50, 0, 250)  # 红色

# 线条宽度
thickness = 2

# 绘制第一条直线
cv2.line(image, line1_start, line1_end, color1, thickness)

# 绘制第二条直线
cv2.line(image, line2_start, line2_end, color2, thickness)


# 计算交点
def find_intersection(line1_start, line1_end, line2_start, line2_end):
    x1, y1 = line1_start
    x2, y2 = line1_end
    x3, y3 = line2_start
    x4, y4 = line2_end

    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)

    if denom == 0:
        return None  # 平行无交点

    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom

    ub = ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    if ua > 0 and ua < 1:
        print("use ua")
        x = x1 + ua * (x2 - x1)
        y = y1 + ua * (y2 - y1)
    elif ub > 0 and ub < 1:
        print("use ub")
        x = x3 + ub * (x4 - x3)
        y = y3 + ub * (y4 - y3)
    else:
        x = (x4 + x2) / 2.0
        y = (y2 + y2) / 2.0
        print("error")

    return int(x), int(y)


intersection = find_intersection(line1_start, line1_end, line2_start, line2_end)

if intersection:
    cv2.circle(image, intersection, 5, (250, 150, 0), -1)  # 绘制交点
    print("交点坐标:", intersection)
else:
    print("直线平行，无交点")

# 显示图像
cv2.imshow('Image with Lines and Intersection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存图像
cv2.imwrite('lines_with_intersection.png', image)