import cv2
import numpy as np
import matplotlib.pyplot as plt

# 全局变量
img = None
gray_img = None
img_copy = None

# 鼠标事件回调函数
def click_event(event, x, y, flags, params):
    global img_copy, gray_img
    
    # 检查是否是左键点击事件
    if event == cv2.EVENT_LBUTTONDOWN:
        # 1. 捕获点击坐标
        clicked_x, clicked_y = x, y
        
        # 重置图像副本，以便清除上一次的标记
        img_feedback = img_copy.copy()
        
        # 在图像上绘制标记点和十字线
        cv2.line(img_feedback, (x, 0), (x, img_feedback.shape[0]), (0, 255, 255), 1) # 垂直线 (黄色)
        cv2.line(img_feedback, (0, y), (img_feedback.shape[1], y), (0, 255, 255), 1) # 水平线 (黄色)
        cv2.circle(img_feedback, (x, y), 5, (0, 0, 255), -1) # 标记点 (红色)
        
        # 更新窗口显示
        cv2.imshow('Image Picker and Discrete Profile Tool', img_feedback)
        
        # 打印点击点的坐标和灰度值
        pixel_bgr = img[y, x]
        pixel_gray = gray_img[y, x]
        
        print(f"\n======================================")
        print(f"✅ 点击的绝对坐标（X, Y）: ({clicked_x}, {clicked_y})")
        print(f"   该点的 BGR 颜色值: ({pixel_bgr[0]}, {pixel_bgr[1]}, {pixel_bgr[2]})")
        print(f"   该点的灰度值: {pixel_gray}")
        print(f"======================================")
        
        print("提示: 按键盘上任意键（非 ESC），将基于该点击点生成**离散点图**，方便分析边界。")
        
        # 等待按键，确认点位并生成图表
        key = cv2.waitKey(0) 
        
        if key == 27: # 如果按了 ESC 键则退出
             cv2.destroyAllWindows()
             return

        # 2. 提取并绘制剖面图
        
        # --- 垂直剖面 (使用点击的 X 坐标) ---
        vertical_profile_data = gray_img[:, clicked_x]
        
        plt.figure(figsize=(8, 10))
        # 关键更改: 使用 marker='o' 和 linestyle='None' 绘制离散点图
        plt.plot(vertical_profile_data, range(len(vertical_profile_data)), marker='o', linestyle='None', markersize=3, color='blue') 
        plt.title(f'Discrete Vertical Intensity Profile at X={clicked_x}')
        plt.xlabel('Intensity Value (0-255)')
        plt.ylabel('Absolute Y-Coordinate')
        plt.gca().invert_yaxis() 
        plt.grid(True)
        plt.show()
        
        # --- 水平剖面 (使用点击的 Y 坐标) ---
        horizontal_profile_data = gray_img[clicked_y, :]
        
        plt.figure(figsize=(12, 5))
        # 关键更改: 使用 marker='o' 和 linestyle='None' 绘制离散点图
        plt.plot(horizontal_profile_data, marker='o', linestyle='None', markersize=3, color='blue')
        plt.title(f'Discrete Horizontal Intensity Profile at Y={clicked_y}')
        plt.xlabel('Absolute X-Coordinate')
        plt.ylabel('Intensity Value (0-255)')
        plt.grid(True)
        plt.show()
        
        print("\n图表已生成。请在图像窗口中再次点击，以选择下一个分析点或按 ESC 退出。")
        
        # 重置图像显示，准备下一次选择
        cv2.imshow('Image Picker and Discrete Profile Tool', img_copy)


# --- 主程序 ---
# 替换为您的图片文件名
image_path = 'example.png' 

# 读取图像
img = cv2.imread(image_path)

if img is None:
    print(f"错误：无法读取图片文件 {image_path}。请检查文件名和路径。")
else:
    # 准备图像副本和灰度图
    img_copy = img.copy()
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 创建窗口并设置鼠标回调函数
    cv2.namedWindow('Image Picker and Discrete Profile Tool', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Image Picker and Discrete Profile Tool', click_event)

    print("--- 交互式离散点剖面工具已启动 ---")
    print("1. 单击图像中的任意点，终端会打印该点的坐标。")
    print("2. 按键盘任意键，将生成离散点图（散点图）。")
    print("3. 按 ESC 键退出程序。")
    
    # 显示图像
    cv2.imshow('Image Picker and Discrete Profile Tool', img_copy)

    # 等待按键，直到按下 ESC 键 (ASCII 27)
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            break

    cv2.destroyAllWindows()