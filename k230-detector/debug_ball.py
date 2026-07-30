# -*- coding: utf-8 -*-
# 钢球检测调试脚本 — 形态学校验调参
# 上屏：摄像头+检测结果  下屏：二值化图像
# 和main.py一致的处理管线（HSV绿掩码+OTSU+过滤+腐蚀回退）

import os, gc, time, sys
import cv2
import image
from media.sensor import *
from media.display import *
from media.media import *
from ulab import numpy as np

# ===== 可调参数（与main.py一致）=====
CAMERA_WIDTH  = 320
CAMERA_HEIGHT = 240

ROI_X = 10
ROI_Y = 145
ROI_W = 300
ROI_H = 15

# HSV 绿色阈值 — 手动填入标定结果
COLOR_LO1 = np.array([37, 139, 96], dtype=np.uint8)
COLOR_HI1 = np.array([58, 225, 234], dtype=np.uint8)

# 形态学核大小（调试这个值，1=不用，3=3x3椭圆，5=5x5椭圆）
MORPH_KSIZE = 5
MORPH_ITERS = 1

# 滤波参数
THRESH_ALPHA = 0.05
MIN_AREA = 10
MAX_AREA = 250
MIN_ASPECT = 0.6
MAX_ASPECT = 1.7

GROOVE_CENTER_X = ROI_X + ROI_W // 2
# ==============================

ROI_END_X = ROI_X + ROI_W
ROI_END_Y = ROI_Y + ROI_H

buf_hsv = buf_gray = buf_green = buf_ngreen = buf_masked = None
buf_open = morph_kernel = None


def alloc_buf():
    global buf_hsv, buf_gray, buf_green, buf_ngreen, buf_masked
    global buf_open, morph_kernel
    buf_hsv    = np.zeros((ROI_H, ROI_W, 3), dtype=np.uint8)
    buf_gray   = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_green  = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_ngreen = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_masked = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_open   = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                              (1, MORPH_KSIZE))  # 横向核 (1,N)


def find_ball(img_np, smooth_th):
    """和main.py一致的管线，返回 (result, stats, th, binary, opened_binary, morph_used)"""
    roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]

    cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=buf_hsv)
    cv2.inRange(buf_hsv, COLOR_LO1, COLOR_HI1, dst=buf_green)
    cv2.bitwise_not(buf_green, dst=buf_ngreen)

    cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY, dst=buf_gray)
    cv2.bitwise_and(buf_gray, buf_ngreen, dst=buf_masked)

    retval, _ = cv2.threshold(buf_masked, 0, 255,
                               cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if smooth_th < 0:
        smooth_th = float(retval)
    else:
        smooth_th = THRESH_ALPHA * float(retval) + (1.0 - THRESH_ALPHA) * smooth_th

    th = int(smooth_th)
    _, binary = cv2.threshold(buf_masked, th, 255, cv2.THRESH_BINARY)

    # 全程形态学
    cv2.morphologyEx(binary, cv2.MORPH_OPEN, morph_kernel,
                     dst=buf_open, iterations=MORPH_ITERS)
    best = _contour_pass(buf_open)

    if best is not None:
        cx, cy, best_area = best
        stats = (0, 0, 0, 1)
        return (cx, cy, best_area), stats, th, binary, buf_open
    else:
        return None, (0, 0, 0, 0), th, binary, buf_open


def _contour_pass(src):
    """对二值图跑一次轮廓分析，返回(cx, cy, area)或None"""
    contours, _ = cv2.findContours(src, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    best_area = 0
    best_cx = best_cy = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = cv2.contourArea(cnt)
        if a < MIN_AREA or a > MAX_AREA:
            continue
        is_ball_shape = True
        if a > 12 and h > 0:
            aspect = float(w) / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                is_ball_shape = False
        if not is_ball_shape:
            edge_tb = (y <= 0 and y + h >= ROI_H - 1)
            edge_lr = (x <= 0 and x + w >= ROI_W - 1)
            if edge_tb or edge_lr:
                continue
        if not is_ball_shape:
            continue
        if a > best_area:
            best_area = a
            best_cx = ROI_X + x + w // 2
            best_cy = ROI_Y + y + h // 2

    if best_area > 0:
        return (best_cx, best_cy, best_area)
    return None


# ===== 主程序 =====

sensor = None

try:
    sensor = Sensor(width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)

    MediaManager.init()
    sensor.run()
    time.sleep(0.5)
    alloc_buf()
    clock = time.clock()
    smooth_th = -1.0

    print("调试脚本启动 横向核:(1,%d) 迭代:%d" % (MORPH_KSIZE, MORPH_ITERS))

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        result, stats, th, binary_np, opened_np = find_ball(img_np, smooth_th)

        # ---- 上屏：检测结果 ----
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X, ROI_END_Y,
                      color=(128, 128, 128), thickness=1)

        if result is not None:
            cx, cy, area = result
            offset_pct = (cx - GROOVE_CENTER_X) / (ROI_W // 2) * 100.0
            img.draw_cross(cx, cy, color=(255, 255, 255), size=8, thickness=2)
            tag = "%.1f%% A=%d" % (offset_pct, area)
            sc = (0, 255, 0)
        else:
            tag = "NO BALL"
            sc = (255, 0, 0)

        img.draw_string_advanced(2, 2, 14,
                                 "%.1ffps %s" % (clock.fps(), tag), color=sc)
        img.draw_string_advanced(2, 18, 12,
                                 "k=%d it=%d" % (MORPH_KSIZE, MORPH_ITERS),
                                 color=(255, 255, 0))

        # ---- 下屏：开运算后的二值化 ----
        bot_img = image.Image(ROI_W, ROI_H, image.GRAYSCALE,
                              alloc=image.ALLOC_REF, data=opened_np)
        bot_label = "T:%d OPEN H(1,%d)" % (th, MORPH_KSIZE)

        # 画布
        top = img.to_rgb565()
        bot = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT, image.RGB565)
        bot.draw_rectangle(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT,
                           color=(0, 0, 0), fill=True)
        bot.draw_image(bot_img.to_rgb565(), ROI_X, ROI_Y)
        bot.draw_string_advanced(2, 4, 14, bot_label, color=(255, 255, 0))
        canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT * 2, image.RGB565)
        canvas.draw_image(top, 0, 0)
        canvas.draw_image(bot, 0, CAMERA_HEIGHT)
        Display.show_image(canvas)

        gc.collect()

except KeyboardInterrupt:
    print("用户停止")
except BaseException as e:
    import sys
    sys.print_exception(e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    print("资源已释放")
