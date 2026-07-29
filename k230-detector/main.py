# -*- coding: utf-8 -*-
# 钢球位置检测 — 立创庐山派 K230
# 方案：OTSU自适应二值化（ROI内自动分离钢球与背景）
#       绿色矩形=凹槽ROI → OTSU二值化 → 最大色块=钢球 → UART
# 输出：UART 偏移百分比 给主控

import os, gc, time, sys
import cv2
import image
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA
from ulab import numpy as np

# ===== 可调参数 =====
CAMERA_WIDTH  = 320
CAMERA_HEIGHT = 240

# 凹槽ROI矩形
ROI_X = 10
ROI_Y = 110             # 画面中间
ROI_W = 300
ROI_H = 20              # 中间20行

# OTSU模式：False=球亮背景暗(THRESH_BINARY), True=球暗背景亮(THRESH_BINARY_INV)
INVERT = False

# 面积过滤
MIN_AREA = 5            # 最小面积（过滤噪点）
MAX_AREA = 30           # 最大面积（过滤凹槽外壁）

# 形状过滤（排除条状反光，保留近圆形钢球）
MIN_ASPECT = 0.4        # 最小宽高比（w/h），条状<0.4被排除
MAX_ASPECT = 2.5        # 最大宽高比（w/h），条状>2.5被排除

# 凹槽中心X
GROOVE_CENTER_X = ROI_X + ROI_W // 2
HALF_WIDTH      = ROI_W // 2

# UART
UART_BAUD   = 115200
UART_TX_PIN = 5
UART_RX_PIN = 6
# ======================

ROI_END_X = ROI_X + ROI_W
ROI_END_Y = ROI_Y + ROI_H

if INVERT:
    OTSU_MODE = cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
else:
    OTSU_MODE = cv2.THRESH_BINARY + cv2.THRESH_OTSU

sensor = None
uart = None


def find_ball(img_np):
    """ROI内OTSU二值化+找钢球色块，返回(cx,cy,rx,ry,rw,rh,area)或None"""
    roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]
    _, binary = cv2.threshold(roi, 0, 255, OTSU_MODE)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 筛选：面积在范围内 + 不贴ROI边缘（排除外壁）
    best_cnt, best_area = None, 0
    best_x = best_y = best_w = best_h = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = cv2.contourArea(cnt)

        # 面积过滤
        if a < MIN_AREA or a > MAX_AREA:
            continue

        # 贴边过滤（外壁一定挨着ROI上下边缘）
        if y <= 1 or y + h >= ROI_H - 1:
            continue

        # 形状过滤（条状反光 vs 紧凑钢球）
        if h > 0:
            aspect = float(w) / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                continue

        if a > best_area:
            best_area = a
            best_cnt = cnt
            best_x, best_y, best_w, best_h = x, y, w, h

    if best_cnt is None:
        return None

    cx = ROI_X + best_x + best_w // 2
    cy = ROI_Y + best_y + best_h // 2
    rx = ROI_X + best_x
    ry = ROI_Y + best_y
    return cx, cy, rx, ry, best_w, best_h, best_area


try:
    # --- 摄像头 ---
    sensor = Sensor(width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)

    # --- LCD ---
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)

    # --- UART ---
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, fpioa.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, fpioa.UART2_RXD)
    uart = UART(UART.UART2, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    # --- 启动 ---
    MediaManager.init()
    sensor.run()
    time.sleep(0.5)
    clock = time.clock()

    print("钢球检测器启动完成（OTSU模式）")

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        # === 检测 ===
        result = find_ball(img_np)

        if result is not None:
            ball_x, ball_y, rx, ry, rw, rh, area = result
            offset_pct = (ball_x - GROOVE_CENTER_X) / HALF_WIDTH * 100.0
            uart.write("%.1f\n" % offset_pct)

            img.draw_rectangle(rx, ry, rw, rh,
                               color=(255, 255, 255), thickness=2)
            img.draw_cross(ball_x, ball_y, color=(255, 255, 255),
                           size=8, thickness=2)
            status = "%+.1f%% A:%d" % (offset_pct, area)
            sc = (0, 255, 0)
        else:
            status = "NO BALL"
            sc = (255, 0, 0)

        # === 显示 ===
        # 上屏：原图 + ROI + 检测
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X, ROI_END_Y,
                      color=(128, 128, 128), thickness=1)
        img.draw_string_advanced(2, 2, 14,
                                 "FPS:%.1f %s" % (clock.fps(), status),
                                 color=sc)

        top_rgb565 = img.to_rgb565()

        # 下屏：OTSU二值图（原始尺寸）
        roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]
        _, binary_np = cv2.threshold(roi, 0, 255, OTSU_MODE)
        binary_img = image.Image(ROI_W, ROI_H, image.GRAYSCALE,
                                 alloc=image.ALLOC_REF, data=binary_np)

        bar_canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT, image.RGB565)
        bar_canvas.draw_image(binary_img.to_rgb565(), ROI_X, ROI_Y)
        bar_canvas.draw_string_advanced(2, 4, 14,
                                        "OTSU %s" %
                                        ("INV" if INVERT else "BIN"),
                                        color=(255, 255, 0))

        # 拼接
        canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT * 2, image.RGB565)
        canvas.draw_image(top_rgb565, 0, 0)
        canvas.draw_image(bar_canvas, 0, CAMERA_HEIGHT)

        Display.show_image(canvas)

        del img_np, roi, binary_np, binary_img, top_rgb565, bar_canvas, canvas
        gc.collect()

except KeyboardInterrupt:
    print("用户停止")
except BaseException as e:
    print("异常: %s" % str(e))
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    print("资源已释放")
