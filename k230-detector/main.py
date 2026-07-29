# -*- coding: utf-8 -*-
# 钢球位置检测 — 立创庐山派 K230
# 方案：HSV绿色掩码 + OTSU（排除绿色凹槽，只留非绿色钢球反光）
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

ROI_X = 10
ROI_Y = 110
ROI_W = 300
ROI_H = 20

# 绿色HSV阈值（用阈值测试.py标定后填入）
# 格式：np.array([H, S, V], dtype=np.uint8), H:0-180 S:0-255 V:0-255
COLOR_LO1 = np.array([40, 113, 107], dtype=np.uint8)
COLOR_HI1 = np.array([60, 225, 214], dtype=np.uint8)
# 双区间（绿色一般不跨边界，保留备用）
USE_DUAL = False
COLOR_LO2 = np.array([0, 0, 0], dtype=np.uint8)
COLOR_HI2 = np.array([0, 0, 0], dtype=np.uint8)

THRESH_ALPHA = 0.05

MIN_AREA = 1
MAX_AREA = 100
MIN_ASPECT = 0.3
MAX_ASPECT = 3

GROOVE_CENTER_X = ROI_X + ROI_W // 2
HALF_WIDTH      = ROI_W // 2

UART_BAUD   = 115200
UART_TX_PIN = 5
UART_RX_PIN = 6
# ======================

ROI_END_X = ROI_X + ROI_W
ROI_END_Y = ROI_Y + ROI_H

sensor = None
uart = None
smooth_th = -1.0
buf_hsv   = None
buf_gray  = None
buf_green = None
buf_ngreen = None
buf_masked = None


def alloc_buf():
    global buf_hsv, buf_gray, buf_green, buf_ngreen, buf_masked
    buf_hsv    = np.zeros((ROI_H, ROI_W, 3), dtype=np.uint8)
    buf_gray   = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_green  = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_ngreen = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)
    buf_masked = np.zeros((ROI_H, ROI_W),    dtype=np.uint8)


def find_ball(img_np):
    global smooth_th

    roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]

    # 1. 绿色掩码（HSV空间，双区间支持）
    cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=buf_hsv)
    cv2.inRange(buf_hsv, COLOR_LO1, COLOR_HI1, dst=buf_green)
    if USE_DUAL:
        cv2.inRange(buf_hsv, COLOR_LO2, COLOR_HI2, dst=buf_ngreen)
        cv2.bitwise_or(buf_green, buf_ngreen, dst=buf_green)
    cv2.bitwise_not(buf_green, dst=buf_ngreen)

    # 2. 灰度 + 掩掉绿色区域（灰度 AND 反绿掩码）
    cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY, dst=buf_gray)
    cv2.bitwise_and(buf_gray, buf_ngreen, dst=buf_masked)

    # 3. OTSU + EMA平滑
    retval, _ = cv2.threshold(buf_masked, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if smooth_th < 0:
        smooth_th = float(retval)
    else:
        smooth_th = THRESH_ALPHA * float(retval) + (1.0 - THRESH_ALPHA) * smooth_th

    th = int(smooth_th)
    _, binary = cv2.threshold(buf_masked, th, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, (0, 0, 0, 0), th, binary

    best_cnt, best_area = None, 0
    best_x = best_y = best_w = best_h = 0
    n_area = n_edge = n_shape = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = cv2.contourArea(cnt)

        if a < MIN_AREA or a > MAX_AREA:
            continue
        n_area += 1
        if (y <= 1 or y + h >= ROI_H - 1) and w > 20:
            continue
        n_edge += 1
        if a > 12 and h > 0:
            aspect = float(w) / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                continue
        n_shape += 1

        if a > best_area:
            best_area = a
            best_cnt = cnt
            best_x, best_y, best_w, best_h = x, y, w, h

    stats = (len(contours), n_area, n_edge, n_shape)
    if best_cnt is None:
        return None, stats, th, binary

    cx = ROI_X + best_x + best_w // 2
    cy = ROI_Y + best_y + best_h // 2
    rx = ROI_X + best_x
    ry = ROI_Y + best_y
    return (cx, cy, rx, ry, best_w, best_h, best_area), stats, th, binary


try:
    sensor = Sensor(width=1280, height=960, fps=90)
    sensor.reset()
    sensor.set_framesize(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
    sensor.set_pixformat(Sensor.RGB888)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)

    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, fpioa.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, fpioa.UART2_RXD)
    uart = UART(UART.UART2, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    MediaManager.init()
    sensor.run()
    time.sleep(0.5)
    alloc_buf()
    clock = time.clock()

    print("OTSU+GreenMask启动")
    fc = 0

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        result, stats, th, binary_np = find_ball(img_np)
        raw_n, n_area, n_edge, n_shape = stats

        if result is not None:
            ball_x, ball_y, rx, ry, rw, rh, area = result
            offset_pct = (ball_x - GROOVE_CENTER_X) / HALF_WIDTH * 100.0
            uart.write("%.1f\n" % offset_pct)

            img.draw_rectangle(rx, ry, rw, rh,
                               color=(255, 255, 255), thickness=2)
            img.draw_cross(ball_x, ball_y, color=(255, 255, 255),
                           size=8, thickness=2)
        else:
            offset_pct = 0.0

        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X, ROI_END_Y,
                      color=(128, 128, 128), thickness=1)

        pipe = "R%d>A%d>E%d>S%d" % (raw_n, n_area, n_edge, n_shape)
        img.draw_string_advanced(2, 2, 14, "%.1f %+.1f%%" % (clock.fps(), offset_pct),
                                 color=(255, 255, 255))
        img.draw_string_advanced(2, 18, 14, pipe, color=(255, 255, 255))

        top_rgb565 = img.to_rgb565()

        # 下屏：掩码后二值图
        binary_img = image.Image(ROI_W, ROI_H, image.GRAYSCALE,
                                 alloc=image.ALLOC_REF, data=binary_np)
        bar_canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT, image.RGB565)
        bar_canvas.draw_rectangle(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT,
                                  color=(0, 0, 0), fill=True)
        bar_canvas.draw_image(binary_img.to_rgb565(), ROI_X, ROI_Y)
        line = "T:%d R%d>A%d>E%d>S%d" % (th, raw_n, n_area, n_edge, n_shape)
        bar_canvas.draw_string_advanced(2, 4, 14, line, color=(255, 255, 0))

        canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT * 2, image.RGB565)
        canvas.draw_image(top_rgb565, 0, 0)
        canvas.draw_image(bar_canvas, 0, CAMERA_HEIGHT)
        Display.show_image(canvas)

        fc += 1
        if fc % 20 == 0:
            print("%.1ffps %+.1f%% T:%d R%d>A%d>E%d>S%d" %
                  (clock.fps(), offset_pct, th,
                   raw_n, n_area, n_edge, n_shape))

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
