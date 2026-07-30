# -*- coding: utf-8 -*-
# 钢球位置检测 — 立创庐山派 K230
# 方案：上电标定绿色HSV → OTSU + 绿色掩码 → UART
# 输出：UART 偏移百分比 给主控

import os, gc, time, sys
import cv2
import image
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA, Pin
from ulab import numpy as np

# ===== 可调参数 =====
CAMERA_WIDTH  = 320
CAMERA_HEIGHT = 240

ROI_X = 10
ROI_Y = 145
ROI_W = 300

# 校准 / 识别 ROI 高度
CALIB_ROI_H  = 12
DETECT_ROI_H = 15

# HSV 余量
H_MARGIN = 8
S_MARGIN = 20
V_MARGIN = 30

THRESH_ALPHA = 0.05

MIN_AREA = 10
MAX_AREA = 250
MIN_ASPECT = 0.6
MAX_ASPECT = 1.7

GROOVE_CENTER_X = ROI_X + ROI_W // 2
HALF_WIDTH      = ROI_W // 2

# 跳变过滤
MAX_JUMP     = 20   # 相邻帧允许的最大X位移（像素）
LOST_TIMEOUT = 20   # 连续丢失/跳变超过此帧数则重新搜索全ROI

# 形态学
MORPH_KSIZE = 5    # 横向矩形核宽度
MORPH_ITERS = 1    # 开运算迭代次数

UART_BAUD   = 115200
UART_TX_PIN = 11
UART_RX_PIN = 12
USR_KEY_NUM = 53
# ======================

# 可变阈值（校准阶段更新）
color_lo1 = np.array([40, 113, 107], dtype=np.uint8)
color_hi1 = np.array([60, 225, 214], dtype=np.uint8)

sensor = None
uart = None
usr_key = None
smooth_th = -1.0
ROI_H = CALIB_ROI_H  # 当前ROI高度
# 跳变过滤跟踪状态
last_valid_ball = None   # (cx, cy, offset_pct) 或 None
lost_count = 0
ROI_END_X = ROI_X + ROI_W
ROI_END_Y = ROI_Y + ROI_H

buf_hsv = buf_gray = buf_green = buf_ngreen = buf_masked = None
buf_th = buf_open = morph_kernel = None


def alloc_buf():
    global buf_hsv, buf_gray, buf_green, buf_ngreen, buf_masked
    global buf_th, buf_open, morph_kernel
    h = DETECT_ROI_H
    buf_hsv    = np.zeros((h, ROI_W, 3), dtype=np.uint8)
    buf_gray   = np.zeros((h, ROI_W),    dtype=np.uint8)
    buf_green  = np.zeros((h, ROI_W),    dtype=np.uint8)
    buf_ngreen = np.zeros((h, ROI_W),    dtype=np.uint8)
    buf_masked = np.zeros((h, ROI_W),    dtype=np.uint8)
    buf_th     = np.zeros((h, ROI_W),    dtype=np.uint8)
    buf_open   = np.zeros((h, ROI_W),    dtype=np.uint8)
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                              (1, MORPH_KSIZE))  # 横向核


def set_roi(h):
    global ROI_H, ROI_END_Y
    ROI_H = h
    ROI_END_Y = ROI_Y + h


def show_frame(img, bot_label=None, bot_img=None, bot_color=(255, 255, 0)):
    """显示上下分屏画布"""
    top = img.to_rgb565()
    bot = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT, image.RGB565)
    bot.draw_rectangle(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT,
                       color=(0, 0, 0), fill=True)
    if bot_img is not None:
        bot.draw_image(bot_img.to_rgb565(), ROI_X, ROI_Y)
    if bot_label is not None:
        bot.draw_string_advanced(2, 4, 14, bot_label, color=bot_color)
    canvas = image.Image(CAMERA_WIDTH, CAMERA_HEIGHT * 2, image.RGB565)
    canvas.draw_image(top, 0, 0)
    canvas.draw_image(bot, 0, CAMERA_HEIGHT)
    Display.show_image(canvas)


def calibrate_hsv(img_np):
    """在校准ROI内统计HSV min/max，更新 color_lo1/color_hi1"""
    global color_lo1, color_hi1
    set_roi(CALIB_ROI_H)
    roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    h_min = s_min = v_min = 255
    h_max = s_max = v_max = 0

    for row in range(ROI_H):
        for col in range(ROI_W):
            px = hsv[row, col]
            h, s, v = int(px[0]), int(px[1]), int(px[2])
            if h < h_min: h_min = h
            if h > h_max: h_max = h
            if s < s_min: s_min = s
            if s > s_max: s_max = s
            if v < v_min: v_min = v
            if v > v_max: v_max = v

    lo_h = max(h_min - H_MARGIN, 0)
    lo_s = max(s_min - S_MARGIN, 0)
    lo_v = max(v_min - V_MARGIN, 0)
    hi_h = min(h_max + H_MARGIN, 180)
    hi_s = min(s_max + S_MARGIN, 255)
    hi_v = min(v_max + V_MARGIN, 255)

    color_lo1 = np.array([lo_h, lo_s, lo_v], dtype=np.uint8)
    color_hi1 = np.array([hi_h, hi_s, hi_v], dtype=np.uint8)

    print("标定完成 H[%d-%d] S[%d-%d] V[%d-%d]" %
          (h_min, h_max, s_min, s_max, v_min, v_max))
    print("COLOR_LO1 = [%d, %d, %d]" % (lo_h, lo_s, lo_v))
    print("COLOR_HI1 = [%d, %d, %d]" % (hi_h, hi_s, hi_v))


def find_ball(img_np):
    global smooth_th

    roi = img_np[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]

    cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=buf_hsv)
    cv2.inRange(buf_hsv, color_lo1, color_hi1, dst=buf_green)
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
    cv2.threshold(buf_masked, th, 255, cv2.THRESH_BINARY, dst=buf_th)

    # 全程形态学开运算：断开球与横向反光的粘连
    cv2.morphologyEx(buf_th, cv2.MORPH_OPEN, morph_kernel,
                     dst=buf_open, iterations=MORPH_ITERS)

    contours, _ = cv2.findContours(buf_open, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, (0, 0, 0, 0), th, buf_open

    best_cnt, best_area = None, 0
    best_x = best_y = best_w = best_h = 0
    n_area = n_edge = n_shape = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = cv2.contourArea(cnt)

        if a < MIN_AREA or a > MAX_AREA:
            continue
        n_area += 1
        # 形状判断（面积>12才做，避免小噪点误判）
        is_ball_shape = True
        if a > 12 and h > 0:
            aspect = float(w) / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                is_ball_shape = False
        # 贴边过滤：球形态的色块不杀（球贴边正常），只杀非球形态的贯穿条状物
        if not is_ball_shape:
            edge_tb = (y <= 0 and y + h >= ROI_H - 1)
            edge_lr = (x <= 0 and x + w >= ROI_W - 1)
            if edge_tb or edge_lr:
                continue
        n_edge += 1
        if not is_ball_shape:
            continue
        n_shape += 1

        if a > best_area:
            best_area = a
            best_cnt = cnt
            best_x, best_y, best_w, best_h = x, y, w, h

    stats = (len(contours), n_area, n_edge, n_shape)
    if best_cnt is None:
        return None, stats, th, buf_open

    cx = ROI_X + best_x + best_w // 2
    cy = ROI_Y + best_y + best_h // 2
    rx = ROI_X + best_x
    ry = ROI_Y + best_y
    return (cx, cy, rx, ry, best_w, best_h, best_area), stats, th, buf_open


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

    fpioa.set_function(USR_KEY_NUM, FPIOA.GPIO0 + USR_KEY_NUM)
    usr_key = Pin(USR_KEY_NUM, Pin.IN, Pin.PULL_DOWN)

    MediaManager.init()
    sensor.run()
    time.sleep(0.5)
    alloc_buf()
    clock = time.clock()

    # ===== 校准模式 =====
    set_roi(CALIB_ROI_H)
    print("校准模式 — 将凹槽对准绿框，按USR键")
    key_prev = False
    calib_done = False

    while not calib_done:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        # 画校准框
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, CALIB_ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X,
                      ROI_Y + CALIB_ROI_H, color=(128, 128, 128), thickness=1)
        img.draw_string_advanced(2, 2, 14, "Press USR to calib",
                                 color=(255, 255, 0))

        # USR按键检测
        if usr_key.value() == 1 and not key_prev:
            key_prev = True
            time.sleep_ms(10)
            if usr_key.value() == 1:
                # 倒数3秒
                for n in range(3, 0, -1):
                    img2 = sensor.snapshot()
                    img2.draw_rectangle(ROI_X, ROI_Y, ROI_W, CALIB_ROI_H,
                                        color=(0, 255, 0), thickness=2)
                    img2.draw_string_advanced(
                        CAMERA_WIDTH // 2 - 20, CAMERA_HEIGHT // 2 - 10,
                        32, "%d" % n, color=(255, 255, 0))
                    show_frame(img2)
                    time.sleep(1)

                # 采样标定
                img3 = sensor.snapshot()
                img3_np = img3.to_numpy_ref()
                calibrate_hsv(img3_np)

                img3.draw_rectangle(ROI_X, ROI_Y, ROI_W, CALIB_ROI_H,
                                    color=(0, 255, 0), thickness=2)
                img3.draw_string_advanced(CAMERA_WIDTH // 2 - 40,
                                          CAMERA_HEIGHT // 2 - 10,
                                          32, "Calib OK", color=(0, 255, 0))
                show_frame(img3)
                time.sleep(1)
                calib_done = True
        elif usr_key.value() == 0:
            key_prev = False

        show_frame(img)
        gc.collect()

    # ===== 识别模式 =====
    set_roi(DETECT_ROI_H)
    smooth_th = -1.0  # 重置平滑阈值
    print("识别模式启动")
    fc = 0

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        result, stats, th, binary_np = find_ball(img_np)
        raw_n, n_area, n_edge, n_shape = stats

        # ---- 跳变过滤 ----
        if result is not None:
            ball_x, ball_y, rx, ry, rw, rh, area = result
            cur_pct = (ball_x - GROOVE_CENTER_X) / HALF_WIDTH * 100.0

            accept = False
            if last_valid_ball is None:
                accept = True
            else:
                last_x, last_y, _ = last_valid_ball
                if abs(ball_x - last_x) <= MAX_JUMP:
                    accept = True
                else:
                    lost_count += 1
                    if lost_count >= LOST_TIMEOUT:
                        accept = True

            if accept:
                last_valid_ball = (ball_x, ball_y, cur_pct)
                lost_count = 0
                uart.write("[%.1f]" % cur_pct)
        else:
            if last_valid_ball is not None:
                lost_count += 1
                if lost_count >= LOST_TIMEOUT:
                    last_valid_ball = None

        # 显示用有效位置
        if last_valid_ball is not None:
            _, _, offset_pct = last_valid_ball
        else:
            offset_pct = 0.0

        # ---- 画面叠加 ----
        if last_valid_ball is not None:
            lx, ly, _ = last_valid_ball
            img.draw_cross(lx, ly, color=(255, 255, 255),
                           size=8, thickness=2)
        if result is not None:
            _, _, rx, ry, rw, rh, _ = result
            img.draw_rectangle(rx, ry, rw, rh,
                               color=(255, 255, 255), thickness=2)

        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, DETECT_ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X, ROI_END_Y,
                      color=(128, 128, 128), thickness=1)

        pipe = "R%d>A%d>E%d>S%d" % (raw_n, n_area, n_edge, n_shape)
        img.draw_string_advanced(2, 2, 14, "%.1f %+.1f%%" % (clock.fps(), offset_pct),
                                 color=(255, 255, 255))
        img.draw_string_advanced(2, 18, 14, pipe, color=(255, 255, 255))

        binary_img = image.Image(ROI_W, DETECT_ROI_H, image.GRAYSCALE,
                                 alloc=image.ALLOC_REF, data=binary_np)
        label = "T:%d OP(1,%d) R%d>A%d>E%d>S%d" % (th, MORPH_KSIZE, raw_n, n_area, n_edge, n_shape)
        show_frame(img, label, binary_img)

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
