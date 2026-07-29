# -*- coding: utf-8 -*-
# 钢球检测调试脚本 — 色块编号 + 过滤原因列表
# 上屏：编号色块框  下屏：每个色块的过滤原因

import os, gc, time, sys
import cv2
import image
from media.sensor import *
from media.display import *
from media.media import *
from ulab import numpy as np

# ===== 参数（与main.py保持一致）=====
CAMERA_WIDTH  = 320
CAMERA_HEIGHT = 240

ROI_X = 10
ROI_Y = 110
ROI_W = 300
ROI_H = 20

INVERT = False
MIN_AREA = 5
MAX_AREA = 30
MIN_ASPECT = 0.4
MAX_ASPECT = 2.5

DEBUG_MIN_AREA = 4        # 调试显示的最小面积（过滤1-2像素噪点）
GROOVE_CENTER_X = ROI_X + ROI_W // 2
# ======================

ROI_END_X = ROI_X + ROI_W
ROI_END_Y = ROI_Y + ROI_H

if INVERT:
    OTSU_MODE = cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
else:
    OTSU_MODE = cv2.THRESH_BINARY + cv2.THRESH_OTSU

REASON_TXT = {0: "OK->SEND", 1: "AREA out", 2: "EDGE touch", 3: "ASPECT bad", 4: "2nd small"}
REASON_COL = {0: (0, 255, 0), 1: (255, 0, 0), 2: (0, 0, 255), 3: (255, 0, 255), 4: (255, 255, 0)}

buf_gray = None


def alloc_buf():
    global buf_gray
    buf_gray = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH), dtype=np.uint8)


def analyze(img_np):
    """返回 (best, info_list, binary)
       info_list: [(id, rx,ry,rw,rh,area,aspect,reason), ...]
    """
    cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY, dst=buf_gray)
    roi = buf_gray[ROI_Y:ROI_END_Y, ROI_X:ROI_END_X]
    _, binary = cv2.threshold(roi, 0, 255, OTSU_MODE)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    info_list = []
    best = None
    best_area = 0
    best_blob = None
    blob_id = 0
    noise_count = 0

    if not contours:
        return None, info_list, binary, noise_count

    # 第一遍：分类
    pass1 = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = cv2.contourArea(cnt)
        aspect = float(w) / float(h) if h > 0 else 99.0
        rx, ry = ROI_X + x, ROI_Y + y

        # 过滤微小噪点（不编号不列出）
        if a < DEBUG_MIN_AREA:
            noise_count += 1
            continue

        bid = blob_id
        blob_id += 1

        if a < MIN_AREA or a > MAX_AREA:
            info_list.append((bid, rx, ry, w, h, a, aspect, 1))
            continue
        if y <= 1 or y + h >= ROI_H - 1:
            info_list.append((bid, rx, ry, w, h, a, aspect, 2))
            continue
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            info_list.append((bid, rx, ry, w, h, a, aspect, 3))
            continue

        pass1.append((bid, rx, ry, w, h, a, aspect, cnt))
        if a > best_area:
            best_area = a
            best_blob = (bid, rx, ry, w, h, a, aspect, cnt)

    # 第二遍：标记OK和2nd
    for item in pass1:
        bid, rx, ry, w, h, a, aspect, cnt = item
        if best_blob is not None and bid == best_blob[0]:
            info_list.append((bid, rx, ry, w, h, a, aspect, 0))
        else:
            info_list.append((bid, rx, ry, w, h, a, aspect, 4))

    if best_blob is not None:
        bid, rx, ry, w, h, a, aspect, cnt = best_blob
        cx = rx + w // 2
        cy = ry + h // 2
        best = (cx, cy, rx, ry, w, h, a, aspect)

    return best, info_list, binary, noise_count


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

    print("调试脚本启动")

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()
        img_np = img.to_numpy_ref()

        best, info_list, binary, noise_n = analyze(img_np)

        # === 上屏：图像 + 编号色块 ===
        img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H,
                           color=(0, 255, 0), thickness=2)
        img.draw_line(GROOVE_CENTER_X, ROI_Y, GROOVE_CENTER_X, ROI_END_Y,
                      color=(128, 128, 128), thickness=1)

        for item in info_list:
            bid, rx, ry, rw, rh, ra, raspect, reason = item
            color = REASON_COL.get(reason, (128, 128, 128))
            img.draw_rectangle(rx, ry, rw, rh, color=color, thickness=1)

        # 高亮选中
        if best is not None:
            bx, by, brx, bry, brw, brh, barea, baspect = best
            offset_pct = (bx - GROOVE_CENTER_X) / (ROI_W // 2) * 100.0
            img.draw_rectangle(brx, bry, brw, brh,
                               color=(255, 255, 255), thickness=2)
            img.draw_cross(bx, by, color=(255, 255, 255), size=8, thickness=2)
            status = "%+.1f%%" % offset_pct
            sc = (0, 255, 0)
        else:
            status = "NO BALL"
            sc = (255, 0, 0)

        img.draw_string_advanced(2, 2, 14,
                                 "FPS:%.1f %s" % (clock.fps(), status),
                                 color=sc)
        img.draw_string_advanced(220, 2, 10,
                                 "G=OK R=AREA B=EDGE P=ASP Y=2nd",
                                 color=(200, 200, 200))

        # 管���统计
        cnt = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
        for item in info_list:
            r = item[6]
            cnt[r] = cnt.get(r, 0) + 1
        pipe = "%d|A%d|E%d|R%d|>>%d" % (len(info_list) + noise_n,
                                         cnt[1], cnt[2], cnt[3], cnt[0])
        if best is not None:
            pipe += " A=%.0f R=%.1f" % (best[6], best[7])
        img.draw_string_advanced(2, 18, 14, pipe, color=(255, 255, 0))

        Display.show_image(img)

        del img_np, binary
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
