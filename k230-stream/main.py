# -*- coding: utf-8 -*-
# K230D 实时图传 — 立创庐山派 Lite K230D
# 方案：GC2093 → JPEG压缩 → MJPEG over HTTP → WiFi AP
# 观看：PC连热点 K230D_Ball 后 VLC 打开 http://192.168.169.1/stream

import os, time, gc
import network
import usocket
from media.sensor import *
from media.display import *
from media.media import *

# ===== 可调参数 =====
WIDTH  = 640
HEIGHT = 360
PORT   = 80

AP_SSID     = "K230D_Ball"
AP_PASSWORD = "12345678"

JPEG_QUALITY = 35
# ======================

MJPEG_BOUNDARY = "--K230_FRAME"
MJPEG_HEADER = (
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=K230_FRAME\r\n"
    "Cache-Control: no-cache\r\n"
    "Connection: close\r\n"
    "\r\n"
).encode()

HTML_PAGE = (
    "<!DOCTYPE html>\n"
    "<html><head><meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>K230D 图传</title></head>\n"
    "<body style=\"margin:0;background:#1a1a2e;text-align:center\">\n"
    "<h3 style=\"color:#eee;padding-top:8px\">K230D 实时图传</h3>\n"
    "<img src=\"/stream\""
    " style=\"width:100%;max-width:640px;border:2px solid #333\">\n"
    "<p style=\"color:#888;font-size:12px\">MJPEG 低延迟实时画面</p>\n"
    "</body></html>"
).encode()
HTML_HEADER = (
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Content-Length: %d\r\n"
    "Connection: close\r\n"
    "\r\n"
) % len(HTML_PAGE)


def send_all(sock, data):
    total = 0
    while total < len(data):
        try:
            sent = sock.send(data[total:])
            if sent <= 0:
                return False
            total += sent
        except OSError:
            return False
    return True


def build_mjpeg_frame(jpeg_data):
    return (
        MJPEG_BOUNDARY + "\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %d\r\n"
        "\r\n" % len(jpeg_data)
    ).encode() + jpeg_data + b"\r\n"


def show_boot(text):
    """更新启动画面"""
    img = image.Image(800, 480, image.RGB565)
    img.draw_rectangle(0, 0, 800, 480, color=(0, 0, 0), fill=True)
    img.draw_string_advanced(100, 200, 28, text, color=(0, 255, 0))
    Display.show_image(img)
    del img


def show_status(display_img, lines, color=(255, 255, 255)):
    for i, text in enumerate(lines):
        display_img.draw_string_advanced(
            10, 10 + i * 22, 18, text, color=color)


# ===== 主程序 =====

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)

    # --- 第1步：立即亮屏 ---
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    show_boot("1/5 屏幕就绪")

    # --- 第2步：WiFi AP ---
    show_boot("2/5 WiFi启动中...")
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    time.sleep(0.5)
    try:
        ap.config(ssid=AP_SSID, key=AP_PASSWORD)
    except Exception:
        ap.config("ssid", AP_SSID)
        ap.config("key", AP_PASSWORD)
    ip = ap.ifconfig()[0]
    show_boot("WiFi OK: %s" % ip)
    time.sleep(1)

    # --- 第3步：摄像头 ---
    show_boot("3/5 摄像头初始化...")
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    MediaManager.init()
    sensor.run()
    show_boot("3/5 摄像头就绪")
    time.sleep(0.5)

    # --- 第4步：HTTP Socket ---
    show_boot("4/5 HTTP服务启动...")
    s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
    s.bind((ip, PORT))
    s.listen(2)
    s.settimeout(0.01)
    show_boot("5/5 就绪! %s" % ip)
    time.sleep(1)

    # --- 第5步：主循环 ---
    stream_client = None
    clock = time.clock()

    fc = 0
    while True:
        os.exitpoint()
        clock.tick()
        fc += 1

        img = sensor.snapshot()
        jpeg = bytes(img.compress(quality=JPEG_QUALITY))

        # 叠加状态信息
        show_status(img, [
            "SSID: %s" % AP_SSID,
            "IP: %s  Port: %d" % (ip, PORT),
            "Stream: %s" % ("ON" if stream_client else "OFF"),
            "FPS: %.1f  %dx%d" % (clock.fps(), WIDTH, HEIGHT),
        ])
        Display.show_image(img)

        # MJPEG 推送
        if stream_client is not None:
            frame = build_mjpeg_frame(jpeg)
            if not send_all(stream_client, frame):
                try:
                    stream_client.close()
                except:
                    pass
                stream_client = None

        # 接受新连接
        try:
            cl, caddr = s.accept()
        except OSError:
            time.sleep_ms(5)
            continue

        try:
            cl.settimeout(0.3)
            req = cl.recv(512).decode()
        except Exception:
            try:
                cl.close()
            except:
                pass
            continue

        try:
            if "/stream" in req:
                if stream_client is not None:
                    try:
                        stream_client.close()
                    except:
                        pass
                stream_client = cl
                send_all(cl, MJPEG_HEADER)
            else:
                send_all(cl, HTML_HEADER + HTML_PAGE)
                time.sleep_ms(50)
                try:
                    cl.close()
                except:
                    pass
        except Exception:
            try:
                cl.close()
            except:
                pass

        if fc % 30 == 0:
            gc.collect()


if __name__ == "__main__":
    main()
