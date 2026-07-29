# -*- coding: utf-8 -*-
# K230D 实时图传 — 立创庐山派 Lite K230D
# 方案：GC2093 → JPEG压缩 → MJPEG over HTTP → WiFi AP
# 观看方式：PC连WiFi热点后，浏览器打开 http://<IP>
#           VLC打开 http://<IP>/stream

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

JPEG_QUALITY = 35  # JPEG质量 1-100，越小体积越小延迟越低
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
    """确保全部数据发送完毕"""
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
    """构造 MJPEG multipart 帧"""
    return (
        MJPEG_BOUNDARY + "\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %d\r\n"
        "\r\n" % len(jpeg_data)
    ).encode() + jpeg_data + b"\r\n"


# ===== WiFi AP =====

def setup_wifi_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(ssid=AP_SSID, key=AP_PASSWORD)
    ip = ap.ifconfig()[0]
    print("WiFi AP: %s  密码: %s  IP: %s" % (AP_SSID, AP_PASSWORD, ip))
    return ip, ap


# ===== 主程序 =====

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)

    print("=" * 40)
    print("K230D MJPEG 实时图传启动")
    print("=" * 40)

    # 1. WiFi AP
    ip, ap = setup_wifi_ap()

    # 2. 摄像头
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    time.sleep(0.5)

    # 3. HTTP Socket
    s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
    s.bind((ip, PORT))
    s.listen(2)
    s.setblocking(False)
    print("HTTP 服务: http://%s" % ip)
    print("VLC: http://%s/stream" % ip)
    print("=" * 40)

    stream_client = None
    stream_frame = 0
    last_jpeg = None
    clock = time.clock()

    try:
        while True:
            os.exitpoint()
            clock.tick()

            # ---- 抓帧 ----
            img = sensor.snapshot()
            last_jpeg = bytes(img.compress(quality=JPEG_QUALITY))
            Display.show_image(img)

            # ---- MJPEG 推帧 ----
            if stream_client is not None:
                part = build_mjpeg_frame(last_jpeg)
                if send_all(stream_client, part):
                    stream_frame += 1
                else:
                    try:
                        stream_client.close()
                    except:
                        pass
                    stream_client = None
                    print("客户端断开（共推 %d 帧）" % stream_frame)
                    stream_frame = 0

            # ---- 接受新连接 ----
            try:
                cl, caddr = s.accept()
            except OSError:
                gc.collect()
                continue

            # ---- 读 HTTP 请求 ----
            try:
                cl.settimeout(0.3)
                req = cl.recv(512).decode()
            except Exception:
                try:
                    cl.close()
                except:
                    pass
                continue

            # ---- 路由 ----
            try:
                if "/stream" in req:
                    if stream_client is not None:
                        try:
                            stream_client.close()
                        except:
                            pass
                    stream_client = cl
                    stream_frame = 0
                    print("MJPEG 客户端: %s" % str(caddr))
                    send_all(cl, MJPEG_HEADER)
                else:
                    print("HTTP 页面: %s" % str(caddr))
                    send_all(cl, HTML_HEADER + HTML_PAGE)
                    time.sleep_ms(50)
                    try:
                        cl.close()
                    except:
                        pass
            except Exception as e:
                try:
                    cl.close()
                except:
                    pass

            gc.collect()

    except KeyboardInterrupt:
        print("用户停止")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        if stream_client is not None:
            try:
                stream_client.close()
            except:
                pass
        sensor.stop()
        time.sleep_ms(200)
        s.close()
        Display.deinit()
        MediaManager.deinit()
        gc.collect()
        print("资源已释放")


if __name__ == "__main__":
    main()
