# -*- coding: utf-8 -*-
# K230D 实时图传 — 立创庐山派 Lite K230D
# 方案：GC2093 → H.264硬编码 → RTSP → WiFi AP
# 推流地址：rtsp://<K230D_IP>:8554/ball

import os, time, gc
import network
import _thread
import multimedia as mm
from media.vencoder import *
from media.sensor import *
from media.media import *
from media.display import *

# ===== 可调参数 =====
WIDTH   = 1280
HEIGHT  = 720
PORT    = 8554
SESSION = "ball"

AP_SSID     = "K230D_Ball"
AP_PASSWORD = "12345678"

# H.264 编码参数
ENC_PROFILE = Encoder.H264_PROFILE_MAIN  # 备选 H264_PROFILE_HIGH
ENC_BITRATE = 2 * 1024 * 1024            # 2 Mbps
# ======================


class RtspServer:
    def __init__(self, session_name="ball", port=8554,
                 video_type=mm.multi_media_type.media_h264):
        self.session_name = session_name
        self.video_type   = video_type
        self.port         = port
        self.server       = mm.rtsp_server()
        self.start_stream = False
        self.runthread_over = False

    # ---- 启动 / 停止 ----

    def start(self):
        self._init_stream()
        self.server.rtspserver_init(self.port)
        self.server.rtspserver_createsession(self.session_name,
                                              self.video_type, False)
        self.server.rtspserver_start()
        self._start_stream()
        self.start_stream = True
        _thread.start_new_thread(self._do_rtsp_stream, ())

    def stop(self):
        if not self.start_stream:
            return
        self.start_stream = False
        while not self.runthread_over:
            time.sleep(0.1)
        self._stop_stream()
        self.server.rtspserver_stop()
        self.server.rtspserver_deinit()

    def get_url(self):
        return self.server.rtspserver_getrtspurl(self.session_name)

    # ---- 内部管线 ----

    def _init_stream(self):
        w = ALIGN_UP(WIDTH, 16)
        # Sensor
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(width=w, height=HEIGHT, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)

        # H.264 硬编码器
        self.encoder = Encoder()
        self.encoder.SetOutBufs(8, w, HEIGHT)
        chn = ChnAttrStr(self.encoder.PAYLOAD_TYPE_H264,
                         ENC_PROFILE, w, HEIGHT)
        self.encoder.Create(chn)

        # 绑定 Sensor → Encoder
        src = self.sensor.bind_info()['src']
        dst = (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.encoder.chn)
        self.link = MediaManager.link(src, dst)

    def _start_stream(self):
        self.encoder.Start()
        self.sensor.run()

    def _stop_stream(self):
        self.sensor.stop()
        del self.link
        self.encoder.Stop()
        self.encoder.Destroy()

    def _do_rtsp_stream(self):
        """推流线程：循环取编码帧→发送"""
        try:
            sd = StreamData()
            while self.start_stream:
                os.exitpoint()
                self.encoder.GetStream(sd)
                for i in range(sd.pack_cnt):
                    data = bytes(uctypes.bytearray_at(sd.data[i],
                                                       sd.data_size[i]))
                    self.server.rtspserver_sendvideodata(
                        self.session_name, data, sd.data_size[i], 1000)
                self.encoder.ReleaseStream(sd)
        except BaseException as e:
            import sys
            sys.print_exception(e)
        finally:
            self.runthread_over = True


# ===== WiFi AP 热点 =====

def setup_wifi_ap():
    ap = network.WLAN(network.AP_IF)
    ap.config(ssid=AP_SSID, key=AP_PASSWORD)
    ip = ap.ifconfig()[0]
    print("WiFi AP 已建立")
    print("  SSID : %s" % AP_SSID)
    print("  密码 : %s" % AP_PASSWORD)
    print("  IP   : %s" % ip)
    return ip


# ===== 主程序 =====

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)

    print("=" * 40)
    print("K230D 实时图传启动")
    print("=" * 40)

    # 1. WiFi AP
    ip = setup_wifi_ap()

    # 2. 初始化显示（状态提示）
    Display.init(Display.ST7701, width=800, height=480, to_ide=True)

    # 3. RTSP 推流
    MediaManager.init()
    rtspserver = RtspServer(SESSION, PORT)
    rtspserver.start()
    url = rtspserver.get_url()
    print("RTSP 推流地址: %s" % url)
    print("VLC/ffplay: rtsp://%s:%d/%s" % (ip, PORT, SESSION))
    print("=" * 40)

    # 4. 状态显示循环
    clock = time.clock()
    try:
        while True:
            os.exitpoint()
            clock.tick()

            # 在 LCD 上显示连接信息
            img = image.Image(800, 480, image.RGB565)
            img.draw_rectangle(0, 0, 800, 480, color=(0, 0, 0), fill=True)
            img.draw_string_advanced(20, 20, 24,
                                     "K230D RTSP Stream",
                                     color=(0, 255, 0))
            img.draw_string_advanced(20, 60, 18,
                                     "SSID: %s" % AP_SSID,
                                     color=(255, 255, 255))
            img.draw_string_advanced(20, 85, 18,
                                     "IP: %s:%d" % (ip, PORT),
                                     color=(255, 255, 255))
            img.draw_string_advanced(20, 110, 18,
                                     "URL: rtsp://%s:%d/%s" % (ip, PORT, SESSION),
                                     color=(200, 200, 200))
            img.draw_string_advanced(20, 150, 14,
                                     "%.1f fps  %dx%d" % (clock.fps(), WIDTH, HEIGHT),
                                     color=(128, 128, 128))
            Display.show_image(img)
            del img
            gc.collect()

    except KeyboardInterrupt:
        print("用户停止")
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        rtspserver.stop()
        Display.deinit()
        MediaManager.deinit()
        print("资源已释放")


if __name__ == "__main__":
    main()
