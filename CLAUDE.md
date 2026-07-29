# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

2026 年全国大学生电子设计竞赛 — 车载平衡滚球运动控制系统。

本项目包含两个基于立创庐山派 K230 系列的独立子系统：

| 子系统 | 硬件 | 功能 |
|--------|------|------|
| **无线图传** | 庐山派 Lite K230D + GC2093 | 采集钢球运动画面 → H.264 编码 → RTSP/WiFi AP → PC 接收显示 |
| **钢球位置检测** | 庐山派 K230 + GC2093 | 识别凹槽中钢球位置 → 计算偏移百分比 → UART 输出给主控 |

两块板子独立工作，分别连接各自的摄像头。

## 硬件架构

```
图传链路：  GC2093 → K230D → H.264硬编码 → WiFi AP热点 → RTSP → 笔记本电脑

识别链路：  GC2093 → K230 → 图像处理 → 钢球偏移% → UART → 主控板
```

### WiFi 图传连接方式

K230D 建立 WiFi AP 热点（SSID: `K230D_Ball`，密码: `12345678`），PC 连接该热点后通过 RTSP 接收视频流。

## 软件架构

```
k230-stream/          # K230D 图传端 MicroPython 代码（CanMV IDE）
  └── main.py         # WiFi AP + H.264硬编码 + RTSP推流

k230-detector/        # K230 #2 识别端 MicroPython 代码（CanMV IDE）
  ├── main.py         # 摄像头采集、钢球检测、UART位置输出
  └── ...

pc-receiver/          # PC 上位机 Python 代码
  ├── main.py         # RTSP接收、OpenCV显示、PyQt/PySide GUI
  ├── recorder.py     # 录像保存（MP4）
  ├── player.py       # 视频回放
  └── ...
```

## 关键接口

- **图传视频流**：RTSP `rtsp://<K230D_IP>:8554/ball`，K230D 建立 WiFi AP `K230D_Ball`
- **钢球位置输出**：UART 串口，偏移百分比格式（如 `-35.2` 偏左，`+20.1` 偏右）
- **UART 数据帧格式**：`[offset_pct]`，无钢球时不发送

## 开发环境

- **K230/K230D 端**：CanMV IDE（立创庐山派官方 IDE），MicroPython 开发，通过 IDE 连接板子下载运行
  - K230（识别端）：`k230-detector/`
  - K230D（图传端）：`k230-stream/`
- **PC 端**：Python 3.x + OpenCV + PyQt/PySide + FFmpeg
- **运行 PC 上位机**：`python main.py`

## 需求文档

详见 `K230_车载平衡滚球图传系统需求说明.md`，包含完整赛题要求、系统架构、功能需求和推荐参数。

## 待定事项

- 凹槽实际尺寸和摄像头安装位置（影响坐标→百分比映射参数）

## K230 vs K230D 差异

| 项目 | K230（识别端） | K230D（图传端） |
|------|---------------|-----------------|
| 板名 | `k230_canmv_lckfb` | `k230d_canmv_lushanpi_lite` |
| USR按键 | GPIO 53 | GPIO 64 |
| 核心 | 双核 | 单核 |
| 分辨率建议 | 320×240（检测） | 1280×720（编码） |
