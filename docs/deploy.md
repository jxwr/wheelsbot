# BalanceBot 部署指南

> 记录完整的编译、烧录和部署步骤

---

## 前置要求

- 安装 [PlatformIO](https://platformio.org/install)
- ESP32-S3 开发板通过 USB 连接电脑
- 可选：安装 `screen` 或 `minicom` 用于串口调试（macOS/Linux）

---

## 快速部署（推荐）

如果你刚修改完代码，需要完整部署：

```bash
# 1. 进入项目目录
cd /Users/jxwr/hardware/wheelsbot

# 2. 编译并烧录固件
pio run -t upload

# 3. 上传前端文件到文件系统
pio run -t uploadfs

# 4. 打开串口监视器查看日志（可选）
pio device monitor
```

---

## 详细步骤说明

### 1. 编译固件

```bash
pio run
```

**成功标志**：
```
Building in release mode
Linking .pio/build/esp32s3/firmware.elf
Building .pio/build/esp32s3/firmware.bin
```

---

### 2. 烧录固件到 ESP32

#### 自动烧录（推荐）

```bash
pio run -t upload
```

**操作步骤**：
1. 命令执行后，会看到 `Connecting...` 提示
2. 如果板子没有自动进入下载模式，**按住 BOOT 键，然后按 RESET 键，松开 BOOT 键**
3. 等待烧录完成（约 30-60 秒）

**成功标志**：
```
Writing at 0x00010000... (10%)
...
Writing at 0x00080000... (100%)
Wrote 1048576 bytes ( XXX compressed) at 0x00010000 in X.X seconds
Leaving...
Hard resetting via RTS pin...
```

#### 手动指定端口（如果自动检测失败）

```bash
# macOS
pio run -t upload --upload-port /dev/cu.usbserial-XXXX

# Linux
pio run -t upload --upload-port /dev/ttyUSB0

# Windows
pio run -t upload --upload-port COM3
```

---

### 3. 上传前端文件（Web界面）

**重要**：修改了 `data/index.html` 或其他前端文件后必须执行

```bash
pio run -t uploadfs
```

**成功标志**：
```
Building file system image...
Adding 'data/index.html'
Adding 'data/three.module.js.gz'
Uploading file system image...
Wrote 8192 bytes
```

**注意**：这个步骤只上传 `data/` 目录下的文件到 ESP32 的 LittleFS 文件系统。

---

### 4. 验证部署

#### 方法1：串口监视器

```bash
pio device monitor
```

**预期输出**：
```
AP IP: 192.168.4.1
HTTP + WebSocket server started
Tasks started
```

按 `Ctrl+C` 退出。

#### 方法2：WiFi 连接

1. 用手机/电脑连接 WiFi：`BalanceBot`
2. 浏览器访问：`http://192.168.4.1`
3. 应该看到调试界面

---

## 常见问题

### 问题1：烧录失败 "Failed to connect to ESP32"

**原因**：ESP32 没有进入下载模式

**解决**：
1. 按住 BOOT 键不放
2. 按一下 RESET 键
3. 松开 BOOT 键
4. 重新执行 `pio run -t upload`

---

### 问题2：网页显示 "File not found"

**原因**：前端文件没有上传到文件系统

**解决**：
```bash
pio run -t uploadfs
```

---

### 问题3：修改了代码但没有生效

**原因**：只编译了固件，可能需要清除缓存

**解决**：
```bash
# 清理编译缓存
pio run -t clean

# 重新编译并上传
pio run -t upload
```

---

### 问题4：串口权限不足（Linux）

**解决**：
```bash
# 临时添加权限
sudo chmod 666 /dev/ttyUSB0

# 或永久添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 然后重新登录
```

---

## 只修改前端文件时的快速部署

如果只修改了 `data/index.html`：

```bash
pio run -t uploadfs
```

无需重新烧录固件。

---

## 只修改控制器参数时的部署

如果只是通过 Web 界面调整参数并保存到 Flash，**无需重新部署**，参数会自动持久化到 `/params/balance.json`。

---

## 完整清理和重新部署

如果遇到奇怪的问题，彻底清理：

```bash
# 1. 清理编译产物
pio run -t clean

# 2. 删除平台缓存
rm -rf .pio

# 3. 重新编译并烧录
pio run -t upload

# 4. 上传前端文件
pio run -t uploadfs
```

---

## 有用的命令

| 命令 | 作用 |
|------|------|
| `pio run` | 只编译，不烧录 |
| `pio run -t upload` | 编译并烧录固件 |
| `pio run -t uploadfs` | 上传前端文件到文件系统 |
| `pio device monitor` | 打开串口监视器 |
| `pio run -t clean` | 清理编译缓存 |
| `pio boards esp32` | 列出支持的 ESP32 板子 |

---

## 文件说明

| 文件/目录 | 说明 |
|-----------|------|
| `src/` | C++ 源代码 |
| `data/` | 前端文件（HTML/JS） |
| `docs/` | 文档 |
| `platformio.ini` | 项目配置 |
| `.pio/build/esp32s3/firmware.bin` | 编译后的固件 |

---

## 备忘录

- **修改 C++ 代码** → `pio run -t upload`
- **修改 HTML/JS** → `pio run -t uploadfs`
- **修改两者** → 先 `upload` 再 `uploadfs`
- **烧录失败** → 按 BOOT + RESET 进入下载模式
- **网页404** → 执行 `uploadfs`

---

最后更新：2026-02-21
