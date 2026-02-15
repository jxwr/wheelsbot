# Web 控制界面重构提案

**提案编号**: 2602141404
**提案日期**: 2026-02-14
**提案人**: wheelsbot 维护者
**状态**: 设计中

---

## 1. 执行摘要

### 1.1 问题陈述

当前 Web 控制界面存在以下问题：

1. **页面分离**: control.html 和 index.html 分离，用户需要在调试和控制模式间频繁切换
2. **信息密度低**: 大量屏幕空间未被有效利用，关键数据分散
3. **触控体验差**: 按钮和控件尺寸不适合 iPad Mini 等平板设备
4. **调试能力弱**: 无法查看传感器原始数据、控制器内部状态
5. **视觉风格不统一**: 两个页面设计风格存在差异

### 1.2 解决方案概述

将现有双页结构合并为**单页应用 (SPA)**，采用标签页导航组织功能：

- **遥控标签**: 3D视图 + 实时曲线 + 双摇杆控制
- **监控标签**: 传感器原始数据可视化 + 控制器内部状态
- **调试标签**: PID参数调节 + 实时响应曲线
- **设置标签**: 系统配置 + 校准工具

### 1.3 预期收益

- 开发效率提升：统一代码库，减少重复逻辑
- 用户体验提升：单页切换无需重新连接 WebSocket
- 调试效率提升：完整的数据可视化和参数调节能力
- 维护成本降低：一致的组件库和样式系统

---

## 2. 现状分析

### 2.1 现有架构

```
data/
├── control.html    # 遥控器界面 (650行)
├── index.html      # 调试控制台 (560行)
└── three.module.js.gz  # Three.js 库
```

### 2.2 当前数据流

```
┌─────────────┐     WebSocket      ┌─────────────┐
│   control   │◄──────────────────►│  ESP32      │
│   .html     │   (20Hz telemetry) │  WebSocket  │
└─────────────┘                    └─────────────┘
                                          ▲
┌─────────────┐     WebSocket           │
│   index     │◄─────────────────────────┘
│   .html     │   (参数设置 + 遥测)
└─────────────┘
```

### 2.3 技术债务

1. **代码重复**: 两个页面各自实现 WebSocket 连接管理、遥测解析
2. **状态丢失**: 切换页面需要重新建立 WebSocket 连接
3. **紧耦合**: HTML/CSS/JS 混合，难以维护和测试
4. **扩展性差**: 添加新功能需要修改两个文件

---

## 3. 设计目标

### 3.1 功能性目标

| 优先级 | 目标 | 描述 |
|-------|------|------|
| P0 | 单页整合 | 所有功能集成到单个 HTML 文件 |
| P0 | 实时可视化 | 3D姿态 + 数据曲线 + 传感器条形图 |
| P0 | 完整调试 | 查看所有传感器原始值和控制器内部状态 |
| P1 | 参数调节 | 实时 PID 调参，带响应曲线反馈 |
| P1 | 触控优化 | iPad Mini 优先设计，最小触控目标 44px |
| P2 | 数据记录 | 支持遥测数据录制和导出 |

### 3.2 非功能性目标

- **性能**: 60fps 渲染，WebSocket 延迟 < 50ms
- **兼容性**: iOS Safari 14+, Chrome 90+
- **离线支持**: 基础 PWA 能力（manifest + service worker）
- **可访问性**: 基础键盘导航支持

---

## 4. 架构设计

### 4.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                    dashboard.html (SPA)                     │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  View Layer  │  │  State Store │  │  WebSocket   │      │
│  │              │  │              │  │   Client     │      │
│  │ • TabControl │  │              │  │              │      │
│  │ • Joystick   │◄─┤ • telemetry  │◄─┤ • connect    │      │
│  │ • Chart      │  │ • params     │  │ • reconnect  │      │
│  │ • Panel3D    │  │ • commands   │  │ • protocol   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
├─────────────────────────────────────────────────────────────┤
│                      Component Library                      │
│  (Button, Slider, Switch, DataCard, ProgressBar, ...)       │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 模块职责

| 模块 | 职责 | 文件/位置 |
|------|------|----------|
| StateStore | 全局状态管理，视图间数据共享 | 内联 JS Class |
| WSClient | WebSocket 连接管理，自动重连 | 内联 JS Class |
| TelemetryProcessor | 遥测数据解析和格式化 | 内联 JS |
| ViewController | 标签页切换和视图生命周期 | 内联 JS |
| UIComponents | 可复用 UI 组件 | CSS + JS |

### 4.3 状态管理

```typescript
// 全局状态结构
interface AppState {
  // 连接状态
  connection: {
    connected: boolean;
    lastPing: number;
    reconnectCount: number;
  };

  // 遥测数据 (最新帧)
  telemetry: {
    timestamp: number;
    attitude: { pitch: number; roll: number; yaw: number };
    motion: { velocity: number; heading: number };
    motors: { left: number; right: number };
    sensors: {
      accel: { x: number; y: number; z: number };
      gyro: { x: number; y: number; z: number };
    };
    controller: {
      angleContrib: number;
      gyroContrib: number;
      distContrib: number;
      speedContrib: number;
      yawOutput: number;
      lqrU: number;
      headingTarget: number;
      distZero: number;
    };
    system: {
      state: number;
      fault: number;
      lifted: boolean;
      runtime: number;
    };
  };

  // 控制状态
  control: {
    motorEnabled: boolean;
    balanceEnabled: boolean;
    maxLinearVel: number;
    maxYawRate: number;
    leftJoystick: { x: number; y: number };
    rightJoystick: { x: number; y: number };
  };

  // 参数缓存
  params: {
    balance: BalanceParams;
    foc: FOCParams;
  };

  // UI 状态
  ui: {
    activeTab: 'control' | 'monitor' | 'debug' | 'settings';
    chartVisible: string[];  // 可见曲线
    panelExpanded: Record<string, boolean>;
  };
}
```

---

## 5. 数据结构定义

### 5.1 WebSocket 协议 v2

#### 5.1.1 客户端 → 服务器

```typescript
// 命令基础类型
type Command =
  | SetTargetCommand
  | SetControlCommand
  | SetPidCommand
  | SetParamCommand
  | GetParamsCommand
  | SaveParamsCommand
  | ResetCommand;

// 设置目标速度/转向
interface SetTargetCommand {
  type: 'set_target';
  target: 'linear_vel' | 'yaw_rate';
  value: number;
}

// 设置控制开关
interface SetControlCommand {
  type: 'set_ctrl';
  key: 'motor_enable' | 'balance_enable' | 'remote_mode';
  value: boolean | number;
}

// 设置 PID 参数
interface SetPidCommand {
  type: 'set_pid';
  group: 'angle' | 'gyro' | 'distance' | 'speed' | 'yaw_angle' | 'yaw_gyro' | 'lqr_u' | 'zeropoint';
  params: {
    kp?: number;
    ki?: number;
    kd?: number;
  };
}

// 设置其他参数
interface SetParamCommand {
  type: 'set_param';
  category: 'safety' | 'filter' | 'foc';
  key: string;
  value: number;
}

// 获取所有参数
interface GetParamsCommand {
  type: 'get_params';
}

// 保存参数到 Flash
interface SaveParamsCommand {
  type: 'save_params';
}

// 重置控制器
interface ResetCommand {
  type: 'reset';
  target: 'balance' | 'heading' | 'all';
}
```

#### 5.1.2 服务器 → 客户端

```typescript
// 遥测数据 (20Hz)
interface TelemetryMessage {
  type: 'telem';
  t: number;              // 时间戳 (ms)

  // ===== 姿态数据 =====
  pitch: number;          // 俯仰角 (rad)
  pitch_deg: number;      // 俯仰角 (deg)
  pitch_rate: number;     // 俯仰角速度 (rad/s)
  roll_deg: number;       // 横滚角 (deg)
  yaw_deg: number;        // 偏航角 (deg)
  yaw_rate: number;       // 偏航角速度 (rad/s)

  // ===== 轮部数据 =====
  wheel_pos: number;      // 轮子位置 (rad, 平均)
  wheel_vel: number;      // 轮子速度 (rad/s, 平均)
  wheel_lifted: boolean;  // 轮部离地检测

  // ===== 电机输出 =====
  lmotor: number;         // 左电机电压 (V)
  rmotor: number;         // 右电机电压 (V)

  // ===== 控制器内部状态 =====
  ctrl: {
    angle_ctrl: number;      // 角度环输出
    gyro_ctrl: number;       // 角速度环输出
    dist_ctrl: number;       // 位移环输出
    speed_ctrl: number;      // 速度环输出
    yaw_output: number;      // Yaw控制输出
    lqr_u: number;           // 总平衡输出 (补偿前)
    lqr_u_comp: number;      // 总平衡输出 (补偿后)
    heading_target: number;  // 目标航向 (rad)
    heading_current: number; // 当前航向 (rad)
    dist_zero: number;       // 位移零点
    pitch_offset: number;    // 自适应重心偏移 (deg)
  };

  // ===== IMU 原始数据 =====
  imu: {
    ax: number; ay: number; az: number;  // 加速度 (m/s²)
    gx: number; gy: number; gz: number;  // 陀螺仪 (rad/s)
  };

  // ===== 系统状态 =====
  state: number;          // 0=停止, 1=运行, 2=故障
  fault: number;          // 故障位图
  runtime: number;        // 运行时间 (s)
}

// 参数同步
interface ParamsMessage {
  type: 'params';
  balance: {
    angle_kp: number;     gyro_kp: number;
    distance_kp: number;  speed_kp: number;
    yaw_angle_kp: number; yaw_gyro_kp: number;
    lqr_u_kp: number;     lqr_u_ki: number;
    zeropoint_kp: number;
    lpf_target_vel_tf: number;
    lpf_zeropoint_tf: number;
    max_tilt_deg: number;
    pitch_offset: number;
    pid_limit: number;
    lift_accel_thresh: number;
    lift_vel_thresh: number;
  };
  foc: {
    voltage_limit: number;
    velocity_limit: number;
    pid_p: number;
    pid_i: number;
    pid_d: number;
    motion_mode: number;
    torque_mode: number;
  };
  ctrl: {
    motor_enable: boolean;
    balance_enable: boolean;
    max_linear_vel: number;
    max_yaw_rate: number;
  };
}

// 确认消息
interface AckMessage {
  type: 'ack';
  cmd: string;
  status: 'ok' | 'error';
  msg?: string;
}
```

### 5.2 数据更新频率

| 数据类型 | 频率 | 说明 |
|---------|------|------|
| 姿态/电机 | 20Hz | 基础遥测 |
| 控制器状态 | 20Hz | 与遥测同帧 |
| IMU 原始值 | 10Hz | 节流发送 |
| 参数同步 | 按需 | 连接时/参数变更时 |
| 命令响应 | 即时 | < 50ms |

---

## 6. 界面设计方案

### 6.1 布局结构

```
┌─────────────────────────────────────────────────────────────┐
│  Status Bar (50px, Fixed)                                   │
│  [●已连接] [00:12:34] [无故障]        [遥控][监控][调试][设置] │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Tab Content Area                                           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  TAB: 遥控 (默认)                                     │  │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐  │  │
│  │  │              │ │              │ │              │  │  │
│  │  │   3D 姿态    │ │   实时曲线    │ │   关键指标    │  │  │
│  │  │   220×180    │ │   220×180    │ │  140×180     │  │  │
│  │  │              │ │              │ │              │  │  │
│  │  └──────────────┘ └──────────────┘ └──────────────┘  │  │
│  │  ┌────────────────────────┐ ┌──────────────────────┐  │  │
│  │  │     双摇杆控制区        │ │    快捷参数面板      │  │  │
│  │  │   ┌────┐    ┌────┐     │ │  [速度滑块]         │  │  │
│  │  │   │  ● │    │  ● │     │ │  [转向滑块]         │  │  │
│  │  │   └────┘    └────┘     │ │  [使能开关]         │  │  │
│  │  │   移动      转向       │ │  [紧急停止]         │  │  │
│  │  └────────────────────────┘ └──────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  TAB: 监控                                            │  │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐  │  │
│  │  │ IMU原始数据   │ │ 控制器分解   │ │  电机状态    │  │  │
│  │  │ 条形图可视化  │ │ 进度条显示   │ │  实时数值    │  │  │
│  │  └──────────────┘ └──────────────┘ └──────────────┘  │  │
│  │  ┌────────────────────────────────────────────────┐  │  │
│  │  │         完整遥测数据表格 (可滚动)                │  │  │
│  │  │  时间戳 | 俯仰 | 航向 | 左电机 | 右电机 | 状态   │  │  │
│  │  └────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  TAB: 调试                                            │  │
│  │  ┌─────────────────────┐ ┌─────────────────────────┐  │  │
│  │  │  PID 参数分组        │ │                         │  │  │
│  │  │  ▼ 角度环           │ │    响应曲线 (调参时)     │  │  │
│  │  │  [P] [I] [D] [应用]  │ │    ┌──────────────┐    │  │  │
│  │  │  ▼ 速度环           │ │    │  目标 vs 实际  │    │  │  │
│  │  │  [P] [I] [D] [应用]  │ │    │  实时对比     │    │  │  │
│  │  │  ▼ Yaw 控制         │ │    └──────────────┘    │  │  │
│  │  │  [P] [I] [D] [应用]  │ │                         │  │  │
│  │  └─────────────────────┘ └─────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
│  Telemetry Bar (60px, Fixed, 可折叠)                        │
│  [俯仰 0°] [航向 0°] [速度 0] [左 0V] [右 0V] [离地 否]      │
└─────────────────────────────────────────────────────────────┘
```

### 6.2 组件规范

#### 6.2.1 摇杆组件 (Joystick)

```
尺寸: 140×140px (外圈), 50×50px (摇杆头)
边框: 2px solid #30363d, active 时 #58a6ff
背景: #21262d
死区: 中心 10% 范围
最大行程: 45px
输出范围: -1.0 ~ +1.0 (X, Y)
更新频率: 20Hz (50ms 节流)
```

**交互状态**:
- 默认: 摇杆头居中，边框灰色
- 激活: 边框变亮蓝色，摇杆头跟随手指
- 释放: 弹簧动画回弹至中心 (200ms ease-out)

#### 6.2.2 数据卡片 (DataCard)

```
背景: #161b22
边框: 1px solid #30363d, 圆角 12px
内边距: 12px
标题: 14px, #58a6ff, 大写
数值: 18px, monospace, #3fb950
标签: 11px, #8b949e
```

#### 6.2.3 条形指示器 (BarIndicator)

```
容器高度: 24px
背景: #0d1117, 圆角 4px
填充: 渐变 (负值红色, 正值绿色, 零点居中)
范围: 可配置 (如 ±20 m/s²)
动画: width 过渡 100ms
数值标签: 右侧, 固定宽度, monospace
```

#### 6.2.4 实时曲线 (RealTimeChart)

```
Canvas 尺寸: 自适应父容器
时间窗口: 10秒 (200个数据点)
Y轴: 自动缩放或固定范围
网格线: #21262d, 1px
曲线: 2px, 抗锯齿
图例: 可点击切换显示/隐藏
```

曲线颜色定义:
- 俯仰角: #58a6ff (蓝)
- 电机输出: #f85149 (红)
- 目标值: #8957e5 (紫)
- 角速度: #3fb950 (绿)

### 6.3 响应式适配

#### iPad Mini 横屏 (默认)

```css
/* 4列网格 */
.control-layout {
  display: grid;
  grid-template-columns: 1fr 1fr 200px;
  grid-template-rows: 180px 220px;
}
```

#### iPad Mini 竖屏

```css
/* 2列堆叠 */
.control-layout {
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 150px 150px 200px auto;
}
```

#### 手机横屏

- 隐藏 3D 视图
- 曲线简化
- 摇杆并排置于底部

---

## 7. 技术实现细节

### 7.1 代码组织

```javascript
// 模块化结构 (IIFE 模式，单文件)
(function() {
  'use strict';

  // ===== 配置 =====
  const CONFIG = {
    WS_URL: `ws://${location.host}/ws`,
    TELEMETRY_RATE: 50,  // ms
    CHART_POINTS: 200,
    JOYSTICK_MAX_DIST: 45,
    TOUCH_MIN_SIZE: 44
  };

  // ===== 状态管理 =====
  class StateStore {
    constructor() {
      this.state = { /* 初始状态 */ };
      this.listeners = new Map();
    }
    get(path) { /* ... */ }
    set(path, value) { /* ... */ }
    subscribe(path, callback) { /* ... */ }
  }

  // ===== WebSocket 客户端 =====
  class WSClient {
    constructor(url, store) { /* ... */ }
    connect() { /* ... */ }
    send(command) { /* ... */ }
    onMessage(handler) { /* ... */ }
  }

  // ===== UI 组件 =====
  class Joystick {
    constructor(element, options) { /* ... */ }
    onMove(callback) { /* ... */ }
    onEnd(callback) { /* ... */ }
  }

  class RealTimeChart {
    constructor(canvas, options) { /* ... */ }
    addSeries(name, color) { /* ... */ }
    pushData(timestamp, values) { /* ... */ }
    render() { /* ... */ }
  }

  class BarIndicator {
    constructor(element, options) { /* ... */ }
    setValue(value) { /* ... */ }
  }

  // ===== 视图控制器 =====
  class ViewController {
    constructor(store) { /* ... */ }
    switchTab(tabName) { /* ... */ }
    updateTelemetry(data) { /* ... */ }
  }

  // ===== 初始化 =====
  document.addEventListener('DOMContentLoaded', () => {
    const store = new StateStore();
    const ws = new WSClient(CONFIG.WS_URL, store);
    const view = new ViewController(store);

    // 初始化组件...
    // 连接 WebSocket...
  });
})();
```

### 7.2 性能优化

#### 渲染优化

```javascript
// 1. 使用 requestAnimationFrame 节流渲染
let pendingRender = false;
function scheduleRender() {
  if (pendingRender) return;
  pendingRender = true;
  requestAnimationFrame(() => {
    render();
    pendingRender = false;
  });
}

// 2. Canvas 离屏渲染 (曲线)
const offscreenCanvas = document.createElement('canvas');
const offCtx = offscreenCanvas.getContext('2d');
// 在离屏 canvas 绘制，完成后一次 blit

// 3. 虚拟滚动 (长列表)
// 只渲染可视区域的数据行
```

#### 内存管理

```javascript
// 环形缓冲区实现
class RingBuffer {
  constructor(size) {
    this.size = size;
    this.buffer = new Array(size);
    this.index = 0;
    this.full = false;
  }

  push(value) {
    this.buffer[this.index] = value;
    this.index = (this.index + 1) % this.size;
    if (this.index === 0) this.full = true;
  }

  *values() {
    const start = this.full ? this.index : 0;
    for (let i = 0; i < (this.full ? this.size : this.index); i++) {
      yield this.buffer[(start + i) % this.size];
    }
  }
}
```

### 7.3 错误处理

```javascript
// WebSocket 重连策略
class ReconnectPolicy {
  constructor() {
    this.attempts = 0;
    this.maxDelay = 30000;  // 最大 30s
  }

  nextDelay() {
    // 指数退避: 1s, 2s, 4s, 8s... 最大 30s
    const delay = Math.min(1000 * Math.pow(2, this.attempts), this.maxDelay);
    this.attempts++;
    return delay;
  }

  reset() {
    this.attempts = 0;
  }
}

// 用户通知
function notifyUser(message, type = 'info') {
  // 非阻塞 toast 通知
  const toast = document.createElement('div');
  toast.className = `toast toast-${type}`;
  toast.textContent = message;
  document.body.appendChild(toast);
  setTimeout(() => toast.remove(), 3000);
}
```

---

## 8. 后端适配

### 8.1 需要修改的文件

| 文件 | 修改内容 |
|------|---------|
| `src/wifi_debug.cpp` | 更新 `sendTelemetry()`，添加新的数据字段 |
| `src/wifi_debug.cpp` | 添加新的命令解析处理 |
| `src/shared_state.h` | 可能需要扩展共享状态结构 |
| `src/control/balance_controller.h` | 添加 `getDebug()` 方法获取内部状态 |

### 8.2 Telemetry 扩展

```cpp
// 在 sendTelemetry() 中添加
void sendTelemetry() {
  BalanceDebug dbg;
  s_ctx->balance.getDebug(dbg);

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{"
    "\"type\":\"telem\","
    "\"t\":%lu,"
    // ... 原有字段 ...
    // 新增控制器内部状态
    "\"ctrl\":{"
      "\"angle_ctrl\":%.3f,"
      "\"gyro_ctrl\":%.3f,"
      "\"dist_ctrl\":%.3f,"
      "\"speed_ctrl\":%.3f,"
      "\"yaw_output\":%.3f,"
      "\"lqr_u\":%.3f,"
      "\"lqr_u_comp\":%.3f,"
      "\"heading_target\":%.3f,"
      "\"heading_current\":%.3f,"
      "\"dist_zero\":%.2f,"
      "\"pitch_offset\":%.4f"
    "},"
    // 新增 IMU 原始数据
    "\"imu\":{"
      "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
      "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f"
    "}"
    "}",
    millis(),
    // ... 对应变量
  );

  ws.textAll(buf);
}
```

### 8.3 命令处理扩展

```cpp
// 添加 PID 参数设置处理
if (type == "set_pid") {
  const char* group = doc["group"];
  auto params = doc["params"];

  PIDController* pid = nullptr;
  if (strcmp(group, "angle") == 0) pid = &s_ctx->balance.pidAngle();
  else if (strcmp(group, "gyro") == 0) pid = &s_ctx->balance.pidGyro();
  // ... 其他 PID

  if (pid) {
    if (params["kp"]) pid->P = params["kp"];
    if (params["ki"]) pid->I = params["ki"];
    if (params["kd"]) pid->D = params["kd"];
    sendAck(client, "set_pid", true);
  }
}
```

---

## 9. 实施计划

### Phase 1: 基础设施 (2-3 天)

- [ ] 创建新的 `dashboard.html` 框架
- [ ] 实现 StateStore 状态管理
- [ ] 实现 WSClient WebSocket 客户端
- [ ] 实现标签页导航系统
- [ ] 创建基础 CSS 样式系统

**验证标准**: 页面能加载，标签页可切换，能连接 WebSocket

### Phase 2: 核心功能 (3-4 天)

- [ ] 实现双摇杆组件
- [ ] 实现遥测数据显示 (数据卡片)
- [ ] 迁移 3D 姿态显示
- [ ] 迁移实时曲线显示
- [ ] 实现底部遥测栏

**验证标准**: 遥控功能完整可用，数据实时更新

### Phase 3: 监控功能 (2-3 天)

- [ ] 实现传感器条形可视化
- [ ] 实现控制器状态分解面板
- [ ] 实现遥测数据表格
- [ ] 后端添加新的 telemetry 字段

**验证标准**: 能看到所有传感器原始值和控制器内部状态

### Phase 4: 调试功能 (2-3 天)

- [ ] 实现 PID 参数调节界面
- [ ] 实现参数保存/加载
- [ ] 实现响应曲线叠加
- [ ] 后端添加 PID 设置命令

**验证标准**: 能实时调节 PID 并看到效果

### Phase 5: 优化与测试 (2-3 天)

- [ ] iPad Mini 真机测试
- [ ] 性能优化 (渲染、内存)
- [ ] 错误处理和边界情况
- [ ] 代码清理和文档

**验证标准**: iPad Mini 上 60fps，无内存泄漏

### Phase 6: 迁移与废弃 (1 天)

- [ ] 替换 `control.html` 和 `index.html`
- [ ] 更新构建脚本
- [ ] 更新文档

---

## 10. 风险评估

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|---------|
| iPad Safari 兼容性问题 | 高 | 中 | 早期真机测试，使用标准 API |
| WebSocket 消息过大 | 中 | 中 | 数据节流，分包发送 |
| 内存泄漏导致崩溃 | 高 | 低 | 定期测试，使用环形缓冲区 |
| 开发时间超期 | 中 | 中 | 分阶段交付，保留旧页面回退 |

---

## 11. 附录

### 11.1 参考资源

- [Apple Human Interface Guidelines - iPad](https://developer.apple.com/design/human-interface-guidelines/ipad)
- [WebSocket Best Practices](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [Canvas Performance Optimization](https://developer.mozilla.org/en-US/docs/Web/API/Canvas_API/Tutorial/Optimizing_canvas)

### 11.2 变更记录

| 日期 | 版本 | 变更 |
|------|------|------|
| 2026-02-14 | 1.0 | 初始提案 |

---

**评审人**: _________________  **日期**: _________________

**批准**: _________________  **日期**: _________________
