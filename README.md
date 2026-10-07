# NEXarm 工程工作区（A:\NEXarm\Claude）

美团低空经济挑战赛 · NexArm 六轴机械臂（ESP32 主控 + AT32 运动控制 + 6×HX-30HM 舵机 + K230 视觉）

## 入口导航

| 我想… | 从这里开始 |
|---|---|
| **了解目录怎么分的、文件在哪** | **`INDEX.md`** ← 分类依据、每类用途、给 agent 的读取建议 |
| **让机械臂动起来 / 复现运动逻辑** | `舵机基础运行逻辑/`（规格 + 机器可读 profile + 可运行库 + 示例） |
| 看当前进度、待办、未决问题 | `PROJECT_STATE.md` |
| 查历史实验的原始数据 | `runs/<实验名>/` |
| 查编译与烧录记录 | `logs/` |
| 找厂商教程原文 | `reference/` |

## 目录速览

```
Claude/
├─ INDEX.md                  ★ 分类依据与索引（先读这个）
├─ PROJECT_STATE.md            项目状态与交接
├─ README.md                   本文件
├─ docs/                       文档：notes 复盘 / reviews 审查
├─ reference/                  厂商 PDF 教程
├─ scripts/                    构建(build) / 采集(capture) / 自检(test) / 工具(tools)
├─ logs/                       编译、烧录、哈希清单
├─ runs/                       每次实验的 .bin/.json/.log 采集，按实验分目录
├─ artifacts/                  固件产物、diff、临时检查文本
├─ 舵机基础运行逻辑/            ★ 运动控制交付物（规格 + 库 + 示例 + 证据）
├─ <各实验目录>/                源码与探针工程（被构建脚本以绝对路径引用，勿移动）
├─ .toolchains/ .arduino-build-*/ …     构建缓存（勿移动，可重建）
└─ .workbuddy/                 记忆与工具数据（勿动）
```

详细分类依据、完整文件归属、以及**为什么有些东西必须留在原地**，见 `INDEX.md`。
