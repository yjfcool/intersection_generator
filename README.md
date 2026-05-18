# 高精度地图路口内车道生成系统

## 概述

本系统以路口外已知车道组数据、连通关系、障碍物和道路边缘线为输入，在路口内自动生成：

- **路口内中心线**（G1平滑贝塞尔曲线、避障、非相交约束）
- **路口内车道边线**（端点与路口外严格重合、局部平滑）
- **精细路口面**（封闭多边形）

支持场景：十字、T字、Y字、井字、环岛等。

## 依赖

| 库 | 用途 | 必须 |
|----|------|------|
| C++17 | 语言标准 | ✅ |
| CMake ≥ 3.14 | 构建系统 | ✅ |
| nlohmann/json | JSON（已内嵌于 third_party/） | ✅ 已包含 |
| GEOS | 精确几何运算（可选增强） | ❌ 可选 |
| proj | 精确坐标变换（可选增强） | ❌ 可选 |

> **零外部依赖即可编译运行**：GEOS 和 proj 均为可选项，系统已内置等效实现。

## 快速开始

```bash
# 1. 构建
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 2. 运行（十字路口示例）
./intersection_gen \
    -c ../config.json \
    -f both \
    -o ./output/cross \
    ../test/integration/data/cross_intersection.json

# 3. 运行测试
ctest --output-on-failure
```

## 命令行参数

```
用法: intersection_gen [选项] <输入JSON文件>

必需参数:
  <输入JSON文件>           输入数据文件路径

可选参数:
  -c, --config <文件>      配置文件路径（默认：./config.json）
  -o, --output-dir <目录>  输出目录（默认：./output）
  -f, --format <格式>      输出格式: shapefile | json | both（默认：both）
  --safe-margin <米>       障碍物安全距离（默认：0.5m）
  --sampling <模式>        采样模式: adaptive | fixed_spacing | fixed_count
  --spacing <米>           固定采样间距（--sampling=fixed_spacing 时有效）
  --no-phase2              禁用 Phase2 数值优化（加快速度）
  --verbose                输出详细调试日志
  -h, --help               显示帮助信息
```

## 输入 JSON 格式

```json
{
  "scenario_id": "cross_01",
  "ref_point": { "lon": 116.391, "lat": 39.907 },
  "coord_type": "projected",
  "lane_groups":  [ ... ],
  "centerlines":  [ ... ],
  "edgelines":    [ ... ],
  "connections":  [ ... ],
  "obstacles":    [ ... ],
  "stop_lines":   [ ... ],
  "road_edges":   [ ... ],
  "rough_area":   { ... }
}
```

### coord_type
- `"projected"`：输入坐标为平面投影坐标（米），直接使用
- `"wgs84"`：输入坐标为 WGS84 经纬度，系统自动转换

### 连通关系 turn_type 取值
`straight` | `left` | `right` | `u_turn_left` | `u_turn_right`

### 障碍物 geom_type 取值
`point` | `line` | `polygon`

## 输出文件

### Shapefile（推荐用 QGIS 打开）
| 文件 | 内容 |
|------|------|
| `int_centerlines.*` | 路口内中心线（POLYLINE） |
| `int_edgelines.*`   | 路口内车道边线（POLYLINE） |
| `int_polygon.*`     | 精细路口面（POLYGON） |

### JSON
| 文件 | 内容 |
|------|------|
| `<scenario_id>_output.json` | 完整输出含质量标记 |

## 质量标记（quality_flags 位掩码）

| 值 | 含义 |
|----|------|
| 0x000 | 正常，无问题 |
| 0x001 | 障碍物穿越警告（避障失败，非相交优先） |
| 0x002 | 非相交约束残余警告 |
| 0x004 | 走廊过窄警告 |
| 0x008 | 曲率过高警告（驾驶舒适性） |
| 0x010 | 边线端点对齐失败警告 |
| 0x020 | 无解错误（几何严重退化） |
| 0x040 | 输入数据退化错误 |
| 0x080 | 中间调头排除非相交约束（信息） |
| 0x100 | 使用两段复合贝塞尔（信息） |

## 配置参数说明

主要参数见 `config.json`，关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `obstacle.safe_margin` | 0.5m | 障碍物安全距离 |
| `sampling.mode` | adaptive | 采样模式 |
| `bezier.alpha_straight` | 0.35 | 直行控制点比例 |
| `bezier.alpha_left` | 0.45 | 左转控制点比例 |
| `conflict.priority` | non_intersect | 冲突优先级 |
| `non_intersect.enable_mid_uturn_exclude` | true | 中间调头排除 |
| `output.format` | both | 输出格式 |

## 项目结构

```
intersection_generator/
├── CMakeLists.txt
├── config.json                      # 默认配置
├── README.md
├── src/
│   ├── main.cpp                     # 程序入口
│   ├── core/
│   │   ├── data_types.h             # 核心数据结构
│   │   ├── config.h / config.cpp    # 配置
│   │   ├── preprocess/              # 预处理模块
│   │   │   ├── coord_transformer.h  # 坐标变换
│   │   │   ├── topology_analyzer.h  # 拓扑分析
│   │   │   ├── tangent_estimator.h  # 切线估算
│   │   │   └── obstacle_index.h     # 障碍物空间索引
│   │   ├── bezier/
│   │   │   └── cubic_bezier.h       # 贝塞尔曲线
│   │   ├── generator/               # 核心生成
│   │   │   ├── corridor_allocator.h
│   │   │   ├── control_point_init.h
│   │   │   ├── obstacle_avoider.h
│   │   │   ├── non_intersect_enforcer.h
│   │   │   ├── conflict_resolver.h
│   │   │   └── curve_generator.h    # 主生成器
│   │   ├── postprocess/
│   │   │   ├── edge_line_generator.h
│   │   │   └── polygon_builder.h
│   │   └── special/
│   │       └── roundabout_handler.h
│   ├── io/
│   │   ├── json_parser.h
│   │   ├── json_writer.h
│   │   └── shp_writer.h             # 自实现Shapefile写入
│   └── utils/
│       ├── geom_utils.h
│       └── logger.h
├── third_party/
│   └── nlohmann/json.hpp            # 内嵌JSON库
└── test/
    └── integration/data/
        ├── cross_intersection.json  # 十字路口
        ├── t_intersection.json      # T字路口
        ├── y_intersection.json      # Y字路口
        └── roundabout.json          # 环岛
```

## 算法简介

### 曲线生成
- 三次贝塞尔曲线（G1连续，端点精确锁定）
- 调头/大曲率场景自动降级为两段复合贝塞尔
- 控制点自由度：α（P1距离比例）、β（P2距离比例）

### 避障（两阶段）
- Phase 1：控制点走廊推移迭代（快速）
- Phase 2：梯度下降数值优化（精确）

### 非相交约束
- 生成顺序：直行 → 右转（内→外）→ 左转（内→外）→ 调头
- 走廊约束：每条线在已生成相邻线之间的空间内生成
- 中间调头车道可排除非相交约束

### 冲突协调
- 默认：非相交优先，避障降级为警告
- 可配置为：避障优先 / 两者均标记错误

## 版本历史
- v1.0.0 (2025-08): 初始版本，全功能实现
