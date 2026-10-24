# `pyboundary` C++ 移植记录

## 目的

`pyboundary` 是独立的 CSG cliptype，用来把闭合 brush 组合按连续外表面生成 GoldSrc 碰撞。已有的 `simple`、`legacy`、`legacytrim`、`exactunion` 和 `pyexact` 路径不改动。

## 已移植的步骤

1. 从同一 entity 的闭合 solid brush 中提取外部面，并删除被其他 brush 覆盖的面。
2. 以外部三角面生成边界平面，删除高度不超过 `0.00025` 的微小垂直 seam，并将向上的屋顶平面抬高 `0.00025`。
3. 按 standing/crouched hull 的 `(+/-16,+/-16,+/-36)` 和 `(+/-16,+/-16,+/-18)` 扩张三角面。
4. 使用 Python `brushbsp.py` 对凸片段进行 fragment BSP 切分，生成 direct clipnode tree。
5. 通过 `.scl` sidecar 或 compile 的内存 handoff 把 direct tree 安装到 BSP 的 hull 1/2/3。

所有 entity 都使用同一套 whole-entity 自适应凸体组合；不再按 brush 数量切换到逐 brush 的 bounded source-term 近似，也不再进入 `solid_bsp`、`legacytrim` 或 `exactunion`。

## 验证样例

临时测试输入：`build/tmp/untitledx_pyboundary.map`。

```powershell
cmake --build build/vs --config Release --parallel 4 --target hltools
bin/hltools.exe compile -threads 2 -cliptype pyboundary build/tmp/untitledx_pyboundary.map
```

当前样例的 C++ CSG 阶段已完成：外部输入 1828 faces、942 triangles；删除 48 个微 seam planes；自适应组合访问约 900 个中间片段，生成 250 个 whole-entity terms；direct hull1/hull3 分别为 4014/3967 nodes。Python 基准为 252 terms，差异来自 C++/Qhull 顶点重建的数值路径，已经不再是 bounded source-term 近似。

## 使用方式

```text
-cliptype pyboundary <map>
```

也可以在 entity 的 `zhlt_cliptype` 中指定 `pyboundary`。原始 MAP 不会被修改。

## 后续工作

当前 C++ 已移植 Python `compose_source()` 的自适应流程，并在每次裁剪后按支撑半空间重建凸包顶点，避免共面/内部交点造成内存爆炸。约束表采用写时复制，普通多面体优先使用 incidence 边；source subtraction 的 disjoint fragment 和裁剪输入改用移动语义，减少顶点/incidence 的重复深拷贝。hull1/hull3 共享同一个外表面自适应分割拓扑，只分别执行扩张后的 fragment BSP，并行构建两个 hull。自适应分割对每个片段的平面范围只计算一次，同时复用成员判定和候选分割排序；支撑面检查改为无临时数组的直接扫描。新的增量约束缓存会沿 split 继承平面极值，仅在旧极值被切掉时重算。

## 2026-10-24 结构性优化

- source brush envelope 和外表面提取都建立一次静态 BVH；查询结果按原 brush 顺序恢复，只跳过 AABB 已证明不相交的 brush，不改变裁剪顺序。
- 移植 `brushbsp.py` 的 `compact()`：后序访问树，折叠两个相同子树，并用 `(plane, front, back)` 去重，再反转索引保持父节点先于子节点。
- `untitledx_pyboundary.map` 的直接树先从 `4014/3967` 压缩到约 `2541/2405`；随后 fragment incidence/sides/used 改为位集，当前实测 hull1/hull3 为 `2682/2454`，最终 BSP clipnodes 为 `5136`。随机 10,000 点对两个版本的 hull1/2/3 判定均为 0 差异。
- 位集版本完整 `compile -threads 2 -cliptype pyboundary` 约 17.4 秒（CSG 单独运行约 14～17 秒，受机器负载影响）；`ctest` 193/193 通过。
- 位集版本的节点数略高于纯压缩版本，后续仍需优化 splitter 评分，使节点数和生成时间同时下降。
- boundary poly 现在缓存自身的 hull-edge 拓扑；同一片段的正反裁剪共享这份缓存。`untitledx` CSG 实测降至约 `12.8s`，完整编译约 `16.1s`，clipnodes 仍为 `5136`，随机 hull 判定仍为 0 差异。

## 2026-10-24 splitter scratch 优化

- `split_fragment()` 现在在每个树节点复用 `side`、cut 顶点和 cut incidence 缓冲；`build_tree()` 复用候选平面 generation 标记数组，并按评分阶段得到的 front/back/cut 数量预留子树容量。
- 这些改动不改变候选排序、EPS、切割顺序或 compact 规则；`untitledx` 仍生成 hull1/hull3 `2682/2454` direct nodes 和 `5136` final clipnodes。
- Release 实测完整 `compile -threads 2 -cliptype pyboundary` 约 `16.1～16.3s`；193/193 项测试通过。相对单次运行的差异应以同机重复测量为准。
- `split_fragment()` 的边交集检查进一步改为先无分配统计共同 incidence 位数，只有确认至少两个共同约束时才创建交集 bitset；输出拓扑保持不变，单次完整编译受机器负载影响约 `16.7～16.9s`，暂未观察到稳定的总耗时下降。

## 2026-10-24 splitter 候选集取舍

- 将 fragment BSP 每层的随机候选上限从 `24` 降为 `12`。`untitledx` 的 direct nodes 变为 hull1/hull3 `2406/2501`，最终 `4907` 个 clipnodes，完整编译约 `15.9s`。
- 与上一份参考 BSP 随机抽取 300,000 个点比较，hull1、hull2、hull3 判定差异均为 `0`；`cliptype_sl` 回归也正常生成 hull1/hull3 各 `25` nodes。
- 上限 `48` 的实验反而生成 `2764/2486` direct nodes、`5250` final clipnodes，因此不采用。候选上限是当前算法级的可调参数：`12` 偏向节点数和速度，继续降低可能增加随机拆分波动。

## 算法级实验记录

- closure 前缀缓存实验未采用：虽然 direct nodes 不变，但 `std::map/shared_ptr` 的缓存管理使完整编译从约 `15.9s` 退化到 `19.1s`。
- composition pending 窗口的 `std::async` 预取实验未采用：几何结果不变，但由于 pending 顺序导致部分预取浪费，总耗时约 `16.2s`，没有超过当前基线。
- 当前保留的算法级取舍仍是 splitter 候选上限 `12`；若继续追求小秒级，需要改成共享裁剪 DAG 或固定线程池批处理，而不是继续增加临时缓存。

## 2026-10-24 composition profiling

- `untitledx` 的当前基线 composition 约 `8.7s`：326 次独立 evaluation 中 closure 约 `1.4s`、source subtraction 约 `7.2s`、adaptive split 约 `0.04s`。
- subtraction 统计约 `270,974` 次 `clip_boundary_poly`、`123,120` 次 hull-edge 构建和 `9,959,286` 次 source plane vertex 检查。这里才是继续压到小秒级的主要目标。
- 将 hull-edge 的顶点对扫描改为按 supporting face 用 `std::map` 计数的实验反而使 composition 增至约 `9.9s`，已撤回。后续若继续，应实现裁剪过程的增量 edge topology，避免每个新 fragment 重建通用 hull-edge 集合。

验证副本：`build/tmp/untitledx_pyboundary.bsp`；上一版纯压缩副本：`build/tmp/untitledx_pyboundary_compacted.bsp`。

最新验证：`untitledx` 的 CSG 阶段约 15～16 秒，完整 CSG/BSP/VIS/RAD 编译耗时约 19.3 秒；最终仍为 7,981 个 clipnodes（32,767 上限的 24.4%）、hull1/hull3 为 4,014/3,967，几何统计与前一版一致。193/193 项测试通过。
