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

## 2026-10-24 vector incidence 优化

- 将 boundary poly 顶点 incidence 从 `std::set<int>` 改为保持排序的 `vector<int>`，插入、包含、并集和交集都保持原有集合语义。

## 2026-10-24 direct BSP 输出修复

- `pyboundary` 保持 `untitledx_pyboundary.bsp` 的外部三角面扩张路线；brush-union 只作为穿墙诊断实验，不作为最终算法。
- BSP 收尾阶段不再对存在 direct surface tree 的编译运行 `legacytrim`。legacytrim 会重写共享 clipnode 图，破坏 direct tree 的局部编号关系。
- 安装 direct tree 时，轴向负法向统一转换为 GoldSrc 的正轴表示，并交换子节点；否则 BSP 中的负轴平面会被引擎按轴向平面错误解释，表现为整面墙可穿越。
- `untitledx_pyboundary.bsp` 是当前形态参考：hull1/hull3 分别约 2406/2501 个节点，总计 4907 个 clipnodes。
- `cliptype_sl` 的下一轮验证必须同时检查墙体封闭和斜坡贴合，不能只用单个墙内点判断算法是否达标。
- `untitledx` 的 direct nodes 保持 hull1/hull3 `2406/2501`，最终 BSP clipnodes 保持 `4907`；完整编译两次约 `11.8s/12.0s`。
- 与参考 BSP 随机比较 300,000 点，hull1/2/3 判定差异均为 `0`；`cliptype_sl` 回归生成 hull1/hull3 各 `25` nodes；`ctest` 193/193 通过。

## 2026-10-24 增量 edge topology 实验

- 实验实现让 `clip_boundary_poly()` 从输入 fragment 继承未切断的 hull edges，只为新切割面生成边环，并在顶点合并/压缩后做索引重映射。
- 为避免闭合阶段的数值拓扑漂移，closure 仍使用原有 hull-edge 重建；增量路径只覆盖 source subtraction 和 fragment split。曾加入面环完整性检查，随后改为 `3V-6` 最小边数快速判定。
- 在 `untitledx` 上，增量版本与当前参考 BSP 随机 300,000 点的 hull1/2/3 判定均为 `0` 差异，`cliptype_sl` 仍为 hull1/hull3 各 `25` nodes。
- 但同机基线 CSG 约 `8.8s`，增量版本约 `10.7～14.6s`；cut-face 环生成、边集合映射和增量拓扑校验的开销超过了省下的 hull-edge 重建时间。增量源码已撤回，当前正式实现保持基线版本。
- 结论：仅维护 fragment edge topology 不是当前机器上的有效算法级取舍；后续应优先针对 source-plane 顶点分类（约 1,000 万次检查）做批量或缓存化，而不是继续堆叠边拓扑修补。

## 2026-10-24 source-plane side cache

- `boundary_outside_parts()` 现在只计算一次当前 fragment 对 source plane 的 signed distances，并把该数组传给正向/反向裁剪；`clip_boundary_poly()` 在有缓存时跳过重复点积，保持原有 EPS、交边筛选和顶点顺序。
- `untitledx` CSG 同机实测约 `8.7s`，相比约 `8.8s` 基线只有小幅收益；composition terms 和 direct nodes 保持 `250`、hull1/hull3 `2406/2501`。这是低风险的局部优化，后续仍需通过完整编译和随机判定验证。

## 2026-10-24 pyboundary gameplay leak fix

- `pyboundary` 的最终 hull 构建已改为 Python `brushbsp.py` 的直接路线：每个外部三角面分别做 box hull 扩张，再进入 fragment BSP；不再把三角面重新合并为 whole-entity 凸项。
- 旧的 whole-entity composition 树在 `cliptype_sl` 的凹接缝处漏碰撞。坐标 `(-30, 60, 20)`、hull1：SDHLT simple 与 Python 参考为 `CONTENTS_SOLID`，旧 C++ pyboundary 为 `CONTENTS_EMPTY`。
- 修复后同一点与 Python 参考一致；随机 200,000 个点的 hull1 判定差异为 `0`。
- `cliptype_sl` 修复版 direct tree：hull1 `146` nodes、hull3 `165` nodes；未压缩树分别为 `473/493`，与 Python fragment 路线的语义一致。
- 可测试文件：`D:/steam/steamapps/common/Half-Life/cstrike/maps/cliptype_sl_pyboundary_direct.bsp`，原 SDHLT simple `cliptype_sl.bsp` 未覆盖。
- 同一修改在 `untitledx_pyboundary.map` 上完成：942 个外部三角面，hull1/hull3 压缩后 `7553/7586` direct nodes，最终 `15139` clipnodes，完整编译约 `5.9s`，未触及 32,767 限制。

## 2026-10-24 恢复参考几何路线

- 上述 direct fragment 版本是穿墙诊断实验，不等于 `untitledx_pyboundary.bsp` 的参考形态；它会把同一地形拆成 7553/7586 个节点，体感上偏向 simple。
- 正式 `pyboundary` 恢复为外部表面提取后执行 `prepare_boundary_composition()`，再用 whole-entity terms 扩张并生成 fragment BSP。
- `untitledx` 当前恢复为 hull1/hull3 `2406/2501`、总 clipnodes `4907`，与参考 BSP 的 300,000 个随机 hull1 判定差异为 `0`。
- direct tree 安装保留原始平面方向和子节点顺序；这与参考 BSP 的 clipnode 编码一致，不能再对轴向平面做正轴规范化。
- 最新 C++ 输出与参考 `untitledx_pyboundary.bsp` 的 planes、clipnodes、models、visibility、lighting 等 BSP lump 均逐字节一致，只有实体文本中的编译信息不同。
