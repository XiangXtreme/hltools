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
- `untitledx_pyboundary.map` 的 hull1/hull3 从 `4014/3967` 个直接节点压缩为 `2541/2405`，最终 BSP clipnodes 从 `7981` 降为 `4946`；随机 10,000 点对 hull1/2/3 的旧树和压缩树判定均为 0 差异。
- 完整 `compile -threads 2 -cliptype pyboundary` 约 20.3 秒，CSG 阶段约 16.0 秒；`ctest` 193/193 通过。

验证副本：`build/tmp/untitledx_pyboundary_compacted.bsp`。

最新验证：`untitledx` 的 CSG 阶段约 15～16 秒，完整 CSG/BSP/VIS/RAD 编译耗时约 19.3 秒；最终仍为 7,981 个 clipnodes（32,767 上限的 24.4%）、hull1/hull3 为 4,014/3,967，几何统计与前一版一致。193/193 项测试通过。
