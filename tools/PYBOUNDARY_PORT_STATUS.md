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

当前 C++ 已移植 Python `compose_source()` 的自适应流程，并在每次裁剪后按支撑半空间重建凸包顶点，避免共面/内部交点造成内存爆炸。约束表采用写时复制，普通多面体优先使用 incidence 边；source subtraction 的 disjoint fragment 和裁剪输入改用移动语义，减少顶点/incidence 的重复深拷贝。

最新验证：`untitledx` 的 CSG 阶段耗时约 28.5 秒，完整 CSG/BSP/VIS/RAD 编译耗时约 33.3 秒；最终仍为 7,981 个 clipnodes（32,767 上限的 24.4%）、hull1/hull3 为 4,014/3,967，几何统计与前一版一致。193/193 项测试通过。
