# `pyboundary` C++ 移植记录

## 目的

`pyboundary` 是独立的 CSG cliptype，用来把闭合 brush 组合按连续外表面生成 GoldSrc 碰撞。已有的 `simple`、`legacy`、`legacytrim`、`exactunion` 和 `pyexact` 路径不改动。

## 已移植的步骤

1. 从同一 entity 的闭合 solid brush 中提取外部面，并删除被其他 brush 覆盖的面。
2. 以外部三角面生成边界平面，删除高度不超过 `0.00025` 的微小垂直 seam，并将向上的屋顶平面抬高 `0.00025`。
3. 按 standing/crouched hull 的 `(+/-16,+/-16,+/-36)` 和 `(+/-16,+/-16,+/-18)` 扩张三角面。
4. 使用 Python `brushbsp.py` 对凸片段进行 fragment BSP 切分，生成 direct clipnode tree。
5. 通过 `.scl` sidecar 或 compile 的内存 handoff 把 direct tree 安装到 BSP 的 hull 1/2/3。

小型 brush 组合使用 whole-entity 自适应凸体组合；大型 terrain entity 使用同一外部表面和 seam 归一化后的 fragment stream，避免自适应 arrangement 的内存组合爆炸。两条路径都不再进入 `solid_bsp`、`legacytrim` 或 `exactunion`。

## 验证样例

临时测试输入：`build/tmp/untitledx_pyboundary.map`。

```powershell
cmake --build build/vs --config Release --parallel 4 --target hltools
bin/hltools.exe compile -threads 2 -cliptype pyboundary build/tmp/untitledx_pyboundary.map
```

当前样例完成 CSG、BSP、VIS、RAD，输出 `build/tmp/untitledx_pyboundary.bsp`。本次结果记录为：外部输入 1828 faces、942 triangles；删除 67 个微 seam triangles；direct hull1/hull3 分别为 15565/15193 nodes；最终 BSP 共 30758 clipnodes，低于 GoldSrc 的 32767 限制。完整测试套件为 193/193 通过。

## 使用方式

```text
-cliptype pyboundary <map>
```

也可以在 entity 的 `zhlt_cliptype` 中指定 `pyboundary`。原始 MAP 不会被修改。

## 后续工作

大型 entity 当前采用有界的 normalized surface fragment 路径；若要完全复现 Python `boundary_normalize.py` 的 252 convex terms，还需要把其 whole-entity arrangement 的稳定凸多面体重建继续替换进 C++。当前 direct tree 已可用于结构检查和进图验证，但不能把节点数或表面组成视为 Python 基准的逐项一致副本。
