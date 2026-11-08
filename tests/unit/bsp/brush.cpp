#include "bsp/internal.h"
#include "support/test.h"

namespace
{
void check_fragment(const bsp::brush *fragment, double min_x, double max_x)
{
    int sides = 0;
    for (const bsp::side *side = fragment->sides; side; side = side->next)
    {
        ++sides;
        expect(side->winding_.size() == 4);
        expect(side->winding_.area() > 0);
        for (const auto &point : side->winding_.points())
        {
            expect(point.x >= min_x - 1e-6);
            expect(point.x <= max_x + 1e-6);
            expect(point.z >= 63.99 - 1e-6);
            expect(point.z <= 64 + 1e-6);
        }
    }
    // All six closing planes must survive. A leaf containing an open
    // detail fragment can otherwise hide unrelated exterior faces.
    expect(sides == 6);
}
} // namespace

test("detail brush clipping preserves closed fragments thinner than on-epsilon")
{
    bsp::brush *brush = bsp::brush_from_box({0, 0, 0}, {64, 64, 64});
    require(brush != nullptr);
    bsp::plane shallow;
    shallow.normal = {0, 0, 1};
    shallow.dist = 63.99;
    bsp::clip_brush(&brush, &shallow);
    require(brush != nullptr);
    check_fragment(brush, 0, 64);

    bsp::plane split;
    split.normal = {1, 0, 0};
    split.dist = 32;
    bsp::brush *front = nullptr;
    bsp::brush *back = nullptr;
    bsp::split_brush(brush, &split, &front, &back);
    require(front != nullptr);
    require(back != nullptr);
    check_fragment(front, 32, 64);
    check_fragment(back, 0, 32);
    bsp::free_brush(front);
    bsp::free_brush(back);
}
