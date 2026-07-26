// Tests for the LiDAR close-range down-weighting core (utils/range_weighting.h).
//
// The mechanism inflates a return's variance by (range_ref/r)^2 below range_ref
// so one nearby surface cannot dominate H^T R^-1 H in an otherwise-open scene.
// Its ABSOLUTE form is scene-blind, which starves confined spaces (tunnel /
// culvert / truck bed) where every return is close: every row is inflated by
// the same factor, which re-balances nothing and merely scales the whole LiDAR
// information block down against the prior. These tests pin both the legacy
// behaviour and the relative form that fixes it.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "utils/range_weighting.h"

using resple::range::RangeNoiseConfig;
using resple::range::rangeNoiseNormalizer;
using resple::range::rangeNoiseScale;
using resple::range::rangeNoiseWeight;

namespace {

// Total LiDAR information a scan contributes, up to the shared 1/var_pt factor:
// each row enters the IEKF as 1/(var_pt * weight).
double totalInformation(const RangeNoiseConfig& cfg,
                        const std::vector<float>& ranges)
{
    float r_max = 0.f;
    for (float r : ranges) r_max = std::max(r_max, r);
    const double norm = rangeNoiseNormalizer(cfg, r_max);
    double info = 0.0;
    for (float r : ranges) info += 1.0 / rangeNoiseWeight(cfg, r, norm);
    return info;
}

}  // namespace

TEST(RangeWeighting, AbsoluteScaleMatchesLegacyFormula)
{
    RangeNoiseConfig cfg;  // range_ref 3.0, scale_max 900, relative false
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 3.0f), 1.0);
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 10.0f), 1.0);   // beyond ref
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 1.0f), 9.0);    // (3/1)^2
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 0.5f), 36.0);   // (3/0.5)^2
    // 0.1 m range floor, then the scale_max ceiling. NB: float 0.1f widens to
    // 0.10000000149..., so a 0.1f input lands a hair under the 900 ceiling
    // while 0.0f/-5.0f (clamped to the exact 0.1 literal) hit it exactly.
    EXPECT_NEAR(rangeNoiseScale(cfg, 0.1f), 900.0, 1e-3);
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 0.0f), 900.0);
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, -5.0f), 900.0);
    cfg.scale_max = 16.0;
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 0.5f), 16.0);   // clamped
}

TEST(RangeWeighting, DisabledByNonPositiveRef)
{
    RangeNoiseConfig cfg;
    cfg.range_ref = 0.0;
    EXPECT_DOUBLE_EQ(rangeNoiseScale(cfg, 0.2f), 1.0);
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, 0.2f), 1.0);
    cfg.relative = true;  // relative is inert when the mechanism is off
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, 0.2f), 1.0);
    EXPECT_DOUBLE_EQ(rangeNoiseWeight(cfg, 0.2f, 1.0), 1.0);
}

// Absolute mode is bit-identical to the legacy path: the normalizer is 1.
TEST(RangeWeighting, AbsoluteModeNormalizerIsOne)
{
    RangeNoiseConfig cfg;  // relative = false
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, 1.0f), 1.0);
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, 50.0f), 1.0);
    for (float r : {0.2f, 0.5f, 1.0f, 2.9f, 3.0f, 25.0f}) {
        EXPECT_DOUBLE_EQ(rangeNoiseWeight(cfg, r, rangeNoiseNormalizer(cfg, r)),
                         rangeNoiseScale(cfg, r))
            << "absolute mode must not change the legacy weight at r=" << r;
    }
}

// No valid return supplied -> normalizer 1.0, so callers can divide blindly.
TEST(RangeWeighting, RelativeModeHandlesEmptyScan)
{
    RangeNoiseConfig cfg;
    cfg.relative = true;
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, 0.0f), 1.0);
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(cfg, -1.0f), 1.0);
}

// THE OPEN-SCENE PROPERTY: with any return at/beyond range_ref the normalizer is
// 1, so relative mode is identical to absolute — the anti-domination behaviour
// the mechanism exists for is preserved exactly.
TEST(RangeWeighting, RelativeModeUnchangedInOpenScene)
{
    RangeNoiseConfig abs_cfg;
    RangeNoiseConfig rel_cfg;
    rel_cfg.relative = true;

    // One intruding surface at 0.5 m, the rest of the scene open.
    const std::vector<float> ranges{0.5f, 4.0f, 8.0f, 15.0f, 30.0f};
    EXPECT_DOUBLE_EQ(rangeNoiseNormalizer(rel_cfg, 30.0f), 1.0);
    for (float r : ranges) {
        EXPECT_DOUBLE_EQ(rangeNoiseWeight(rel_cfg, r, 1.0),
                         rangeNoiseScale(abs_cfg, r));
    }
    EXPECT_DOUBLE_EQ(totalInformation(rel_cfg, ranges),
                     totalInformation(abs_cfg, ranges));
    // ...and the close return is still suppressed 36x relative to a far one.
    EXPECT_DOUBLE_EQ(rangeNoiseWeight(rel_cfg, 0.5f, 1.0) /
                         rangeNoiseWeight(rel_cfg, 8.0f, 1.0),
                     36.0);
}

// THE CONFINED-SPACE FIX: every return close (tunnel / truck bed). Absolute mode
// starves the update uniformly; relative mode restores full information while
// keeping the (here: uniform) relative weighting.
TEST(RangeWeighting, RelativeModeRestoresInformationInConfinedSpace)
{
    RangeNoiseConfig abs_cfg;
    RangeNoiseConfig rel_cfg;
    rel_cfg.relative = true;

    // Truck bed / narrow culvert: walls 0.8-1.2 m away, nothing beyond.
    const std::vector<float> ranges{0.8f, 0.9f, 1.0f, 1.1f, 1.2f};
    const double n = static_cast<double>(ranges.size());

    const double info_abs = totalInformation(abs_cfg, ranges);
    const double info_rel = totalInformation(rel_cfg, ranges);
    const double norm = rangeNoiseNormalizer(rel_cfg, 1.2f);

    // The reference is the farthest return: (3/1.2)^2 (1e-6 tolerance because
    // 1.2f widens to 1.20000004...).
    EXPECT_NEAR(norm, 6.25, 1e-5);
    // Absolute: every row inflated 6-14x, so the update carries ~1/9th of the
    // information the same geometry would contribute in an open scene.
    EXPECT_LT(info_abs, n / 5.0) << "absolute mode should be starved here";
    // Relative: the farthest return carries weight exactly 1...
    EXPECT_DOUBLE_EQ(rangeNoiseWeight(rel_cfg, 1.2f, norm), 1.0);
    // ...so the whole scan's information is recovered by exactly the
    // normalizer. It does NOT reach n=5: the scan still spans 0.8-1.2 m, and
    // that genuine 2.25x relative spread is deliberately preserved (see the
    // ordering check below). Anchoring, not flattening, is the goal.
    EXPECT_NEAR(info_rel, info_abs * norm, 1e-9);
    EXPECT_GT(info_rel, 0.6 * n) << "relative mode should not be starved";
    EXPECT_GT(info_rel / info_abs, 5.0)
        << "relative mode recovered only " << (info_rel / info_abs) << "x";

    // Relative ORDERING within the scan is untouched — closer returns are still
    // down-weighted against farther ones by exactly the same ratio.
    for (float r : ranges) {
        EXPECT_NEAR(rangeNoiseWeight(rel_cfg, r, norm) /
                        rangeNoiseWeight(rel_cfg, 1.2f, norm),
                    rangeNoiseScale(abs_cfg, r) / rangeNoiseScale(abs_cfg, 1.2f),
                    1e-12)
            << "relative weighting changed at r=" << r;
    }
}

// A uniformly-close scan must never be weighted worse than the same scan would
// be if the mechanism were disabled outright (that is the starvation symptom).
TEST(RangeWeighting, RelativeModeNeverStarvesVersusDisabled)
{
    RangeNoiseConfig off_cfg;
    off_cfg.range_ref = 0.0;
    RangeNoiseConfig rel_cfg;
    rel_cfg.relative = true;

    for (float wall : {0.3f, 0.5f, 0.8f, 1.5f, 2.5f}) {
        const std::vector<float> ranges{wall, wall, wall};
        EXPECT_NEAR(totalInformation(rel_cfg, ranges),
                    totalInformation(off_cfg, ranges), 1e-12)
            << "uniform scan at " << wall << " m lost information";
    }
}
