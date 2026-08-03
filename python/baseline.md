# Phase 3 baseline - complementary filter

Seeds [1, 2, 3, 4, 5], 300 s, 101.2229 Hz, dt = 0.0098792 s. Tau swept over [0.005, 100.0] s, 41 points. Values are means over seeds.

| scenario | profile | tau_opt (s) | RMS all | RMS static | RMS dynamic | peak | accel-only |
|---|---|---|---|---|---|---|---|
| clean | gentle | 0.0221 | 0.0194 | 0.0192 | 0.0196 | 0.0830 | 0.0398 |
| clean | brisk | 0.0221 | 0.0194 | 0.0192 | 0.0196 | 0.0830 | 0.0398 |
| lever 30 mm | brisk | 0.2050 | 0.1548 | 0.0881 | 0.1826 | 0.3806 | 0.1891 |
| lever 30 mm + vib | brisk | 0.2050 | 0.1559 | 0.0907 | 0.1833 | 0.4889 | 0.3482 |
| lever + vib + taps | brisk | 0.2050 | 0.1598 | 0.0974 | 0.1866 | 0.7828 | 0.5385 |
| legacy dist 0.5 | gentle | 1.0187 | 1.0334 | 1.1756 | 0.9288 | 10.2539 | 2.9211 |

All errors in degrees RMS unless stated.

Caveats: synthetic data only, validated against a simulator built from the Phase 2 measurements rather than against independent truth. The 1/f flicker component is not synthesised, which flatters any filter carrying an explicit bias state more than it flatters the complementary filter.
