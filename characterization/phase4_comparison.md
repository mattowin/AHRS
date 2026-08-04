# Phase 4 - filter comparison

Synthetic records, 300 s at 101.2229 Hz (dt = 0.0098792 s), seeds [1, 2, 3, 4, 5], values are means over seeds. All errors in degrees.

Kalman Q and R derived from the Phase 2 Allan deviation results; no tuning against these records.


## clean, gentle

| variant | tau (s) | RMS static | RMS dynamic | peak | bias RMS (deg/s) | bias settle (s) | updates used | tuning inputs |
|---|---|---|---|---|---|---|---|---|
| CF, tuned per scenario | 0.0221 | 0.0192 | 0.0196 | 0.0830 | n/a | n/a | 100.0% | 1 swept |
| CF, tuned on clean only | 0.0221 | 0.0192 | 0.0196 | 0.0830 | n/a | n/a | 100.0% | 1 swept |
| KF, bare | 0.8148 | 0.0045 | 0.0042 | 0.0453 | 0.0119 | 1.5 | 100.0% | 3 measured |
| KF + gate | 0.8148 | 0.0045 | 0.0042 | 0.0453 | 0.0119 | 1.5 | 100.0% | 3 measured + 2 tuned |
| KF + lever comp | 0.8148 | 0.0046 | 0.0098 | 0.0476 | 0.0124 | 1.5 | 100.0% | 3 measured + r |
| KF + gate + comp | 0.8148 | 0.0046 | 0.0098 | 0.0476 | 0.0124 | 1.5 | 100.0% | 3 measured + 2 + r |

## clean, brisk

| variant | tau (s) | RMS static | RMS dynamic | peak | bias RMS (deg/s) | bias settle (s) | updates used | tuning inputs |
|---|---|---|---|---|---|---|---|---|
| CF, tuned per scenario | 0.0221 | 0.0192 | 0.0196 | 0.0830 | n/a | n/a | 100.0% | 1 swept |
| CF, tuned on clean only | 0.0221 | 0.0192 | 0.0196 | 0.0830 | n/a | n/a | 100.0% | 1 swept |
| KF, bare | 0.8148 | 0.0045 | 0.0042 | 0.0453 | 0.0119 | 1.5 | 100.0% | 3 measured |
| KF + gate | 0.8148 | 0.0045 | 0.0042 | 0.0453 | 0.0119 | 1.5 | 100.0% | 3 measured + 2 tuned |
| KF + lever comp | 0.8148 | 0.0046 | 0.0554 | 0.1134 | 0.0124 | 1.5 | 100.0% | 3 measured + r |
| KF + gate + comp | 0.8148 | 0.0046 | 0.0554 | 0.1134 | 0.0124 | 1.5 | 100.0% | 3 measured + 2 + r |

## lever 30 mm, brisk

| variant | tau (s) | RMS static | RMS dynamic | peak | bias RMS (deg/s) | bias settle (s) | updates used | tuning inputs |
|---|---|---|---|---|---|---|---|---|
| CF, tuned per scenario | 0.2050 | 0.0881 | 0.1826 | 0.3806 | n/a | n/a | 100.0% | 1 swept |
| CF, tuned on clean only | 0.0221 | 0.0194 | 0.2310 | 0.4415 | n/a | n/a | 100.0% | 1 swept |
| KF, bare | 0.8148 | 0.0046 | 0.0555 | 0.1121 | 0.0119 | 1.5 | 100.0% | 3 measured |
| KF + gate | 0.8148 | 0.0046 | 0.0555 | 0.1121 | 0.0119 | 1.5 | 100.0% | 3 measured + 2 tuned |
| KF + lever comp | 0.8148 | 0.0046 | 0.0060 | 0.0472 | 0.0124 | 1.5 | 100.0% | 3 measured + r |
| KF + gate + comp | 0.8148 | 0.0046 | 0.0060 | 0.0472 | 0.0124 | 1.5 | 100.0% | 3 measured + 2 + r |

## lever+vib+taps, brisk

| variant | tau (s) | RMS static | RMS dynamic | peak | bias RMS (deg/s) | bias settle (s) | updates used | tuning inputs |
|---|---|---|---|---|---|---|---|---|
| CF, tuned per scenario | 0.2050 | 0.0974 | 0.1866 | 0.7828 | n/a | n/a | 100.0% | 1 swept |
| CF, tuned on clean only | 0.0221 | 0.1691 | 0.2861 | 3.8892 | n/a | n/a | 100.0% | 1 swept |
| KF, bare | 0.8148 | 0.0220 | 0.0583 | 0.6354 | 0.1004 | 9.6 | 100.0% | 3 measured |
| KF + gate | 0.8148 | 0.0292 | 0.0749 | 0.6049 | 0.0963 | 8.2 | 34.4% | 3 measured + 2 tuned |
| KF + lever comp | 0.8148 | 0.0220 | 0.0183 | 0.6193 | 0.1000 | 9.6 | 100.0% | 3 measured + r |
| KF + gate + comp | 0.8148 | 0.0292 | 0.0172 | 0.6056 | 0.0957 | 8.3 | 41.8% | 3 measured + 2 + r |

## Caveats

* Synthetic data throughout. The filters are validated against a simulator built from the Phase 2 measurements, not against independent truth.
* The 1/f flicker component responsible for the bias-instability minimum is not synthesised. Its omission flatters any filter carrying an explicit bias state, so it flatters the Kalman filter more than the complementary filter.
* The gate uses the in-plane specific-force magnitude only, since this is a single-axis testbench. A three-axis implementation would use the full norm.
* Lever-arm compensation assumes r is known. The sensitivity of the result to an incorrect r is not swept here.

