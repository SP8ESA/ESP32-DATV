"""Recommended QO-100 WB spots: BATC/AMSAT-DL bandplan v3 (6 February 2021).

Verified against https://wiki.batc.org.uk/QO-100_WB_Bandplan.
Frequencies are MHz. The ESP transmits on the uplink, not the 10 GHz downlink.
"""
from dataclasses import dataclass

SOURCE_URL = "https://wiki.batc.org.uk/QO-100_WB_Bandplan"
PLAN_VERSION = "BATC v3 · 2021-02-06"
UPLINK_MIN, UPLINK_MAX = 2401., 2410.
TRANSPONDER_OFFSET = 8089.5
BEACON_UPLINK = 2402.
PREFERRED_UPLINK_MIN = 2407.5


@dataclass(frozen=True)
class Channel:
    name: str
    row: int
    uplink: float
    rates: tuple

    @property
    def downlink(self):
        return self.uplink + TRANSPONDER_OFFSET

    def preferred(self, baud):
        return baud <= 333000 and self.uplink > PREFERRED_UPLINK_MIN

    def supports(self, baud):
        if self.row == 0:
            return 500000 < baud <= 1000000
        if self.row == 1:
            return 125000 < baud <= 333000 or (333000 < baud <= 500000 and self.uplink < PREFERRED_UPLINK_MIN)
        return baud <= 125000


CHANNELS = tuple(
    [Channel(f"W{i + 1}", 0, f, (1000000,)) for i, f in enumerate((2403.75, 2405.25, 2406.75))]
    + [Channel(f"N{i + 1}", 1, 2403.25 + .5 * i, (250000, 333000, 500000) if i < 9 else (250000, 333000)) for i in range(14)]
    + [Channel(f"V{i + 1}", 2, 2403.25 + .25 * i, (33000, 66000, 125000)) for i in range(27)]
)


def channel_warning(frequency, baud):
    """Warn on unusual WB use; local test frequencies do not use this plan."""
    if not UPLINK_MIN <= frequency <= UPLINK_MAX:
        return None
    rate = f"{baud / 1e6:g} MS/s" if baud >= 1000000 else f"{baud / 1000:g} kS/s"
    where = f"{rate} at {frequency:.3f} MHz"
    if frequency < 2403.:
        return f"{where} is in the beacon section."
    half_bandwidth = baud * 1.35 / 2e6
    if frequency - half_bandwidth < UPLINK_MIN or frequency + half_bandwidth > UPLINK_MAX:
        return f"{where} extends beyond the WB band edge."
    if frequency >= PREFERRED_UPLINK_MIN and baud > 333000:
        return f"{where} is too wide for the narrow WB section."
    if any(c.supports(baud) and abs(c.uplink - frequency) <= .001 for c in CHANNELS):
        return None
    return f"{where} is off the recommended WB channel grid."
