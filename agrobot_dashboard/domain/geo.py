"""Geographic coordinate formatting."""


def dms(value, is_lat):
    """Format a decimal-degree coordinate as degrees-minutes-seconds with a
    hemisphere suffix, e.g. 51.5074 → 51°30'26.64"N.  Returns "" for None."""
    if value is None:
        return ""
    hemi = ("N" if value >= 0 else "S") if is_lat else ("E" if value >= 0 else "W")
    v = abs(float(value))
    deg = int(v)
    minutes_full = (v - deg) * 60.0
    minutes = int(minutes_full)
    seconds = (minutes_full - minutes) * 60.0
    return f"{deg}°{minutes:02d}'{seconds:05.2f}\"{hemi}"
