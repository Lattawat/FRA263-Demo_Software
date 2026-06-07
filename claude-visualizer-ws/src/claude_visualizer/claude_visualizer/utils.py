#!/usr/bin/python3
import pylsl

def create_outlet(stream_cfg: dict, msg_format) -> pylsl.StreamOutlet:
    outlet_type = stream_cfg.get("outlet_type")
    if outlet_type == "REGULAR":
        nominal_sampling_rate = float(stream_cfg.get("sampling_rate_hz", 100.0))
    elif outlet_type == "IRREGULAR":
        nominal_sampling_rate = pylsl.IRREGULAR_RATE
    else:
        raise ValueError(
            f"unknown outlet_type {outlet_type!r} for stream "
            f"{stream_cfg.get('name', '?')!r}; expected 'REGULAR' or 'IRREGULAR'"
        )

    info = pylsl.StreamInfo(
        name=str(stream_cfg.get("name", "")),
        type=str(stream_cfg.get("type", "")),
        channel_count=len(stream_cfg.get("channel", [])),
        nominal_srate=nominal_sampling_rate,
        channel_format=msg_format,
        source_id=str(stream_cfg.get("source_id", "")),
    )

    channels = info.desc().append_child("channels")
    for label in stream_cfg.get("channel", []):
        ch = channels.append_child("channel")
        ch.append_child_value("label", label)
    return pylsl.StreamOutlet(info)