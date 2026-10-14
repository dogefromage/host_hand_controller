import shutil
import sys

_drawn_lines = 0
PARTIAL_BLOCKS = " ▏▎▍▌▋▊▉"


def print_sensor_bars(values, max_value):
    """Redraw one terminal-wide bar per sensor value (0..SENSOR_MAX) in place."""
    global _drawn_lines

    width = shutil.get_terminal_size().columns
    index_width = len(str(len(values) - 1))
    value_width = len(str(max_value))
    # "NN │" + bar + "│ VVVV"
    bar_width = max(width - index_width - value_width - 5, 1)

    lines = []
    for i, value in enumerate(values):
        fraction = min(max(value, 0), max_value) / max_value
        eighths = round(fraction * bar_width * 8)
        full, partial = divmod(eighths, 8)
        bar = "█" * full
        if full < bar_width:
            bar += PARTIAL_BLOCKS[partial]
        bar = bar.ljust(bar_width)
        lines.append(f"{i:>{index_width}} │{bar}│ {value:>{value_width}}")

    out = ""
    if _drawn_lines:
        # move cursor back to the start of the previous drawing and clear below it
        out += f"\x1b[{_drawn_lines}F\x1b[J"
    out += "\n".join(lines) + "\n"
    sys.stdout.write(out)
    sys.stdout.flush()
    _drawn_lines = len(lines)
