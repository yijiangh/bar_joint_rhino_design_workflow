"""Embed the toolbar tab icons from ``icon/`` into ``scaffolding_toolbar.rui``.

Rhino stores toolbar icons *inside* the .rui as three base64 PNG sprite
sheets -- one per UI size (16 / 24 / 32 px cells) -- laid out as a grid that
is filled left to right.  A ``<bitmap_item guid="…" index="N">`` maps a guid
to cell *N*, and whatever references that guid gets that picture:

* ``<tool_bar bitmap_id="…">``   -> the **tab** icon (what this tool sets),
* ``<macro_item bitmap_id="…">`` -> a button icon.

So an icon cannot just be dropped next to the .rui as a file; it has to be
packed into the sheets.  This tool does that from ``icon/icon_<Toolbar>.png``
(e.g. ``icon/icon_RSDesign.png`` for the ``RSDesign`` toolbar), and is
idempotent: each toolbar's guid is derived from its name, so re-running
replaces the sheets rather than accumulating cells.

**Re-run it after Rhino saves the toolbar.**  Rhino re-serialises the whole
.rui from memory whenever the UI layout changes, which silently drops
hand-authored sections; one run of this script puts the icons back.

Usage (from the repo root, any Python 3.8+, no third-party packages)::

    python scripts/apply_toolbar_icons.py            # apply
    python scripts/apply_toolbar_icons.py --check    # report, write nothing

Everything outside the ``<bitmaps>`` block and the ``bitmap_id`` attributes
is left byte-for-byte alone, so the diff stays reviewable.
"""

from __future__ import annotations

import argparse
import base64
import os
import re
import struct
import sys
import uuid
import zlib


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUI_PATH = os.path.join(REPO_ROOT, "scaffolding_toolbar.rui")
ICON_DIR = os.path.join(REPO_ROOT, "icon")

#: Cell sizes Rhino expects, in the order the three sheets appear in the file.
SHEET_SIZES = (("small_bitmap", 16), ("normal_bitmap", 24), ("large_bitmap", 32))

#: Namespace for deriving a stable ``bitmap_id`` from a toolbar name, so the
#: guid in the .rui never changes between runs.
_GUID_NAMESPACE = uuid.UUID("b7a1f4c2-6d3e-4c0a-9f21-8f5a0c1d2e34")

_LOCALE_TAG = "locale_1033"


def bitmap_guid(toolbar_name):
    """Return the ``bitmap_id`` guid to use for *toolbar_name*.

    Derived with UUID v5 (a hash of the name inside a fixed namespace) rather
    than randomly generated, so every run of this script produces the *same*
    guid for the same toolbar.  That is what makes the tool idempotent: a
    re-run rewrites the existing mapping instead of leaving a stale guid
    behind that no bitmap_item points at any more.

    Args:
        toolbar_name (str): the toolbar's ``<text>`` value, e.g. ``"RSDesign"``.

    Returns:
        str: the guid, formatted as Rhino writes them (lowercase, hyphenated).
    """
    return str(uuid.uuid5(_GUID_NAMESPACE, f"toolbar-icon:{toolbar_name}"))


# ---------------------------------------------------------------------------
# Minimal PNG codec (stdlib only -- Rhino's Python has no Pillow either)
# ---------------------------------------------------------------------------


def _png_chunks(raw):
    """Walk the chunk structure of a PNG byte string.

    A PNG is an 8-byte signature followed by chunks, each laid out as
    ``length (4) | type (4) | data (length) | crc (4)``.  Iterating them is
    all we need to find IHDR (the header) and concatenate the IDAT payloads.

    Args:
        raw (bytes): the whole file.

    Yields:
        tuple[bytes, bytes]: ``(chunk_type, chunk_data)`` in file order.

    Raises:
        ValueError: if the PNG signature is missing.
    """
    if raw[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG file")
    offset = 8
    while offset < len(raw):
        (length,) = struct.unpack(">I", raw[offset:offset + 4])
        chunk_type = raw[offset + 4:offset + 8]
        data = raw[offset + 8:offset + 8 + length]
        yield chunk_type, data
        offset += 12 + length  # length + type + data + crc


def _paeth(a, b, c):
    """The PNG Paeth predictor: pick whichever neighbour the gradient favours.

    Used by row filter type 4.  *a* is the pixel to the left, *b* the one
    above, *c* the one above-left; the predictor returns the neighbour
    closest to ``a + b - c``.

    Args:
        a (int), b (int), c (int): the three neighbouring byte values.

    Returns:
        int: the predicted byte to add back when un-filtering.
    """
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def read_png_rgba(path):
    """Decode an 8-bit RGBA PNG into flat pixel bytes.

    Inflates the IDAT stream and undoes the per-row filter (PNG stores each
    scanline as a filter byte plus filtered data, where the filter predicts
    each byte from its left / upper / upper-left neighbour).

    Only the one format our icons use is supported -- colour type 6, bit
    depth 8, non-interlaced.  Anything else raises rather than silently
    producing a garbled sheet.

    Args:
        path (str): PNG file to read.

    Returns:
        tuple[int, int, bytearray]: ``(width, height, pixels)`` where *pixels*
        is ``width * height * 4`` bytes in RGBA order, row-major.

    Raises:
        ValueError: on a non-PNG, an unsupported variant, or a truncated
            pixel stream.
    """
    with open(path, "rb") as stream:
        raw = stream.read()

    header = None
    idat = bytearray()
    for chunk_type, data in _png_chunks(raw):
        if chunk_type == b"IHDR":
            header = struct.unpack(">IIBBBBB", data)
        elif chunk_type == b"IDAT":
            idat += data  # IDAT may be split across several chunks
        elif chunk_type == b"IEND":
            break
    if header is None:
        raise ValueError(f"{path}: no IHDR chunk")

    width, height, depth, color_type, compression, filter_method, interlace = header
    if (depth, color_type, interlace) != (8, 6, 0):
        raise ValueError(
            f"{path}: need an 8-bit RGBA non-interlaced PNG "
            f"(got depth={depth}, color_type={color_type}, interlace={interlace})"
        )
    if compression != 0 or filter_method != 0:
        raise ValueError(f"{path}: unsupported compression/filter method")

    stride = width * 4  # bytes per row; 4 = one byte each of R, G, B, A
    data = zlib.decompress(bytes(idat))
    expected = (stride + 1) * height  # +1 for each row's filter-type byte
    if len(data) != expected:
        raise ValueError(f"{path}: {len(data)} raw bytes, expected {expected}")

    pixels = bytearray(stride * height)
    previous = bytearray(stride)  # the already-unfiltered row above
    pos = 0
    for row in range(height):
        filter_type = data[pos]
        pos += 1
        line = bytearray(data[pos:pos + stride])
        pos += stride
        if filter_type == 1:  # Sub: predict from the pixel to the left
            for i in range(4, stride):
                line[i] = (line[i] + line[i - 4]) & 0xFF
        elif filter_type == 2:  # Up: predict from the pixel above
            for i in range(stride):
                line[i] = (line[i] + previous[i]) & 0xFF
        elif filter_type == 3:  # Average: mean of left and above
            for i in range(stride):
                left = line[i - 4] if i >= 4 else 0
                line[i] = (line[i] + ((left + previous[i]) >> 1)) & 0xFF
        elif filter_type == 4:  # Paeth
            for i in range(stride):
                left = line[i - 4] if i >= 4 else 0
                upper_left = previous[i - 4] if i >= 4 else 0
                line[i] = (line[i] + _paeth(left, previous[i], upper_left)) & 0xFF
        elif filter_type != 0:  # 0 = None, nothing to undo
            raise ValueError(f"{path}: unknown row filter {filter_type}")
        pixels[row * stride:(row + 1) * stride] = line
        previous = line
    return width, height, pixels


def write_png_rgba(width, height, pixels):
    """Encode flat RGBA bytes as a PNG byte string.

    Writes the simplest legal PNG: one IHDR, one deflated IDAT with every row
    using filter type 0 (none), one IEND.  Filtering would shrink the file,
    but these sheets are a few kilobytes and the .rui is text, so clarity
    wins over bytes here.

    Args:
        width (int), height (int): sheet dimensions in pixels.
        pixels (bytearray): ``width * height * 4`` bytes, RGBA, row-major.

    Returns:
        bytes: the complete PNG file.
    """
    stride = width * 4
    raw = bytearray()
    for row in range(height):
        raw.append(0)  # filter type 0 = None
        raw += pixels[row * stride:(row + 1) * stride]

    def chunk(chunk_type, data):
        """Frame *data* as a PNG chunk with its length and CRC32."""
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + chunk(b"IEND", b"")
    )


def resize_rgba(width, height, pixels, target):
    """Box-filter an RGBA image down to ``target`` x ``target``.

    Each output pixel averages the source pixels that fall inside its cell,
    which is what keeps a 48 px icon legible at 16 px (plain nearest-neighbour
    would drop most of the strokes).  The source rectangle is computed with
    integer arithmetic so non-integer ratios (48 -> 32) still tile exactly.

    Colour is averaged **premultiplied by alpha** and then divided back out,
    so fully transparent border pixels -- whose RGB is arbitrary -- cannot
    bleed into the edges of the shrunken icon.

    Args:
        width (int), height (int): source dimensions.
        pixels (bytearray): source RGBA bytes, row-major.
        target (int): output edge length in pixels (square).

    Returns:
        bytearray: ``target * target * 4`` RGBA bytes.
    """
    stride = width * 4
    out = bytearray(target * target * 4)
    for out_y in range(target):
        # Source rows covered by this output row (at least one, even when
        # target > height, so the loop below never runs empty).
        y0 = out_y * height // target
        y1 = max((out_y + 1) * height // target, y0 + 1)
        for out_x in range(target):
            x0 = out_x * width // target
            x1 = max((out_x + 1) * width // target, x0 + 1)
            acc_r = acc_g = acc_b = acc_a = 0
            count = 0
            for y in range(y0, y1):
                base = y * stride + x0 * 4
                for _ in range(x1 - x0):
                    alpha = pixels[base + 3]
                    acc_r += pixels[base] * alpha
                    acc_g += pixels[base + 1] * alpha
                    acc_b += pixels[base + 2] * alpha
                    acc_a += alpha
                    base += 4
                    count += 1
            index = (out_y * target + out_x) * 4
            if acc_a:  # un-premultiply; all-transparent cells stay zeroed
                out[index] = min(255, acc_r // acc_a)
                out[index + 1] = min(255, acc_g // acc_a)
                out[index + 2] = min(255, acc_b // acc_a)
            out[index + 3] = acc_a // count if count else 0
    return out


def build_sheet(icons, cell):
    """Pack decoded icons into one horizontal sprite strip.

    Rhino reads a sheet as a grid filled left to right, so laying the icons
    out in a single row makes each icon's ``index`` simply its position --
    no row/column arithmetic to get wrong.  (Rhino's own ``default.rui`` uses
    250 columns because it holds ~1500 icons; the column count is just
    whatever the PNG width implies.)

    Args:
        icons (list): ``(width, height, pixels)`` triples from
            :func:`read_png_rgba`, in the order they should be indexed.
        cell (int): edge length of one slot, i.e. 16, 24 or 32.

    Returns:
        bytes: the strip as a PNG, ``cell * len(icons)`` wide, ``cell`` tall.
    """
    sheet_width = cell * len(icons)
    sheet = bytearray(sheet_width * cell * 4)
    for slot, (width, height, pixels) in enumerate(icons):
        scaled = resize_rgba(width, height, pixels, cell)
        for row in range(cell):
            src = row * cell * 4
            dst = (row * sheet_width + slot * cell) * 4
            sheet[dst:dst + cell * 4] = scaled[src:src + cell * 4]
    return write_png_rgba(sheet_width, cell, sheet)


# ---------------------------------------------------------------------------
# .rui surgery
# ---------------------------------------------------------------------------
#
# Edited as text rather than through ElementTree on purpose: a full XML
# round trip would re-indent all ~650 lines and drop the section comments,
# burying the actual change in noise.  The two patterns below are the only
# parts of the file this tool touches.

#: Matches a toolbar's opening tag together with its <text> element, so the
#: guid, the other attributes, and the toolbar's name are all captured at once.
_TOOLBAR_RE = re.compile(
    r'(<tool_bar\s+guid="(?P<guid>[^"]+)"(?P<attrs>[^>]*)>\s*'
    r"<text>\s*<" + _LOCALE_TAG + r">(?P<name>[^<]+)</" + _LOCALE_TAG + r">)"
)
#: Matches the whole <bitmaps> block, in both the populated and the empty
#: self-closing form Rhino writes when there are no icons.
_BITMAPS_RE = re.compile(r"[ \t]*<bitmaps>.*?</bitmaps>\n|[ \t]*<bitmaps\s*/>\n", re.S)


def toolbar_names(text):
    """List the toolbars declared in the .rui.

    Args:
        text (str): the whole .rui file contents.

    Returns:
        list[tuple[str, str, str | None]]: ``(name, guid, existing_bitmap_id)``
        per toolbar, in file order.  The third element is ``None`` for a
        toolbar that has no icon yet.
    """
    found = []
    for match in _TOOLBAR_RE.finditer(text):
        existing = re.search(r'bitmap_id="([^"]+)"', match.group("attrs"))
        found.append(
            (match.group("name"), match.group("guid"), existing.group(1) if existing else None)
        )
    return found


def set_bitmap_ids(text, guid_by_name):
    """Add or replace the ``bitmap_id`` attribute on each named toolbar.

    Toolbars missing from *guid_by_name* (no icon file on disk) are left
    exactly as they are, so a partial icon set does not strip the icons off
    the toolbars that do have one.

    Args:
        text (str): the whole .rui file contents.
        guid_by_name (dict[str, str]): toolbar name -> bitmap guid.

    Returns:
        str: the updated file contents.
    """
    def _replace(match):
        guid = guid_by_name.get(match.group("name"))
        if guid is None:
            return match.group(0)
        # Rebuild only the opening tag, dropping any previous bitmap_id;
        # everything from the first '>' on (whitespace + <text> element) is
        # passed through untouched.
        attrs = re.sub(r'\s*bitmap_id="[^"]*"', "", match.group("attrs"))
        opening = f'<tool_bar guid="{match.group("guid")}" bitmap_id="{guid}"{attrs}>'
        return opening + match.group(0).split(">", 1)[1]

    return _TOOLBAR_RE.sub(_replace, text)


def render_bitmaps_block(names, sheets):
    """Serialise the three sprite sheets as the file's ``<bitmaps>`` block.

    All three sizes carry the same guid list with the same indices -- Rhino
    picks a sheet according to the user's toolbar-size setting and looks the
    icon up by guid, so the mappings have to agree.

    Args:
        names (list[str]): toolbar names, in the order they were packed.
        sheets (list[bytes]): one PNG per entry of :data:`SHEET_SIZES`.

    Returns:
        str: the block, indented to match the rest of the file and ending in
        a newline.
    """
    lines = ["  <bitmaps>"]
    for (tag, cell), png in zip(SHEET_SIZES, sheets):
        lines.append(f'    <{tag} item_width="{cell}" item_height="{cell}">')
        lines.append(
            "      <bitmap>" + base64.b64encode(png).decode("ascii") + "</bitmap>"
        )
        for index, name in enumerate(names):
            lines.append(
                f'      <bitmap_item guid="{bitmap_guid(name)}" index="{index}" />'
            )
        lines.append(f"    </{tag}>")
    lines.append("  </bitmaps>")
    return "\n".join(lines) + "\n"


def icon_path(name):
    """Return the icon file a toolbar is expected to use.

    Args:
        name (str): the toolbar name, e.g. ``"RSStability"``.

    Returns:
        str: absolute path to ``icon/icon_<name>.png`` (which need not exist).
    """
    return os.path.join(ICON_DIR, f"icon_{name}.png")


def main(argv=None):
    """Load the icons, rebuild the sheets, and patch the .rui.

    Toolbars with no icon file are reported and skipped rather than treated
    as an error, so the tool stays usable while an icon set is still being
    drawn.

    Args:
        argv (list[str] | None): command-line arguments, for testing.
            ``None`` uses ``sys.argv``.

    Returns:
        int: process exit code -- 0 on success (including "already up to
        date"), 1 if the .rui has no toolbars, no icon matched, or the
        ``<bitmaps>`` block could not be located.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--check",
        action="store_true",
        help="report what would change and write nothing",
    )
    args = parser.parse_args(argv)

    with open(RUI_PATH, "r", encoding="utf-8") as stream:
        text = stream.read()

    toolbars = toolbar_names(text)
    if not toolbars:
        print(f"apply_toolbar_icons: no <tool_bar> elements in {RUI_PATH}.")
        return 1

    names, icons, missing = [], [], []
    for name, _guid, _existing in toolbars:
        path = icon_path(name)
        if not os.path.isfile(path):
            missing.append(name)
            continue
        names.append(name)
        icons.append(read_png_rgba(path))

    for name in missing:
        print(f"  no icon for '{name}' (expected {icon_path(name)}); tab stays blank")
    if not names:
        print("apply_toolbar_icons: no matching icons found; nothing to do.")
        return 1

    sheets = [build_sheet(icons, cell) for _tag, cell in SHEET_SIZES]
    updated = set_bitmap_ids(text, {name: bitmap_guid(name) for name in names})
    if not _BITMAPS_RE.search(updated):
        print("apply_toolbar_icons: could not find the <bitmaps> block to replace.")
        return 1
    updated = _BITMAPS_RE.sub(render_bitmaps_block(names, sheets), updated, count=1)

    for (tag, cell), png in zip(SHEET_SIZES, sheets):
        print(f"  {tag}: {len(names)} cell(s) of {cell}px, {len(png)} byte PNG")
    for index, name in enumerate(names):
        print(f"  {name} -> index {index}, bitmap_id {bitmap_guid(name)}")

    if args.check:
        state = "would change" if updated != text else "already up to date"
        print(f"apply_toolbar_icons: {state} ({RUI_PATH}).")
        return 0
    if updated == text:
        print(f"apply_toolbar_icons: already up to date ({RUI_PATH}).")
        return 0
    with open(RUI_PATH, "w", encoding="utf-8") as stream:
        stream.write(updated)
    print(f"apply_toolbar_icons: wrote {RUI_PATH}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
