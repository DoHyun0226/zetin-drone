#!/usr/bin/env python3
"""Compare two firmware ELF files for program identity.

Used to prove that a source reorganisation (moving code between files,
changing comments) produced the same firmware. Compares what the chip loads:

- ELF entry point;
- every SHF_ALLOC section: name, type, flags, address, size, alignment
  (NOBITS sections such as .bss included, in order);
- the bytes of every allocated section that has file contents.

The ESP-IDF application descriptor (``esp_app_desc_t``, symbol
``esp_app_desc``) carries the build time, date and the ELF's own SHA-256;
only those three fields are masked. Everything else in it (secure_version,
eFuse revisions, MMU page size, project name, IDF version) is compared.
Non-allocated sections (debug info, symbol tables) are ignored because they
are not loaded and record source paths and line numbers.

Do not compare ``.bin`` files: their checksum and appended SHA change with
any masked field, so a byte count of differences says nothing about where.

Usage: compare_firmware_elf.py BASELINE.elf CANDIDATE.elf
Exit 0 when identical, 1 when different (differences are printed).
"""

import struct
import sys
from pathlib import Path

SHF_ALLOC = 0x2
SHT_NOBITS = 8
SHT_SYMTAB = 2

# esp_app_desc_t (ESP-IDF 5.x, esp_app_desc.h): byte ranges that change per build.
APP_DESC_SYMBOL = "esp_app_desc"
APP_DESC_SIZE = 256
APP_DESC_MASKED = (
    ("time", 80, 96),
    ("date", 96, 112),
    ("app_elf_sha256", 144, 176),
)


class Elf32:
    def __init__(self, path):
        self.path = Path(path)
        self.data = self.path.read_bytes()
        d = self.data
        if d[:4] != b"\x7fELF":
            raise ValueError(f"{path}: not an ELF file")
        if d[4] != 1 or d[5] != 1:
            raise ValueError(f"{path}: only 32-bit little-endian ELF is supported")
        (self.e_type, self.e_machine, _, self.e_entry, _, self.e_shoff, _, _, _, _,
         self.e_shentsize, self.e_shnum, self.e_shstrndx) = struct.unpack_from(
            "<HHIIIIIHHHHHH", d, 16)
        self.sections = [self._section(i) for i in range(self.e_shnum)]
        shstr = self.sections[self.e_shstrndx]
        for s in self.sections:
            s["name"] = self._cstr(shstr["offset"] + s["name_off"])

    def _section(self, index):
        off = self.e_shoff + index * self.e_shentsize
        (name_off, sh_type, flags, addr, offset, size, link, info, align,
         entsize) = struct.unpack_from("<IIIIIIIIII", self.data, off)
        return {"index": index, "name_off": name_off, "type": sh_type, "flags": flags,
                "addr": addr, "offset": offset, "size": size, "link": link,
                "align": align, "entsize": entsize}

    def _cstr(self, off):
        end = self.data.index(b"\x00", off)
        return self.data[off:end].decode("utf-8", "replace")

    def allocated(self):
        return [s for s in self.sections if s["flags"] & SHF_ALLOC]

    def contents(self, section):
        if section["type"] == SHT_NOBITS:
            return b""
        return self.data[section["offset"]:section["offset"] + section["size"]]

    def symbols(self):
        out = {}
        for s in self.sections:
            if s["type"] != SHT_SYMTAB:
                continue
            strtab = self.sections[s["link"]]
            for i in range(s["size"] // 16):
                name_off, value, size, _, _, _ = struct.unpack_from(
                    "<IIIBBH", self.data, s["offset"] + i * 16)
                if name_off:
                    out.setdefault(self._cstr(strtab["offset"] + name_off), (value, size))
        return out


def _layout(section):
    return (section["name"], section["type"], section["flags"], section["addr"],
            section["size"], section["align"])


def _mask_app_desc(elf, buffers):
    """Mask the per-build fields of esp_app_desc in place. Returns notes."""
    sym = elf.symbols().get(APP_DESC_SYMBOL)
    if sym is None:
        return [f"{elf.path.name}: no {APP_DESC_SYMBOL} symbol (nothing masked)"]
    addr, size = sym
    if size not in (0, APP_DESC_SIZE):
        raise ValueError(f"{elf.path}: {APP_DESC_SYMBOL} has size {size}, expected 256")
    for s in elf.allocated():
        if s["type"] != SHT_NOBITS and s["addr"] <= addr < s["addr"] + s["size"]:
            buf = buffers[s["index"]]
            base = addr - s["addr"]
            for _, lo, hi in APP_DESC_MASKED:
                buf[base + lo:base + hi] = b"\x00" * (hi - lo)
            return []
    raise ValueError(f"{elf.path}: {APP_DESC_SYMBOL} is not inside an allocated section")


def compare(baseline_path, candidate_path):
    a, b = Elf32(baseline_path), Elf32(candidate_path)
    diffs = []
    notes = []
    if a.e_entry != b.e_entry:
        diffs.append(f"entry 0x{a.e_entry:08x} != 0x{b.e_entry:08x}")
    la = [_layout(s) for s in a.allocated()]
    lb = [_layout(s) for s in b.allocated()]
    if la != lb:
        for x, y in zip(la, lb):
            if x != y:
                diffs.append(f"section layout {x} != {y}")
        if len(la) != len(lb):
            diffs.append(f"allocated section count {len(la)} != {len(lb)}")
        return diffs, notes
    bufs_a = {s["index"]: bytearray(a.contents(s)) for s in a.allocated()}
    bufs_b = {s["index"]: bytearray(b.contents(s)) for s in b.allocated()}
    notes += _mask_app_desc(a, bufs_a)
    notes += _mask_app_desc(b, bufs_b)
    for sa, sb in zip(a.allocated(), b.allocated()):
        ca, cb = bufs_a[sa["index"]], bufs_b[sb["index"]]
        if ca != cb:
            first = next(i for i in range(min(len(ca), len(cb))) if ca[i] != cb[i])
            count = sum(1 for x, y in zip(ca, cb) if x != y)
            diffs.append(f"{sa['name']}: {count} byte(s) differ, first at "
                         f"0x{sa['addr'] + first:08x}")
    return diffs, notes


def main(argv):
    if len(argv) != 3:
        print(__doc__.strip().splitlines()[-3], file=sys.stderr)
        return 2
    diffs, notes = compare(argv[1], argv[2])
    for note in notes:
        print(f"note: {note}")
    if diffs:
        for d in diffs:
            print(f"DIFF {d}")
        return 1
    count = len(Elf32(argv[1]).allocated())
    print(f"identical: entry, {count} allocated sections (layout and bytes; "
          f"app descriptor time/date/sha256 masked)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
