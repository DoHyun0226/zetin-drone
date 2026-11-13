#!/usr/bin/env python3
"""두 펌웨어 ELF 가 같은 프로그램인지 비교한다.

소스 재배치(파일 사이 코드 이동, 주석 변경)가 같은 펌웨어를 만들었음을 증명하는 데
쓴다. 칩이 올리는 것을 비교한다:

- ELF 진입점
- 모든 SHF_ALLOC 섹션의 이름·종류·flags·주소·크기·정렬(.bss 같은 NOBITS 포함, 순서대로)
- 파일 내용이 있는 할당 섹션의 바이트

ESP-IDF 앱 설명자(``esp_app_desc_t``, 심볼 ``esp_app_desc``)에는 빌드 시각·날짜와
ELF 자신의 SHA-256 이 들어 있다. 이 세 필드만 가린다. 나머지(secure_version, eFuse
리비전, MMU 페이지 크기, 프로젝트 이름, IDF 버전)는 비교한다. 할당되지 않는 섹션
(디버그 정보, 심볼 표)은 칩에 올라가지 않고 소스 경로·줄 번호를 담으므로 보지 않는다.

``.bin`` 은 비교하지 않는다. 가린 필드 하나만 달라도 체크섬과 덧붙인 SHA 가 바뀌어,
차이 바이트 수로는 어디가 다른지 알 수 없다.

사용: compare_firmware_elf.py BASELINE.elf CANDIDATE.elf
같으면 종료 코드 0, 다르면 1(차이를 출력).
"""

import struct
import sys
from pathlib import Path

SHF_ALLOC = 0x2
SHT_NOBITS = 8
SHT_SYMTAB = 2

# esp_app_desc_t(ESP-IDF 5.x, esp_app_desc.h)에서 빌드마다 바뀌는 바이트 범위.
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
    """esp_app_desc 의 빌드별 필드를 제자리에서 가린다. 알림 목록을 돌려준다."""
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
