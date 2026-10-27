#!/usr/bin/env python3
"""
elf2uf2.py -- convert an RP2040 flash ELF to UF2.

Group AAI18.  A pure-Python port of the flash-binary path of the template's
tools/elf2uf2/main.cpp, so the CMake build needs no host C++ compiler
(the Pico Windows installer ships Python but no host compiler).  For a
flash binary it produces byte-for-byte the same UF2 as the C++ tool:

  - PT_LOAD segments whose physical address is in flash (0x10000000 to
    0x15000000) are written; segments in RAM are skipped (they are copied
    from flash at boot).
  - Data is split into 256-byte pages; gaps inside a touched 4 KB erase
    sector are filled with zero pages up to the last page, as the bootrom
    requires.
  - Family ID 0xe48bff56 (RP2040).

Usage: python elf2uf2.py input.elf output.uf2
"""

import struct
import sys

PAGE_SIZE = 256
SECTOR_SIZE = 4096
FLASH_START, FLASH_END = 0x10000000, 0x15000000
RAM_RANGES = ((0x20000000, 0x20042000), (0x21000000, 0x21040000))
MAGIC_START0, MAGIC_START1, MAGIC_END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
FLAG_FAMILY_ID_PRESENT = 0x00002000
RP2040_FAMILY_ID = 0xE48BFF56
PT_LOAD = 1


def fail(message):
    sys.stderr.write("ERROR: %s\n" % message)
    sys.exit(1)


def in_ranges(addr, size, ranges):
    return any(lo <= addr and addr + size <= hi for lo, hi in ranges)


def load_pages(elf):
    if elf[:4] != b"\x7fELF" or elf[4] != 1 or elf[5] != 1:
        fail("not a little-endian ELF32 file")

    e_entry, e_phoff = struct.unpack_from("<II", elf, 24)
    e_phentsize, e_phnum = struct.unpack_from("<HH", elf, 42)

    pages = {}
    for idx in range(e_phnum):
        (p_type, p_offset, _vaddr, p_paddr, p_filesz, p_memsz,
         _flags, _align) = struct.unpack_from("<8I", elf,
                                              e_phoff + idx * e_phentsize)
        if p_type != PT_LOAD or p_memsz == 0:
            continue

        mapped = min(p_filesz, p_memsz)
        if mapped:
            if in_ranges(p_paddr, mapped, ((FLASH_START, FLASH_END),)):
                addr, remaining, offset = p_paddr, mapped, p_offset
                while remaining:
                    page_off = addr & (PAGE_SIZE - 1)
                    length = min(remaining, PAGE_SIZE - page_off)
                    page = pages.setdefault(addr - page_off,
                                            bytearray(PAGE_SIZE))
                    page[page_off:page_off + length] = \
                        elf[offset:offset + length]
                    addr += length
                    offset += length
                    remaining -= length
            elif not in_ranges(p_paddr, mapped, RAM_RANGES):
                fail("segment at %08x is outside flash and RAM" % p_paddr)

    if not pages:
        fail("no flash contents found (is this a flash binary?)")

    if not (FLASH_START <= e_entry < FLASH_END):
        sys.stderr.write("note: entry point %08x is not in flash\n" % e_entry)

    # Pad touched erase sectors with zero pages, up to the last page.
    last_page = max(pages)
    for sector in {page // SECTOR_SIZE for page in pages}:
        for page in range(sector * SECTOR_SIZE, (sector + 1) * SECTOR_SIZE,
                          PAGE_SIZE):
            if page < last_page:
                pages.setdefault(page, bytearray(PAGE_SIZE))

    return pages


def main(argv):
    if len(argv) != 3:
        sys.stderr.write("usage: elf2uf2.py input.elf output.uf2\n")
        return 2

    with open(argv[1], "rb") as handle:
        pages = load_pages(handle.read())

    total = len(pages)
    with open(argv[2], "wb") as out:
        for number, addr in enumerate(sorted(pages)):
            header = struct.pack("<8I", MAGIC_START0, MAGIC_START1,
                                 FLAG_FAMILY_ID_PRESENT, addr, PAGE_SIZE,
                                 number, total, RP2040_FAMILY_ID)
            data = bytes(pages[addr]) + bytes(476 - PAGE_SIZE)
            out.write(header + data + struct.pack("<I", MAGIC_END))

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
