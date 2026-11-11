"""Run only the test ELF; no target or host file writes from emulated firmware."""
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_PC

cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0, 0x200000)
cpu.mem_map(0x20000000, 0x40000)
cpu.mem_map(0xE000E000, 0x1000)
with open(sys.argv[1], "rb") as source:
    elf = ELFFile(source)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            cpu.mem_write(segment["p_vaddr"], segment.data())

status = None


def trap(machine, interrupt, _):
    global status
    if interrupt != 7:
        raise RuntimeError(f"Unexpected ARM exception {interrupt}")
    op, arg = machine.reg_read(UC_ARM_REG_R0), machine.reg_read(UC_ARM_REG_R1)
    if op == 4:
        message = bytearray()
        while len(message) < 4096:
            char = machine.mem_read(arg + len(message), 1)[0]
            if not char:
                break
            message.append(char)
        print(message.decode("ascii"), end="")
    elif op == 0x20:
        reason, status = struct.unpack("<II", machine.mem_read(arg, 8))
        if reason != 0x20026:
            status = 1
        machine.emu_stop()
    else:
        raise RuntimeError(f"Unexpected semihosting operation {op}")
    pc = machine.reg_read(UC_ARM_REG_PC)
    if bytes(machine.mem_read(pc, 2)) == b"\xab\xbe":
        machine.reg_write(UC_ARM_REG_PC, (pc + 2) | 1)


sp, entry = struct.unpack("<II", cpu.mem_read(0, 8))
cpu.reg_write(UC_ARM_REG_SP, sp)
cpu.hook_add(UC_HOOK_INTR, trap)
cpu.emu_start(entry, 0x1FFFFE, timeout=20_000_000, count=100_000_000)
if status != 0:
    raise SystemExit(f"Test failed: status={status}, PC={cpu.reg_read(UC_ARM_REG_PC):x}")
