"""Check the actual ELF member offsets consumed by Orbit's objdump parser.

Run with --objdump PATH followed by one or more firmware ELFs. No hardware access
or third-party Python packages are needed. DWARF2 block expressions deliberately
fail: Orbit 1.2.2 parses their length as a member offset.
"""
import argparse
from pathlib import Path
import re
import subprocess


EXPECTED = {
    'INS_t': {
        'q': 0, 'MotionAccel_b': 16, 'MotionAccel_n': 28, 'AccelLPF': 40,
        'xn': 44, 'yn': 56, 'zn': 68, 'Gyro': 80, 'Accel': 92,
        'Roll': 104, 'Pitch': 108, 'Yaw': 112, 'YawTotalAngle': 116, 'init': 120,
    },
    'attitude_t': {
        'Gyro': 0, 'Accel': 12, 'Roll': 24, 'Pitch': 28, 'Yaw': 32,
        'YawTotalAngle': 36,
    },
}


def read_dies(output):
    dies, stack = {}, []
    for line in output.splitlines():
        match = re.match(r'\s*<(\d+)><([0-9a-fA-F]+)>:.*\((DW_TAG_\w+)\)', line)
        if match:
            depth, offset, tag = match.groups()
            die = {'depth': int(depth), 'tag': tag, 'attrs': {}, 'children': []}
            dies[int(offset, 16)] = die
            while stack and stack[-1]['depth'] >= die['depth']:
                stack.pop()
            if stack:
                stack[-1]['children'].append(die)
            stack.append(die)
            continue
        match = re.match(r'\s*<[0-9a-fA-F]+>\s+(DW_AT_\w+)\s*:\s*(.+)', line)
        if match and stack:
            key, value = match.groups()
            value = re.sub(r'^\(indirect string, offset: 0x[0-9a-fA-F]+\):\s*', '', value)
            stack[-1]['attrs'][key] = value.strip()
    return dies


def check_elf(objdump, elf, bench=False):
    output = subprocess.check_output([objdump, '--dwarf=info', str(elf)], text=True,
                                     encoding='utf-8', errors='replace')
    dies = read_dies(output)
    found = set()
    for die in dies.values():
        name = die['attrs'].get('DW_AT_name')
        if die['tag'] != 'DW_TAG_typedef' or name not in EXPECTED:
            continue
        reference = die['attrs']['DW_AT_type']
        struct = dies[int(re.search(r'<0x([0-9a-fA-F]+)>', reference).group(1), 16)]
        members = {member['attrs'].get('DW_AT_name'): member for member in struct['children']
                   if member['tag'] == 'DW_TAG_member'}
        for field, offset in EXPECTED[name].items():
            raw = members[field]['attrs'].get('DW_AT_data_member_location', '')
            if not re.fullmatch(r'\d+', raw) or int(raw) != offset:
                raise SystemExit(f'FAIL: {elf}: {name}.{field}: expected numeric offset '
                                 f'{offset}, got {raw!r}; Orbit cannot read this layout correctly')
        found.add(name)
    required = {'attitude_t'} if bench else set(EXPECTED)
    if not required <= found:
        raise SystemExit(f'FAIL: {elf}: missing types {required - found}')
    print(f'PASS: {elf}: {sorted(found)} member offsets are Orbit compatible; '
          'Roll/Pitch/Yaw occupy separate addresses')
    if bench:
        print('NOTE: bench firmware disables INS; its attitude type is checked, not live sensor data')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objdump', default='arm-none-eabi-objdump')
    parser.add_argument('--bench', action='store_true', help='Bench firmware disables INS; require only attitude_t')
    parser.add_argument('elf', type=Path, nargs='+')
    args = parser.parse_args()
    for elf_path in args.elf:
        check_elf(args.objdump, elf_path, args.bench)
