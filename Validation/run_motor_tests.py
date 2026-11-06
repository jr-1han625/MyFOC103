"""编译实际电机启动与 L1 源码并执行主机数值验收，结果集中写入本目录。"""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime, timezone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', help='主机 clang/gcc/zig 编译器的完整路径')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    validation = root / 'Validation'
    compiler = args.cc or shutil.which('clang') or shutil.which('gcc') or shutil.which('zig')
    if not compiler:
        parser.error('没有找到 C 编译器，请通过 --cc 提供 clang/gcc/zig 路径。')
    driver = [compiler]
    if Path(compiler).stem.lower() == 'zig':
        driver.append('cc')

    env = os.environ.copy()
    for name, suffix in [('ZIG_GLOBAL_CACHE_DIR', 'global'), ('ZIG_LOCAL_CACHE_DIR', 'local'),
                         ('TEMP', 'tmp'), ('TMP', 'tmp')]:
        directory = validation / 'cache' / suffix
        directory.mkdir(parents=True, exist_ok=True)
        env[name] = str(directory)
    sources = ['L1_Drivers/MotorIo.c', 'L2_Middleware/algos/foc_math.c', 'L2_Middleware/algos/foc_openloop.c', 'L2_Middleware/motor_start.c', 'Validation/test_motor.c']
    recorded = sources + ['Hardware/motor_board.c', 'Hardware/motor_board.h', 'User/xy_select.h', 'User/main.c', 'User/stm32f10x_it.c', 'User/driver_assembly.c', 'Project.uvprojx', 'Validation/run_motor_tests.py']
    report = ['UTC: ' + datetime.now(timezone.utc).isoformat(), 'SOURCE SHA256:']
    for name in recorded:
        report.append(hashlib.sha256((root / name).read_bytes()).hexdigest() + '  ' + name)
    result_path = validation / 'motor_test_results.txt'

    try:
        for optimization in ['-O0', '-O2']:
            binary = validation / ('motor_tests' + ('.exe' if os.name == 'nt' else ''))
            command = driver + ['-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', optimization,
                                '-IL1_Drivers', '-IL2_Middleware', '-IL2_Middleware/algos', '-IUser', '-IHardware'] + sources + ['-o', str(binary)]
            if os.name != 'nt':
                command.append('-lm')
            report.append('BUILD: ' + subprocess.list2cmdline(command))
            compiled = subprocess.run(command, cwd=root, env=env, text=True,
                                      encoding='utf-8', errors='strict', capture_output=True)
            report.extend([compiled.stdout, compiled.stderr])
            if compiled.returncode:
                raise RuntimeError('C compilation failed')
            report.append('BUILD PASS: ' + optimization)
            executed = subprocess.run([str(binary)], cwd=root, env=env, text=True,
                                      encoding='utf-8', errors='strict', capture_output=True)
            report.extend([executed.stdout, executed.stderr])
            if executed.returncode:
                raise RuntimeError('C acceptance tests failed')
        report.append('RESULT: PASS (host numerical verification; no Keil build or hardware timing claim)')
    except Exception as exc:
        report.append('RESULT: FAIL: ' + str(exc))
        result_path.write_text('\n'.join(report), encoding='utf-8')
        print('\n'.join(report))
        return 1
    result_path.write_text('\n'.join(report), encoding='utf-8')
    print('\n'.join(report))
    return 0


if __name__ == '__main__':
    sys.exit(main())
