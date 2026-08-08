#!/usr/bin/env python3
"""通过 Windows 蓝牙 SPP 虚拟串口调试智能车。需要安装 pyserial。"""

import argparse
import threading

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("缺少 pyserial，请运行：python -m pip install pyserial") from exc


def receive_loop(port: serial.Serial) -> None:
    try:
        while port.is_open:
            line = port.readline()
            if line:
                print(line.decode("utf-8", errors="replace").rstrip())
    except serial.SerialException as exc:
        print(f"\n[接收停止: {exc}]")


def main() -> None:
    parser = argparse.ArgumentParser(description="HC-04 智能车蓝牙调试终端")
    parser.add_argument("port", nargs="?", help="Windows 蓝牙 SPP 串口，例如 COM8")
    parser.add_argument("--list", action="store_true", help="列出可用串口")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    if args.list:
        for item in list_ports.comports():
            print(f"{item.device}: {item.description}")
        return
    if not args.port:
        parser.error("请提供蓝牙串口号，或使用 --list 查看")

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        print(f"已打开 {args.port} @ {args.baud}。输入 HELP；输入 quit 退出。")
        threading.Thread(target=receive_loop, args=(port,), daemon=True).start()
        while True:
            try:
                command = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if command.lower() in {"quit", "exit"}:
                break
            if command:
                port.write((command + "\n").encode("ascii"))


if __name__ == "__main__":
    main()
