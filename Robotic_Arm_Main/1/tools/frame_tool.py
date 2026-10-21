#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂串口帧工具：构造 / 解析 / 自检。零第三方依赖，只用标准库。

帧格式（见 Presentation（MD）/架构/design_idea.md 第 5 章）：
    0xAA | LEN | CMD | PAYLOAD... | SUM
    LEN = 整帧字节数
    SUM = 前 LEN-1 字节累加和，取低 8 位
    多字节字段一律小端

命令：
    0x01 MOVE      x,y,z (int16 mm) + phi (int16, 0.1°)   LEN=12
    0x02 HOME                                             LEN=4
    0x03 STOP                                             LEN=4
    0x04 QUERY                                            LEN=4
    0x05 SET_JOINT joint(u8) + deg(int16, 0.1°)           LEN=7

应答：
    0x81 ACK  四关节角 (int16, 0.1°)                       LEN=12
    0x82 ERR  错误码 u8                                    LEN=5

用法：
    python frame_tool.py query
    python frame_tool.py home
    python frame_tool.py stop
    python frame_tool.py move -59 143 32 45
    python frame_tool.py joint 0 180
    python frame_tool.py parse "AA 0C 81 65 04 C2 01 7C FC 84 03 62"
    python frame_tool.py table
    python frame_tool.py selftest

说明：
    本工具只算字节，不打开串口。把输出的 hex 粘到串口助手（SSCOM / VOFA+ 等）
    的 HEX 发送框即可；收到的 hex 粘回 parse 子命令核对。
    未在实机上验证过收发时序，只验证过帧字节本身（见 selftest）。
"""

import argparse
import struct
import sys

# Windows 控制台默认 GBK，强制 UTF-8 避免中文乱码
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

CMD_MOVE = 0x01
CMD_HOME = 0x02
CMD_STOP = 0x03
CMD_QUERY = 0x04
CMD_JOINT = 0x05
RSP_ACK = 0x81
RSP_ERR = 0x82

CMD_NAME = {
    CMD_MOVE: "MOVE",
    CMD_HOME: "HOME",
    CMD_STOP: "STOP",
    CMD_QUERY: "QUERY",
    CMD_JOINT: "SET_JOINT",
    RSP_ACK: "ACK",
    RSP_ERR: "ERR",
}

ERR_NAME = {
    0x00: "NONE",
    0x01: "WORKSPACE/IK 不可达",
    0x02: "校验和错误",
    0x03: "参数非法",
    0x04: "未知命令",
}

JNAME = ["BASE", "UPPER", "FOREARM", "END"]
LEN_OF = {CMD_MOVE: 12, CMD_HOME: 4, CMD_STOP: 4, CMD_QUERY: 4, CMD_JOINT: 7}


def sum8(data):
    return sum(data) & 0xFF


def build(cmd, payload=b""):
    length = 4 + len(payload)
    body = bytes([0xAA, length, cmd]) + payload
    return body + bytes([sum8(body)])


def frame_move(x, y, z, phi_deg):
    phi10 = int(round(phi_deg * 10))
    payload = struct.pack("<hhh", int(x), int(y), int(z)) + struct.pack("<h", phi10)
    return build(CMD_MOVE, payload)


def frame_joint(joint, deg):
    deg10 = int(round(deg * 10))
    return build(CMD_JOINT, bytes([joint]) + struct.pack("<h", deg10))


def frame_simple(cmd):
    if cmd not in (CMD_HOME, CMD_STOP, CMD_QUERY):
        raise ValueError("不是简单命令")
    return build(cmd)


def hexs(data):
    return " ".join("%02X" % b for b in data)


def _is_frame_like(data, off):
    if len(data) - off < 2:
        return False
    if data[off] != 0xAA:
        return False
    length = data[off + 1]
    return 4 <= length <= 16


def parse_bytes(data):
    """在任意字节流里找帧并解析，返回 [(offset, dict)]。"""
    out = []
    off = 0
    while off < len(data):
        if not _is_frame_like(data, off):
            off += 1
            continue
        length = data[off + 1]
        if len(data) - off < length:
            break  # 半帧
        frame = data[off:off + length]
        calc = sum8(frame[:-1])
        item = {
            "offset": off,
            "raw": frame,
            "len": length,
            "cmd": frame[2],
            "sum_ok": calc == frame[-1],
            "sum_calc": calc,
            "sum_recv": frame[-1],
        }
        cmd = frame[2]
        if cmd == RSP_ACK and length == 12:
            vals = struct.unpack("<hhhh", frame[3:11])
            item["theta"] = [v / 10.0 for v in vals]
        elif cmd == RSP_ERR and length == 5:
            item["err"] = frame[3]
        elif cmd == CMD_MOVE and length == 12:
            x, y, z, phi10 = struct.unpack("<hhhh", frame[3:11])
            item["pose"] = (x, y, z, phi10 / 10.0)
        elif cmd == CMD_JOINT and length == 7:
            deg10, = struct.unpack("<h", frame[4:6])
            item["joint"] = (frame[3], deg10 / 10.0)
        out.append(item)
        off += length
    return out


def describe(item):
    cmd = item["cmd"]
    name = CMD_NAME.get(cmd, "0x%02X" % cmd)
    parts = ["[%02d] %-9s len=%d %s" % (
        item["offset"], name, item["len"],
        "SUM_OK" if item["sum_ok"] else
        "SUM_BAD(calc=%02X recv=%02X)" % (item["sum_calc"], item["sum_recv"]),
    )]
    if "theta" in item:
        t = item["theta"]
        parts.append("  theta(BASE/UPPER/FOREARM/END) = %s" % ", ".join("%.1f" % v for v in t))
    if "err" in item:
        parts.append("  err = 0x%02X (%s)" % (item["err"], ERR_NAME.get(item["err"], "?")))
    if "pose" in item:
        parts.append("  pose = x=%d y=%d z=%d phi=%.1f" % item["pose"])
    if "joint" in item:
        j, d = item["joint"]
        parts.append("  joint = %d (%s) deg=%.1f" % (j, JNAME[j] if j < 4 else "?", d))
    parts.append("  hex = %s" % hexs(item["raw"]))
    return "\n".join(parts)


def parse_hex_text(text):
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = cleaned.split()
    data = bytearray()
    for tok in tokens:
        tok = tok.strip()
        if not tok:
            continue
        if len(tok) == 1:
            tok = "0" + tok
        data.append(int(tok, 16))
    return bytes(data)


def cmd_table():
    rows = [
        ("QUERY", frame_simple(CMD_QUERY)),
        ("HOME", frame_simple(CMD_HOME)),
        ("STOP", frame_simple(CMD_STOP)),
    ]
    for j in range(4):
        rows.append(("SET_JOINT %s=0" % JNAME[j], frame_joint(j, 0)))
        rows.append(("SET_JOINT %s=180" % JNAME[j], frame_joint(j, 180)))
    # home 位姿（由 FK 算得，见 DAY1 文档）
    rows.append(("MOVE home(-59,143,32,45)", frame_move(-59, 143, 32, 45)))
    print("%-28s %s" % ("用途", "帧（HEX）"))
    print("-" * 70)
    for label, f in rows:
        print("%-28s %s" % (label, hexs(f)))


def selftest():
    ok = True

    def check(label, got, want):
        nonlocal ok
        if got != want:
            ok = False
            print("FAIL %-34s got=%s want=%s" % (label, got, want))
        else:
            print("pass %-34s %s" % (label, got))

    # 1. 简单帧
    check("QUERY", hexs(frame_simple(CMD_QUERY)), "AA 04 04 B2")
    check("HOME", hexs(frame_simple(CMD_HOME)), "AA 04 02 B0")
    check("STOP", hexs(frame_simple(CMD_STOP)), "AA 04 03 B1")

    # 2. home 位姿 ACK（112.5 / 45 / -90 / 90）
    ack_home = "AA 0C 81 65 04 C2 01 7C FC 84 03 62"
    items = parse_bytes(parse_hex_text(ack_home))
    check("ACK home 帧数", len(items), 1)
    if items:
        check("ACK home 校验", items[0]["sum_ok"], True)
        check("ACK home 角度", items[0].get("theta"),
              [112.5, 45.0, -90.0, 90.0])

    # 3. MOVE 帧 round-trip
    f = frame_move(-59, 143, 32, 45)
    items = parse_bytes(f)
    check("MOVE 校验", items[0]["sum_ok"], True)
    check("MOVE 载荷", items[0]["pose"], (-59, 143, 32, 45.0))

    # 4. SET_JOINT round-trip
    f = frame_joint(0, 180)
    items = parse_bytes(f)
    check("SET_JOINT 校验", items[0]["sum_ok"], True)
    check("SET_JOINT 载荷", items[0]["joint"], (0, 180.0))

    # 5. 错误帧
    items = parse_bytes(bytes([0xAA, 0x05, 0x82, 0x01, 0x82 + 0xAA + 0x05 + 0x01 & 0xFF]))
    check("ERR 解析", items[0].get("err"), 0x01)

    # 6. 坏校验要能识别
    bad = bytearray(f)
    bad[-1] ^= 0xFF
    items = parse_bytes(bytes(bad))
    check("坏校验识别", items[0]["sum_ok"], False)

    print()
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="机械臂串口帧构造/解析工具（零依赖）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("query", help="生成 QUERY 帧")
    sub.add_parser("home", help="生成 HOME 帧")
    sub.add_parser("stop", help="生成 STOP 帧")

    p = sub.add_parser("move", help="生成 MOVE 帧")
    p.add_argument("x", type=float)
    p.add_argument("y", type=float)
    p.add_argument("z", type=float)
    p.add_argument("phi", type=float, help="末端板绝对角，单位度")

    p = sub.add_parser("joint", help="生成 SET_JOINT 帧")
    p.add_argument("index", type=int, choices=[0, 1, 2, 3],
                   help="0=BASE 1=UPPER 2=FOREARM 3=END")
    p.add_argument("deg", type=float)

    p = sub.add_parser("parse", help="解析一串 HEX（可含空格/0x/逗号）")
    p.add_argument("hex", nargs="+")

    sub.add_parser("table", help="打印常用帧对照表")
    sub.add_parser("selftest", help="运行内置自检")

    args = ap.parse_args()

    if args.cmd == "query":
        print(hexs(frame_simple(CMD_QUERY)))
    elif args.cmd == "home":
        print(hexs(frame_simple(CMD_HOME)))
    elif args.cmd == "stop":
        print(hexs(frame_simple(CMD_STOP)))
    elif args.cmd == "move":
        print(hexs(frame_move(args.x, args.y, args.z, args.phi)))
    elif args.cmd == "joint":
        print(hexs(frame_joint(args.index, args.deg)))
    elif args.cmd == "parse":
        data = parse_hex_text(" ".join(args.hex))
        items = parse_bytes(data)
        if not items:
            print("没有找到合法帧")
            return 1
        print("共 %d 字节，解析到 %d 帧" % (len(data), len(items)))
        for it in items:
            print(describe(it))
        return 0 if all(it["sum_ok"] for it in items) else 1
    elif args.cmd == "table":
        cmd_table()
    elif args.cmd == "selftest":
        return selftest()

    return 0


if __name__ == "__main__":
    sys.exit(main())
