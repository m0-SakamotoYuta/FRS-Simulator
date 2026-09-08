"""FRS-SIMULATOR.py へのパッチ適用ヘルパー（CRLF を保つ）。"""
import io
import sys

SRC = r"C:\Users\yutas\Desktop\FRS-Simulator\FRS-SIMULATOR.py"


def read():
	with io.open(SRC, "r", encoding="utf-8", newline="") as f:
		return f.read()


def write(s):
	with io.open(SRC, "w", encoding="utf-8", newline="") as f:
		f.write(s)


def replace_once(s, old, new, label=""):
	n = s.count(old)
	if n != 1:
		raise SystemExit(f"[FAIL] {label}: 出現回数 {n} (1 でなければならない)\n---\n{old[:300]}\n---")
	return s.replace(old, new, 1)


def insert_before(s, anchor, block, label=""):
	return replace_once(s, anchor, block + anchor, label)


def insert_after(s, anchor, block, label=""):
	return replace_once(s, anchor, anchor + block, label)
