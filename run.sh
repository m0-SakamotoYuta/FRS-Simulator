#!/bin/sh
# FRS-SIMULATOR を .venv の Python で起動する（Mac/Linux 用。Windows は run.bat）
# 初回セットアップ: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
cd "$(dirname "$0")" || exit 1
if [ -x ".venv/bin/python" ]; then
    exec .venv/bin/python FRS-SIMULATOR.py "$@"
fi
echo ".venv が見つかりません。以下でセットアップしてください:"
echo "  python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"
exit 1
