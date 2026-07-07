@echo off
rem FRS-SIMULATOR を .venv の Python で起動する（tkinterdnd2 等の依存が揃った環境）
cd /d "%~dp0"
".venv\Scripts\python.exe" "FRS-SIMULATOR.py" %*
