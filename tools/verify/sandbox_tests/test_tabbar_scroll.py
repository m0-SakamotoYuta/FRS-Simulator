import importlib.util, os, time
HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
spec = importlib.util.spec_from_file_location("frs", os.path.join(HERE, "FRS-SIMULATOR.py"))
frs = importlib.util.module_from_spec(spec); spec.loader.exec_module(frs)
fails = []
def check(c, l):
    print(("  OK  " if c else "  NG  ") + l); (None if c else fails.append(l))
def pump(app, n=3):
    for _ in range(n): app.update_idletasks(); app.update()
app = frs.MainMenuGUI(); pump(app, 5)
for i in range(app.notebook.index("end")):
    if "ankle" in app.notebook.tab(i, "text"): app.notebook.select(i)
pump(app, 5)
for _ in range(10): time.sleep(0.1); pump(app, 3)
ui = app._tabbar_ui["ankle"]; cv = ui["canvas"]
def visible(btn):
    lo, hi = cv.xview(); tot = ui["inner"].winfo_reqwidth()
    x0, x1 = btn.winfo_x(), btn.winfo_x() + btn.winfo_width()
    return lo * tot - 1 <= x0 and x1 <= hi * tot + 1
act = ui["active"]
check(act is not None and visible(act), f"起動直後: 開いているタブ({app._ankle_active_tab}) が帯に見えている {cv.xview()}")
cv.xview_moveto(0.0); app._tabbar_update_arrows("ankle"); pump(app)
prev = cv.xview()[0]; steps = []
for k in range(6):
    app._tabbar_scroll("ankle", +1); pump(app)
    steps.append(round(cv.xview()[0] - prev, 4)); prev = cv.xview()[0]
print("   ▶ 1回ごとの移動量:", steps)
check(min(steps) > 0 and max(steps) - min(steps) < 0.002 and max(steps) < 0.1, "▶ は少しずつ右へ進む（開いているタブへ飛ばない）")
ui["end"].invoke(); pump(app, 5)
check(cv.xview()[1] > 0.999, f"右端へ → 右端のまま {cv.xview()}")
for k in range(3):
    app._tabbar_scroll("ankle", -1); pump(app)
check(cv.xview()[1] < 0.999, f"右端から ◀ で少し戻る {cv.xview()}")
app.on_ankle_tab_select(0); pump(app, 5)
check(cv.xview()[0] < 0.01, f"左端のタブを開くと、帯はそのタブへ寄る {cv.xview()}")
app.on_ankle_tab_select(len(app._ankle_tabs) - 1); pump(app, 5)
check(cv.xview()[1] > 0.999, f"右端のタブを開くと、帯は右端へ寄る {cv.xview()}")
app.on_ankle_tab_add(); pump(app, 5)
check(visible(ui["active"]), "＋で作ったタブが帯に見えている")
app.destroy()
print("RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL")
