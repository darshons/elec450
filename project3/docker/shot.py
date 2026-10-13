import sys, time, pathlib
from playwright.sync_api import sync_playwright
port, out = sys.argv[1], sys.argv[2]
with sync_playwright() as p:
    b = p.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"])
    pg = b.new_page(viewport={"width": 1100, "height": 900})
    for _ in range(60):
        try:
            pg.goto(f"http://localhost:{port}"); break
        except Exception:
            time.sleep(1)
    time.sleep(12)
    pg.evaluate("document.querySelectorAll('[class*=Notification]').forEach(e => e.remove())")
    time.sleep(1)
    pg.screenshot(timeout=120000, path=out, clip={"x": 0, "y": 0, "width": 840, "height": 900}); b.close()
pathlib.Path(f"/tmp/p3/done{port}").touch()
