#!/usr/bin/env python3
import os
import sys

print("[TEST] Starting manager_smoke.py", flush=True)

app_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'atelier_slam')
os.chdir(app_dir)
sys.path.insert(0, app_dir)

print("[TEST] Current directory:", os.getcwd(), flush=True)

print("[TEST] Importing manager module...", flush=True)
from atelier import TB3Manager
print("[TEST] ✓ Manager module imported", flush=True)

import tkinter as tk
print("[TEST] Creating Tk root...", flush=True)
root = tk.Tk()
print("[TEST] ✓ Tk root created", flush=True)

print("[TEST] Creating TB3Manager instance...", flush=True)
try:
    app = TB3Manager(root)
    print("[TEST] ✓ TB3Manager instance created!", flush=True)
except Exception as e:
    print(f"[TEST] ✗ Error creating TB3Manager: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("[TEST] Setting root to quit after 500ms...", flush=True)
root.after(500, app.on_closing)

print("[TEST] Starting mainloop...", flush=True)
try:
    root.mainloop()
    print("[TEST] ✓ Mainloop completed!", flush=True)
except Exception as e:
    print(f"[TEST] ✗ Mainloop error: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("[TEST] Test completed successfully!", flush=True)
