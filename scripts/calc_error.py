"""Похибка досягнення цілі (варіант 9).
Використання: python3 scripts/calc_error.py X_ФАКТ Y_ФАКТ [X_ЦІЛЬ Y_ЦІЛЬ]"""
import sys, math
xa, ya = float(sys.argv[1]), float(sys.argv[2])
xt = float(sys.argv[3]) if len(sys.argv) > 3 else -5.200
yt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.100
e = math.hypot(xa - xt, ya - yt)
print(f"Фактичні координати: ({xa:.3f}; {ya:.3f})")
print(f"Ціль: ({xt:.3f}; {yt:.3f})")
print(f"Похибка e = {e:.3f} м")
