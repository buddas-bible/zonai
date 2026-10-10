from pathlib import Path

path = Path("src/dynamics/world.cpp")
text = path.read_text(encoding="utf-8")
old = "mouseJointSim2>>>>>>>;"
new = "mouseJointSim2>>>>>>>>;"
count = text.count(old)
if count != 1:
    raise RuntimeError(f"expected one simType closing marker, found {count}")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
print("Fixed Pogo simType conditional closing")
