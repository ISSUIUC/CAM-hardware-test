from pathlib import Path
import os
import time
import shutil
from tqdm import tqdm

home=Path(os.environ["USERPROFILE"]); dev=Path("C:/dev"); src=home/".platformio"; dst=dev/".platformio"
dst.mkdir(parents=True, exist_ok=True)
print("Copying .platformio (This will take a while...)")
if src.exists() and not src.is_symlink():
    for p in tqdm(src.iterdir()):
        t=dst/p.name
        shutil.copytree(p,t,dirs_exist_ok=True) if p.is_dir() else shutil.copy2(p,t)
    src.rename(src.with_name(f".platformio.bak_{int(time.time())}"))
elif src.is_symlink(): src.unlink()
print("Generating symlink..")
src.symlink_to(dst, target_is_directory=True)
(dev/"proj").mkdir(parents=True, exist_ok=True); (dev/"proj"/Path.cwd().name).symlink_to(Path.cwd(), target_is_directory=True)
print("OK: copied old .platformio -> C:/dev/.platformio, linked ~/.platformio and safe project path")

print("\n------ NEXT STEPS ------")
print("1) Set the proper $PLATFORMIO_CORE_DIR variable:")
print("\nsetx PLATFORMIO_CORE_DIR C:/dev/.platformio")
print("\n2) Build the project again (this will trigger a full rebuild, so sit tight!)")
print("\nNOTE: This will ONLY work on windows! I'm too lazy to make a platform-independent script, sorry.\n  -Michael")