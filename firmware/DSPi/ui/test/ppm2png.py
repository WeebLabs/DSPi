import sys, glob, os
from PIL import Image
for f in glob.glob(os.path.join(sys.argv[1], "*.ppm")):
    Image.open(f).save(f[:-4] + ".png")
    os.remove(f)
