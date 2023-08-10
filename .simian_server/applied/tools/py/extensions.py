import sys

DYNAMIC_LIBRARY = "so" if sys.platform == "linux" else "dll"
