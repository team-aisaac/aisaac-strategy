import sys

if sys.version_info.major != 2:
    from pathlib import Path
    sys.path.append(str(Path(__file__).parent))

