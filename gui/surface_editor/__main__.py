"""Entry point: python -m surface_editor [file.3dm]"""

import sys

if "--ai" in sys.argv:
    sys.argv.remove("--ai")
    from .ai_app import main
else:
    from .editor import main

main()
