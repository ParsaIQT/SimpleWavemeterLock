"""Double-click launcher (on Windows .pyw files start without a console window)."""
from wmlock.__main__ import main

raise SystemExit(main())
