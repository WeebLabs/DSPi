"""Test modules.  Importing this package registers every test via the
@test decorator side effects.  Add new modules to the import list below."""

from . import identity        # noqa: F401
from . import diagnostics     # noqa: F401
from . import eq              # noqa: F401
from . import dynamics        # noqa: F401
from . import outputs         # noqa: F401
from . import volume          # noqa: F401
from . import inputs          # noqa: F401
from . import crosscut        # noqa: F401
from . import presets         # noqa: F401
from . import stress          # noqa: F401
