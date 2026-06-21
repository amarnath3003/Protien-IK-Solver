# tests/conftest.py
# pytest configuration: makes the `app` package importable from `tests/`
# when pytest is run from the `backend/` directory.
import sys
import os

# Ensure `app` is importable (it lives one directory up from `tests/`)
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
