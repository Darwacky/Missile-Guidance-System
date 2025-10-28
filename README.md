Missile Guidance Simulation

This small script simulates a proportional navigation missile guidance example and plots the missile and target paths.

Setup

1. Ensure Python 3.10+ is installed and available on PATH (or note the full path to your python executable).
2. From this folder, install dependencies:

   python -m pip install --upgrade pip
   python -m pip install -r requirements.txt

Run

   python "c:\Coding\missile guidance system.py"

Notes

- If VS Code shows "Import could not be resolved", select the Python interpreter that has the dependencies installed (e.g., the WindowsApps python3.12 executable shown in the workspace).
- The code was updated to avoid a NumPy 2.0 deprecation by using a 3D cross product for 2D vectors.
