# Setup guide

You need three things: **Python 3.10+**, **Git**, and an editor. JSBSim itself
comes from `pip` as a prebuilt package (`jsbsim==1.3.1`). That package includes
the C++ flight dynamics engine, the Python bindings, the `jsbsim` command-line
program and about 60 aircraft models. You do **not** need a C++ compiler until
Module 17.

- [Windows 10 (primary)](#windows-10-primary)
- [WSL2 / Linux / macOS](#wsl2--linux--macos)
- [Daily workflow](#daily-workflow)
- [Troubleshooting](#troubleshooting)

---

## Windows 10 (primary)

### 1. Install the tools (one time)

| Tool | Where | Notes |
|---|---|---|
| Python 3.12 (64-bit) | <https://www.python.org/downloads/windows/> | In the installer, tick **"Add python.exe to PATH"** and keep **"py launcher"** ticked. |
| Git for Windows | <https://git-scm.com/download/win> | The defaults are fine. Choose "Checkout as-is, commit as-is" if you are asked about line endings; the repo's `.gitattributes` handles them. |
| VS Code | <https://code.visualstudio.com/> | Install the **Python** extension (Microsoft). Optional: "XML" (Red Hat) for editing JSBSim files. |

Open a **new** PowerShell window afterwards and check:

```powershell
py --version        # Python 3.12.x
git --version
```

### 2. Clone the repo and run setup

```powershell
cd $HOME\Documents          # or wherever you keep code
git clone https://github.com/lanemusgrave/JSBSim.git JSBSim
cd JSBSim
powershell -ExecutionPolicy Bypass -File scripts\setup.ps1
```

`setup.ps1` creates a virtual environment in `.venv\`, installs everything in
`requirements.txt` plus this repo's helper package `gnclab`, and then runs
`scripts\verify_install.py`. You should see seven `[ OK ]` lines ending with
**"All good."**

### 3. Activate the environment (every new terminal)

```powershell
.\.venv\Scripts\Activate.ps1
```

If PowerShell says *"running scripts is disabled on this system"*, allow
local scripts for your user account once:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

In VS Code, press `Ctrl+Shift+P` → **Python: Select Interpreter** → pick
`.venv\Scripts\python.exe`. New VS Code terminals then activate it automatically.

### 4. First flight

```powershell
python modules\00_setup\solutions\first_flight.py --show
```

---

## WSL2 / Linux / macOS

WSL2 gives you a real Ubuntu on your Windows 10 machine. GNC teams often run
Linux, and WSL2 is also the recommended route for building JSBSim from source
in Module 17.

```powershell
# Windows 10 version 2004+ : in an *administrator* PowerShell, then reboot
wsl --install -d Ubuntu
```

Then, inside Ubuntu (or on any Linux/macOS machine):

```bash
sudo apt update && sudo apt install -y python3 python3-venv python3-pip git   # Ubuntu
git clone https://github.com/lanemusgrave/JSBSim.git ~/JSBSim
cd ~/JSBSim
bash scripts/setup.sh
source .venv/bin/activate
```

Tip: keep the WSL copy of the repo inside the Linux file system (`~/JSBSim`),
not under `/mnt/c/...`. File access across the boundary is slow.
VS Code's **WSL** extension opens it directly.

---

## Daily workflow

```text
activate .venv  →  open modules/NN_.../README.md  →  read  →  do exercises/  →
compare with solutions/  →  pytest  →  git commit your work
```

- Every lesson script saves its plots to `outputs/<module>/` (gitignored).
  Add `--show` to also open plot windows.
- `pytest -m "not slow"` runs the quick checks; `pytest` runs everything,
  including every module solution (a few minutes).
- Commit your exercise work often. The `solutions/` folders stay as the
  reference answers.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `py` / `python` not recognized | Re-run the Python installer → *Modify* → tick "Add Python to environment variables". Open a new terminal. |
| *running scripts is disabled* | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| `jsbsim` not recognized | Activate the venv. The program lives at `.venv\Scripts\jsbsim.exe`. |
| `ModuleNotFoundError: gnclab` | Activate the venv, then `pip install -e .` from the repo root. |
| `jsbsim` CLI cannot find an aircraft | The CLI looks for `aircraft/` under `--root` (default: current folder). Pass the package data folder: `jsbsim --root="$(python -c 'import jsbsim;print(jsbsim.get_default_root_dir())')" ...` (see Module 00). |
| Plot windows don't appear | They only open with `--show`. PNGs are always written to `outputs/`. |
| Windows Firewall prompt in Module 16 | It's the UDP link between JSBSim and your controller on `localhost`. "Private networks" is enough. |
| `pip install` fails behind a corporate proxy | `pip install --proxy http://user@proxy:port -r requirements.txt`, or ask IT for the pip config. |
| A trim fails with `TrimFailureError` | The requested condition is not achievable (too slow, too fast, bad altitude). See Module 05. |
