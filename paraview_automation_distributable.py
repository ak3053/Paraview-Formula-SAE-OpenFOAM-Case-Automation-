r"""
SimScale FOAM -> ParaView automation   (run:  pvpython paraview_automation_distributable.py)

Pipeline Overview:
  1. Auto-resolves non-overwriting output directory:
     - Checks if BASE_OUTPUT_DIR exists; if so, increments to paraview_output(1), (2), etc.
  2. Prints configuration summary upfront and prompts for slice-sweep videos:
     - Static Pressure (p) Front-to-Rear sweep video [y/N]
     - Total Pressure (total(p)) Front-to-Rear sweep video [y/N]
     - Velocity (U) Front-to-Rear sweep video [y/N]
  3. Loads boundary patches for 3D body surface (internalMesh strictly excluded):
     - Static surface views for wallShearStress (Surface LIC + vectors, Turbo [0, 1] Pa)
     - Static surface views for pressure:
       * Clean Surface (p_surface_*, Turbo [-200, 80] Pa)
       * Surface LIC with wallShearStress vectors (p_lic_*, Turbo [-200, 80] Pa)
     - Static surface views for totalPressure (clean Surface, no vectors, Turbo [-25, 75] Pa)
     - Static surface views for velocity (clean Surface, no vectors, Turbo [0, 20] m/s)
     - Perspectives for all static measurements: Iso, Side, Front, Rear, Top, PLUS Bottom view (_bottom)
  4. Loads air volume (internalMesh) for 2D planar contour slices:
     - Spanwise Y-normal slices (y = 0.01, 0.15, 0.20, 0.30, 0.40, 0.50 m):
       Stored directly in velocity_Y/, pressure_Y/, totalPressure_Y/
       Generates both standard and '_streamlines' (Surface LIC with U vectors) views.
     - Horizontal Z-normal slices (z = 0.05, 0.10, 0.20, 0.30 m):
       Stored directly in velocity_Z/, pressure_Z/, totalPressure_Z/
       Generates both standard and '_streamlines' views (viewed top-down).
  5. Front-to-Rear sweep videos (+X down to -X):
     - Compiled MP4s stored in respective field directories:
       - pressure/pressure_slice_sweep_60fps.mp4
       - totalPressure/totalPressure_slice_sweep_60fps.mp4
       - velocity/velocity_slice_sweep_60fps.mp4
     - Individual PNG frames retained under slice_frames/:
       - slice_frames/slice_frames_pressure/
       - slice_frames/slice_frames_totalPressure/
       - slice_frames/slice_frames_velocity/
"""

try:
    from paraview.simple import (
        OpenFOAMReader, CellDatatoPointData, MergeBlocks,
        ExtractSurface, Slice,
        GetActiveViewOrCreate, Show, Hide,
        ColorBy, GetColorTransferFunction, GetOpacityTransferFunction, GetScalarBar,
        Render, RenderAllViews,
        SaveScreenshot, LoadPlugin, GetAnimationScene,
        _DisableFirstRenderCameraReset,
    )
except ImportError:
    print("\n" + "=" * 78)
    print(" [ERROR] ParaView Python module ('paraview.simple') was not found.")
    print("=" * 78)
    print(" This automation script must be executed using ParaView's Python interpreter, please see docs ParaView Post-Processing Script SOP:")
    print("     pvpython paraview_automation_distributable.py")
    print(" Or for headless execution on a cluster/server:")
    print("     pvbatch paraview_automation_distributable.py")
    print("=" * 78 + "\n")
    import sys
    sys.exit(1)

import os
import re
import sys
import math
import shutil
import subprocess
import json

# Ensure console standard input is bound to genuine OS console in pvpython
if hasattr(sys, '__stdin__') and sys.__stdin__ is not None:
    sys.stdin = sys.__stdin__

def rgb_fb(r1, g1, b1, r2, g2, b2):
    """Sets top pixel (foreground) and bottom pixel (background)"""
    return f"\x1b[38;2;{r1};{g1};{b1};48;2;{r2};{g2};{b2}m▀\x1b[0m"

rd = (255,0,0)
gr = (0,255,0)
yl = (255,255,0)
wh = (255,255,255)

def safe_input(prompt=""):
    """
    Robust console prompt reader for pvpython.
    Directly accesses the true console standard input, ensuring that pressing
    Enter cleanly submits an empty string rather than advancing lines or hanging.
    """
    sys.stdout.write(prompt)
    sys.stdout.flush()
    stream = sys.__stdin__ if (hasattr(sys, '__stdin__') and sys.__stdin__ is not None) else sys.stdin
    try:
        line = stream.readline()
    except Exception:
        line = ""
    if line is None or (len(line) == 0):
        raise EOFError()
    return line.rstrip("\r\n")

# =============================================================================
# PERSISTENT PATH CONFIGURATION (Auto-updated by script)
# =============================================================================
# These values are configured and saved automatically when running the script.
# Initialized as empty ("") for generic distribution to FSAE team members.
# On first run, the script will interactively guide the user to set their paths.
CONFIGURED_FOAM_FILE    = ''
CONFIGURED_OUTPUT_DIR   = ''
CONFIG_FALLBACK_FILE    = "paraview_config.json"

# ---- Boundary Patches (Car Body) -------------------------------------------
# Filter mode:
#   'remove'  : Keeps all boundary patches EXCEPT those in FACE_NUMBERS.
#               (Recommended: removes outer wind tunnel walls & bounding box)
#   'isolate' : Keeps ONLY the boundary patches in FACE_NUMBERS.
FACE_FILTER_MODE = 'remove'

# Face numbers: matched against patch names ending in that integer (e.g. 298 matches 'patch/face298').
FACE_NUMBERS     = [298, 299, 300, 301, 302, 303]

# ---- Coordinate Convention -------------------------------------------------
# +X : Front (nose cone)
# -X : Rear (rear wing / diffuser)
# +Y : Exterior bodywork (symmetry plane is at Y=0 facing -Y)
# +Z : Vertical (up)

# ---- Fields & Ranges -------------------------------------------------------
COLOR_PRESET       = 'Turbo'

WSS_ARRAY          = 'wallShearStress'
WSS_RANGE          = (0.0, 1.0)        # Pa: 0 to 1

PRESSURE_ARRAY     = 'p'
PRESSURE_RANGE     = (-200.0, 80.0)    # Pa: -200 to 80

TOTAL_P_CANDIDATES = ['total(p)', 'totalPressure', 'p_total', 'p0']
TOTAL_P_RANGE      = (-25.0, 75.0)     # Pa: -25 to 75

VELOCITY_ARRAY     = 'U'
VELOCITY_RANGE     = (0.0, 20.0)       # m/s: 0 to 20

# ---- Surface LIC Settings (for wallShearStress only) -----------------------
LIC_NUM_STEPS      = 40
LIC_STEP_SIZE      = 0.5
LIC_INTENSITY      = 0.8

# ---- Slice Locations -------------------------------------------------------
# Y-Normal spanwise vertical cut planes (side-on view)
Y_SLICE_LOCATIONS  = [0.01, 0.15, 0.20, 0.30, 0.40, 0.50]

# Z-Normal horizontal cut planes (top-down view)
Z_SLICE_LOCATIONS  = [0.05, 0.10, 0.20, 0.30]

# ---- Video Sweep Settings (Front to Rear: +X -> -X) ------------------------
N_FRAMES           = 1000
VIDEO_FPS          = 60
SWEEP_PAD          = 0.15              # Fraction of body X-extent added before/after sweep
FRAME_PAD          = 1.6               # Framing zoom factor
SHOW_BODY_OUTLINE  = True              # Black silhouette line where cut plane intersects body

# ---- General Rendering Settings --------------------------------------------
USE_LAST_TIMESTEP      = True
RESOLUTION             = [1920, 1080]
CAMERA_DISTANCE_FACTOR = 2.5           # Multiple of bounding-box diagonal

# Static 3D surface camera presets: (direction from object centre, view-up)
CAMERA_PRESETS = {
    '01_iso':   ([ 1.0,  1.0,  0.7], [0, 0, 1]),  # Front 3/4 isometric of outer bodywork
    '02_side':  ([ 0.0,  1.0,  0.0], [0, 0, 1]),  # Exterior side profile looking along -Y
    '03_front': ([ 1.0,  0.0,  0.0], [0, 0, 1]),  # Front view looking at nose along -X
    '04_rear':  ([-1.0,  0.0,  0.0], [0, 0, 1]),  # Rear view looking at diffuser along +X
    '05_top':   ([ 0.0,  0.0,  1.0], [1, 0, 0]),  # Top-down view with nose pointing up
}

# Bottom view camera parameters (looking from underneath up toward +Z)
BOTTOM_CAM_POS   = [1.0, 0.5, -6.0]
BOTTOM_CAM_FOCAL = [1.0, 0.5, 0.6]
BOTTOM_CAM_UP    = [0.0, 1.0, 0.0]    # Exterior side (+Y) points toward the top of the image

_DisableFirstRenderCameraReset()

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def resolve_unique_output_dir(base_dir):
    """
    Resolves a unique, non-overwriting output directory.
    - If base_dir is a parent folder (e.g. Downloads, Documents, Desktop, user profile,
      or any directory not named 'paraview_output'), it places a dedicated 'paraview_output'
      subfolder inside it, incrementing to paraview_output(1), (2), etc.
      It will NEVER rename or touch the parent directory (e.g. will NEVER create Downloads(1)).
    - If base_dir already specifies a target run folder, it increments only that child folder.
    """
    base_dir = os.path.abspath(base_dir)
    base_name = os.path.basename(base_dir).lower()
    home = os.path.expanduser("~").lower()

    is_parent_folder = (
        base_dir.lower() in (home, os.path.dirname(home))
        or base_name in ("downloads", "documents", "desktop", "temp", "tmp")
        or (os.path.isdir(base_dir) and not base_name.startswith("paraview_output"))
    )

    if is_parent_folder:
        target_dir = os.path.join(base_dir, "paraview_output")
    else:
        target_dir = base_dir

    if not os.path.exists(target_dir):
        return target_dir

    parent = os.path.dirname(target_dir)
    stem = os.path.basename(target_dir)
    m = re.match(r"^(.*?)(?:\(\d+\))?$", stem)
    clean_stem = m.group(1) if m else stem

    i = 1
    while True:
        candidate = os.path.join(parent, f"{clean_stem}({i})")
        if not os.path.exists(candidate):
            return candidate
        i += 1


def get_default_output_dir():
    """Returns the default output directory (~/Downloads/paraview_output)."""
    home = os.path.expanduser("~")
    downloads = os.path.join(home, "Downloads")
    if os.path.isdir(downloads):
        return os.path.join(downloads, "paraview_output")
    return os.path.join(home, "paraview_output")


def get_path_examples():
    """Returns platform-specific example paths for guidance."""
    if sys.platform.startswith("win"):
        return (
            r"C:\Users\YourName\Downloads\SimScale_Run_1\case.foam",
            r"C:\Users\YourName\Downloads\paraview_output"
        )
    return (
        "/home/yourname/Downloads/SimScale_Run_1/case.foam",
        "/home/yourname/Downloads/paraview_output"
    )


def browse_file_dialog(title="Select OpenFOAM case.foam File"):
    """
    Lightweight, zero-external-dependency native file selection dialog.
    Tries Tkinter first, falls back to PowerShell on Windows, AppleScript on macOS, or zenity on Linux.
    """
    # 1. Tkinter if available in this python environment
    try:
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        path = filedialog.askopenfilename(
            title=title,
            filetypes=[("OpenFOAM Case", "*.foam *.OpenFOAM"), ("All Files", "*.*")]
        )
        root.destroy()
        if path:
            return os.path.normpath(path)
    except Exception:
        pass

    # 2. Windows fallback using PowerShell Single-Threaded Apartment (STA) OpenFileDialog
    if sys.platform.startswith("win"):
        ps_script = (
            "[System.Reflection.Assembly]::LoadWithPartialName('System.Windows.Forms') | Out-Null; "
            "$f = New-Object System.Windows.Forms.OpenFileDialog; "
            "$f.Filter = 'OpenFOAM Case (*.foam;*.OpenFOAM)|*.foam;*.OpenFOAM|All Files (*.*)|*.*'; "
            f"$f.Title = '{title}'; "
            "if ($f.ShowDialog() -eq [System.Windows.Forms.DialogResult]::OK) { [Console]::WriteLine($f.FileName) }"
        )
        try:
            res = subprocess.run(
                ["powershell", "-NoProfile", "-STA", "-Command", ps_script],
                capture_output=True, text=True, timeout=120
            )
            out = res.stdout.strip()
            if out:
                return os.path.normpath(out)
        except Exception:
            pass

    # 3. macOS fallback using AppleScript
    if sys.platform == "darwin":
        try:
            cmd = f'POSIX path of (choose file of type {{"foam", "OpenFOAM"}} with prompt "{title}")'
            res = subprocess.run(["osascript", "-e", cmd], capture_output=True, text=True, timeout=120)
            out = res.stdout.strip()
            if out:
                return os.path.normpath(out)
        except Exception:
            pass

    # 4. Linux fallback using zenity
    if not sys.platform.startswith("win") and sys.platform != "darwin":
        try:
            res = subprocess.run(
                ["zenity", "--file-selection", f"--title={title}", "--file-filter=*.foam *.OpenFOAM"],
                capture_output=True, text=True, timeout=120
            )
            out = res.stdout.strip()
            if out:
                return os.path.normpath(out)
        except Exception:
            pass

    return None


def browse_directory_dialog(title="Select Output Directory for Visualizations", initial_dir=None):
    """
    Lightweight, zero-external-dependency native directory selection dialog.
    """
    # 1. Tkinter if available
    try:
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        path = filedialog.askdirectory(title=title, initialdir=initial_dir)
        root.destroy()
        if path:
            return os.path.normpath(path)
    except Exception:
        pass

    # 2. Windows fallback using modern OpenFileDialog (looks identical to case file browser)
    if sys.platform.startswith("win"):
        init_part = f"$f.InitialDirectory = '{initial_dir}'; " if initial_dir and os.path.isdir(initial_dir) else ""
        ps_script = (
            "[System.Reflection.Assembly]::LoadWithPartialName('System.Windows.Forms') | Out-Null; "
            "$f = New-Object System.Windows.Forms.OpenFileDialog; "
            "$f.ValidateNames = $false; "
            "$f.CheckFileExists = $false; "
            "$f.CheckPathExists = $true; "
            "$f.FileName = 'Select Folder'; "
            f"$f.Title = '{title}'; "
            + init_part +
            "if ($f.ShowDialog() -eq [System.Windows.Forms.DialogResult]::OK) { "
            "  $p = $f.FileName; "
            "  if ([System.IO.Directory]::Exists($p)) { [Console]::WriteLine($p) } "
            "  else { [Console]::WriteLine([System.IO.Path]::GetDirectoryName($p)) } "
            "}"
        )
        try:
            res = subprocess.run(
                ["powershell", "-NoProfile", "-STA", "-Command", ps_script],
                capture_output=True, text=True, timeout=120
            )
            out = res.stdout.strip()
            if out:
                p = os.path.normpath(out)
                if not os.path.isdir(p):
                    p = os.path.dirname(p)
                return p
        except Exception:
            pass

    # 3. macOS fallback using AppleScript
    if sys.platform == "darwin":
        try:
            cmd = f'POSIX path of (choose folder with prompt "{title}")'
            res = subprocess.run(["osascript", "-e", cmd], capture_output=True, text=True, timeout=120)
            out = res.stdout.strip()
            if out:
                return os.path.normpath(out)
        except Exception:
            pass

    # 4. Linux fallback using zenity
    if not sys.platform.startswith("win") and sys.platform != "darwin":
        try:
            res = subprocess.run(
                ["zenity", "--file-selection", "--directory", f"--title={title}"],
                capture_output=True, text=True, timeout=120
            )
            out = res.stdout.strip()
            if out:
                return os.path.normpath(out)
        except Exception:
            pass

    return None


def clean_path_input(raw):
    """Sanitizes user input by stripping quotes, whitespace, and resolving relative paths/home."""
    if not raw:
        return ""
    val = raw.strip()
    if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
        val = val[1:-1].strip()
        print("  [Action Taken] Removed surrounding quotation marks from path.")
    if val.startswith("~"):
        val = os.path.expanduser(val)
        print("  [Action Taken] Expanded '~' to user home directory.")
    return os.path.normpath(val)


def persist_configuration(foam_file, output_dir):
    """
    Saves input/output paths. First attempts to modify this Python script in-place so no
    extra files are created and the script remains fully portable. If script self-modification
    fails (e.g. read-only permissions), falls back to saving paraview_config.json.
    """
    script_path = None
    try:
        if "__file__" in globals() and globals()["__file__"]:
            cand = os.path.abspath(__file__)
            if os.path.isfile(cand):
                script_path = cand
    except Exception:
        pass

    saved_in_script = False
    if script_path:
        try:
            with open(script_path, "r", encoding="utf-8") as f:
                content = f.read()

            foam_repr = repr(os.path.normpath(foam_file))
            output_repr = repr(os.path.normpath(output_dir))

            new_content = re.sub(
                r'^CONFIGURED_FOAM_FILE\s*=.*$',
                lambda m: f'CONFIGURED_FOAM_FILE    = {foam_repr}',
                content,
                flags=re.MULTILINE
            )
            new_content = re.sub(
                r'^CONFIGURED_OUTPUT_DIR\s*=.*$',
                lambda m: f'CONFIGURED_OUTPUT_DIR   = {output_repr}',
                new_content,
                flags=re.MULTILINE
            )

            if new_content != content:
                with open(script_path, "w", encoding="utf-8") as f:
                    f.write(new_content)
                print(f"[Saved] Configuration saved directly inside '{os.path.basename(script_path)}'!")
                saved_in_script = True
        except Exception:
            # Script modification blocked by OS/environment; will fallback below
            pass

    if not saved_in_script:
        script_dir = os.path.dirname(script_path) if script_path else os.getcwd()
        json_path = os.path.join(script_dir, CONFIG_FALLBACK_FILE)
        try:
            cfg = {}
            if os.path.isfile(json_path):
                try:
                    with open(json_path, "r", encoding="utf-8") as f:
                        cfg = json.load(f)
                except Exception:
                    cfg = {}
            cfg["foam_file"] = os.path.normpath(foam_file)
            cfg["output_dir"] = os.path.normpath(output_dir)
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            print(f"[Saved] Configuration saved to '{CONFIG_FALLBACK_FILE}'.")
            print(f"[Notice] Please ensure '{CONFIG_FALLBACK_FILE}' and this script remain in the same directory.")
        except Exception as e:
            # Last-resort fallback to user home directory
            home_cfg = os.path.join(os.path.expanduser("~"), ".simscale_paraview_config.json")
            try:
                cfg = {}
                if os.path.isfile(home_cfg):
                    try:
                        with open(home_cfg, "r", encoding="utf-8") as f:
                            cfg = json.load(f)
                    except Exception:
                        cfg = {}
                cfg["foam_file"] = os.path.normpath(foam_file)
                cfg["output_dir"] = os.path.normpath(output_dir)
                with open(home_cfg, "w", encoding="utf-8") as f:
                    json.dump(cfg, f, indent=2)
                print(f"[Saved] Configuration saved to '{home_cfg}'.")
            except Exception as e2:
                print(f"[Warning] Could not save paths to configuration file: {e2}")


def persist_face_numbers(face_numbers):
    """
    Saves face numbers to remove. First attempts to modify this Python script in-place so no
    extra files are created and the script remains fully portable. If script self-modification
    fails (e.g. read-only permissions), falls back to updating paraview_config.json.
    """
    script_path = None
    try:
        if "__file__" in globals() and globals()["__file__"]:
            cand = os.path.abspath(__file__)
            if os.path.isfile(cand):
                script_path = cand
    except Exception:
        pass

    faces_sorted = sorted(list(set(int(x) for x in face_numbers)))
    saved_in_script = False
    if script_path:
        try:
            with open(script_path, "r", encoding="utf-8") as f:
                content = f.read()

            new_content = re.sub(
                r'^(# Face numbers: matched against patch names.*\n)FACE_NUMBERS\s*=.*$',
                lambda m: m.group(1) + f'FACE_NUMBERS     = {faces_sorted}',
                content,
                flags=re.MULTILINE
            )

            if new_content != content:
                with open(script_path, "w", encoding="utf-8") as f:
                    f.write(new_content)
                print(f"[Saved] Default face numbers saved directly inside '{os.path.basename(script_path)}'!")
                saved_in_script = True
        except Exception:
            # Script modification blocked by OS/environment; will fallback below
            pass

    # Save to local paraview_config.json
    script_dir = os.path.dirname(script_path) if script_path else os.getcwd()
    json_path = os.path.join(script_dir, CONFIG_FALLBACK_FILE)
    try:
        cfg = {}
        if os.path.isfile(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
            except Exception:
                cfg = {}
        cfg["face_numbers"] = faces_sorted
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        if not saved_in_script:
            print(f"[Saved] Face numbers saved to '{CONFIG_FALLBACK_FILE}'.")
    except Exception:
        pass

    # Fallback to user home directory
    home_cfg = os.path.join(os.path.expanduser("~"), ".simscale_paraview_config.json")
    try:
        cfg = {}
        if os.path.isfile(home_cfg):
            try:
                with open(home_cfg, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
            except Exception:
                cfg = {}
        cfg["face_numbers"] = faces_sorted
        with open(home_cfg, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


def load_saved_face_numbers():
    """Loads previously saved face numbers if available, falling back to embedded FACE_NUMBERS."""
    script_dir = os.path.dirname(os.path.abspath(__file__)) if ("__file__" in globals() and globals()["__file__"]) else os.getcwd()
    for cfg_path in [
        os.path.join(script_dir, CONFIG_FALLBACK_FILE),
        os.path.join(os.path.expanduser("~"), ".simscale_paraview_config.json")
    ]:
        if os.path.isfile(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    faces = data.get("face_numbers")
                    if faces is not None and isinstance(faces, list):
                        return [int(x) for x in faces]
            except Exception:
                pass
    return list(FACE_NUMBERS)


def parse_face_numbers_input(raw_input):
    """
    Parses comma-separated list of face numbers.
    Accepts raw integers (e.g. '298, 299, 300') as well as patch names with prefixes
    (e.g. 'face298, patch/face299'), automatically extracting trailing integer numbers.
    Returns (list_of_ints, is_valid, error_msg).
    """
    cleaned = raw_input.strip()
    if not cleaned:
        return [], True, ""

    if cleaned.lower() in ('none', 'empty', 'clear', '[]'):
        return [], True, ""

    tokens = [t.strip() for t in re.split(r'[,;\s]+', cleaned) if t.strip()]
    if not tokens:
        return [], True, ""

    parsed = []
    for t in tokens:
        # Match trailing integer or any integer inside the token
        m = re.search(r'(\d+)$', t)
        if not m:
            m = re.search(r'(\d+)', t)
        if m:
            parsed.append(int(m.group(1)))
        else:
            return None, False, f"Could not extract a valid face number from '{t}'."

    # Deduplicate and sort
    return sorted(list(set(parsed))), True, ""


def prompt_for_face_numbers(current_default=None):
    """
    Prompts the user for face numbers to remove:
      1. Displays current default face numbers.
      2. Accepts a comma-separated list of integers / patch names.
         Pressing [Enter] without input keeps the current default.
      3. Validates and displays the parsed selection.
      4. Asks for confirmation with a [Y/n] question. If 'n', loops and tries again.
      5. Automatically saves changed face numbers back into the script as new defaults.
    """
    curr = current_default if current_default is not None else list(FACE_NUMBERS)

    print("\n" + "=" * 78)
    print("      BOUNDARY PATCH SELECTION (FACE NUMBERS TO REMOVE)")
    print("=" * 78)
    print(f" Current Face Numbers Default: {curr}")
    print(" Explanation: Enter the patch/face numbers of outer wind tunnel walls or")
    print("              bounding box to remove them from the 3D car body surface.")
    print(" Options:")
    print("   - Press [Enter] to keep the current default face numbers")
    print("   - Enter a comma-separated list of face numbers or patch names")
    print("     (e.g. 298, 299, 300, 301, 302, 303 or face298, patch/face299)")
    print("   - Enter 'none' if you do not want to remove any patches")
    print("-" * 78)

    while True:
        try:
            raw_input = safe_input("Enter face numbers to remove [Press Enter to keep current]: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n[Aborted] Setup canceled by user.")
            sys.exit(0)

        # Press Enter without input -> keep current default
        if not raw_input:
            chosen = list(curr)
        else:
            chosen, ok, err_msg = parse_face_numbers_input(raw_input)
            if not ok:
                print(f"  [Warning] {err_msg}")
                print("  Please try again.\n")
                continue

        print(f"\n  [Selected] Face numbers to remove: {chosen}")

        confirmed = False
        while True:
            try:
                confirm = safe_input("Confirm face numbers selection? [Y/n]: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                print("\n[Aborted] Setup canceled by user.")
                sys.exit(0)

            if confirm in ('', 'y', 'yes'):
                confirmed = True
                break
            elif confirm in ('n', 'no'):
                print("  [Notice] Selection canceled. Let's try again.\n")
                break
            else:
                print("  Please enter 'y' to confirm or 'n' to re-enter.")

        if confirmed:
            if chosen != curr:
                persist_face_numbers(chosen)
            return chosen


def load_saved_configuration():
    """Loads previously saved foam_file and output_dir if available."""
    # 1. Embedded script variables
    if CONFIGURED_FOAM_FILE and CONFIGURED_OUTPUT_DIR:
        return CONFIGURED_FOAM_FILE, CONFIGURED_OUTPUT_DIR

    # 2. Local paraview_config.json
    script_dir = os.path.dirname(os.path.abspath(__file__)) if ("__file__" in globals() and globals()["__file__"]) else os.getcwd()
    json_path = os.path.join(script_dir, CONFIG_FALLBACK_FILE)
    if os.path.isfile(json_path):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
                f_path = data.get("foam_file", "")
                o_path = data.get("output_dir", "")
                if f_path and o_path:
                    return f_path, o_path
        except Exception:
            pass

    # 3. User home directory config
    home_cfg = os.path.join(os.path.expanduser("~"), ".simscale_paraview_config.json")
    if os.path.isfile(home_cfg):
        try:
            with open(home_cfg, "r", encoding="utf-8") as f:
                data = json.load(f)
                f_path = data.get("foam_file", "")
                o_path = data.get("output_dir", "")
                if f_path and o_path:
                    return f_path, o_path
        except Exception:
            pass

    return None, None


def validate_and_resolve_foam_path(raw_path):
    """
    Validates input path, explains errors, auto-fixes common beginner issues (quotes, folder pasted instead
    of file, finds .foam file), verifies OpenFOAM case structure, and reports all actions.
    Returns (resolved_path, is_valid).
    """
    cleaned = clean_path_input(raw_path)
    if not cleaned:
        print("  [Warning] Path cannot be empty.")
        return None, False

    if not os.path.exists(cleaned):
        print(f"\n  [Warning] Invalid path: File or directory not found at:")
        print(f"            '{cleaned}'")
        print("  Explanation: The path does not exist on your computer.")
        print("  Tip: Verify that you copied the complete path and that the case folder was not moved.")
        return None, False

    # Check if a folder was pasted instead of the .foam file
    if os.path.isdir(cleaned):
        print(f"\n  [Warning] The path entered is a folder, not a file:")
        print(f"            '{cleaned}'")
        print("  Explanation: ParaView requires the specific '.foam' case file, not just the folder.")
        try:
            candidates = [
                f for f in os.listdir(cleaned)
                if f.lower().endswith(('.foam', '.openfoam'))
            ]
        except Exception:
            candidates = []

        if candidates:
            chosen = "case.foam" if "case.foam" in candidates else candidates[0]
            resolved = os.path.join(cleaned, chosen)
            print(f"  [Action Taken] Located OpenFOAM case file inside folder: '{chosen}'")
            print(f"                 Auto-selected: '{resolved}'")
            cleaned = resolved
        else:
            print(f"  [Error] No '.foam' file was found directly inside '{cleaned}'.")
            print("  Tip: In SimScale exported results, there is typically a file named 'case.foam'.")
            print("       If missing, you can create an empty text file named 'case.foam' in that folder.")
            return None, False

    # Check file extension
    if not cleaned.lower().endswith(('.foam', '.openfoam')):
        print(f"\n  [Warning] File '{os.path.basename(cleaned)}' does not have a '.foam' extension.")
        print("  Explanation: ParaView's OpenFOAM reader requires a file ending in '.foam' or '.OpenFOAM'.")
        try:
            ans = safe_input("  Would you like to try loading it anyway? [y/N]: ").strip().lower()
            if ans not in ('y', 'yes'):
                return None, False
        except (KeyboardInterrupt, EOFError):
            return None, False

    # Check OpenFOAM constant/ directory structure
    case_dir = os.path.dirname(cleaned)
    has_constant = (
        os.path.isdir(os.path.join(case_dir, "constant"))
        or os.path.isdir(os.path.join(case_dir, "processor0", "constant"))
    )
    if not has_constant:
        print(f"\n  [Warning] Missing OpenFOAM mesh directory in: '{case_dir}'")
        print("  Explanation: No 'constant/' directory was found alongside the case file.")
        print("               ParaView needs the OpenFOAM mesh data inside 'constant/' to load CFD fields.")
        print("  Tip: Make sure you extracted the complete downloaded SimScale zip archive.")
        try:
            ans = safe_input("  Would you like to proceed with this file anyway? [y/N]: ").strip().lower()
            if ans not in ('y', 'yes'):
                return None, False
        except (KeyboardInterrupt, EOFError):
            return None, False

    return os.path.abspath(cleaned), True


def prompt_for_foam_file(current_default=None):
    """Interactively prompts the user for the .foam file path with examples and browse option."""
    ex_foam, _ = get_path_examples()

    while True:
        print("\n" + "-" * 78)
        print(" OpenFOAM Input Case File (.foam):")
        print(f"   For example: '{ex_foam}'")
        print("   Options:")
        print("     - Paste or type the full file path directly")
        print("     - Type 'b' to open File Explorer and browse")
        if current_default:
            print(f"     - Press [Enter] to keep current: '{current_default}'")
        print("-" * 78)

        try:
            prompt_label = "Case .foam Path [Enter to keep, 'b' to browse]: " if current_default else "Case .foam Path (or 'b' to browse): "
            user_input = safe_input(prompt_label).strip()
        except (KeyboardInterrupt, EOFError):
            print("\n[Aborted] Setup canceled by user.")
            sys.exit(0)

        # Press Enter to keep current
        if not user_input and current_default:
            res, ok = validate_and_resolve_foam_path(current_default)
            if ok:
                return res
            print("  Please provide an updated path.")
            continue

        # Browse option
        if user_input.lower() in ('b', 'browse'):
            print("  [Explorer] Opening file dialog... (select your .foam file)")
            chosen = browse_file_dialog(title="Select SimScale OpenFOAM case.foam File")
            if chosen:
                print(f"  [Selected via Explorer] '{chosen}'")
                res, ok = validate_and_resolve_foam_path(chosen)
                if ok:
                    return res
            else:
                print("  [Browse] No file selected or dialog canceled.")
            continue

        # Direct path entered
        if user_input.lower() in ('exit', 'quit', 'q'):
            print("\n[Aborted] Setup canceled by user.")
            sys.exit(0)

        res, ok = validate_and_resolve_foam_path(user_input)
        if ok:
            return res


def prompt_for_output_dir(current_default=None):
    """Interactively prompts the user for the output directory with examples, default, and browse option."""
    suggested = current_default if current_default else get_default_output_dir()
    _, ex_out = get_path_examples()

    while True:
        print("\n" + "-" * 78)
        print(" Visualization Output Directory:")
        print(f"   For example: '{ex_out}'")
        print("   Options:")
        print(f"     - Press [Enter] to use default: '{suggested}'")
        print("     - Paste or type a custom destination folder path")
        print("     - Type 'b' to open File Explorer and choose folder")
        print("-" * 78)

        try:
            user_input = safe_input("Output Directory [Enter for default, 'b' to browse]: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n[Aborted] Setup canceled by user.")
            sys.exit(0)

        # Press Enter to use suggested default
        if not user_input:
            out_dir = os.path.abspath(suggested)
            print(f"  [Selected Default] Using output directory: '{out_dir}'")
            return out_dir

        # Browse option
        if user_input.lower() in ('b', 'browse'):
            print("  [Explorer] Opening folder dialog... (select destination folder)")
            chosen = browse_directory_dialog(
                title="Select Output Directory for Visualizations",
                initial_dir=suggested
            )
            if chosen:
                print(f"  [Selected via Explorer] '{chosen}'")
                return os.path.abspath(chosen)
            else:
                print("  [Browse] No folder selected or dialog canceled.")
            continue

        # Direct path entered
        if user_input.lower() in ('exit', 'quit', 'q'):
            print("\n[Aborted] Setup canceled by user.")
            sys.exit(0)

        cleaned = clean_path_input(user_input)
        if cleaned:
            out_dir = os.path.abspath(cleaned)
            print(f"  [Selected] Output directory set to: '{out_dir}'")
            return out_dir
        else:
            print("  [Warning] Please enter a valid directory path or press [Enter] for default.")


def interactive_path_setup():
    """
    Coordinates interactive path setup:
      1. Checks CLI arguments
      2. If paths are saved: presents y/n confirmation prompt before running any visualizations
      3. If first-time startup or user selects 'n': prompts for paths and saves them
    """
    # 1. Check command-line arguments
    cli_foam = None
    if len(sys.argv) > 1 and sys.argv[1].endswith(('.foam', '.OpenFOAM')):
        cli_cand = os.path.abspath(sys.argv[1])
        if os.path.exists(cli_cand):
            cli_foam = cli_cand

    saved_foam, saved_output = load_saved_configuration()

    # If CLI argument was supplied
    if cli_foam:
        print("\n" + "=" * 78)
        print(" [CLI] Case file provided via command line:")
        print(f"       {cli_foam}")
        print("=" * 78)
        resolved_foam, ok = validate_and_resolve_foam_path(cli_foam)
        if resolved_foam and ok:
            out_dir = saved_output if saved_output else get_default_output_dir()
            if resolved_foam != saved_foam:
                try:
                    save_ans = safe_input("\nSave this case path as your new default for future runs? [y/N]: ").strip().lower()
                    if save_ans in ('y', 'yes'):
                        persist_configuration(resolved_foam, out_dir)
                except (KeyboardInterrupt, EOFError):
                    pass
            return resolved_foam, out_dir
        else:
            print("  [Notice] The CLI-provided case file could not be resolved. Falling back to interactive setup.")

    # 2. Subsequent Runs: prompt to confirm saved paths
    if saved_foam and saved_output:
        print("\n" + "=" * 78)
        print("             SIMSCALE OPENFOAM PARAVIEW AUTOMATION SETUP")
        print("=" * 78)
        print(" Current Saved Paths:")
        print(f"   - Input Case (.foam):  {saved_foam}")
        print(f"   - Output Directory:    {saved_output}")
        print("-" * 78)

        while True:
            try:
                confirm = safe_input("Confirm paths and proceed? [Y/n]: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                print("\n[Aborted] Canceled by user.")
                sys.exit(0)

            if confirm in ('', 'y', 'yes'):
                res_foam, ok = validate_and_resolve_foam_path(saved_foam)
                if ok:
                    return res_foam, saved_output
                else:
                    print("\n[Notice] Saved case path is no longer accessible. Please re-enter your paths.")
                    break
            elif confirm in ('n', 'no'):
                break
            else:
                print("Please enter 'y' to confirm or 'n' to update paths.")

        # Re-prompt for paths with current as defaults
        print("\nUpdating paths (press [Enter] on either prompt to keep the current value):")
        new_foam = prompt_for_foam_file(current_default=saved_foam)
        new_output = prompt_for_output_dir(current_default=saved_output)
        persist_configuration(new_foam, new_output)
        return new_foam, new_output

    # 3. First-time Startup: guided prompts
    print("\n" + "=" * 78)
    print("             FIRST-TIME SETUP: SIMSCALE PARAVIEW AUTOMATION")
    print("=" * 78)
    print(" Welcome to the FSAE SimScale ParaView Automation tool!")
    print(" This quick setup will configure your input and output directories.")
    print(" Your paths will be saved automatically so you won't need to re-enter them")
    print(" on future runs.")
    print("=" * 78)

    new_foam = prompt_for_foam_file()
    new_output = prompt_for_output_dir()
    persist_configuration(new_foam, new_output)
    print("\n[Setup Complete] Initial configuration saved successfully!\n")
    return new_foam, new_output


def pick_body_regions(all_regions):
    """Filter boundary mesh regions for the body (internalMesh and groups strictly excluded)."""
    candidates = [
        r for r in all_regions
        if 'internalmesh' not in r.lower()
        and not r.lower().startswith('group/')
    ]

    if not FACE_NUMBERS:
        return candidates

    def matches_face_numbers(region_name):
        basename = region_name.split('/')[-1].lower()
        for n in FACE_NUMBERS:
            pat = r'(?<!\d)%d$' % int(n)
            if re.search(pat, basename) or basename == f"face{n}" or basename == f"patch{n}":
                return True
        return False

    if FACE_FILTER_MODE.lower() == 'remove':
        return [r for r in candidates if not matches_face_numbers(r)]
    else:
        return [r for r in candidates if matches_face_numbers(r)]


def bounds_of(proxy, time):
    proxy.UpdatePipeline(time)
    return list(proxy.GetDataInformation().GetBounds())


def check_bounds(b, label):
    if b[0] > b[1] or b[2] > b[3] or b[4] > b[5]:
        raise RuntimeError(f"'{label}' is EMPTY (bounds {b}). Verify FACE_NUMBERS and FACE_FILTER_MODE.")


def diag_of(b):
    return math.sqrt((b[1]-b[0])**2 + (b[3]-b[2])**2 + (b[5]-b[4])**2)


def set_static_camera(view, b, preset_key):
    cx = 0.5*(b[0]+b[1]); cy = 0.5*(b[2]+b[3]); cz = 0.5*(b[4]+b[5])
    diag = diag_of(b)
    direction, up = CAMERA_PRESETS[preset_key]
    n = math.sqrt(sum(d*d for d in direction))
    view.CameraParallelProjection = 0
    view.CameraFocalPoint  = [cx, cy, cz]
    view.CameraPosition    = [cx + direction[0]/n * diag * CAMERA_DISTANCE_FACTOR,
                               cy + direction[1]/n * diag * CAMERA_DISTANCE_FACTOR,
                               cz + direction[2]/n * diag * CAMERA_DISTANCE_FACTOR]
    view.CameraViewUp = up


def set_bottom_camera(view):
    view.CameraParallelProjection = 0
    view.CameraPosition   = BOTTOM_CAM_POS
    view.CameraFocalPoint = BOTTOM_CAM_FOCAL
    view.CameraViewUp     = BOTTOM_CAM_UP


def shot(path, view):
    RenderAllViews()
    try:
        view.ResetCameraClippingRange()
    except Exception:
        pass
    SaveScreenshot(path, view, ImageResolution=RESOLUTION, TransparentBackground=0)
    print(f"  [Saved] {os.path.basename(path)}")


def compile_video_if_possible(frame_dir, output_mp4, fps=60):
    """Automatically invoke FFmpeg to assemble image sequence into MP4 if available."""
    ffmpeg_exe = shutil.which("ffmpeg")
    os.makedirs(os.path.dirname(output_mp4), exist_ok=True)

    if not ffmpeg_exe:
        print("\n[Notice] FFmpeg was not found on your system PATH.")
        print("To compile the video manually, open terminal and run:")
        print(f'  cd "{frame_dir}"')
        print(f'  ffmpeg -y -framerate {fps} -start_number 0 -i slice_%04d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -pix_fmt yuv420p -crf 18 "{output_mp4}"')
        return

    print(f"\n[FFmpeg] Compiling {fps} FPS video -> {output_mp4} ...")
    cmd = [
        ffmpeg_exe,
        "-y",
        "-framerate", str(fps),
        "-start_number", "0",
        "-i", "slice_%04d.png",
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-crf", "18",
        output_mp4
    ]
    try:
        res = subprocess.run(cmd, cwd=frame_dir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode == 0:
            print(f"[FFmpeg] Video compilation completed successfully: {output_mp4}")
        else:
            print(f"[FFmpeg] Non-zero return code ({res.returncode}):")
            print(res.stderr if res.stderr else "Unknown error")
            print("\nTo compile manually from the terminal, run:")
            print(f'  cd "{frame_dir}"')
            print(f'  ffmpeg -y -framerate {fps} -start_number 0 -i slice_%04d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -pix_fmt yuv420p -crf 18 "{output_mp4}"')
    except Exception as e:
        print(f"[FFmpeg] Execution error: {e}")


# =============================================================================
# 1.  PRE-RUN CONFIGURATION SUMMARY & USER SETUP
# =============================================================================
FOAM_FILE, BASE_OUTPUT_DIR = interactive_path_setup()

# Prompt for boundary face numbers to remove (after path setup, before videos)
FACE_NUMBERS = prompt_for_face_numbers(current_default=load_saved_face_numbers())

OUTPUT_DIR = resolve_unique_output_dir(BASE_OUTPUT_DIR)
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Subdirectories directly under paraview_output
WSS_OUTPUT_DIR   = os.path.join(OUTPUT_DIR, "wallShearStress")
P_OUTPUT_DIR     = os.path.join(OUTPUT_DIR, "pressure")
TOTP_OUTPUT_DIR  = os.path.join(OUTPUT_DIR, "totalPressure")
U_OUTPUT_DIR     = os.path.join(OUTPUT_DIR, "velocity")

Y_SLICE_U_DIR    = os.path.join(OUTPUT_DIR, "velocity_Y")
Y_SLICE_P_DIR    = os.path.join(OUTPUT_DIR, "pressure_Y")
Y_SLICE_TOTP_DIR = os.path.join(OUTPUT_DIR, "totalPressure_Y")

Z_SLICE_U_DIR    = os.path.join(OUTPUT_DIR, "velocity_Z")
Z_SLICE_P_DIR    = os.path.join(OUTPUT_DIR, "pressure_Z")
Z_SLICE_TOTP_DIR = os.path.join(OUTPUT_DIR, "totalPressure_Z")

# All slice_frames subdirectories are organized inside a parent 'slice_frames' folder
BASE_SLICE_FRAMES_DIR = os.path.join(OUTPUT_DIR, "slice_frames")
FRAME_DIR_P      = os.path.join(BASE_SLICE_FRAMES_DIR, "slice_frames_pressure")
FRAME_DIR_TOTP   = os.path.join(BASE_SLICE_FRAMES_DIR, "slice_frames_totalPressure")
FRAME_DIR_U      = os.path.join(BASE_SLICE_FRAMES_DIR, "slice_frames_velocity")

for d in [WSS_OUTPUT_DIR, P_OUTPUT_DIR, TOTP_OUTPUT_DIR, U_OUTPUT_DIR,
          Y_SLICE_U_DIR, Y_SLICE_P_DIR, Y_SLICE_TOTP_DIR,
          Z_SLICE_U_DIR, Z_SLICE_P_DIR, Z_SLICE_TOTP_DIR,
          BASE_SLICE_FRAMES_DIR]:
    os.makedirs(d, exist_ok=True)

case_file = FOAM_FILE

print("\n" + "=" * 78)
print("             SIMSCALE OPENFOAM PARAVIEW AUTOMATION SETUP")
print("=" * 78)
print(f" Case File:          {case_file}")
print(f" Output Directory:   {OUTPUT_DIR}")
print(f" Face Filter Mode:   {FACE_FILTER_MODE.upper()} (Faces: {FACE_NUMBERS})")
print(f" Coordinate Frame:   +X = Front (Nose) | -X = Rear | +Y = Exterior | +Z = Up")
print("-" * 78)
print(" Planned Visualizations:")
print(f"   1. Static Surface:    wallShearStress (Surface LIC + vectors),")
print(f"                         pressure (both clean 'p_surface' & LIC 'p_lic' + WSS vectors),")
print(f"                         totalPressure (clean), velocity (clean)")
print(f"                         (5 standard perspectives + bottom view each)")
print(f"   2. Spanwise Slices:   Y = {Y_SLICE_LOCATIONS} m (clean and _streamlines)")
print(f"                         - Velocity ({VELOCITY_RANGE} m/s) -> velocity_Y/")
print(f"                         - Static Pressure ({PRESSURE_RANGE} Pa) -> pressure_Y/")
print(f"                         - Total Pressure ({TOTAL_P_RANGE} Pa) -> totalPressure_Y/")
print(f"   3. Horizontal Slices: Z = {Z_SLICE_LOCATIONS} m (clean and _streamlines)")
print(f"                         - Velocity ({VELOCITY_RANGE} m/s) -> velocity_Z/")
print(f"                         - Static Pressure ({PRESSURE_RANGE} Pa) -> pressure_Z/")
print(f"                         - Total Pressure ({TOTAL_P_RANGE} Pa) -> totalPressure_Z/")
print("=" * 78)

# Upfront prompts for the three video sweeps
try:
    ans_p = safe_input("\nWould you like to generate the slice-sweep video for Static Pressure (p)? [y/N]: ").strip().lower()
    run_video_p = ans_p in ('y', 'yes')
except (EOFError, KeyboardInterrupt):
    run_video_p = False

try:
    ans_totp = safe_input("Would you like to generate the slice-sweep video for Total Pressure (total(p))? [y/N]: ").strip().lower()
    run_video_totp = ans_totp in ('y', 'yes')
except (EOFError, KeyboardInterrupt):
    run_video_totp = False

try:
    ans_u = safe_input("Would you like to generate the slice-sweep video for Velocity (U)? [y/N]: ").strip().lower()
    run_video_u = ans_u in ('y', 'yes')
except (EOFError, KeyboardInterrupt):
    run_video_u = False

print("\nVideo Generation Configuration:")
print(f"  - Static Pressure Video:      {'ENABLED' if run_video_p else 'SKIPPED'}")
print(f"  - Total Pressure Video:       {'ENABLED' if run_video_totp else 'SKIPPED'}")
print(f"  - Velocity Video:             {'ENABLED' if run_video_u else 'SKIPPED'}")
print("\n>>> TIP: Press Ctrl+C in this terminal at any time to abort execution safely. <<<\n")

# =============================================================================
# 2.  CASE INITIALIZATION & REGION SELECTION
# =============================================================================
print("Inspecting OpenFOAM case mesh regions...")
body_reader = OpenFOAMReader(registrationName='body_reader', FileName=case_file)
body_reader.UpdatePipelineInformation()

try:
    all_regions = list(body_reader.MeshRegions.Available)
except Exception:
    all_regions = ['internalMesh']

print(f"Total available mesh regions: {len(all_regions)}")
if len(all_regions) <= 12:
    for r in all_regions:
        print(f"  - {r}")
else:
    for r in all_regions[:5]:
        print(f"  - {r}")
    print(f"  ... [{len(all_regions) - 10} intermediate patch regions omitted] ...")
    for r in all_regions[-5:]:
        print(f"  - {r}")

body_regions = pick_body_regions(all_regions)
print(f"\nFiltered body patches: {len(body_regions)} patches selected for car surface (internalMesh excluded).")

if not body_regions:
    raise RuntimeError("No body regions matched. Check FACE_NUMBERS and FACE_FILTER_MODE.")

body_reader.MeshRegions = body_regions

try:
    body_reader.DecomposePolyhedra = 1
except Exception:
    pass

try:
    body_reader.CellArrays = list(body_reader.CellArrays.Available)
except Exception:
    pass
try:
    body_reader.PointArrays = list(body_reader.PointArrays.Available)
except Exception:
    pass

scene = GetAnimationScene()
scene.UpdateAnimationUsingDataTimeSteps()

if USE_LAST_TIMESTEP and len(body_reader.TimestepValues) > 0:
    scene.AnimationTime = body_reader.TimestepValues[-1]
    print(f"Using simulation timestep: {scene.AnimationTime}")
time = scene.AnimationTime
body_reader.UpdatePipeline(time)

# =============================================================================
# 3.  BODY PIPELINE (reader -> c2p -> MergeBlocks -> ExtractSurface)
# =============================================================================
body_c2p = CellDatatoPointData(registrationName='body_c2p', Input=body_reader)
body_c2p.ProcessAllArrays = 1
body_c2p.PassCellData     = 1
body_c2p.UpdatePipeline(time)

pt_arrays = [body_c2p.PointData.GetArray(i).Name
             for i in range(body_c2p.PointData.GetNumberOfArrays())]
print(f"Point data fields available on body: {pt_arrays}")

# Identify total pressure array if present
matched_tot_p = None
for cand in TOTAL_P_CANDIDATES:
    if cand in pt_arrays:
        matched_tot_p = cand
        break

body_merge = MergeBlocks(registrationName='MergeBlocks_Body', Input=body_c2p)
body_surf  = ExtractSurface(registrationName='ExtractSurface_Body', Input=body_merge)
body_b     = bounds_of(body_surf, time)
check_bounds(body_b, 'Body surface')
print(f"Body bounding box: X=[{body_b[0]:.3f}, {body_b[1]:.3f}], Y=[{body_b[2]:.3f}, {body_b[3]:.3f}], Z=[{body_b[4]:.3f}, {body_b[5]:.3f}]")

# =============================================================================
# 4.  VIEW SETUP
# =============================================================================
view = GetActiveViewOrCreate('RenderView')
view.ViewSize = RESOLUTION
view.OrientationAxesVisibility = 1
view.Background = [0.12, 0.14, 0.18]
view.UseColorPaletteForBackground = 0

try:
    LoadPlugin('SurfaceLIC', remote=False, ns=globals())
except Exception:
    pass

body_disp = Show(body_surf, view)

# Hide upstream readers/blocks so internalMesh is never visible
Hide(body_reader, view)
Hide(body_c2p,    view)
Hide(body_merge,  view)

# =============================================================================
# 5.  STATIC SCREENSHOTS: wallShearStress (Surface LIC + Vectors + Bottom View)
# =============================================================================
print("\n" + "=" * 78)
print(f" 1/4 EXPORTING STATIC SCREENSHOTS: {WSS_ARRAY} (Surface LIC + Vectors + Bottom View)")
print(f" Destination: {WSS_OUTPUT_DIR}")
print("=" * 78)

if WSS_ARRAY in pt_arrays:
    body_disp.SetRepresentationType('Surface LIC')
    ColorBy(body_disp, ('POINTS', WSS_ARRAY, 'Magnitude'))

    wss_lut = GetColorTransferFunction(WSS_ARRAY)
    wss_lut.ApplyPreset(COLOR_PRESET, True)
    wss_lut.AutomaticRescaleRangeMode = 'Never'
    try:
        wss_lut.LockDataRange = 1
    except Exception:
        pass
    wss_lut.RescaleTransferFunction(WSS_RANGE[0], WSS_RANGE[1])
    GetOpacityTransferFunction(WSS_ARRAY).RescaleTransferFunction(WSS_RANGE[0], WSS_RANGE[1])
    body_disp.SetScalarBarVisibility(view, True)

    wss_sb = GetScalarBar(wss_lut, view)
    wss_sb.Title           = f'{WSS_ARRAY} (Pa)'
    wss_sb.ComponentTitle  = 'Magnitude'
    wss_sb.Orientation     = 'Horizontal'
    wss_sb.WindowLocation  = 'Lower Center'
    wss_sb.ScalarBarLength = 0.45

    try:
        body_disp.SelectInputVectors = ['POINTS', WSS_ARRAY]
        for prop, val in [
            ('NumberOfSteps', LIC_NUM_STEPS),
            ('StepSize', LIC_STEP_SIZE),
            ('EnhancedLIC', 1),
            ('ColorMode', 'Blend'),
            ('LICIntensity', LIC_INTENSITY)
        ]:
            try:
                setattr(body_disp, prop, val)
            except Exception:
                pass
    except Exception:
        pass

    # 5 Standard perspectives
    for cam_key in sorted(CAMERA_PRESETS.keys()):
        set_static_camera(view, body_b, cam_key)
        path = os.path.join(WSS_OUTPUT_DIR, f"wss_lic_{cam_key}.png")
        shot(path, view)

    # Bottom view
    set_bottom_camera(view)
    shot(os.path.join(WSS_OUTPUT_DIR, "wss_lic_bottom.png"), view)

    body_disp.SetScalarBarVisibility(view, False)
    try:
        wss_sb.Visibility = 0
    except Exception:
        pass
else:
    print(f"  [Warning] '{WSS_ARRAY}' not found in point data. Skipping WSS views.")

# =============================================================================
# 6.  STATIC SCREENSHOTS: pressure, totalPressure, velocity (Clean Surface + Bottom)
# =============================================================================
print("\n" + "=" * 78)
print(" 2/4 EXPORTING STATIC SCREENSHOTS: Clean Surface (pressure, totalPressure, velocity)")
print("=" * 78)

surface_static_tasks = [
    (PRESSURE_ARRAY, PRESSURE_RANGE, 'pressure', f'{PRESSURE_ARRAY} (Pa)', P_OUTPUT_DIR, 'p_surface'),
]
if matched_tot_p:
    surface_static_tasks.append((matched_tot_p, TOTAL_P_RANGE, 'totalPressure', f'Total Pressure [{matched_tot_p}] (Pa)', TOTP_OUTPUT_DIR, 'totalp_surface'))
else:
    print("  [Notice] Total pressure field not found on body surface. Skipping total pressure static views.")

surface_static_tasks.append((VELOCITY_ARRAY, VELOCITY_RANGE, 'velocity', 'Velocity (m/s)', U_OUTPUT_DIR, 'u_surface'))

for field_name, f_range, desc, sb_title, out_dir, f_prefix in surface_static_tasks:
    if field_name not in pt_arrays:
        print(f"  [Warning] '{field_name}' not found on body surface. Skipping {desc} static views.")
        continue

    print(f"\n Exporting static surface views: {desc} -> {out_dir}")
    body_disp.SetRepresentationType('Surface')
    ColorBy(body_disp, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))

    lut = GetColorTransferFunction(field_name)
    lut.ApplyPreset(COLOR_PRESET, True)
    lut.AutomaticRescaleRangeMode = 'Never'
    try:
        lut.LockDataRange = 1
    except Exception:
        pass
    lut.RescaleTransferFunction(f_range[0], f_range[1])
    GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])
    body_disp.SetScalarBarVisibility(view, True)

    sb = GetScalarBar(lut, view)
    sb.Title           = sb_title
    sb.ComponentTitle  = ''
    sb.Orientation     = 'Horizontal'
    sb.WindowLocation  = 'Lower Center'
    sb.ScalarBarLength = 0.45

    # 5 Standard perspectives
    for cam_key in sorted(CAMERA_PRESETS.keys()):
        set_static_camera(view, body_b, cam_key)
        path = os.path.join(out_dir, f"{f_prefix}_{cam_key}.png")
        shot(path, view)

    # Bottom view
    set_bottom_camera(view)
    shot(os.path.join(out_dir, f"{f_prefix}_bottom.png"), view)

    # For pressure only: also generate Surface LIC with wallShearStress vectors (p_lic_*)
    if field_name == PRESSURE_ARRAY and WSS_ARRAY in pt_arrays:
        print(f"  -> Exporting Surface LIC + wallShearStress vectors for {desc} (p_lic_*)")
        body_disp.SetRepresentationType('Surface LIC')
        ColorBy(body_disp, ('POINTS', field_name))
        try:
            body_disp.SelectInputVectors = ['POINTS', WSS_ARRAY]
            for prop, val in [
                ('NumberOfSteps', LIC_NUM_STEPS),
                ('StepSize', LIC_STEP_SIZE),
                ('EnhancedLIC', 1),
                ('ColorMode', 'Blend'),
                ('LICIntensity', LIC_INTENSITY)
            ]:
                try:
                    setattr(body_disp, prop, val)
                except Exception:
                    pass
        except Exception:
            pass

        # Strictly enforce and lock the pressure color map to [-200.0, 80.0] Pa
        p_lut = GetColorTransferFunction(field_name)
        p_lut.ApplyPreset(COLOR_PRESET, True)
        p_lut.AutomaticRescaleRangeMode = 'Never'
        try:
            p_lut.LockDataRange = 1
        except Exception:
            pass
        p_lut.RescaleTransferFunction(f_range[0], f_range[1])
        GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])

        body_disp.SetScalarBarVisibility(view, True)
        p_lic_sb = GetScalarBar(p_lut, view)
        p_lic_sb.Title           = sb_title
        p_lic_sb.ComponentTitle  = ''
        p_lic_sb.Orientation     = 'Horizontal'
        p_lic_sb.WindowLocation  = 'Lower Center'
        p_lic_sb.ScalarBarLength = 0.45

        # 5 Standard perspectives with LIC
        for cam_key in sorted(CAMERA_PRESETS.keys()):
            set_static_camera(view, body_b, cam_key)
            path = os.path.join(out_dir, f"p_lic_{cam_key}.png")
            shot(path, view)

        # Bottom view with LIC
        set_bottom_camera(view)
        shot(os.path.join(out_dir, "p_lic_bottom.png"), view)

        body_disp.SetScalarBarVisibility(view, False)
        try:
            p_lic_sb.Visibility = 0
        except Exception:
            pass

    body_disp.SetScalarBarVisibility(view, False)
    try:
        sb.Visibility = 0
    except Exception:
        pass

# =============================================================================
# 7.  LOAD AIR VOLUME FOR SLICES & VIDEOS
# =============================================================================
internal_regions = [r for r in all_regions if 'internalmesh' in r.lower()]
if not internal_regions:
    print("  [Error] No internalMesh found in case. Cannot generate volume slices or videos.")
    air_merge = None
else:
    print("\nLoading internalMesh (air volume) for planar slices and videos...")
    air_reader = OpenFOAMReader(registrationName='air_reader', FileName=case_file)
    air_reader.UpdatePipelineInformation()
    try:
        air_reader.DecomposePolyhedra = 1
    except Exception:
        pass
    air_reader.MeshRegions = internal_regions
    try:
        air_reader.CellArrays = list(air_reader.CellArrays.Available)
    except Exception:
        pass
    try:
        air_reader.PointArrays = list(air_reader.PointArrays.Available)
    except Exception:
        pass
    air_reader.UpdatePipeline(time)
    Hide(air_reader, view)

    air_c2p = CellDatatoPointData(registrationName='air_c2p', Input=air_reader)
    air_c2p.ProcessAllArrays = 1
    air_c2p.PassCellData     = 1
    air_c2p.UpdatePipeline(time)
    Hide(air_c2p, view)

    air_merge = MergeBlocks(registrationName='MergeBlocks_Air', Input=air_c2p)
    air_merge.UpdatePipeline(time)
    Hide(air_merge, view)

    dom_b = bounds_of(air_merge, time)
    check_bounds(dom_b, 'internalMesh (air)')

    # Identify total pressure in air volume if not already matched
    air_arrays = [air_c2p.PointData.GetArray(i).Name for i in range(air_c2p.PointData.GetNumberOfArrays())]
    if not matched_tot_p:
        for cand in TOTAL_P_CANDIDATES:
            if cand in air_arrays:
                matched_tot_p = cand
                break

# Hide 3D body surface during planar slice rendering
Hide(body_surf, view)

# =============================================================================
# 8.  Y-NORMAL SPANWISE SLICES (with & without Surface LIC Streamlines)
# =============================================================================
if air_merge is not None:
    print("\n" + "=" * 78)
    print(f" 3/4 EXPORTING SPANWISE Y-NORMAL SLICES (y = {Y_SLICE_LOCATIONS} m)")
    print("     (Each slice exported clean and with _streamlines driven by velocity)")
    print("=" * 78)

    y_slc = Slice(registrationName='Slice_Y_Air', Input=air_merge)
    y_slc.SliceType        = 'Plane'
    y_slc.SliceType.Normal = [0.0, 1.0, 0.0]

    y_sdisp = Show(y_slc, view)

    y_slc_body = Slice(registrationName='Slice_Y_Body', Input=body_surf)
    y_slc_body.SliceType        = 'Plane'
    y_slc_body.SliceType.Normal = [0.0, 1.0, 0.0]

    y_odisp = Show(y_slc_body, view)
    ColorBy(y_odisp, None)
    y_odisp.AmbientColor = [0, 0, 0]
    y_odisp.DiffuseColor = [0, 0, 0]
    y_odisp.LineWidth    = 3.0

    # Side-on camera framing: camera at +Y looking toward -Y, view-up = +Z
    cx = 0.5 * (body_b[0] + body_b[1])
    cz = 0.5 * (body_b[4] + body_b[5])
    dx = (body_b[1] - body_b[0]) * 1.3
    dz = (body_b[5] - body_b[4]) * 1.5
    asp = float(RESOLUTION[0]) / RESOLUTION[1]
    y_par_scale = 0.5 * max(dz, dx / asp)
    cam_dist_y  = diag_of(dom_b)

    view.CameraParallelProjection = 1
    view.CameraParallelScale      = y_par_scale
    view.CameraViewUp             = [0, 0, 1]

    y_fields = [
        (VELOCITY_ARRAY, VELOCITY_RANGE, 'Velocity (m/s)', Y_SLICE_U_DIR, 'u_slice'),
        (PRESSURE_ARRAY, PRESSURE_RANGE, 'Static Pressure (Pa)', Y_SLICE_P_DIR, 'p_slice'),
    ]
    if matched_tot_p:
        y_fields.append((matched_tot_p, TOTAL_P_RANGE, f'Total Pressure [{matched_tot_p}] (Pa)', Y_SLICE_TOTP_DIR, 'totalp_slice'))

    for field_name, f_range, sb_title, out_dir, f_prefix in y_fields:
        if field_name not in air_arrays:
            continue

        print(f"\n Exporting Y-slices for {sb_title} -> {out_dir}:")
        lut = GetColorTransferFunction(field_name)
        lut.ApplyPreset(COLOR_PRESET, True)
        lut.AutomaticRescaleRangeMode = 'Never'
        try:
            lut.LockDataRange = 1
        except Exception:
            pass
        lut.RescaleTransferFunction(f_range[0], f_range[1])
        GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])

        sb = GetScalarBar(lut, view)
        sb.Title           = sb_title
        sb.ComponentTitle  = ''
        sb.Orientation     = 'Horizontal'
        sb.WindowLocation  = 'Lower Center'
        sb.ScalarBarLength = 0.45

        for y_pos in Y_SLICE_LOCATIONS:
            y_slc.SliceType.Origin      = [cx, y_pos, cz]
            y_slc_body.SliceType.Origin = [cx, y_pos, cz]

            view.CameraFocalPoint    = [cx, y_pos, cz]
            view.CameraPosition      = [cx, y_pos + cam_dist_y, cz]
            view.CameraViewUp        = [0, 0, 1]
            view.CameraParallelScale = y_par_scale

            # 1. Standard slice view (without streamlines)
            y_sdisp.SetRepresentationType('Surface')
            ColorBy(y_sdisp, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))
            y_sdisp.SetScalarBarVisibility(view, True)
            path_std = os.path.join(out_dir, f"{f_prefix}_y_{y_pos:.3f}.png")
            shot(path_std, view)

            # 2. Surface LIC slice view (with streamlines driven by velocity U)
            y_sdisp.SetRepresentationType('Surface LIC')
            ColorBy(y_sdisp, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))
            try:
                y_sdisp.SelectInputVectors = ['POINTS', 'U']
                for prop, val in [
                    ('NumberOfSteps', LIC_NUM_STEPS),
                    ('StepSize', LIC_STEP_SIZE),
                    ('EnhancedLIC', 1),
                    ('ColorMode', 'Blend'),
                    ('LICIntensity', LIC_INTENSITY)
                ]:
                    try:
                        setattr(y_sdisp, prop, val)
                    except Exception:
                        pass
            except Exception:
                pass

            # Re-lock color range after Surface LIC switch (ParaView resets it on representation change)
            lut.AutomaticRescaleRangeMode = 'Never'
            try:
                lut.LockDataRange = 1
            except Exception:
                pass
            lut.RescaleTransferFunction(f_range[0], f_range[1])
            GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])
            y_sdisp.SetScalarBarVisibility(view, True)

            path_lic = os.path.join(out_dir, f"{f_prefix}_y_{y_pos:.3f}_streamlines.png")
            shot(path_lic, view)

        y_sdisp.SetScalarBarVisibility(view, False)
        try:
            sb.Visibility = 0
        except Exception:
            pass

    Hide(y_slc, view)
    Hide(y_slc_body, view)

# =============================================================================
# 9.  Z-NORMAL HORIZONTAL SLICES (with & without Surface LIC Streamlines)
# =============================================================================
if air_merge is not None:
    print("\n" + "=" * 78)
    print(f" 4/4 EXPORTING HORIZONTAL Z-NORMAL SLICES (z = {Z_SLICE_LOCATIONS} m)")
    print("     (Viewed top-down: +X to right, +Y up; each with & without _streamlines)")
    print("=" * 78)

    z_slc = Slice(registrationName='Slice_Z_Air', Input=air_merge)
    z_slc.SliceType        = 'Plane'
    z_slc.SliceType.Normal = [0.0, 0.0, 1.0]

    z_sdisp = Show(z_slc, view)

    z_slc_body = Slice(registrationName='Slice_Z_Body', Input=body_surf)
    z_slc_body.SliceType        = 'Plane'
    z_slc_body.SliceType.Normal = [0.0, 0.0, 1.0]

    z_odisp = Show(z_slc_body, view)
    ColorBy(z_odisp, None)
    z_odisp.AmbientColor = [0, 0, 0]
    z_odisp.DiffuseColor = [0, 0, 0]
    z_odisp.LineWidth    = 3.0

    # Top-down camera framing: looking along -Z, +X points right, +Y points up (ViewUp = [0, 1, 0])
    cy = 0.5 * (body_b[2] + body_b[3])
    dx_z = (body_b[1] - body_b[0]) * 1.3
    dy_z = (body_b[3] - body_b[2]) * 1.5
    z_par_scale = 0.5 * max(dy_z, dx_z / asp)
    cam_dist_z  = diag_of(dom_b)

    view.CameraParallelProjection = 1
    view.CameraParallelScale      = z_par_scale
    view.CameraViewUp             = [0, 1, 0]

    z_fields = [
        (VELOCITY_ARRAY, VELOCITY_RANGE, 'Velocity (m/s)', Z_SLICE_U_DIR, 'u_slice'),
        (PRESSURE_ARRAY, PRESSURE_RANGE, 'Static Pressure (Pa)', Z_SLICE_P_DIR, 'p_slice'),
    ]
    if matched_tot_p:
        z_fields.append((matched_tot_p, TOTAL_P_RANGE, f'Total Pressure [{matched_tot_p}] (Pa)', Z_SLICE_TOTP_DIR, 'totalp_slice'))

    for field_name, f_range, sb_title, out_dir, f_prefix in z_fields:
        if field_name not in air_arrays:
            continue

        print(f"\n Exporting Z-slices for {sb_title} -> {out_dir}:")
        lut = GetColorTransferFunction(field_name)
        lut.ApplyPreset(COLOR_PRESET, True)
        lut.AutomaticRescaleRangeMode = 'Never'
        try:
            lut.LockDataRange = 1
        except Exception:
            pass
        lut.RescaleTransferFunction(f_range[0], f_range[1])
        GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])

        sb = GetScalarBar(lut, view)
        sb.Title           = sb_title
        sb.ComponentTitle  = ''
        sb.Orientation     = 'Horizontal'
        sb.WindowLocation  = 'Lower Center'
        sb.ScalarBarLength = 0.45

        for z_pos in Z_SLICE_LOCATIONS:
            z_slc.SliceType.Origin      = [cx, cy, z_pos]
            z_slc_body.SliceType.Origin = [cx, cy, z_pos]

            view.CameraFocalPoint    = [cx, cy, z_pos]
            view.CameraPosition      = [cx, cy, z_pos + cam_dist_z]
            view.CameraViewUp        = [0, 1, 0]
            view.CameraParallelScale = z_par_scale

            # 1. Standard slice view (without streamlines)
            z_sdisp.SetRepresentationType('Surface')
            ColorBy(z_sdisp, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))
            z_sdisp.SetScalarBarVisibility(view, True)
            path_std = os.path.join(out_dir, f"{f_prefix}_z_{z_pos:.3f}.png")
            shot(path_std, view)

            # 2. Surface LIC slice view (with streamlines driven by velocity U)
            z_sdisp.SetRepresentationType('Surface LIC')
            ColorBy(z_sdisp, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))
            try:
                z_sdisp.SelectInputVectors = ['POINTS', 'U']
                for prop, val in [
                    ('NumberOfSteps', LIC_NUM_STEPS),
                    ('StepSize', LIC_STEP_SIZE),
                    ('EnhancedLIC', 1),
                    ('ColorMode', 'Blend'),
                    ('LICIntensity', LIC_INTENSITY)
                ]:
                    try:
                        setattr(z_sdisp, prop, val)
                    except Exception:
                        pass
            except Exception:
                pass

            # Re-lock color range after Surface LIC switch (ParaView resets it on representation change)
            lut.AutomaticRescaleRangeMode = 'Never'
            try:
                lut.LockDataRange = 1
            except Exception:
                pass
            lut.RescaleTransferFunction(f_range[0], f_range[1])
            GetOpacityTransferFunction(field_name).RescaleTransferFunction(f_range[0], f_range[1])
            z_sdisp.SetScalarBarVisibility(view, True)

            path_lic = os.path.join(out_dir, f"{f_prefix}_z_{z_pos:.3f}_streamlines.png")
            shot(path_lic, view)

        z_sdisp.SetScalarBarVisibility(view, False)
        try:
            sb.Visibility = 0
        except Exception:
            pass

    Hide(z_slc, view)
    Hide(z_slc_body, view)

# =============================================================================
# 10. FRONT-TO-REAR VIDEO SWEEPS (Static Pressure, Total Pressure, and/or Velocity)
# =============================================================================
video_tasks = []
if run_video_p:
    video_tasks.append((PRESSURE_ARRAY, PRESSURE_RANGE, 'Static Pressure (Pa)',
                        FRAME_DIR_P, os.path.join(P_OUTPUT_DIR, "pressure_slice_sweep_60fps.mp4")))
if run_video_totp:
    if matched_tot_p:
        video_tasks.append((matched_tot_p, TOTAL_P_RANGE, f'Total Pressure [{matched_tot_p}] (Pa)',
                            FRAME_DIR_TOTP, os.path.join(TOTP_OUTPUT_DIR, "totalPressure_slice_sweep_60fps.mp4")))
    else:
        print("  [Notice] Total pressure field not found in case. Skipping Total Pressure video sweep.")
if run_video_u:
    video_tasks.append((VELOCITY_ARRAY, VELOCITY_RANGE, 'Velocity (m/s)',
                        FRAME_DIR_U, os.path.join(U_OUTPUT_DIR, "velocity_slice_sweep_60fps.mp4")))

if video_tasks and air_merge is not None:
    print("\n" + "=" * 78)
    print(f" GENERATING FRONT-TO-REAR SLICE VIDEOS ({len(video_tasks)} task(s), {N_FRAMES} frames each)")
    print(f" Direction: Front (+X = {body_b[1]:.2f}) -> Rear (-X = {body_b[0]:.2f})")
    print("=" * 78)

    slc_x = Slice(registrationName='Slice_X_Air', Input=air_merge)
    slc_x.SliceType        = 'Plane'
    slc_x.SliceType.Normal = [1.0, 0.0, 0.0]

    slc_x_body = None
    if SHOW_BODY_OUTLINE:
        slc_x_body             = Slice(registrationName='Slice_X_Body', Input=body_surf)
        slc_x_body.SliceType   = 'Plane'
        slc_x_body.SliceType.Normal = [1.0, 0.0, 0.0]
        odisp_x = Show(slc_x_body, view)
        ColorBy(odisp_x, None)
        odisp_x.AmbientColor = [0, 0, 0]
        odisp_x.DiffuseColor = [0, 0, 0]
        odisp_x.LineWidth    = 3.0

    span = body_b[1] - body_b[0]
    x_front = min(dom_b[1], body_b[1] + SWEEP_PAD * span)
    x_rear  = max(dom_b[0], body_b[0] - SWEEP_PAD * span)

    dy_f = (body_b[3] - body_b[2]) * FRAME_PAD
    dz_f = (body_b[5] - body_b[4]) * FRAME_PAD
    fy   = 0.5 * (body_b[2] + body_b[3])
    fz   = 0.5 * (body_b[4] + body_b[5])
    par_scale_x = 0.5 * max(dz_f, dy_f / asp)
    cam_dist_x  = diag_of(dom_b)

    sdisp_x = Show(slc_x, view)
    sdisp_x.SetRepresentationType('Surface')

    view.CameraParallelProjection  = 1
    view.CameraParallelScale       = par_scale_x
    view.CameraViewUp              = [0, 0, 1]
    view.OrientationAxesVisibility = 1

    for field_name, f_range, sb_title, frame_dir, mp4_path in video_tasks:
        os.makedirs(frame_dir, exist_ok=True)
        print(f"\n-> Rendering Video: {sb_title}")
        print(f"   Frames: {frame_dir}")
        print(f"   Output: {mp4_path}")

        ColorBy(sdisp_x, ('POINTS', field_name, 'Magnitude' if field_name == 'U' else ''))
        slut = GetColorTransferFunction(field_name)
        slut.ApplyPreset(COLOR_PRESET, True)
        slut.AutomaticRescaleRangeMode = 'Never'
        slut.RescaleTransferFunction(f_range[0], f_range[1])
        sdisp_x.SetScalarBarVisibility(view, True)

        ssb = GetScalarBar(slut, view)
        ssb.Title           = sb_title
        ssb.ComponentTitle  = ''
        ssb.Orientation     = 'Horizontal'
        ssb.WindowLocation  = 'Lower Center'
        ssb.ScalarBarLength = 0.45

        for i in range(N_FRAMES):
            t_frac = i / max(N_FRAMES - 1, 1)
            x_curr = x_front - (x_front - x_rear) * t_frac

            slc_x.SliceType.Origin = [x_curr, fy, fz]
            if slc_x_body:
                slc_x_body.SliceType.Origin = [x_curr, fy, fz]

            view.CameraFocalPoint    = [x_curr, fy, fz]
            view.CameraPosition      = [x_curr + cam_dist_x, fy, fz]
            view.CameraViewUp        = [0, 0, 1]
            view.CameraParallelScale = par_scale_x

            RenderAllViews()
            try:
                view.ResetCameraClippingRange()
            except Exception:
                pass

            SaveScreenshot(
                os.path.join(frame_dir, f"slice_{i:04d}.png"),
                view,
                ImageResolution=RESOLUTION,
                TransparentBackground=0,
            )
            if i % 100 == 0 or i == N_FRAMES - 1:
                print(f"  Frame {i + 1}/{N_FRAMES} (X = {x_curr:.3f} m)")

        sdisp_x.SetScalarBarVisibility(view, False)
        try:
            ssb.Visibility = 0
        except Exception:
            pass

        print(f"Frames saved to: {frame_dir}")
        compile_video_if_possible(frame_dir, mp4_path, fps=VIDEO_FPS)

    Hide(slc_x, view)
    if slc_x_body:
        Hide(slc_x_body, view)

# =============================================================================
# SUMMARY & COMPLETION
# =============================================================================
print("\n" + "=" * 78)
print("                       ALL AUTOMATION TASKS COMPLETED")
print("=" * 78)
print(f" Output Root Directory: {OUTPUT_DIR}")
print(f"  |-- wallShearStress/  : 5 static surface views + bottom view (Surface LIC + vectors)")
print(f"  |-- pressure/         : 5 static surface views + bottom view (clean 'p_surface' & LIC 'p_lic')")
if run_video_p:
    print(f"  |    \\-- pressure_slice_sweep_60fps.mp4")
if matched_tot_p:
    print(f"  |-- totalPressure/    : 5 static surface views + bottom view (clean surface)")
    if run_video_totp:
        print(f"  |    \\-- totalPressure_slice_sweep_60fps.mp4")
print(f"  |-- velocity/         : 5 static surface views + bottom view (clean surface)")
if run_video_u:
    print(f"  |    \\-- velocity_slice_sweep_60fps.mp4")
print(f"  |-- velocity_Y/       : Spanwise Y-slices (clean and _streamlines)")
print(f"  |-- pressure_Y/       : Spanwise Y-slices (clean and _streamlines)")
if matched_tot_p:
    print(f"  |-- totalPressure_Y/  : Spanwise Y-slices (clean and _streamlines)")
print(f"  |-- velocity_Z/       : Horizontal Z-slices (clean and _streamlines)")
print(f"  |-- pressure_Z/       : Horizontal Z-slices (clean and _streamlines)")
if matched_tot_p:
    print(f"  |-- totalPressure_Z/  : Horizontal Z-slices (clean and _streamlines)")
if run_video_p or (run_video_totp and matched_tot_p) or run_video_u:
    print(f"  \\-- slice_frames/")
    if run_video_p:
        print(f"       |-- slice_frames_pressure/      : {N_FRAMES} frames")
    if run_video_totp and matched_tot_p:
        print(f"       |-- slice_frames_totalPressure/ : {N_FRAMES} frames")
    if run_video_u:
        print(f"       \\-- slice_frames_velocity/      : {N_FRAMES} frames")
print("=" * 78 + "\n")
