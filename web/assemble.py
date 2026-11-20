"""Assemble the public Pages artifact from the web app and shared solver."""

from pathlib import Path
from shutil import copyfile


ROOT = Path(__file__).resolve().parent.parent
SITE = ROOT / "dist"
WEB_FILES = (
    "index.html", "method.html", "method.css", "styles.css", "suspension-icon.svg", "app.mjs",
    "visuals.mjs", "downloads.mjs", "worker.js", "lab_backend.py",
)
PACKAGE_FILES = (
    "__init__.py", "model.py", "project.py", "results.py", "solver.py",
    "suspension.py", "wheel_frame.py", "examples/hardpoints.csv",
    "examples/reference_settings.json",
)


def assemble():
    for source_dir, target_dir, files in (
        (ROOT / "web", SITE, WEB_FILES),
        (ROOT / "JacobianKinematicsLab", SITE / "JacobianKinematicsLab", PACKAGE_FILES),
        (ROOT, SITE, ("LICENSE",)),
    ):
        for name in files:
            target = target_dir / name
            target.parent.mkdir(parents=True, exist_ok=True)
            copyfile(source_dir / name, target)
    (SITE / ".nojekyll").touch()


if __name__ == "__main__":
    assemble()
