# Suspension-Kinematics

[SuspensionKinematicsLab](https://dannyhsuc10.github.io/Suspension-Kinematics/) is the English
web version of the [desktop JacobianKinematicsLab](JacobianKinematicsLab/README.md). It runs
the same solver in a browser worker. Import hardpoints or a project, set wheel and rack motion,
and inspect the 3D mechanism, kinematic histories, motion ratios, and suspension sweeps.
The header's **Import JSON** and **Download JSON** support shared suspension hardpoints from
Suspension Geometry, Suspension Statics, and Multibody Dynamics. Kinematics JSON also stores
the original motion and mechanism settings. Use **Import CSV / mechanism** for CSV files and
**More project actions** for CSV project downloads. Solved histories download as CSV.
The [Jacobian method guide](https://dannyhsuc10.github.io/Suspension-Kinematics/method.html)
describes the coordinate conventions and equations.

## Preview and deploy

From the repository root, assemble the public files and serve them over HTTP.

```text
python web/assemble.py
python -m http.server 8000 --directory dist
```

Open http://localhost:8000/. The solver loads Pyodide and NumPy from jsDelivr.
GitHub Actions assembles the same files and deploys them to Pages when changes reach `main`.
