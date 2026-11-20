const runtime = 'https://cdn.jsdelivr.net/pyodide/v0.27.7/full/';
const packageName = 'JacobianKinematicsLab';
const sources = ['__init__.py', 'model.py', 'project.py', 'results.py', 'solver.py', 'suspension.py',
  'wheel_frame.py', 'examples/hardpoints.csv', 'examples/reference_settings.json'];

async function read(path) {
  const response = await fetch(path);
  if (!response.ok) throw new Error(`Could not load ${path} (${response.status}).`);
  return new Uint8Array(await response.arrayBuffer());
}

async function initialize() {
  importScripts(`${runtime}pyodide.js`);
  const pyodide = await loadPyodide({ indexURL: runtime });
  const [files, bridge] = await Promise.all([
    Promise.all(sources.map(path => read(`./${packageName}/${path}`))),
    read('./lab_backend.py'),
    pyodide.loadPackage('numpy')
  ]);
  const root = `/home/pyodide/${packageName}`;
  pyodide.FS.mkdirTree(`${root}/examples`);
  sources.forEach((path, index) => pyodide.FS.writeFile(`${root}/${path}`, files[index]));
  pyodide.runPython(new TextDecoder().decode(bridge));
  return pyodide.globals.get('dispatch');
}

let ready;
self.onmessage = async ({ data: { id, operation, payload } }) => {
  try {
    ready ??= initialize();
    const dispatch = await ready;
    const result = JSON.parse(dispatch(operation, JSON.stringify(payload)));
    self.postMessage({ id, result });
  } catch (error) {
    const lines = String(error.message || error).trim().split('\n');
    const message = lines.at(-1).replace(/^(?:\w+\.)*\w*(?:Error|Unreachable):\s*/, '');
    self.postMessage({ id, error: message });
  }
};
