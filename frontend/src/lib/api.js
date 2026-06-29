// API client for the ProteinIK backend (FastAPI, default localhost:8000).
// All solving happens server-side; this module is a thin, typed-ish
// wrapper around fetch/WebSocket.

const API_BASE = import.meta.env.VITE_API_BASE || 'http://localhost:8000';
const WS_BASE = API_BASE.replace(/^http/, 'ws');

async function request(path, options = {}) {
  const res = await fetch(`${API_BASE}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    const detail = typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail || res.statusText);
    throw new Error(detail);
  }
  return res.json();
}

export function getRobots() {
  return request('/api/robots');
}

export async function getRobotSpec(robot) {
  const url = robot ? `/api/robot?robot=${encodeURIComponent(robot)}` : '/api/robot';
  const data = await request(url);
  // Normalize snake_case API fields to camelCase for the frontend kinematics lib
  return {
    name: data.name,
    a: data.a,
    d: data.d,
    alpha: data.alpha,
    linkRadius: data.link_radius,
    joint_limits: data.joint_limits,
  };
}

export function getSolvers() {
  return request('/api/solvers');
}

export function getRandomTarget(seed, robot = 'ur5') {
  return request('/api/random-target', {
    method: 'POST',
    body: JSON.stringify({ seed: seed ?? null, robot }),
  });
}

export function solveOnce({ solver, robot, q0, target, seed, collectSteps = true }) {
  return request('/api/solve', {
    method: 'POST',
    body: JSON.stringify({
      solver, robot, q0: q0 ?? null, target, seed: seed ?? null, collect_steps: collectSteps,
    }),
  });
}

export function runBenchmark({ solvers, nTrials = 100, seed = 1, scenario = 'open_space', robot = 'ur5' }) {
  return request('/api/benchmark', {
    method: 'POST',
    body: JSON.stringify({ solvers, n_trials: nTrials, seed, scenario, robot }),
  });
}

/**
 * Opens a WebSocket connection for live step-by-step streaming of a
 * single solve. Returns a controller with `.send(msg)` and `.close()`;
 * callbacks fire as messages arrive.
 */
export function connectSolveStream({ onStart, onStep, onDone, onError, onClose }) {
  const ws = new WebSocket(`${WS_BASE}/ws/solve`);

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'start') onStart?.(msg);
    else if (msg.type === 'step') onStep?.(msg.data);
    else if (msg.type === 'done') onDone?.(msg.data);
    else if (msg.type === 'error') onError?.(msg.message);
  };
  ws.onerror = () => onError?.('WebSocket connection error');
  ws.onclose = () => onClose?.();

  return {
    ready: () => new Promise((resolve, reject) => {
      if (ws.readyState === WebSocket.OPEN) return resolve();
      ws.addEventListener('open', () => resolve(), { once: true });
      ws.addEventListener('error', () => reject(new Error('WebSocket failed to open')), { once: true });
    }),
    send: (msg) => ws.send(JSON.stringify(msg)),
    close: () => ws.close(),
  };
}

export { API_BASE };
