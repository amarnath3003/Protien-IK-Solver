import { useCallback, useEffect, useMemo, useState } from 'react';
import { ArmScene } from './components/ArmScene';
import { RobotArm } from './components/RobotArm';
import { TargetMarker } from './components/TargetMarker';
import { EnergyFunnel } from './components/EnergyFunnel';
import { BenchmarkPanel } from './components/BenchmarkPanel';
import { useLiveSolve } from './hooks/useLiveSolve';
import { getRandomTarget, runBenchmark, API_BASE } from './lib/api';
import { SOLVERS, SOLVER_ORDER, phaseLabel } from './lib/solverMeta';
import './styles/global.css';
import './styles/app.css';

function App() {
  const [target, setTarget] = useState(null);
  const [seed] = useState(1);
  const [apiOk, setApiOk] = useState(null);
  const [focusedSolver, setFocusedSolver] = useState('protein_ik');

  const [scenario, setScenario] = useState('open_space');
  const [benchResults, setBenchResults] = useState(null);
  const [benchLoading, setBenchLoading] = useState(false);
  const nTrials = 60;

  // one live-solve hook instance per solver, so all six can run/animate
  // independently and simultaneously in the grid
  const dls = useLiveSolve('jacobian_dls');
  const ccd = useLiveSolve('ccd');
  const fabrik = useLiveSolve('fabrik');
  const trac = useLiveSolve('trac_ik_style');
  const multiStart = useLiveSolve('multi_start');
  const protein = useLiveSolve('protein_ik');

  const solveHooks = useMemo(() => ({
    jacobian_dls: dls, ccd, fabrik, trac_ik_style: trac, multi_start: multiStart, protein_ik: protein,
  }), [dls, ccd, fabrik, trac, multiStart, protein]);

  const focused = solveHooks[focusedSolver];

  const fetchNewTarget = useCallback(async (newSeed) => {
    try {
      const data = await getRandomTarget(newSeed);
      setApiOk(true);
      setTarget({ position: data.position, quaternion: data.quaternion });
      return data;
    } catch {
      setApiOk(false);
      return null;
    }
  }, []);

  useEffect(() => {
    // eslint-disable-next-line react-hooks/set-state-in-effect
    fetchNewTarget(seed);
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const solveAll = useCallback(async () => {
    const data = await fetchNewTarget(Math.floor(Math.random() * 1e9));
    if (!data) return;
    const t = { position: data.position, quaternion: data.quaternion };
    SOLVER_ORDER.forEach((id, i) => {
      solveHooks[id].run({ target: t, seed: 2000 + i, stepDelayMs: 30 });
    });
  }, [fetchNewTarget, solveHooks]);

  const runBench = useCallback(async () => {
    setBenchLoading(true);
    try {
      const data = await runBenchmark({ solvers: SOLVER_ORDER, nTrials, scenario, seed: 1 });
      setBenchResults(data.results);
      setApiOk(true);
    } catch {
      setApiOk(false);
    } finally {
      setBenchLoading(false);
    }
  }, [scenario]);

  return (
    <div className="app">
      <header className="app-header">
        <div className="app-header__title">
          <span className="app-header__eyebrow">folding-inspired inverse kinematics</span>
          <h1>ProteinIK <span className="app-header__lab">/ lab bench</span></h1>
        </div>
        <div className="app-header__status">
          <span className={`api-dot ${apiOk === true ? 'ok' : apiOk === false ? 'bad' : ''}`} />
          <span>{apiOk === false ? `cannot reach ${API_BASE}` : apiOk === null ? 'connecting…' : 'connected'}</span>
        </div>
      </header>

      {apiOk === false && (
        <div className="api-warning">
          Can't reach the backend at <code>{API_BASE}</code>. Start it with{' '}
          <code>uvicorn app.main:app --reload --port 8000</code> from the backend directory, then reload this page.
        </div>
      )}

      <main className="app-main">
        <section className="solve-controls">
          <button className="run-button run-button--primary" onClick={solveAll}>
            solve a new target — all six solvers
          </button>
          <span className="solve-controls__hint">
            Generates one random reachable pose and streams every solver's attempt live, side by side.
          </span>
        </section>

        <section className="focus-panel">
          <div className="focus-panel__viewport">
            <ArmScene>
              {target && (
                <>
                  <RobotArm q={focused.q} accentColor={SOLVERS[focusedSolver].color} />
                  <TargetMarker position={target.position} quaternion={target.quaternion} />
                </>
              )}
            </ArmScene>
          </div>
          <div className="focus-panel__readout">
            <div className="focus-panel__tabs">
              {SOLVER_ORDER.map((id) => (
                <button
                  key={id}
                  className={`focus-tab ${focusedSolver === id ? 'is-active' : ''}`}
                  style={focusedSolver === id ? { borderColor: SOLVERS[id].color, color: SOLVERS[id].color } : undefined}
                  onClick={() => setFocusedSolver(id)}
                >
                  {SOLVERS[id].short}
                </button>
              ))}
            </div>

            <h3 className="focus-panel__name">{SOLVERS[focusedSolver].name}</h3>
            <div className="focus-panel__phase">{focused.phase ? phaseLabel(focused.phase) : focused.status === 'idle' ? 'awaiting target' : '—'}</div>

            <EnergyFunnel
              posError={focused.step?.pos_error}
              orientError={focused.step?.orient_error}
              phase={focused.phase}
              status={focused.status}
            />

            <dl className="focus-panel__stats">
              <Stat label="status" value={focused.status} />
              <Stat label="iteration" value={focused.step?.iteration ?? '—'} />
              <Stat label="pos error" value={focused.step ? focused.step.pos_error.toFixed(4) : '—'} />
              <Stat label="orient error" value={focused.step ? focused.step.orient_error.toFixed(4) : '—'} />
              <Stat label="self-clearance" value={focused.step ? focused.step.min_self_distance.toFixed(4) : '—'} />
              {focused.result && (
                <Stat
                  label="result"
                  value={focused.result.success ? 'converged' : 'did not converge'}
                  tone={focused.result.success ? 'good' : 'bad'}
                />
              )}
            </dl>
          </div>
        </section>

        <section className="grid-section">
          <h2 className="panel-title">All solvers, same target</h2>
          <div className="solver-grid">
            {SOLVER_ORDER.map((id) => {
              const h = solveHooks[id];
              return (
                <button
                  key={id}
                  className={`solver-tile ${focusedSolver === id ? 'is-focused' : ''}`}
                  onClick={() => setFocusedSolver(id)}
                  aria-label={`Focus ${SOLVERS[id].name}`}
                >
                  <div className="solver-tile__viewport">
                    <ArmScene compact>
                      <RobotArm q={h.q} accentColor={SOLVERS[id].color} scale={2.4} />
                      {target && <TargetMarker position={target.position} quaternion={target.quaternion} scale={2.4} />}
                    </ArmScene>
                  </div>
                  <div className="solver-tile__label">
                    <span className="solver-card__dot" style={{ background: SOLVERS[id].color }} />
                    {SOLVERS[id].short}
                    <span className={`solver-tile__status solver-tile__status--${h.status}`}>{h.status}</span>
                  </div>
                </button>
              );
            })}
          </div>
        </section>

        <BenchmarkPanel
          results={benchResults}
          scenario={scenario}
          onScenarioChange={setScenario}
          loading={benchLoading}
          onRun={runBench}
          nTrials={nTrials}
        />

        <footer className="app-footer">
          <p>
            ProteinIK is a staged, energy-based solver inspired by protein folding (target-blind local relaxation,
            coarse collapse, funnel narrowing, chaperone-style rescue). Across tested scenarios it outperforms
            classical baselines (Jacobian DLS, CCD, FABRIK) but does not beat the production-style baselines
            (TRAC-IK-style restart, Multi-start) on success rate or speed — these results are shown as measured,
            not adjusted to flatter ProteinIK.
          </p>
        </footer>
      </main>
    </div>
  );
}

function Stat({ label, value, tone }) {
  return (
    <div className={`stat stat--${tone || 'neutral'}`}>
      <dt>{label}</dt>
      <dd>{value}</dd>
    </div>
  );
}

export default App;
