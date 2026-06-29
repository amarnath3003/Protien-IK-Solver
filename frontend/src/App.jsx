import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ArmScene } from './components/ArmScene';
import { RobotArm } from './components/RobotArm';
import { TargetMarker } from './components/TargetMarker';
import { EnergyFunnel } from './components/EnergyFunnel';
import { BenchmarkPanel } from './components/BenchmarkPanel';
import { useLiveSolve } from './hooks/useLiveSolve';
import { getRandomTarget, runBenchmark, API_BASE } from './lib/api';
import { SOLVERS, SOLVER_ORDER, ROBOTS, ROBOT_ORDER, ROBOT_SOLVER_COMPAT, phaseLabel } from './lib/solverMeta';
import { ROBOT_SPECS } from './lib/kinematics';
import './styles/global.css';
import './styles/app.css';

function App() {
  const [target, setTarget] = useState(null);
  const [seed] = useState(() => Math.floor(Math.random() * 1e9));
  const [apiOk, setApiOk] = useState(null);
  const [focusedSolver, setFocusedSolver] = useState('protein_ik');
  const [robot, setRobot] = useState('ur5');

  const [scenario, setScenario] = useState('open_space');
  const [benchResults, setBenchResults] = useState(null);
  const [benchLoading, setBenchLoading] = useState(false);
  const nTrials = 60;

  // Ordered solver list for the current robot. Uses compat list as the source of truth
  // so analytical_planar3dof appears for planar3dof but not for other robots.
  const activeSolverOrder = useMemo(() => {
    const compat = ROBOT_SOLVER_COMPAT[robot];
    if (!compat) return SOLVER_ORDER;
    // Preserve SOLVER_ORDER ordering for common solvers, append any extras (e.g. analytical)
    const ordered = SOLVER_ORDER.filter((id) => compat.includes(id));
    const extras = compat.filter((id) => !SOLVER_ORDER.includes(id));
    return [...ordered, ...extras];
  }, [robot]);

  // One live-solve hook per solver — hooks are always mounted (can't be conditional)
  const dls            = useLiveSolve('jacobian_dls');
  const ccd            = useLiveSolve('ccd');
  const fabrik         = useLiveSolve('fabrik');
  const trac           = useLiveSolve('trac_ik_style');
  const multiStart     = useLiveSolve('multi_start');
  const protein        = useLiveSolve('protein_ik');
  const proteinFast    = useLiveSolve('protein_fast');
  const fixedLambda    = useLiveSolve('fixed_lambda_ik');
  const proteinHomotopy = useLiveSolve('protein_homotopy');
  const analytical     = useLiveSolve('analytical_planar3dof');

  const solveHooks = useMemo(() => ({
    jacobian_dls: dls, ccd, fabrik, trac_ik_style: trac, multi_start: multiStart,
    protein_ik: protein, protein_fast: proteinFast, fixed_lambda_ik: fixedLambda,
    protein_homotopy: proteinHomotopy, analytical_planar3dof: analytical,
  }), [dls, ccd, fabrik, trac, multiStart, protein, proteinFast, fixedLambda, proteinHomotopy, analytical]);

  const focused = solveHooks[focusedSolver] ?? dls;

  const fetchNewTarget = useCallback(async (newSeed, currentRobot) => {
    try {
      const data = await getRandomTarget(newSeed, currentRobot);
      setApiOk(true);
      setTarget({ position: data.position, quaternion: data.quaternion });
      return data;
    } catch {
      setApiOk(false);
      return null;
    }
  }, []);

  const solveAll = useCallback(async (currentRobot) => {
    const r = currentRobot ?? robot;
    const data = await fetchNewTarget(Math.floor(Math.random() * 1e9), r);
    if (!data) return;
    const t = { position: data.position, quaternion: data.quaternion };
    const compat = ROBOT_SOLVER_COMPAT[r];
    const base = compat ? SOLVER_ORDER.filter((id) => compat.includes(id)) : SOLVER_ORDER;
    const extras = compat ? compat.filter((id) => !SOLVER_ORDER.includes(id)) : [];
    const order = [...base, ...extras];
    order.forEach((id, i) => {
      setTimeout(() => {
        solveHooks[id].run({ target: t, seed: 2000 + i, stepDelayMs: 30, robot: r });
      }, i * 80);
    });
  }, [fetchNewTarget, solveHooks, robot]);

  useEffect(() => {
    // On load: fetch a target and kick off all solvers for the default robot
    (async () => {
      const data = await fetchNewTarget(seed, 'ur5');
      if (!data) return;
      const t = { position: data.position, quaternion: data.quaternion };
      SOLVER_ORDER.filter((id) => id !== 'analytical_planar3dof').forEach((id, i) => {
        setTimeout(() => {
          solveHooks[id].run({ target: t, seed: 2000 + i, stepDelayMs: 30, robot: 'ur5' });
        }, i * 80);
      });
    })();
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const handleRobotChange = useCallback((newRobot) => {
    setRobot(newRobot);
    setBenchResults(null);
    const compat = ROBOT_SOLVER_COMPAT[newRobot];
    if (compat && !compat.includes(focusedSolver)) {
      setFocusedSolver('protein_ik');
    }
    SOLVER_ORDER.forEach((id) => solveHooks[id].reset(newRobot));
    solveAll(newRobot);
  }, [focusedSolver, solveHooks, solveAll]);

  const runBench = useCallback(async () => {
    setBenchLoading(true);
    const compat = ROBOT_SOLVER_COMPAT[robot];
    const base = compat ? SOLVER_ORDER.filter((id) => compat.includes(id)) : SOLVER_ORDER;
    const extras = compat ? compat.filter((id) => !SOLVER_ORDER.includes(id)) : [];
    const solvers = [...base, ...extras];
    try {
      const data = await runBenchmark({ solvers, nTrials, scenario, seed: 1, robot });
      setBenchResults(data.results);
      setApiOk(true);
    } catch {
      setApiOk(false);
    } finally {
      setBenchLoading(false);
    }
  }, [scenario, robot]);

  const robotSpec = ROBOT_SPECS[robot] ?? ROBOT_SPECS.ur5;

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

        {/* ── Robot selector ────────────────────────────────────────────────── */}
        <section className="robot-selector">
          <span className="robot-selector__label">Robot arm</span>
          <div className="robot-tabs" role="tablist" aria-label="Robot arm selection">
            {ROBOT_ORDER.map((id) => (
              <button
                key={id}
                role="tab"
                aria-selected={robot === id}
                className={`robot-tab ${robot === id ? 'is-active' : ''}`}
                onClick={() => handleRobotChange(id)}
                title={ROBOTS[id].description}
              >
                {ROBOTS[id].name}
                <span className="robot-tab__dof">{ROBOTS[id].dof}-DOF</span>
              </button>
            ))}
          </div>
        </section>

        <section className="solve-controls">
          <button className="run-button run-button--primary" onClick={() => solveAll()}>
            solve a new target — all solvers
          </button>
          <span className="solve-controls__hint">
            Generates one random reachable pose and streams every solver's attempt live, side by side.
          </span>
        </section>

        <section className="focus-panel">
          <div className="focus-panel__viewport">
            {/* key={robot} forces clean remount when DOF count changes */}
            <ArmScene key={robot}>
              {target && (
                <>
                  <RobotArm q={focused.q} spec={robotSpec} accentColor={SOLVERS[focusedSolver]?.color ?? '#6FFFB0'} />
                  <TargetMarker position={target.position} quaternion={target.quaternion} />
                </>
              )}
            </ArmScene>
          </div>
          <div className="focus-panel__readout">
            <div className="focus-panel__tabs">
              {activeSolverOrder.map((id) => (
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

            <h3 className="focus-panel__name">{SOLVERS[focusedSolver]?.name}</h3>
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
            {activeSolverOrder.map((id) => {
              const h = solveHooks[id];
              const convergenceClass = h.status === 'done'
                ? (h.result?.success ? 'solver-tile--converged' : 'solver-tile--failed')
                : '';
              return (
                <button
                  key={id}
                  className={`solver-tile ${focusedSolver === id ? 'is-focused' : ''} ${convergenceClass}`}
                  onClick={() => setFocusedSolver(id)}
                  aria-label={`Focus ${SOLVERS[id].name}`}
                >
                  <div className="solver-tile__viewport">
                    <ArmScene compact key={robot}>
                      <RobotArm q={h.q} spec={robotSpec} accentColor={SOLVERS[id].color} scale={2.4} />
                      {target && <TargetMarker position={target.position} quaternion={target.quaternion} scale={2.4} />}
                    </ArmScene>
                  </div>
                  <div className="solver-tile__label">
                    <span className="solver-card__dot" style={{ background: SOLVERS[id].color }} />
                    {SOLVERS[id].short}
                    {h.status === 'done' && h.result != null ? (
                      <span className={`solver-tile__outcome ${h.result.success ? 'solver-tile__outcome--ok' : 'solver-tile__outcome--fail'}`}>
                        {h.result.success ? '✓' : '✗'}
                      </span>
                    ) : (
                      <span className={`solver-tile__status solver-tile__status--${h.status}`}>{h.status}</span>
                    )}
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
          robot={robot}
          onRobotChange={handleRobotChange}
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
