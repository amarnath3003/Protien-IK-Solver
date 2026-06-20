import { SOLVERS, SOLVER_ORDER, SCENARIOS } from '../lib/solverMeta';

export function BenchmarkPanel({ results, scenario, onScenarioChange, loading, onRun, nTrials }) {
  const maxTime = results
    ? Math.max(...Object.values(results).map((r) => r.mean_time_ms))
    : 1;

  return (
    <section className="benchmark-panel">
      <div className="benchmark-panel__header">
        <h2 className="panel-title">Batch benchmark</h2>
        <div className="benchmark-panel__controls">
          <div className="scenario-tabs" role="tablist" aria-label="Benchmark scenario">
            {Object.entries(SCENARIOS).map(([id, meta]) => (
              <button
                key={id}
                role="tab"
                aria-selected={scenario === id}
                className={`scenario-tab ${scenario === id ? 'is-active' : ''}`}
                onClick={() => onScenarioChange(id)}
                title={meta.description}
              >
                {meta.name}
              </button>
            ))}
          </div>
          <button className="run-button" onClick={onRun} disabled={loading}>
            {loading ? `running ${nTrials} trials…` : `run ${nTrials} trials`}
          </button>
        </div>
      </div>

      {!results && !loading && (
        <p className="benchmark-panel__empty">
          Run a batch to compare success rate, solve time, and self-collision behavior across all six solvers on identical targets.
        </p>
      )}

      {results && (
        <div className="benchmark-rows">
          {SOLVER_ORDER.map((id) => {
            const r = results[id];
            if (!r) return null;
            const meta = SOLVERS[id];
            return (
              <div key={id} className="benchmark-row">
                <div className="benchmark-row__label">
                  <span className="solver-card__dot" style={{ background: meta.color }} />
                  {meta.name}
                </div>
                <div className="benchmark-row__bars">
                  <BarStat
                    label="success"
                    value={`${(r.success_rate * 100).toFixed(1)}%`}
                    fraction={r.success_rate}
                    color={meta.color}
                  />
                  <BarStat
                    label="speed"
                    value={`${r.mean_time_ms.toFixed(1)}ms`}
                    fraction={1 - r.mean_time_ms / maxTime}
                    color="var(--steel)"
                  />
                  <BarStat
                    label="clearance"
                    value={r.mean_min_self_distance.toFixed(4)}
                    fraction={Math.min(Math.max((r.mean_min_self_distance + 0.05) / 0.1, 0), 1)}
                    color={r.mean_min_self_distance < 0 ? 'var(--alarm)' : 'var(--phosphor-dim)'}
                  />
                </div>
              </div>
            );
          })}
        </div>
      )}
    </section>
  );
}

function BarStat({ label, value, fraction, color }) {
  const pct = Math.round(Math.min(Math.max(fraction, 0), 1) * 100);
  return (
    <div className="bar-stat">
      <div className="bar-stat__track">
        <div className="bar-stat__fill" style={{ width: `${pct}%`, background: color }} />
      </div>
      <div className="bar-stat__meta">
        <span className="bar-stat__label">{label}</span>
        <span className="bar-stat__value">{value}</span>
      </div>
    </div>
  );
}
