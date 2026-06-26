import { SOLVERS, SOLVER_ORDER, SCENARIOS } from '../lib/solverMeta';

export function BenchmarkPanel({ results, scenario, onScenarioChange, loading, onRun, nTrials }) {
  const maxMean = results
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
          Run a batch to compare success rate, solve time, and self-collision behavior across all solvers on identical targets.
        </p>
      )}

      {results && (
        <div className="benchmark-rows">
          {/* column headers */}
          <div className="benchmark-header-row">
            <div />
            <div className="benchmark-col-labels">
              <span>success rate</span>
              <span>speed (mean / p95)</span>
              <span>clearance</span>
              <span>iters</span>
            </div>
          </div>

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
                  <TimeStat
                    mean={r.mean_time_ms}
                    p95={r.p95_time_ms}
                    maxMean={maxMean}
                  />
                  <BarStat
                    label="clearance"
                    value={r.mean_min_self_distance.toFixed(4)}
                    fraction={Math.min(Math.max((r.mean_min_self_distance + 0.05) / 0.1, 0), 1)}
                    color={r.mean_min_self_distance < 0 ? 'var(--alarm)' : 'var(--phosphor-dim)'}
                  />
                  <NumStat label="iters" value={r.mean_iters.toFixed(1)} />
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

function TimeStat({ mean, p95, maxMean }) {
  const meanPct = Math.round(Math.min(Math.max(mean / (maxMean || 1), 0), 1) * 100);
  const p95Pct  = Math.round(Math.min(Math.max(p95  / (maxMean || 1), 0), 1) * 100);
  return (
    <div className="bar-stat">
      <div className="bar-stat__track bar-stat__track--layered">
        {/* p95 bar (background, wider) */}
        <div className="bar-stat__fill bar-stat__fill--p95" style={{ width: `${p95Pct}%` }} />
        {/* mean bar (foreground, narrower) */}
        <div className="bar-stat__fill bar-stat__fill--mean" style={{ width: `${meanPct}%` }} />
      </div>
      <div className="bar-stat__meta">
        <span className="bar-stat__label">speed</span>
        <span className="bar-stat__value">
          {mean.toFixed(1)}ms
          <span className="bar-stat__p95" title="p95 latency"> / {p95.toFixed(1)}</span>
        </span>
      </div>
    </div>
  );
}

function NumStat({ label, value }) {
  return (
    <div className="bar-stat">
      <div className="bar-stat__num-display">{value}</div>
      <div className="bar-stat__meta">
        <span className="bar-stat__label">{label}</span>
      </div>
    </div>
  );
}
