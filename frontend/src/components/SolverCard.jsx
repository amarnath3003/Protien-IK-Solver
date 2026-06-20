import { RobotArm } from './RobotArm';
import { TargetMarker } from './TargetMarker';
import { ArmScene } from './ArmScene';
import { SOLVERS, phaseLabel } from '../lib/solverMeta';

const STATUS_LABEL = {
  idle: 'idle',
  connecting: 'connecting…',
  running: 'solving',
  done: 'done',
  error: 'error',
};

export function SolverCard({ solverId, q, status, phase, step, result, error, target }) {
  const meta = SOLVERS[solverId];
  const success = result?.success;

  return (
    <div className={`solver-card status-${status}`}>
      <div className="solver-card__header">
        <span className="solver-card__dot" style={{ background: meta.color }} />
        <span className="solver-card__name">{meta.name}</span>
        <span className={`solver-card__status solver-card__status--${status}`}>
          {STATUS_LABEL[status]}
        </span>
      </div>

      <div className="solver-card__viewport">
        <ArmScene compact>
          <RobotArm q={q} accentColor={meta.color} />
          {target && <TargetMarker position={target.position} quaternion={target.quaternion} />}
        </ArmScene>
      </div>

      <div className="solver-card__footer">
        <div className="solver-card__phase">{phase ? phaseLabel(phase) : '—'}</div>
        <div className="solver-card__metrics">
          <Metric label="pos err" value={step ? step.pos_error.toFixed(4) : '—'} />
          <Metric label="iter" value={step ? step.iteration : '—'} />
          {status === 'done' && (
            <Metric
              label="result"
              value={success ? 'converged' : 'failed'}
              tone={success ? 'good' : 'bad'}
            />
          )}
        </div>
        {error && <div className="solver-card__error">{error}</div>}
      </div>
    </div>
  );
}

function Metric({ label, value, tone }) {
  return (
    <div className={`metric metric--${tone || 'neutral'}`}>
      <span className="metric__value">{value}</span>
      <span className="metric__label">{label}</span>
    </div>
  );
}
