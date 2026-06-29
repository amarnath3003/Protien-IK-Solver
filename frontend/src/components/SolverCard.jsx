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
        {status === 'done' && result && ['protein_homotopy', 'fixed_lambda_ik'].includes(solverId) && (
          <div className="solver-card__diagnostic" style={{ marginTop: '0.75rem', fontSize: '0.75rem', background: 'rgba(0,0,0,0.1)', padding: '0.5rem', borderRadius: '4px' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '0.25rem', color: '#ccc' }}>
              <span>Conflict (C): {result.conflict_index.toFixed(3)}</span>
              <span>{result.conflict_index < 0.5 ? 'cooperative' : (result.conflict_index > 1.5 ? 'conflicted' : 'orthogonal')}</span>
            </div>
            <div style={{ width: '100%', height: '4px', background: 'rgba(255,255,255,0.1)', borderRadius: '2px', position: 'relative', marginBottom: '0.75rem' }}>
              <div style={{ position: 'absolute', left: '50%', width: '1px', height: '100%', background: '#fff', opacity: 0.3 }} />
              <div style={{ 
                position: 'absolute', 
                left: '50%', 
                width: `${Math.abs(result.conflict_index) * 50}%`,
                height: '100%', 
                background: result.conflict_index > 0 ? '#ff4444' : '#00D4AA',
                transform: result.conflict_index < 0 ? 'translateX(-100%)' : 'none'
              }} />
            </div>
            
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '0.25rem', color: '#ccc' }}>
              <span>λ Final: {result.lambda_final.toFixed(3)}</span>
              <span style={{ color: result.lambda_final < 1.0 ? '#E8B95C' : '#00D4AA' }}>
                {result.lambda_final < 1.0 ? "⚠ Constraints unfulfilled" : "✓"}
              </span>
            </div>
            <div style={{ width: '100%', height: '4px', background: 'rgba(255,255,255,0.1)', borderRadius: '2px', position: 'relative' }}>
              <div style={{ 
                width: `${result.lambda_final * 100}%`,
                height: '100%', 
                background: '#8B9E9A',
                borderRadius: '2px'
              }} />
            </div>
          </div>
        )}
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
