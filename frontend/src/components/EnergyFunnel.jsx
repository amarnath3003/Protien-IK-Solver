import { useMemo } from 'react';

/**
 * Renders a V-shaped funnel; a marker's vertical position tracks how
 * far the current pos+orient error is from converged (top = far/high
 * energy, bottom = converged/low energy), and horizontal jitter encodes
 * how "wide" the active search currently is (wider near the top,
 * narrowing toward the bottom) -- the literal visual form of a folding
 * funnel, not a decorative chart.
 */
export function EnergyFunnel({ posError = null, orientError = null, phase = '', status = 'idle' }) {
  const t = useMemo(() => {
    if (posError == null) return 0; // idle: sits at the rim (unconverged)
    const combined = posError + 0.3 * (orientError ?? 0);
    const norm = 1 - Math.exp(-combined * 4);
    return Math.min(Math.max(norm, 0), 1);
  }, [posError, orientError]);

  const W = 160, H = 200;
  const rimY = 14, tipY = H - 14;
  const rimHalfWidth = 60, tipHalfWidth = 4;

  const y = rimY + t * (tipY - rimY);
  const halfWidthAtY = rimHalfWidth + (tipHalfWidth - rimHalfWidth) * t;
  const jitterSeed = phase.length + (posError ?? 0) * 137;
  const jitter = status === 'running' ? (Math.sin(jitterSeed * 12.9) * halfWidthAtY * 0.5) : 0;
  const markerX = W / 2 + jitter;

  const funnelPath = `M ${W / 2 - rimHalfWidth} ${rimY} L ${W / 2 - tipHalfWidth} ${tipY} L ${W / 2 + tipHalfWidth} ${tipY} L ${W / 2 + rimHalfWidth} ${rimY} Z`;

  const isRescue = phase.includes('rescue') || phase.includes('unfold');
  const markerColor = isRescue ? 'var(--alarm)' : 'var(--phosphor)';

  return (
    <svg viewBox={`0 0 ${W} ${H}`} width="100%" height={H} role="img" aria-label="Energy funnel readout">
      <path d={funnelPath} fill="none" stroke="var(--steel)" strokeWidth="1" opacity="0.6" />
      {[0.25, 0.5, 0.75].map((f) => {
        const gy = rimY + f * (tipY - rimY);
        const hw = rimHalfWidth + (tipHalfWidth - rimHalfWidth) * f;
        return (
          <line key={f} x1={W / 2 - hw} y1={gy} x2={W / 2 + hw} y2={gy}
            stroke="var(--steel-dim)" strokeWidth="0.75" strokeDasharray="2 3" />
        );
      })}
      {status !== 'idle' && (
        <circle cx={markerX} cy={y} r={status === 'running' ? 4.5 : 5.5} fill={markerColor}>
          {status === 'running' && (
            <animate attributeName="r" values="3.5;5.5;3.5" dur="0.9s" repeatCount="indefinite" />
          )}
        </circle>
      )}
      <text x={W / 2} y={rimY - 4} textAnchor="middle" fontSize="9" fill="var(--steel)" fontFamily="var(--font-mono)">
        unfolded
      </text>
      <text x={W / 2} y={tipY + 14} textAnchor="middle" fontSize="9" fill="var(--steel)" fontFamily="var(--font-mono)">
        native
      </text>
    </svg>
  );
}
