import { useRef, useEffect } from 'react';

/**
 * Renders a V-shaped funnel; a marker's vertical position tracks how
 * far the current pos+orient error is from converged (top = far/high
 * energy, bottom = converged/low energy), and horizontal jitter encodes
 * how "wide" the active search currently is (wider near the top,
 * narrowing toward the bottom) -- the literal visual form of a folding
 * funnel, not a decorative chart.
 *
 * Jitter is driven by an accumulated clock in a requestAnimationFrame loop so it
 * animates continuously while the solver is running, not just when a
 * new WebSocket step arrives.
 */
const W = 160, H = 200;
const rimY = 14, tipY = H - 14;
const rimHalfWidth = 60, tipHalfWidth = 4;

export function EnergyFunnel({ posError = null, orientError = null, phase = '', status = 'idle' }) {
  // Accumulated time for smooth animated jitter
  const timeRef  = useRef(0);
  const circleRef = useRef(null);
  const latestErrors = useRef({ posError, orientError });

  useEffect(() => {
    latestErrors.current = { posError, orientError };
  }, [posError, orientError]);

  useEffect(() => {
    if (status !== 'running') return;

    let lastTime = null;
    let reqId = null;

    const animate = (time) => {
      if (lastTime != null) {
        const delta = (time - lastTime) / 1000;
        timeRef.current += delta;

        if (circleRef.current) {
          const { posError: pe, orientError: oe } = latestErrors.current;
          // Recompute t (error → funnel position) every frame
          const combined = (pe ?? 0) + 0.3 * (oe ?? 0);
          const t = Math.min(Math.max(1 - Math.exp(-combined * 4), 0), 1);

          const y = rimY + t * (tipY - rimY);
          const halfWidthAtY = rimHalfWidth + (tipHalfWidth - rimHalfWidth) * t;
          const jitter = Math.sin(timeRef.current * 7.3) * halfWidthAtY * 0.45;
          const markerX = W / 2 + jitter;

          circleRef.current.setAttribute('cx', markerX);
          circleRef.current.setAttribute('cy', y);
        }
      }
      lastTime = time;
      reqId = requestAnimationFrame(animate);
    };

    reqId = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(reqId);
  }, [status]);

  // Compute static initial position for SSR / first render
  const combined0 = (posError ?? 0) + 0.3 * (orientError ?? 0);
  const t0 = posError == null ? 0 : Math.min(Math.max(1 - Math.exp(-combined0 * 4), 0), 1);
  const y0 = rimY + t0 * (tipY - rimY);

  const isRescue = phase.includes('rescue') || phase.includes('unfold');
  const markerColor = isRescue ? 'var(--alarm)' : 'var(--phosphor)';

  const funnelPath = `M ${W / 2 - rimHalfWidth} ${rimY} L ${W / 2 - tipHalfWidth} ${tipY} L ${W / 2 + tipHalfWidth} ${tipY} L ${W / 2 + rimHalfWidth} ${rimY} Z`;

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
        <circle
          ref={circleRef}
          cx={W / 2}
          cy={y0}
          r={status === 'running' ? 4.5 : 5.5}
          fill={markerColor}
        >
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
