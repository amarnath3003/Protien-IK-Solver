import { useCallback, useEffect, useRef, useState } from 'react';
import { connectSolveStream } from '../lib/api';
import { ROBOT_IDLE_Q } from '../lib/kinematics';

/**
 * Manages a live, streamed solve for one solver. Call `run(target, seed, robot)`
 * to kick off a fresh solve; `q` always reflects the latest joint state
 * (idle pose before any solve, animating through steps during a solve,
 * final pose after). Exposes status/phase/metrics for the UI readouts.
 */
export function useLiveSolve(solverId) {
  const [q, setQ] = useState(ROBOT_IDLE_Q.ur5);
  const [status, setStatus] = useState('idle'); // idle | connecting | running | done | error
  const [phase, setPhase] = useState('');
  const [step, setStep] = useState(null);
  const [result, setResult] = useState(null);
  const [error, setError] = useState(null);
  const controllerRef = useRef(null);

  const run = useCallback(async ({ target, q0, seed, stepDelayMs = 35, robot = 'ur5' }) => {
    controllerRef.current?.close();
    setStatus('connecting');
    setError(null);
    setResult(null);
    setStep(null);
    setQ(ROBOT_IDLE_Q[robot] ?? ROBOT_IDLE_Q.ur5);

    try {
      const controller = connectSolveStream({
        onStart: () => setStatus('running'),
        onStep: (data) => {
          setQ(data.q);
          setPhase(data.phase);
          setStep(data);
        },
        onDone: (data) => {
          setStatus('done');
          setResult(data);
          if (data.q_final) setQ(data.q_final);
        },
        onError: (msg) => {
          setStatus('error');
          setError(msg);
        },
      });
      controllerRef.current = controller;
      await controller.ready();
      controller.send({
        solver: solverId,
        robot,
        target,
        q0: q0 ?? null,
        seed: seed ?? null,
        step_delay_ms: stepDelayMs,
      });
    } catch (e) {
      setStatus('error');
      setError(e.message);
    }
  }, [solverId]);

  const reset = useCallback((robot = 'ur5') => {
    controllerRef.current?.close();
    setQ(ROBOT_IDLE_Q[robot] ?? ROBOT_IDLE_Q.ur5);
    setStatus('idle');
    setPhase('');
    setStep(null);
    setResult(null);
    setError(null);
  }, []);

  useEffect(() => () => controllerRef.current?.close(), []);

  return { q, status, phase, step, result, error, run, reset };
}
