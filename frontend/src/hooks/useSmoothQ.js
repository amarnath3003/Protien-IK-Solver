import { useRef, useState } from 'react';
import { useFrame } from '@react-three/fiber';

/**
 * Smoothly interpolates a joint-angle array `q` toward each new target
 * value using frame-rate-independent exponential decay (critical damping).
 *
 * @param {number[]} targetQ   The "goal" joint angles (updated by WebSocket steps).
 * @param {number}   speed     Controls how quickly the arm catches up.
 *                             Higher = snappier. Default 12 gives ~80 ms settle time at 60fps.
 * @returns {number[]}         The smoothed joint angles to pass to the renderer.
 */
export function useSmoothQ(targetQ, speed = 12) {
  // currentQ holds the live-interpolated angles between React renders.
  // We use a ref so mutations don't cause re-renders on their own.
  const currentQRef = useRef([...targetQ]);
  // smoothQ is the React state we expose — updated only when the values
  // change enough to be visible, keeping re-renders minimal.
  const [smoothQ, setSmoothQ] = useState([...targetQ]);

  useFrame((_, delta) => {
    // Exponential lerp factor: frame-rate-independent via delta (seconds).
    // Factor approaches 1 as delta grows, ensuring the arm always catches up
    // regardless of frame rate drops.
    const alpha = 1 - Math.exp(-speed * delta);

    const curr = currentQRef.current;
    let maxDelta = 0;
    const next = curr.map((c, i) => {
      const t = targetQ[i] ?? c;
      const n = c + (t - c) * alpha;
      maxDelta = Math.max(maxDelta, Math.abs(n - c));
      return n;
    });

    // Only update React state when the arm is actually moving
    if (maxDelta > 5e-5) {
      currentQRef.current = next;
      setSmoothQ([...next]);
    }
  });

  return smoothQ;
}
