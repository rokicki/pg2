import type { KPuzzle } from "../../kpuzzle";

export function lazyKPuzzle(
  getKPuzzle: () => Promise<KPuzzle>,
): () => Promise<KPuzzle> {
  let cachedKPuzzlePromise: Promise<KPuzzle> | null = null;
  return (): Promise<KPuzzle> => {
    return (cachedKPuzzlePromise ??= getKPuzzle());
  };
}
