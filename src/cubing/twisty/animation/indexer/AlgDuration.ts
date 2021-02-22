import {
  Alg,
  Bunch,
  Comment,
  Commutator,
  Conjugate,
  Move,
  Newline,
  Pause,
  TraversalUp,
} from "../../../alg";
import { Duration } from "../cursor/CursorTypes";

export function constantDurationForAmount(_amount: number): Duration {
  return 1000;
}

export function defaultDurationForAmount(amount: number): Duration {
  switch (Math.abs(amount)) {
    case 0:
      return 0;
    case 1:
      return 1000;
    case 2:
      return 1500;
    default:
      return 2000;
  }
}
// eslint-disable-next-line no-inner-declarations
export function ExperimentalScaledDefaultDurationForAmount(
  scale: number,
  amount: number,
): Duration {
  switch (Math.abs(amount)) {
    case 0:
      return 0;
    case 1:
      return scale * 1000;
    case 2:
      return scale * 1500;
    default:
      return scale * 2000;
  }
}

export class AlgDuration extends TraversalUp<Duration> {
  // TODO: Pass durationForAmount as Down type instead?
  constructor(
    public durationForAmount: (
      amount: number,
    ) => Duration = defaultDurationForAmount,
  ) {
    super();
  }

  public traverseAlg(alg: Alg): Duration {
    let total = 0;
    for (const unit of alg.units()) {
      total += this.traverseUnit(unit);
    }
    return total;
  }

  public traverseBunch(bunch: Bunch): Duration {
    return (
      bunch.experimentalEffectiveAmount *
      this.traverseAlg(bunch.experimentalAlg)
    );
  }

  public traverseMove(move: Move): Duration {
    return this.durationForAmount(move.effectiveAmount);
  }

  public traverseCommutator(commutator: Commutator): Duration {
    return (
      commutator.experimentalEffectiveAmount *
      2 *
      (this.traverseAlg(commutator.A) + this.traverseAlg(commutator.B))
    );
  }

  public traverseConjugate(conjugate: Conjugate): Duration {
    return (
      conjugate.experimentalEffectiveAmount *
      (2 * this.traverseAlg(conjugate.A) + this.traverseAlg(conjugate.B))
    );
  }

  public traversePause(_pause: Pause): Duration {
    return this.durationForAmount(1);
  }

  public traverseNewline(_newline: Newline): Duration {
    return this.durationForAmount(1);
  }

  public traverseComment(_comment: Comment): Duration {
    return this.durationForAmount(0);
  }
}
