import { Alg } from "./Alg";
import { AlgBuilder } from "./AlgBuilder";
import type { Unit } from "./units";
import { Commutator } from "./units/containers/Commutator";
import { Conjugate } from "./units/containers/Conjugate";
import { Grouping } from "./units/containers/Grouping";
import { LineComment } from "./units/leaves/LineComment";
import { Move, QuantumMove } from "./units/leaves/Move";
import { Newline } from "./units/leaves/Newline";
import { Pause } from "./units/leaves/Pause";

type StoppingChar = "," | ":" | "]" | ")";

function parseIntWithEmptyFallback<T>(n: string, emptyFallback: T): number | T {
  return n ? parseInt(n) : emptyFallback;
}

const amountRegex = /^(\d+)?('?)/;
const moveStartRegex = /^[_\dA-Za-z]/; // TODO: Handle slash
const quantumMoveRegex = /^((([1-9]\d*)-)?([1-9]\d*))?([_A-Za-z]+)?/;
const commentTextRegex = /^[^\n]*/;
const square1PairStart = /^(-?\d+), ?/; // TODO: match up with other whitespace handling?
const square1PairEnd = /^(-?\d+)\)/; // TODO: match up with other whitespace handling?

export function parseAlg(s: string): Alg {
  return new AlgParser().parseAlg(s);
}

export function parseMove(s: string): Move {
  return new AlgParser().parseMove(s);
}

export function parseQuantumMove(s: string): QuantumMove {
  return new AlgParser().parseQuantumMove(s);
}

export interface ParserIndexed {
  charIndex: number;
}

export type Parsed<T extends Alg | Unit> = T & ParserIndexed;

function addCharIndex<T extends Alg | Unit>(
  t: T,
  charIndex: number,
): Parsed<T> {
  const parsedT = t as ParserIndexed & T;
  parsedT.charIndex = charIndex;
  return parsedT;
}

export function transferCharIndex<T extends Alg | Unit>(from: T, to: T): T {
  if ("charIndex" in from) {
    (to as Parsed<T>).charIndex = (from as Parsed<T>).charIndex;
  }
  return to;
}

type MoveSuffix = "+" | "++" | "-" | "--";

// TODO: support recording string locations for moves.
class AlgParser {
  #input: string = "";
  #idx: number = 0;

  parseAlg(input: string): Parsed<Alg> {
    this.#input = input;
    this.#idx = 0;
    const alg = this.parseAlgWithStopping([]);
    this.mustBeAtEndOfInput();
    return alg;
  }

  parseMove(input: string): Parsed<Move> {
    this.#input = input;
    this.#idx = 0;
    const move = this.parseMoveImpl();
    this.mustBeAtEndOfInput();
    return move;
  }

  parseQuantumMove(input: string): QuantumMove {
    this.#input = input;
    this.#idx = 0;
    const quantumMove = this.parseQuantumMoveImpl();
    this.mustBeAtEndOfInput();
    return quantumMove;
  }

  private mustBeAtEndOfInput() {
    if (this.#idx !== this.#input.length) {
      throw new Error("parsing unexpectedly ended early");
    }
  }

  private parseAlgWithStopping(stopBefore: StoppingChar[]): Parsed<Alg> {
    const algStartIdx = this.#idx;
    const algBuilder = new AlgBuilder();

    // We're "crowded" if there was not a space or newline since the last unit.
    let crowded = false;

    const mustNotBeCrowded = (idx: number): void => {
      if (crowded) {
        throw new Error(
          `Unexpected character at index ${idx}. Are you missing a space?`,
        ); // TODO better error message
      }
    };

    mainLoop: while (this.#idx < this.#input.length) {
      const savedCharIndex = this.#idx;
      if ((stopBefore as string[]).includes(this.#input[this.#idx])) {
        return addCharIndex(algBuilder.toAlg(), algStartIdx);
      }
      if (this.tryConsumeNext(" ")) {
        crowded = false;
        continue mainLoop;
      } else if (moveStartRegex.test(this.#input[this.#idx])) {
        mustNotBeCrowded(savedCharIndex);
        const move = this.parseMoveImpl();
        algBuilder.push(move);
        crowded = true;
        continue mainLoop;
      } else if (this.tryConsumeNext("(")) {
        mustNotBeCrowded(savedCharIndex);
        const sq1PairStartMatch = this.tryRegex(square1PairStart);
        if (sq1PairStartMatch) {
          const savedCharIndexD = this.#idx;
          const sq1PairEndMatch = this.parseRegex(square1PairEnd);
          const uMove = addCharIndex(
            new Move(new QuantumMove("U_SQ_"), parseInt(sq1PairStartMatch[1])),
            savedCharIndex + 1,
          );
          const dMove = addCharIndex(
            new Move(new QuantumMove("D_SQ_"), parseInt(sq1PairEndMatch[1])),
            savedCharIndexD,
          );
          const alg = addCharIndex(new Alg([uMove, dMove]), savedCharIndex + 1);
          algBuilder.push(addCharIndex(new Grouping(alg), savedCharIndex));
          crowded = true;
          continue mainLoop;
        } else {
          const alg = this.parseAlgWithStopping([")"]);
          this.mustConsumeNext(")");
          const amount = this.parseAmount();
          algBuilder.push(
            addCharIndex(new Grouping(alg, amount), savedCharIndex),
          );
          crowded = true;
          continue mainLoop;
        }
      } else if (this.tryConsumeNext("[")) {
        mustNotBeCrowded(savedCharIndex);
        const A = this.parseAlgWithStopping([",", ":"]);
        const separator = this.popNext();
        const B = this.parseAlgWithStopping(["]"]);
        this.mustConsumeNext("]");
        switch (separator) {
          case ":":
            algBuilder.push(addCharIndex(new Conjugate(A, B), savedCharIndex));
            crowded = true;
            continue mainLoop;
          case ",":
            algBuilder.push(addCharIndex(new Commutator(A, B), savedCharIndex));
            crowded = true;
            continue mainLoop;
          default:
            throw "unexpected parsing error";
        }
      } else if (this.tryConsumeNext("\n")) {
        algBuilder.push(addCharIndex(new Newline(), savedCharIndex));
        crowded = false;
        continue mainLoop;
      } else if (this.tryConsumeNext("/")) {
        if (this.tryConsumeNext("/")) {
          mustNotBeCrowded(savedCharIndex);
          const [text] = this.parseRegex(commentTextRegex);
          algBuilder.push(addCharIndex(new LineComment(text), savedCharIndex));
          crowded = false;
          continue mainLoop;
        } else {
          // We allow crowding here to account for csTimer scrambles, which don't have a space between a Square-1 tuple and the following slash.
          algBuilder.push(addCharIndex(new Move("_SLASH_"), savedCharIndex));
          crowded = true;
          continue mainLoop;
        }
      } else if (this.tryConsumeNext(".")) {
        mustNotBeCrowded(savedCharIndex);
        algBuilder.push(addCharIndex(new Pause(), savedCharIndex));
        while (this.tryConsumeNext(".")) {
          algBuilder.push(addCharIndex(new Pause(), this.#idx - 1)); // TODO: Can we precompute index similarly to other units?
        }
        crowded = true;
        continue mainLoop;
      } else {
        throw new Error(`Unexpected character: ${this.popNext()}`);
      }
    }

    if (this.#idx !== this.#input.length) {
      throw new Error("did not finish parsing?");
    }
    if (stopBefore.length > 0) {
      throw new Error("expected stopping");
    }
    return addCharIndex(algBuilder.toAlg(), algStartIdx);
  }

  private parseQuantumMoveImpl(): QuantumMove {
    const [, , , outerLayerStr, innerLayerStr, family] = this.parseRegex(
      quantumMoveRegex,
    );

    return new QuantumMove(
      family,
      parseIntWithEmptyFallback(innerLayerStr, undefined),
      parseIntWithEmptyFallback(outerLayerStr, undefined),
    );
  }

  private parseMoveImpl(): Parsed<Move> {
    const savedCharIndex = this.#idx;

    if (this.tryConsumeNext("/")) {
      return addCharIndex(new Move("_SLASH_"), savedCharIndex);
    }

    let quantumMove = this.parseQuantumMoveImpl();
    let [amount, hadEmptyAbsAmount] = this.parseAmountAndTrackEmptyAbsAmount();
    const suffix = this.parseMoveSuffix();

    if (suffix) {
      if (amount < 0) {
        throw new Error("uh-oh");
      }
      if ((suffix === "++" || suffix === "--") && amount !== 1) {
        // TODO: Handle 1 vs. null
        throw new Error(
          "Pochmann ++ or -- moves cannot have an amount other than 1.",
        );
      }
      if ((suffix === "++" || suffix === "--") && !hadEmptyAbsAmount) {
        throw new Error(
          "Pochmann ++ or -- moves cannot have an amount written as a number.",
        );
      }
      if ((suffix === "+" || suffix === "-") && hadEmptyAbsAmount) {
        throw new Error(
          "Clock dial moves must have an amount written as a natural number followed by + or -.",
        );
      }
      if (suffix.startsWith("+")) {
        quantumMove = quantumMove.modified({
          family: `${quantumMove.family}_${
            suffix === "+" ? "PLUS" : "PLUSPLUS"
          }_`, // TODO
        });
      }
      if (suffix.startsWith("-")) {
        quantumMove = quantumMove.modified({
          family: `${quantumMove.family}_${
            suffix === "-" ? "PLUS" : "PLUSPLUS"
          }_`, // TODO
        });
        amount *= -1;
      }
    }

    const move = addCharIndex(new Move(quantumMove, amount), savedCharIndex);
    return move;
  }

  private parseMoveSuffix(): MoveSuffix | null {
    if (this.tryConsumeNext("+")) {
      if (this.tryConsumeNext("+")) {
        return "++";
      }
      return "+";
    }
    if (this.tryConsumeNext("-")) {
      if (this.tryConsumeNext("-")) {
        return "--";
      }
      return "-";
    }
    return null;
  }

  private parseAmountAndTrackEmptyAbsAmount(): [number, boolean] {
    const [, absAmountStr, primeStr] = this.parseRegex(amountRegex);
    return [
      parseIntWithEmptyFallback(absAmountStr, 1) * (primeStr === "'" ? -1 : 1),
      !absAmountStr,
    ];
  }

  private parseAmount(): number {
    const [, absAmountStr, primeStr] = this.parseRegex(amountRegex);
    return (
      parseIntWithEmptyFallback(absAmountStr, 1) * (primeStr === "'" ? -1 : 1)
    );
  }

  private parseRegex(regex: RegExp): RegExpExecArray {
    const arr = regex.exec(this.remaining());
    if (arr === null) {
      throw new Error("internal parsing error"); // TODO
    }
    this.#idx += arr[0].length;
    return arr;
  }

  // TOD: can we avoid this?
  private tryRegex(regex: RegExp): RegExpExecArray | null {
    const arr = regex.exec(this.remaining());
    if (arr === null) {
      return null;
    }
    this.#idx += arr[0].length;
    return arr;
  }

  private remaining(): string {
    return this.#input.slice(this.#idx);
  }

  private popNext(): string {
    const next = this.#input[this.#idx];
    this.#idx++;
    return next;
  }

  private tryConsumeNext(expected: string): boolean {
    if (this.#input[this.#idx] === expected) {
      this.#idx++;
      return true;
    }
    return false;
  }

  private mustConsumeNext(expected: string): string {
    const next = this.popNext();
    if (next !== expected) {
      throw new Error(
        `expected \`${expected}\` while parsing, encountered ${next}`,
      ); // TODO: be more helpful
    }
    return next;
  }
}
