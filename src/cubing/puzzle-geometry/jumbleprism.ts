import { Quat } from "./Quat";
class Face {
  private coords: number[];
  public length: number;
  constructor(q: Quat[]) {
    this.coords = new Array(q.length * 3);
    for (let i = 0; i < q.length; i++) {
      this.coords[3 * i] = q[i].b;
      this.coords[3 * i + 1] = q[i].c;
      this.coords[3 * i + 2] = q[i].d;
    }
    this.length = q.length;
  }

  get(off: number): Quat {
    return new Quat(
      0,
      this.coords[3 * off],
      this.coords[3 * off + 1],
      this.coords[3 * off + 2],
    );
  }

  centermass(): Quat {
    let sx = 0;
    let sy = 0;
    let sz = 0;
    for (let i = 0; i < this.length; i++) {
      sx += this.coords[3 * i];
      sy += this.coords[3 * i + 1];
      sz += this.coords[3 * i + 2];
    }
    return new Quat(0, sx / this.length, sy / this.length, sz / this.length);
  }

  rotate(q: Quat): Face {
    const a = [];
    for (let i = 0; i < this.length; i++) {
      a.push(this.get(i).rotatepoint(q));
    }
    return new Face(a);
  }

  rotateforward(): Face {
    const a = [];
    for (let i = 1; i < this.length; i++) {
      a.push(this.get(i));
    }
    a.push(this.get(0));
    return new Face(a);
  }
}

export class FaceTree {
  constructor(
    private face: Quat[],
    private left?: FaceTree,
    private right?: FaceTree,
  ) {}

  public split(q: Quat): FaceTree {
    const t = q.cutface(this.face);
    if (t !== null) {
      if (this.left === undefined) {
        this.left = new FaceTree(t[0]);
        this.right = new FaceTree(t[1]);
      } else {
        this.left = this.left?.split(q);
        this.right = this.right?.split(q);
      }
    }
    return this;
  }

  public collect(arr: Face[], leftfirst: boolean): Face[] {
    if (this.left === undefined) {
      arr.push(new Face(this.face));
    } else if (leftfirst) {
      this.left?.collect(arr, false);
      this.right?.collect(arr, true);
    } else {
      this.right?.collect(arr, false);
      this.left?.collect(arr, true);
    }
    return arr;
  }
}

function makeplane(f: Quat[]): Quat {
    // turn three points into a plane.
    const a = f[1].sub(f[0]);
    const b = f[2].sub(f[1]);
    const c = a.cross(b);
    c.a = -f[0].dot(c);
    return c;
}

export function makeJumblePrism() {
  const h = Math.sqrt(3) / 3;
  const p = [
    new Quat(0, 1, h, 1),
    new Quat(0, -1, h, 1),
    new Quat(0, 0, -2 * h, 1),
    new Quat(0, 1, h, 1),
    new Quat(0, -1, h, 1),
    new Quat(0, 0, -2 * h, 1),
  ];
  const f = [
    new Face([p[0], p[1], p[2]]),
    new Face([p[3], p[4], p[5]]),
    new Face([p[0], p[1], p[3], p[4]]),
    new Face([p[1], p[2], p[4], p[5]]),
    new Face([p[2], p[0], p[5], p[3]]),
  ];
  const c = [
    makeplane([p[0], p[4], p[5]]),
    makeplane([p[1], p[5], p[3]]),
    makeplane([p[2], p[3], p[4]]),
    makeplane([p[3], p[1], p[2]]),
    makeplane([p[4], p[2], p[0]]),
    makeplane([p[5], p[0], p[1]]),
  ];
  return [c, f];
}
export const JP = makeJumblePrism();