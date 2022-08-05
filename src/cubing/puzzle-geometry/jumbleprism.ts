import { centermassface, Quat } from "./Quat";
const eps = 1e-9;
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
  const b = f[2].sub(f[0]);
  const c = a.cross(b);
  c.a = -f[0].dot(c);
  return c;
}

// A cubie is a set of faces.  To split it by a plane, we split each face,
// but we need to track the split points so we can introduce new internal faces
// created by the split.
function splitcubie(cubie: Quat[][], plane: Quat, r: Quat[][][]): void {
  const c1: Quat[][] = [];
  const c2: Quat[][] = [];
  let cm1 = new Quat(0,0,0,0);
  let cm2 = new Quat(0,0,0,0);
  const cpts: Quat[] = [];
  for (const face of cubie) {
    const [f1, f2, cp] = plane.splitface(face);
    if (f1.length) {
      c1.push(f1);
      cm1 = cm1.sum(centermassface(f1));
    }
    if (f2.length) {
      c2.push(f2);
      cm2 = cm2.sum(centermassface(f2));
    }
    for (const pt of cp) {
      let found = false;
      for (let i=0; i<cpts.length; i++) {
        if (pt.dist(cpts[i]) < eps) {
          found = true;
          break;
        }
      }
      if (!found) {
        cpts.push(pt);
      }
    }
  }
  if (cpts.length > 2) {
    // now we need to take cpts and turn it into a face
    // and add it to the two new cubies so it faces the
    // correct direction.
    const cm = centermassface(cpts);
    const sortme: [number, Quat][] = [];
    const d1 = cpts[0].sub(cm);
    const d2 = cpts[1].sub(cm);
    for (let i = 0; i < cpts.length; i++) {
      const d3 = cpts[i].sub(cm);
      sortme.push([Math.atan2(d1.dot(d3), d2.dot(d3)), cpts[i]]);
    }
    sortme.sort((a, b) => a[0] - b[0]);
    const cpts2 = [];
    for (const sp of sortme) {
        cpts2.push(sp[1]);
    }
    if (cpts2[1].sub(cpts2[0]).cross(cpts2[2].sub(cpts2[0])).dot(plane) < 0) {
        c1.push(cpts2);
        c2.push(cpts2.slice().reverse());
      } else {
        c2.push(cpts2);
        c1.push(cpts2.slice().reverse());
      }
  }
  if (c1.length) {
    r.push(c1);
  }
  if (c2.length) {
    r.push(c2);
  }
}

function splitcubieset(cubieset: Quat[][][], plane: Quat): Quat[][][] {
  const r: Quat[][][] = [];
  for (const cubie of cubieset) {
    splitcubie(cubie, plane, r);
  }
  return r;
}

export function makeJumblePrism() {
  const h = Math.sqrt(3) / 3;
  const p = [
    new Quat(0, 1, h, 1),
    new Quat(0, -1, h, 1),
    new Quat(0, 0, -2 * h, 1),
    new Quat(0, 1, h, -1),
    new Quat(0, -1, h, -1),
    new Quat(0, 0, -2 * h, -1),
  ];
  const cubie = [
    [p[0], p[1], p[2]],
    [p[3], p[4], p[5]],
    [p[0], p[1], p[3], p[4]],
    [p[1], p[2], p[4], p[5]],
    [p[2], p[0], p[5], p[3]],
  ];
  let cubieset = [cubie];
  const cuts = [
    makeplane([p[0], p[4], p[5]]),
    makeplane([p[1], p[5], p[3]]),
    makeplane([p[2], p[3], p[4]]),
    makeplane([p[3], p[1], p[2]]),
    makeplane([p[4], p[2], p[0]]),
    makeplane([p[5], p[0], p[1]]),
  ];
  for (const cut of cuts) {
    cubieset = splitcubieset(cubieset, cut);
  }
  console.log(cubieset.length);
}
export const JP = makeJumblePrism();