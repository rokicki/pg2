import { centermassface, Quat } from "./Quat";
const eps = 1e-9;

function makeplane(f: Quat[]): Quat {
  // turn three points into a plane.
  const a = f[1].sub(f[0]);
  const b = f[2].sub(f[0]);
  const c = a.cross(b).normalizeplane();
  c.a = f[0].dot(c);
  if (c.a < -eps) {
    console.log("A plane was deeper than deep");
  }
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

function centermasscubie(cubie: Quat[][]): Quat {
  let s = new Quat(0, 0, 0, 0);
  for (const face of cubie) {
    s = s.sum(centermassface(face));
  }
  return s.smul(1/cubie.length);
}

class Jumbler {
  public curstop: number[];
  public cubieshapes: [Quat, Quat[][]][] = [];
  constructor(public cubieset: Quat[][][], public cuts: Quat[], public stops: number[]) {
    this.curstop = Array(cuts.length).fill(0);
    for (const cubie of cubieset) {
      this.cubieshapes.push([centermasscubie(cubie), cubie]);
    }
  }

  public canoncubie(cubie: Quat[][]): number {
    const cm = centermasscubie(cubie);
    for (let i=0; i<this.cubieshapes.length; i++) {
      let ok = true;
      if (cm.dist(this.cubieshapes[i][0]) < eps) {
        for (let j=0; j<cubie.length; j++) {
          let foundface = false;
          const cmf = centermassface(cubie[j]);
          for (const f2 of this.cubieshapes[i][1]) {
            if (centermassface(f2).dist(cmf) < eps) {
              foundface = true;
              break;
            }
          }
          if (!foundface) {
            ok = false;
            break;
          }
        }
        if (ok) {
          return i;
        }
      }
    }
    this.cubieshapes.push([cm, cubie]);
    return this.cubieshapes.length - 1;
  }

  public unblocked(): number[] {
    const r: number[] = [];
    for (let i=0; i<this.cuts.length; i++) {
      const plane = this.cuts[i];
      const d = plane.a;
      let ok = true;
      for (let j=0; ok && j<this.cubieset.length; j++) {
        let seensides = 0;
        for (let k=0; ok && k<this.cubieset[j].length; k++) {
          for (let m=0; m<this.cubieset[j][k].length; m++) {
            seensides |= 1 << (plane.side(this.cubieset[j][k][m].dot(plane) - d) + 1);
            if ((seensides & 5) == 5) {
              ok = false;
              break;
            }
          }
        }
      }
      if (ok) {
        r.push(i);
      }
    }
    return r;
  }

  public move(grip: number, stop: number): void {
    const ang = (this.stops[stop] - this.stops[this.curstop[grip]]) * 0.5;
    const plane = this.cuts[grip];
    const d = plane.a;
    const s = Math.sin(ang);
    const rot = new Quat(Math.cos(ang), plane.b * s, plane.c * s, plane.d * s);
    for (let j=0; j<this.cubieset.length; j++) {
      let side = 0;
      for (let k=0; side===0 && k<this.cubieset[j].length; k++) {
        for (let m=0; m<this.cubieset[j][k].length; m++) {
          side = plane.side(this.cubieset[j][k][m].dot(plane) - d);
          if (side != 0) {
            break;
          }
        }
      }
      if (side === 1) {
        const nc: Quat[][] = [];
        for (let k=0; k<this.cubieset[j].length; k++) {
          nc.push(rot.rotateface(this.cubieset[j][k]));
        }
        this.cubieset[j] = nc;
      }
    }
    this.curstop[grip] = stop;
  }

  public getshape(): number[] {
    const r: number[] = [];
    for (const cubie of this.cubieset) {
      r.push(this.canoncubie(cubie));
    }
    r.sort();
    return r;
  }
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
    [p[0], p[1], p[4], p[3]],
    [p[1], p[2], p[5], p[4]],
    [p[2], p[0], p[3], p[5]],
  ];
  let cubieset = [cubie];
  const cuts = [
    makeplane([p[0], p[5], p[4]]),
    makeplane([p[1], p[3], p[5]]),
    makeplane([p[2], p[4], p[3]]),
    makeplane([p[3], p[1], p[2]]),
    makeplane([p[4], p[2], p[0]]),
    makeplane([p[5], p[0], p[1]]),
  ];
  for (const cut of cuts) {
    cubieset = splitcubieset(cubieset, cut);
  }
  const stops = [0, Math.acos(1/8), Math.acos(-3/4), -Math.acos(-3/4), -Math.acos(1/8)];
  const ju = new Jumbler(cubieset, cuts, stops);
  console.log(stops);
  const seen: {[key: string]: number} = {};
  console.log(ju.cubieset);
  let lastgrip = -1;
  for (let t=0; t<8800000; t++) {
    const unblocked = ju.unblocked();
    if (unblocked.length > 1) {
      seen[ju.getshape().join(" ")] = 1;
    }
    if (unblocked.length == 0) {
      console.log("Failed to do move correctly");
      break;
    }
    let g = unblocked[Math.floor(Math.random() * unblocked.length)];
    while (g === lastgrip && unblocked.length > 1) {
      g = unblocked[Math.floor(Math.random() * unblocked.length)];
    }
    const off = 1 + Math.floor(Math.random() * 4);
    const stop = (ju.curstop[g] + off) % 5;
    ju.move(g, stop);
    lastgrip = g;
    if ((t & (t - 1)) == 0) {
      const len = Object.keys(seen).length;
      console.log("At " + t + " length is " + len + " " + ju.cubieshapes.length);
    }
  }
/*
  for (let i=0; i<ju.cuts.length; i++) {
    for (let j=1; j<ju.stops.length; j++) {
      ju.move(i, j);
      console.log(ju.getshape());
    }
    ju.move(i, 0);
    console.log(ju.getshape());
  }
  */
}
export const JP = makeJumblePrism();