import { ManagedCustomElement } from "../../../cubing/twisty/dom/element/ManagedCustomElement";
import { customElementsShim } from "../../../cubing/twisty/dom/element/node-custom-element-shims";
import { TwistyPlayerInitialConfig } from "../../../cubing/twisty";
import { TwistyPlayer } from "../../../cubing/twisty/index";
import { invert, parseAlg, Sequence } from "../../../cubing/alg";

const DEBUG = true;

const warned = new Set<string>();
function warnOnce(warning: string) {
  if (!warned.has(warning)) {
    console.warn(warning);
    warned.add(warning);
  }
}

// From http://software.rubikscube.info/AnimCube/
const paramsNotImplementedYet: Record<string, true> = {
  config: true, // fff...
  bgcolor: true, // hhhhhh
  butbgcolor: true, // hhhhhh
  colorscheme: true, // cccccc
  colors: true, // hhhhhh...
  position: true, // ppp...
  speed: true, // nn...
  doublespeed: true, // nn...
  perspective: true, // nn...
  scale: true, // nn...
  align: true, // d
  hint: true, // nn...
  buttonbar: true, // d
  edit: true, // d
  movetext: true, // d
  fonttype: true, // d
  metric: true, // d
  move: true, // mmmmmmmmm...
  initmove: true, // mmmmmmmmm... / #
  initrevmove: true, // mmmmmmmmm... / #
  demo: true, // mmmmmmmmm... / #
  facelets: true, // xxxxxx...xxxxxx
  pos: true, // aaaaaa...aaaaaa
};

export class TwistyAnimCube extends ManagedCustomElement {
  twistyPlayer: TwistyPlayer;
  constructor(
    private finishConnectedCallback?: (twistyAnimCube: TwistyAnimCube) => void,
  ) {
    super();
  }

  protected connectedCallback(): void {
    // We set timeout so that we can access the children.
    setTimeout(() => {
      const twistyPlayerConfig: TwistyPlayerInitialConfig = {
        alg: new Sequence([]),
      };

      const process = (
        name: string,
        callback: (value: string) => void,
      ): void => {
        delete paramsNotImplementedYet[name];
        const child = this.children[name as any]; // TODO: Does the DOM API guarantee this always works?
        if (DEBUG) {
          console.info(name, child);
        }
        if (child) {
          const value = child.getAttribute("value");
          if (value) {
            callback(value);
          }
        }
      };

      process("move", (value: string) => {
        twistyPlayerConfig.alg = parseAlg(value ?? "");
      });

      process("initmove", (value: string) => {
        if (value === "#") {
          twistyPlayerConfig.setupAlg = invert(twistyPlayerConfig.alg!);
        } else {
          twistyPlayerConfig.setupAlg = parseAlg(value ?? "");
        }
      });

      // Takes precedenve over `initmove`
      process("initrevmove", (value: string) => {
        if (value === "#") {
          twistyPlayerConfig.setupAlg = invert(twistyPlayerConfig.alg!);
        } else {
          twistyPlayerConfig.setupAlg = invert(parseAlg(value ?? ""));
        }
      });

      process("buttonbar", (value: string) => {
        switch (value) {
          case "0":
            twistyPlayerConfig.controlPanel = "none";
            return;
          case "1":
            warnOnce("buttonbar=1 is not implemented (yet)");
            return;
          case "2":
            return;
          default:
            warnOnce(`invalid buttonbar value: ${value}`);
            return;
        }
      });

      process("hint", (value: string) => {
        switch (value) {
          case "0":
            twistyPlayerConfig.hintFacelets = "none";
            return;
          default:
            twistyPlayerConfig.hintFacelets = "floating";
            return;
        }
      });

      if (DEBUG) {
        console.info(twistyPlayerConfig);
      }
      this.twistyPlayer = new TwistyPlayer(twistyPlayerConfig);

      process("speed", (value: string) => {
        let speed = parseFloat(value);
        if (speed === 0) {
          speed = 10;
        }
        if (speed < 0) {
          warnOnce("speed must be a non-negative number");
        }
        this.twistyPlayer.timeline.tempoScale = 10 / speed;
      });

      // process("width", (value: string) => {
      //   const width = parseFloat(value);
      //   this.twistyPlayer.style.width = `${width.toString()}px`;
      // });

      // process("height", (value: string) => {
      //   const height = parseFloat(value);
      //   this.twistyPlayer.style.height = `${height.toString()}px`;
      // });

      for (const name in paramsNotImplementedYet) {
        if (this.children[name as any]) {
          warnOnce(`Param is not implemented (yet): ${name}`);
          return;
        }
      }

      this.addElement(this.twistyPlayer);
      if (this.finishConnectedCallback) {
        this.finishConnectedCallback(this);
      }
    });
  }
}

customElementsShim.define("twisty-anim-cube", TwistyAnimCube);

window.addEventListener("load", () => {
  const applets = document.querySelectorAll("applet");
  console.log(applets);
  for (const applet of Array.from(applets)) {
    const attributeContains = (attrName: string, substr: string): boolean => {
      return (
        (applet.getAttribute(attrName) ?? "").toLowerCase().indexOf(substr) !==
        1
      );
    };

    let twisty: Element;
    if (
      attributeContains("code", "animcube") ||
      attributeContains("archive", "animcube")
    ) {
      // TODO: pass on width and height
      twisty = new TwistyAnimCube((twistyAnimCube: TwistyAnimCube) => {
        const width = applet.getAttribute("width");
        if (width) {
          twistyAnimCube.twistyPlayer.style.width = `${parseInt(
            width,
          ).toString()}px`;
        }
        const height = applet.getAttribute("height");
        if (height) {
          twistyAnimCube.twistyPlayer.style.height = `${parseInt(
            height,
          ).toString()}px`;
        }
      });
    } else {
      return;
    }
    twisty.append(...Array.from(applet.childNodes));
    applet.insertAdjacentElement("afterend", twisty);
    applet.remove();
  }
});