"""Architecture and model-based-development pipeline diagrams (SVG, light card so they read in light and dark mode)."""
from xml.sax.saxutils import escape
from common import MEDIA

INK, CARD, LINE = "#24292f", "#f6f8fa", "#57606a"
FILL = {"model": "#cfe3f4", "task": "#f6e3b8", "hw": "#cdeadf", "todo": "#ffffff"}


def svg(w, h, body, title, desc):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" role="img" aria-labelledby="t d">'
            f'<title id="t">{escape(title)}</title><desc id="d">{escape(desc)}</desc>'
            f'<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0 0L10 5L0 10z" fill="{LINE}"/></marker></defs>'
            f'<rect width="{w}" height="{h}" rx="10" fill="{CARD}"/>' + body + "</svg>")


def box(x, y, w, h, lines, kind="model", dashed=False):
    t = "".join(f'<text x="{x + w / 2}" y="{y + 20 + 15 * i}" text-anchor="middle" font-size="{12 if i else 13}" '
                f'font-weight="{400 if i else 700}" fill="{INK}">{escape(l)}</text>' for i, l in enumerate(lines))
    return (f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" fill="{FILL[kind]}" stroke="{LINE}" '
            f'{"stroke-dasharray=\"5 4\"" if dashed else ""}/>' + t)


def arrow(x1, y1, x2, y2, dashed=False):
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{LINE}" stroke-width="1.6" marker-end="url(#a)" {"stroke-dasharray=\"5 4\"" if dashed else ""}/>'


def label(x, y, s, size=12, anchor="start", bold=False):
    return f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" fill="{INK}" font-weight="{700 if bold else 400}" font-family="sans-serif">{escape(s)}</text>'


# ---- architecture ----
b = label(20, 28, "Model-based path (MATLAB / Simulink)", 13, bold=True)
b += box(20, 44, 150, 56, ["Simscape plant", "nonlinear, 5 outputs"]) + arrow(170, 72, 200, 72)
b += box(200, 44, 150, 56, ["Linearise at hover", "5 states, 4 inputs"]) + arrow(350, 72, 380, 72)
b += box(380, 44, 150, 56, ["Discretise + dlqr", "Ts = 0.01 s, Q, R"]) + arrow(530, 72, 560, 72)
b += box(560, 44, 160, 56, ["Embedded Coder", "not completed"], "todo", True) + arrow(720, 72, 750, 72, True)
b += box(750, 44, 130, 56, ["Crazyflie 2.1", "BMI088 IMU"], "hw")
b += label(20, 150, "Hand-written C path (FreeRTOS, desktop run)", 13, bold=True)
b += box(20, 166, 150, 66, ["dynamic model task", "1 ms, course template"], "task")
b += box(200, 166, 150, 66, ["filter task (ours)", "5 ms, complementary"], "task")
b += box(380, 166, 150, 66, ["control task", "10 ms (100 Hz), LQ gain K"], "task")
b += box(560, 166, 160, 66, ["reference task", "roll/pitch triangles"], "task")
b += box(750, 166, 130, 66, ["log task", "simulation.csv"], "task")
b += arrow(170, 199, 200, 199) + arrow(350, 199, 380, 199) + arrow(560, 199, 530, 199)
b += label(185, 262, "sensors_sem", 11, "middle") + label(365, 262, "estimate_sem", 11, "middle") + label(545, 262, "references_sem", 11, "middle")
b += f'<path d="M455 232 V292 H95 V232" fill="none" stroke="{LINE}" stroke-width="1.6" stroke-dasharray="5 4" marker-end="url(#a)"/>'
b += label(275, 310, "motors_sem: PWM commands back to the dynamic model", 11, "middle")
b += arrow(455, 100, 455, 166, True) + label(463, 135, "gain K typed into C", 11)
b += label(20, 350, "Shared state (sensors, estimate, references, motors) is copied under a binary semaphore in each task.", 11)
b += label(20, 368, "Blue: model-based tooling. Yellow: C tasks. Green: hardware. Dashed: step not completed, or done by hand.", 11)
(MEDIA / "architecture.svg").write_text(svg(900, 385, b, "System architecture",
    "Two paths: a Simulink model path ending in Embedded Coder (not completed) and the Crazyflie, and a hand-written FreeRTOS path with dynamic model, filter, controller, reference and log tasks sharing state through semaphores."))

# ---- MBD pipeline ----
steps = [("Requirement", "hold roll/pitch,", "track pulses", "model", False), ("Model", "Simscape plant,", "linearised state space", "model", False),
         ("Simulate", "pulse + disturbance,", "tune Q and R by hand", "model", False), ("Test", "manual scenarios;", "no Breach/STL script", "todo", True),
         ("Code generation", "Embedded Coder", "not completed", "todo", True), ("Deploy", "flashed controller", "oscillated in roll", "hw", False)]
b = ""
for i, (a, l1, l2, kind, dash) in enumerate(steps):
    x = 15 + i * 148
    b += box(x, 30, 128, 66, [a, l1, l2], kind, dash)
    if i < len(steps) - 1: b += arrow(x + 128, 63, x + 148, 63, dash)
b += f'<path d="M{15 + 5 * 148 + 64} 96 V130 H{15 + 64} V96" fill="none" stroke="{LINE}" stroke-width="1.6" stroke-dasharray="5 4" marker-end="url(#a)"/>'
b += label(450, 126, "next: feed hardware mismatch back into the model (motor/thrust, delay, filter)", 11, "middle")
b += label(15, 160, "Dashed outlines: step not done, or done by hand. See docs/results.md for the evidence behind each box.", 11)
(MEDIA / "mbd_pipeline.svg").write_text(svg(900, 175, b, "Model-based development pipeline",
    "Requirement, model, simulate, test, code generation and deploy, with code generation not completed and testing done manually."))
print("wrote", [p.name for p in MEDIA.glob("*.svg")])
