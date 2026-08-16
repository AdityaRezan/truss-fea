import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from truss import Truss2D, warren_bridge

RESULTS = os.path.join(os.path.dirname(__file__), "..", "results")


def validate_single_bar():
    nodes = [(0, 0), (2, 0)]
    tr = Truss2D(nodes, [(0, 1)], E=200e9, A=0.001)
    tr.fix(0)
    tr.fix(1, x=False, y=True)
    P = 10e3
    tr.load(1, fx=P)
    tr.solve()
    analytic = P * 2.0 / (200e9 * 0.001)
    fem = tr.u[2]
    err = abs(fem - analytic) / analytic
    print(f"single bar validation: FEM {fem:.6e} m  analytic {analytic:.6e} m  "
          f"rel err {err:.2e}")
    assert err < 1e-10
    return err


def plot_truss(ax, tr, scale=300.0, title=""):
    smax = np.abs(tr.stresses).max()
    for e, (i, j) in enumerate(tr.elements):
        xi, yi = tr.nodes[i]
        xj, yj = tr.nodes[j]
        stress = tr.stresses[e]
        color = plt.cm.coolwarm(0.5 + 0.5 * stress / smax)
        lw = 1.0 + 3.5 * abs(stress) / smax
        ax.plot([xi, xj], [yi, yj], color="lightgray", lw=0.8, ls="--")
        dxi, dyi = tr.u[2 * i] * scale, tr.u[2 * i + 1] * scale
        dxj, dyj = tr.u[2 * j] * scale, tr.u[2 * j + 1] * scale
        ax.plot([xi + dxi, xj + dxj], [yi + dyi, yj + dyj], color=color, lw=lw)
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)


def main():
    os.makedirs(RESULTS, exist_ok=True)
    print("=" * 70)
    print("Truss FEA - direct stiffness method")
    print("=" * 70)
    validate_single_bar()

    nodes, elements = warren_bridge(n_panels=6, span=24.0, height=3.0)
    tr = Truss2D(nodes, elements, E=200e9, A=0.002)
    tr.fix(0)
    tr.fix(6, x=False, y=True)
    truck_kn = 60.0
    for n in range(1, 6):
        tr.load(n, fy=-truck_kn * 1e3)
    tr.solve()

    max_disp = np.abs(tr.u).max() * 1000
    max_tension = tr.stresses.max() / 1e6
    max_compression = tr.stresses.min() / 1e6
    print(f"\nWarren bridge, 24 m span, 5 x {truck_kn:.0f} kN deck loads:")
    print(f"  max displacement : {max_disp:7.2f} mm")
    print(f"  max tension      : {max_tension:7.1f} MPa")
    print(f"  max compression  : {max_compression:7.1f} MPa")
    print(f"  steel weight     : {tr.total_weight()/1e3:7.2f} tonnes")
    yield_mpa = 250.0
    sf = yield_mpa / max(max_tension, -max_compression)
    print(f"  safety factor    : {sf:7.2f} (vs 250 MPa yield)")

    heights = np.linspace(1.2, 6.0, 25)
    weights, max_stresses, disps = [], [], []
    for h in heights:
        n2, e2 = warren_bridge(n_panels=6, span=24.0, height=h)
        t2 = Truss2D(n2, e2, E=200e9, A=0.002)
        t2.fix(0)
        t2.fix(6, x=False, y=True)
        for n in range(1, 6):
            t2.load(n, fy=-truck_kn * 1e3)
        t2.solve()
        weights.append(t2.total_weight() / 1e3)
        max_stresses.append(np.abs(t2.stresses).max() / 1e6)
        disps.append(np.abs(t2.u).max() * 1000)
    best = int(np.argmin(max_stresses))
    print(f"\nheight sweep: min peak stress {max_stresses[best]:.1f} MPa "
          f"at height {heights[best]:.2f} m")

    fig, axes = plt.subplots(2, 1, figsize=(13, 9),
                             gridspec_kw={"height_ratios": [1.3, 1]})
    plot_truss(axes[0], tr, scale=300,
               title="Warren bridge under deck loading - deformation x300, "
                     "color = stress (red tension, blue compression)")
    ax = axes[1]
    ax2 = ax.twinx()
    ax.plot(heights, max_stresses, "o-", color="tab:red", label="peak stress")
    ax2.plot(heights, weights, "s-", color="tab:gray", label="steel weight")
    ax.axvline(heights[best], ls="--", color="tab:red", alpha=0.5)
    ax.set_xlabel("truss height [m]")
    ax.set_ylabel("peak |stress| [MPa]", color="tab:red")
    ax2.set_ylabel("weight [tonnes]", color="tab:gray")
    ax.set_title("Design sweep: truss height vs peak stress and weight")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "bridge_analysis.png"), dpi=130)
    print("saved results/bridge_analysis.png")


if __name__ == "__main__":
    main()
