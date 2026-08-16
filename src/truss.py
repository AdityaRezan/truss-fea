import numpy as np


class Truss2D:
    def __init__(self, nodes, elements, E=200e9, A=0.002):
        self.nodes = np.asarray(nodes, dtype=float)
        self.elements = list(elements)
        self.E = E
        self.A = A
        self.n_dof = 2 * len(self.nodes)
        self.fixed = set()
        self.loads = np.zeros(self.n_dof)

    def fix(self, node, x=True, y=True):
        if x:
            self.fixed.add(2 * node)
        if y:
            self.fixed.add(2 * node + 1)

    def load(self, node, fx=0.0, fy=0.0):
        self.loads[2 * node] += fx
        self.loads[2 * node + 1] += fy

    def element_stiffness(self, e):
        i, j = self.elements[e]
        xi, yi = self.nodes[i]
        xj, yj = self.nodes[j]
        L = np.hypot(xj - xi, yj - yi)
        c = (xj - xi) / L
        s = (yj - yi) / L
        k = self.E * self.A / L
        m = np.array([[c * c, c * s, -c * c, -c * s],
                      [c * s, s * s, -c * s, -s * s],
                      [-c * c, -c * s, c * c, c * s],
                      [-c * s, -s * s, c * s, s * s]])
        return k * m, L, c, s

    def assemble(self):
        K = np.zeros((self.n_dof, self.n_dof))
        for e in range(len(self.elements)):
            ke, _, _, _ = self.element_stiffness(e)
            i, j = self.elements[e]
            dofs = [2 * i, 2 * i + 1, 2 * j, 2 * j + 1]
            for a in range(4):
                for b in range(4):
                    K[dofs[a], dofs[b]] += ke[a, b]
        return K

    def solve(self):
        K = self.assemble()
        free = [d for d in range(self.n_dof) if d not in self.fixed]
        Kff = K[np.ix_(free, free)]
        u = np.zeros(self.n_dof)
        u[free] = np.linalg.solve(Kff, self.loads[free])
        self.u = u
        reactions = K @ u - self.loads
        self.reactions = reactions

        self.forces = np.zeros(len(self.elements))
        self.stresses = np.zeros(len(self.elements))
        for e, (i, j) in enumerate(self.elements):
            _, L, c, s = self.element_stiffness(e)
            ue = np.array([u[2 * i], u[2 * i + 1], u[2 * j], u[2 * j + 1]])
            elong = (-c * ue[0] - s * ue[1] + c * ue[2] + s * ue[3])
            self.forces[e] = self.E * self.A / L * elong
            self.stresses[e] = self.forces[e] / self.A
        return u

    def total_weight(self, density=7850.0):
        w = 0.0
        for e in range(len(self.elements)):
            _, L, _, _ = self.element_stiffness(e)
            w += density * self.A * L
        return w


def warren_bridge(n_panels=6, span=24.0, height=3.0):
    nodes = []
    for i in range(n_panels + 1):
        nodes.append((i * span / n_panels, 0.0))
    for i in range(n_panels):
        nodes.append((i * span / n_panels + span / (2 * n_panels), height))
    elements = []
    for i in range(n_panels):
        elements.append((i, i + 1))
    top = lambda i: n_panels + 1 + i
    for i in range(n_panels - 1):
        elements.append((top(i), top(i + 1)))
    for i in range(n_panels):
        elements.append((i, top(i)))
        elements.append((top(i), i + 1))
    return nodes, elements
