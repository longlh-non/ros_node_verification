#!/usr/bin/env python3
"""
Step-by-step visualisation of the Kripke structure using networkx + matplotlib.
No ROS runtime needed.

Usage:
    python3 visualize_kripke.py

Controls:
    Press any key or click the figure to advance one step.
    Auto-advances every 2 s if no interaction.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx


# ── Pure-Python data model (mirrors simple_node.py, no ROS imports) ───────────

class Edge:
    def __init__(self, label, to_node, output=None):
        self.label = label
        self.to_node = to_node
        self.output = output


class KripkeNode:
    def __init__(self, node_id, ap=None):
        self.node_id = node_id
        self.atomic_props = set(ap or [])
        self.edges = {}

    def add_edge(self, label, to_node, output=None):
        if label not in self.edges:
            self.edges[label] = Edge(label, to_node, output)
            if output:
                self.atomic_props.add(output)

    def get_edge(self, label):
        return self.edges.get(label)


# ── Step-recording builder ────────────────────────────────────────────────────

class StepRecordingKripke:
    """
    Replays the Kripke DFS construction from simple_node.py and records every
    graph-change event as an immutable snapshot so they can be animated.
    """

    def __init__(self):
        self.nodes: dict[int, KripkeNode] = {}
        self._visited: set[int] = set()
        self._steps: list[dict] = []

    # ── snapshot helpers ──────────────────────────────────────────────────────

    def _snapshot(self, description: str, highlight_node=None, highlight_edge=None):
        edges = [
            {'from': nid, 'to': edge.to_node, 'label': edge.label, 'output': edge.output}
            for nid, node in self.nodes.items()
            for edge in node.edges.values()
        ]
        self._steps.append({
            'nodes': set(self.nodes.keys()),
            'edges': edges,
            'highlight_node': highlight_node,
            'highlight_edge': highlight_edge,  # (from_id, to_id, label) | None
            'description': description,
        })

    def _add_node(self, node_id, ap=None):
        if node_id not in self.nodes:
            self.nodes[node_id] = KripkeNode(node_id, ap)

    def _add_edge(self, frm, label, to, output=None):
        self._add_node(frm)
        self._add_node(to)
        self.nodes[frm].add_edge(label, to, output)

    def _get_edge(self, node, label):
        n = self.nodes.get(node)
        return n.get_edge(label) if n else None

    # ── transition / output functions (mirror simple_node.py) ─────────────────

    _TRANSITION = {
        (0, 'a'): 1, (0, 'b'): 2,
        (1, 'a'): 0, (1, 'b'): 2,
        (2, 'a'): 1, (2, 'b'): 0,
    }

    @classmethod
    def _next(cls, state, sym):
        return cls._TRANSITION[(state, sym)]

    @staticmethod
    def _output(state, sym):
        return 'YES' if (state == 1 and sym == 'a') or (state == 2 and sym == 'b') else None

    # ── build (same DFS logic as KripkeStructure._dfs) ────────────────────────

    def build(self):
        self._add_node(0, ap={'in_state_0'})
        self._visited = {0}
        self._snapshot("Initial state: s0", highlight_node=0)

        for current, sym in [(0, 'a'), (0, 'b'), (1, 'a'), (1, 'b'), (2, 'a'), (2, 'b')]:
            self._dfs(current, sym)

    def _dfs(self, current, sym):
        nxt = self._next(current, sym)
        out = self._output(current, sym)

        is_new = nxt not in self._visited
        if is_new:
            self._visited.add(nxt)
            self._add_node(nxt, ap={f'in_state_{nxt}'})
            self._snapshot(f"New state discovered: s{nxt}", highlight_node=nxt)
            # sc (state-change) edges — added silently, filtered out in the visualiser
            for eid in list(self.nodes.keys()):
                self._add_edge(eid, f'sc={nxt}', nxt)
                self._add_edge(nxt, f'sc={eid}', eid)

        if self._get_edge(current, sym) is None:
            out_str = out if out else '∅'
            kind = "new transition" if is_new else "back-edge"
            self._add_edge(current, sym, nxt, out)
            self._snapshot(
                f"s{current} —[{sym} / {out_str}]→ s{nxt}  ({kind})",
                highlight_edge=(current, nxt, sym),
            )


# ── Layout & colour constants ─────────────────────────────────────────────────

# Fixed equilateral-triangle positions for the three states
POS = {0: (0.0, 1.0), 1: (-0.87, -0.5), 2: (0.87, -0.5)}

ECOLOR = {'a': '#1565C0', 'b': '#B71C1C'}   # dark blue / dark red
ECOLOR_HI = '#E65100'                         # highlight orange


# ── Drawing ───────────────────────────────────────────────────────────────────

def _draw(ax, step: dict):
    ax.clear()
    ax.axis('off')
    ax.set_title(step['description'], fontsize=11, pad=10)

    node_ids = sorted(n for n in step['nodes'] if n in POS)
    raw_edges = [e for e in step['edges'] if not e['label'].startswith('sc=')]
    h_node = step['highlight_node']
    h_edge = step['highlight_edge']

    # Build a directed graph (no parallel edges after sc= filter)
    G = nx.DiGraph()
    G.add_nodes_from(node_ids)
    for e in raw_edges:
        if e['from'] in POS and e['to'] in POS:
            G.add_edge(e['from'], e['to'], label=e['label'], output=e['output'])

    pos = {n: POS[n] for n in G.nodes()}

    # ── nodes ─────────────────────────────────────────────────────────────────
    node_colors = ['#FFCC80' if n == h_node else '#BBDEFB' for n in node_ids]
    border_colors = ['#E65100' if n == h_node else '#0D47A1' for n in node_ids]

    nx.draw_networkx_nodes(G, pos, ax=ax,
                           nodelist=node_ids,
                           node_color=node_colors,
                           node_size=1500,
                           edgecolors=border_colors,
                           linewidths=2.5)
    nx.draw_networkx_labels(G, pos, ax=ax,
                            labels={n: f's{n}' for n in node_ids},
                            font_size=13, font_weight='bold')

    # ── edges ─────────────────────────────────────────────────────────────────
    if G.edges():
        e_list = list(G.edges())
        e_colors, e_widths = [], []
        for u, v in e_list:
            lbl = G[u][v]['label']
            hi = h_edge and h_edge[0] == u and h_edge[1] == v and h_edge[2] == lbl
            e_colors.append(ECOLOR_HI if hi else ECOLOR.get(lbl, '#6A1B9A'))
            e_widths.append(3.5 if hi else 1.8)

        nx.draw_networkx_edges(G, pos, ax=ax,
                               edgelist=e_list,
                               edge_color=e_colors,
                               width=e_widths,
                               connectionstyle='arc3,rad=0.25',
                               arrowsize=20,
                               min_source_margin=22,
                               min_target_margin=22)

    # ── edge labels (offset toward the arc bow) ───────────────────────────────
    for u, v, data in G.edges(data=True):
        sx, sy = POS[u]
        ex, ey = POS[v]
        dx, dy = ex - sx, ey - sy
        ln = (dx ** 2 + dy ** 2) ** 0.5 or 1.0
        # Right (clockwise) perpendicular — matches arc3,rad>0 bow direction: (dy, -dx)
        mx = (sx + ex) / 2 + ( dy / ln) * 0.18
        my = (sy + ey) / 2 + (-dx / ln) * 0.18
        lbl = data['label']
        out = data.get('output')
        text = f"{lbl}" + ("/YES" if out else "")
        hi = h_edge and h_edge[0] == u and h_edge[1] == v and h_edge[2] == lbl
        color = ECOLOR_HI if hi else ECOLOR.get(lbl, '#6A1B9A')
        ax.text(mx, my, text, fontsize=9, color=color,
                ha='center', va='center', zorder=5,
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.85, ec='none'))

    # ── initial-state entry arrow ─────────────────────────────────────────────
    if 0 in pos:
        x, y = pos[0]
        ax.annotate('', xy=(x - 0.22, y), xytext=(x - 0.48, y),
                    arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

    # ── step counter ──────────────────────────────────────────────────────────
    total = step.get('_total', '?')
    current = step.get('_index', '?')
    ax.text(1.35, 1.35, f"{current}/{total}",
            fontsize=9, color='grey', ha='right', va='top')

    # ── legend ────────────────────────────────────────────────────────────────
    ax.legend(handles=[
        mpatches.Patch(color=ECOLOR['a'],  label="input 'a'"),
        mpatches.Patch(color=ECOLOR['b'],  label="input 'b'"),
        mpatches.Patch(color=ECOLOR_HI,    label='current step'),
    ], loc='lower left', fontsize=9, framealpha=0.85)

    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.0, 1.5)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    k = StepRecordingKripke()
    k.build()

    steps = k._steps
    total = len(steps)
    for i, s in enumerate(steps):
        s['_index'] = i + 1
        s['_total'] = total

    idx = [0]

    fig, ax = plt.subplots(figsize=(7, 6.5))
    fig.suptitle("Kripke Structure — step-by-step construction",
                 fontsize=13, fontweight='bold')

    def advance(_=None):
        if idx[0] < len(steps):
            _draw(ax, steps[idx[0]])
            idx[0] += 1
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('key_press_event', advance)
    fig.canvas.mpl_connect('button_press_event', advance)

    advance()  # draw step 0 immediately

    try:
        timer = fig.canvas.new_timer(interval=2000)
        timer.add_callback(advance)
        timer.start()
    except Exception:
        pass  # some backends don't support timers

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
