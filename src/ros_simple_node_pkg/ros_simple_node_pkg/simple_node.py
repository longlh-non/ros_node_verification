import queue
import threading
import warnings

import rclpy
from rclpy.node import Node as RosNode
from std_msgs.msg import String

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import networkx as nx
    _VIZ_OK = True
except ImportError:
    _VIZ_OK = False

# ── Visualisation constants ───────────────────────────────────────────────────

# Fixed equilateral-triangle layout for the three states
_POS = {0: (0.0, 1.0), 1: (-0.87, -0.5), 2: (0.87, -0.5)}
_EC = {'a': '#1565C0', 'b': '#B71C1C'}   # edge colours: blue / red
_EC_HI = '#E65100'                         # highlighted edge (last transition)
_NC_CUR = '#66BB6A'                        # current state (green)
_NC_DEF = '#BBDEFB'                        # other states (light blue)


# ── Kripke data model ─────────────────────────────────────────────────────────

class Edge:
    def __init__(self, label, to_node, output=None):
        self.label = label      # input symbol ('a'/'b') or state-change label ('sc=v')
        self.to_node = to_node
        self.output = output


class KripkeNode:
    def __init__(self, node_id, ap=None):
        self.node_id = node_id
        self.atomic_props = set(ap or [])
        self.edges = {}         # label -> Edge

    def add_edge(self, label, to_node, output=None):
        if label not in self.edges:
            self.edges[label] = Edge(label, to_node, output)
            if output:
                self.atomic_props.add(output)

    def get_edge(self, label):
        return self.edges.get(label)


class KripkeStructure:
    def __init__(self):
        self.nodes = {}
        self.initial_node = None
        self.logger = None

    def add_node(self, node_id, ap=None):
        if node_id not in self.nodes:
            self.nodes[node_id] = KripkeNode(node_id, ap)

    def add_edge(self, from_node, label, to_node, output=None):
        self.add_node(from_node)
        self.add_node(to_node)
        self.nodes[from_node].add_edge(label, to_node, output)

    def get_edge(self, current_node, label):
        if current_node not in self.nodes:
            return None
        return self.nodes[current_node].get_edge(label)

    @staticmethod
    def _derive_next_node(current_node, input_val):
        if current_node == 0:
            return 1 if input_val == 'a' else 2
        elif current_node == 1:
            return 0 if input_val == 'a' else 2
        elif current_node == 2:
            return 1 if input_val == 'a' else 0

    def _build_kripke_dfs(self):
        self.initial_node = 0
        self.add_node(0, ap={'in_state_0'})
        self._visited = {0}
        self.logger.info("[Kripke build] Added initial node 0")

    def _dfs(self, current, symbol, derive_output):
        next_node = self._derive_next_node(current, symbol)
        output = derive_output(current, symbol)

        is_new = next_node not in self._visited
        if is_new:
            self._visited.add(next_node)
            self.add_node(next_node, ap={f'in_state_{next_node}'})
            self.logger.info(f"[Kripke build] Discovered new state {next_node}")
            for existing_id in list(self.nodes.keys()):
                self.add_edge(existing_id, f'sc={next_node}', next_node)
                self.add_edge(next_node, f'sc={existing_id}', existing_id)
                self.logger.info(f"[Kripke build]   sc edge: State {existing_id} --[sc={next_node}]--> State {next_node}")

        if self.get_edge(current, symbol) is None:
            out_str = output if output else '∅'
            self.logger.info(f"[Kripke build] Edge: State {current} --[{symbol}]--> State {next_node}  output={out_str}"
                             + (" (back-edge)" if not is_new else ""))
            self.add_edge(current, symbol, next_node, output)

        return next_node

    def print_structure(self, logger):
        logger.info("Kripke Structure:")
        for node_id, node in self.nodes.items():
            state_props = {p for p in node.atomic_props if p.startswith('in_state_')}
            props = ', '.join(sorted(state_props)) if state_props else '∅'
            logger.info(f"  State {node_id}  AP={{{props}}}")
            for label, edge in node.edges.items():
                out_str = f" / {edge.output}" if edge.output else ""
                logger.info(f"    --[{label}{out_str}]--> State {edge.to_node}")


# ── ROS node ──────────────────────────────────────────────────────────────────

class SimpleKripkeNode(RosNode):
    def __init__(self):
        super().__init__('fsm_node')

        self.kripke = KripkeStructure()
        self.kripke.logger = self.get_logger()
        self.kripke._build_kripke_dfs()

        self.current_node = 0
        self._last_edge = None      # (from_id, to_id, label, output) of last transition

        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.output_publisher = self.create_publisher(String, 'output', 10)

        self.create_subscription(String, 'input', self.input_callback, 10)
        self.create_subscription(String, 'state_change', self.state_change_callback, 10)

        # Visualisation setup (main-thread safe; errors disable viz gracefully)
        self._viz_queue: queue.Queue = queue.Queue()
        self._viz_fig = None
        self._viz_ax = None
        if _VIZ_OK:
            self._setup_plot()

        self._publish_state(self.current_node)
        self._enqueue_draw()   # draw initial state (node 0 only)
        self.get_logger().info(f"Simple Kripke Node started. Initial state: {self.current_node}")

    # ── visualisation helpers ─────────────────────────────────────────────────

    def _setup_plot(self):
        try:
            plt.ion()
            self._viz_fig, self._viz_ax = plt.subplots(figsize=(7, 6))
            self._viz_fig.suptitle("Kripke Structure — real-time",
                                   fontsize=13, fontweight='bold')
            plt.tight_layout()
            # Treat non-GUI backend warnings as errors so headless runs fall back cleanly
            with warnings.catch_warnings():
                warnings.filterwarnings('error', category=UserWarning)
                plt.pause(0.001)
        except Exception as exc:
            self.get_logger().warn(f"Visualisation disabled: {exc}")
            if self._viz_fig is not None:
                plt.close(self._viz_fig)
            self._viz_fig = None

    def _enqueue_draw(self):
        """Snapshot current graph state and post it for the main thread to render."""
        edges = [
            {'from': nid, 'to': e.to_node, 'label': e.label, 'output': e.output}
            for nid, node in self.kripke.nodes.items()
            for e in node.edges.values()
        ]
        self._viz_queue.put({
            'nodes': set(self.kripke.nodes.keys()),
            'edges': edges,
            'current': self.current_node,
            'last_edge': self._last_edge,
        })

    def redraw_if_needed(self):
        """Drain the queue and render the latest snapshot. Called from the main thread."""
        if self._viz_fig is None:
            return
        latest = None
        while True:
            try:
                latest = self._viz_queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            try:
                self._draw_graph(latest)
                self._viz_fig.canvas.flush_events()
            except Exception:
                pass  # window was closed or display lost

    def _draw_graph(self, data: dict):
        ax = self._viz_ax
        ax.clear()
        ax.axis('off')

        node_ids = sorted(n for n in data['nodes'] if n in _POS)
        input_edges = [e for e in data['edges'] if not e['label'].startswith('sc=')]
        current = data['current']
        last = data['last_edge']   # (from, to, label, output) | None

        # Title
        if last:
            frm, to, lbl, out = last
            out_str = out if out else '∅'
            ax.set_title(f"Input '{lbl}':  s{frm} → s{to}   output = {out_str}",
                         fontsize=10, pad=8)
        else:
            ax.set_title("Waiting for input…", fontsize=10, pad=8)

        # Build directed graph
        G = nx.DiGraph()
        G.add_nodes_from(node_ids)
        for e in input_edges:
            if e['from'] in _POS and e['to'] in _POS:
                G.add_edge(e['from'], e['to'], label=e['label'], output=e['output'])

        pos = {n: _POS[n] for n in G.nodes()}

        # Node colours
        nc = [_NC_CUR if n == current else _NC_DEF for n in node_ids]
        bc = ['#1B5E20' if n == current else '#0D47A1' for n in node_ids]
        nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=node_ids,
                               node_color=nc, node_size=1500,
                               edgecolors=bc, linewidths=2.5)
        nx.draw_networkx_labels(G, pos, ax=ax,
                                labels={n: f's{n}' for n in node_ids},
                                font_size=13, font_weight='bold')

        # Edges
        if G.edges():
            e_list = list(G.edges())
            e_colors, e_widths = [], []
            for u, v in e_list:
                lbl = G[u][v]['label']
                hi = last and last[0] == u and last[1] == v and last[2] == lbl
                e_colors.append(_EC_HI if hi else _EC.get(lbl, '#6A1B9A'))
                e_widths.append(3.5 if hi else 1.8)
            nx.draw_networkx_edges(G, pos, ax=ax,
                                   edgelist=e_list,
                                   edge_color=e_colors,
                                   width=e_widths,
                                   connectionstyle='arc3,rad=0.25',
                                   arrowsize=20,
                                   min_source_margin=22,
                                   min_target_margin=22)

        # Edge labels (offset toward the arc bow)
        for u, v, d in G.edges(data=True):
            sx, sy = _POS[u]
            ex, ey = _POS[v]
            dx, dy = ex - sx, ey - sy
            ln = (dx ** 2 + dy ** 2) ** 0.5 or 1.0
            mx = (sx + ex) / 2 + ( dy / ln) * 0.18
            my = (sy + ey) / 2 + (-dx / ln) * 0.18
            lbl = d['label']
            out = d.get('output')
            hi = last and last[0] == u and last[1] == v and last[2] == lbl
            color = _EC_HI if hi else _EC.get(lbl, '#6A1B9A')
            ax.text(mx, my, f"{lbl}" + ("/YES" if out == "YES" else ""),
                    fontsize=9, color=color, ha='center', va='center', zorder=5,
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.85, ec='none'))

        # Initial-state entry arrow
        if 0 in pos:
            x, y = pos[0]
            ax.annotate('', xy=(x - 0.22, y), xytext=(x - 0.48, y),
                        arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

        ax.legend(handles=[
            mpatches.Patch(color=_EC['a'],    label="input 'a'"),
            mpatches.Patch(color=_EC['b'],    label="input 'b'"),
            mpatches.Patch(color=_EC_HI,      label='last transition'),
            mpatches.Patch(color=_NC_CUR,     label='current state'),
        ], loc='lower left', fontsize=9, framealpha=0.85)

        ax.set_xlim(-1.4, 1.4)
        ax.set_ylim(-1.0, 1.5)
        self._viz_fig.canvas.draw_idle()

    # ── output function ───────────────────────────────────────────────────────

    def _derive_output(self, current_node, input_val):
        if current_node == 1 and input_val == 'a':
            return 'YES'
        if current_node == 2 and input_val == 'b':
            return 'YES'
        return None

    def _publish_state(self, node):
        msg = String()
        msg.data = str(node)
        self.state_publisher.publish(msg)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def input_callback(self, msg):
        self.get_logger().info(f"{msg}")
        input_val = msg.data.strip()
        prev_node = self.current_node

        next_node = self.kripke._dfs(self.current_node, input_val, self._derive_output)
        output = self._derive_output(prev_node, input_val)
        out_str = output if output else '∅'

        self.get_logger().info(
            f"State {prev_node} --[{input_val}]--> State {next_node}  output={out_str}"
        )

        self._last_edge = (prev_node, next_node, input_val, output)
        self.current_node = next_node
        self._publish_state(self.current_node)

        out_msg = String()
        out_msg.data = out_str
        self.output_publisher.publish(out_msg)

        self._enqueue_draw()

    def state_change_callback(self, msg):
        try:
            v = int(msg.data.strip())
        except ValueError:
            self.get_logger().warn(f"Invalid state_change value: '{msg.data}'")
            return

        edge = self.kripke.get_edge(self.current_node, f'sc={v}')
        if edge is None:
            self.get_logger().warn(f"Unknown state_change target: {v}")
            return

        self.get_logger().info(f"state_change: {self.current_node} -> {edge.to_node}")
        self._last_edge = (self.current_node, edge.to_node, f'sc={v}', None)
        self.current_node = edge.to_node
        self._publish_state(self.current_node)
        self._enqueue_draw()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SimpleKripkeNode()

    if not _VIZ_OK or node._viz_fig is None:
        # No display available — spin normally
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        return

    # Spin ROS in a background thread; keep the main thread for matplotlib
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok() and plt.fignum_exists(node._viz_fig.number):
            node.redraw_if_needed()
            plt.pause(0.05)          # 50 ms tick — processes GUI events
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
