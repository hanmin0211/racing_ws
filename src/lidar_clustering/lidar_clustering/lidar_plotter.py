import matplotlib.pyplot as plt


class LidarPlotter:
    def __init__(
        self,
        on_close=None,
        on_reset=None,
        x_min=0.0,
        x_max=16.0,
        y_min=-8.0,
        y_max=8.0,
    ):
        self.on_close_callback = on_close
        self.on_reset_callback = on_reset

        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.fig.canvas.mpl_connect('close_event', self._on_close)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    def _on_close(self, event):
        if self.on_close_callback:
            self.on_close_callback()

    def _on_key(self, event):
        if (
            event.key
            and event.key.lower() == 'r'
            and self.on_reset_callback
        ):
            self.on_reset_callback()

    def draw(self, points, tracked_clusters, status='LiDAR'):
        self.ax.clear()

        if points:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            self.ax.scatter(xs, ys, s=8, label='LiDAR points')

        for obstacle in tracked_clusters:
            track_id = obstacle.get('track_id', -1)
            cluster = obstacle.get('points', [])

            if cluster:
                xs = [p[0] for p in cluster]
                ys = [p[1] for p in cluster]
                self.ax.scatter(xs, ys, s=28, label=f'ID {track_id}')

            cx = obstacle.get('center_x', 0.0)
            cy = obstacle.get('center_y', 0.0)

            self.ax.scatter(cx, cy, marker='x', s=70)

            self.ax.annotate(
                (
                    f'ID {track_id}\n'
                    f'X={cx:.2f} m\n'
                    f'Y={cy:.2f} m\n'
                    f'D={obstacle.get("center_distance", 0.0):.2f} m'
                ),
                (cx, cy),
                xytext=(5, 5),
                textcoords='offset points',
                fontsize=8,
            )

        self.ax.scatter(0.0, 0.0, marker='^', s=100, label='LiDAR')
        self.ax.axhline(0.0, linewidth=1)
        self.ax.axvline(0.0, linewidth=1)
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('X forward (m)')
        self.ax.set_ylabel('Y left/right (m)')
        self.ax.set_title(status)
        self.ax.grid(True)

        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc='upper right', fontsize=7)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def close(self):
        plt.close(self.fig)
