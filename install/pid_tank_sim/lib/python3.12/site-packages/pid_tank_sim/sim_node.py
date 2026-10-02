# pid_drone_alt_sim/pid_drone_alt_sim/sim_node.py
import tkinter as tk
from tkinter import simpledialog, messagebox
import time
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PIDDroneAltSimNode(Node):
    """
    Simple 1D drone altitude (z) PID control sim:
        m * d2z = thrust - m*g - c * dz
    Control: thrust = PID(z_set - z), saturated to [0, thrust_max]
    GUI: shows a drone in a vertical scene with setpoint line and a live plot.
    """

    def __init__(self):
        super().__init__('pid_drone_alt_sim')

        # Parameters 
        self.declare_parameter('dt', 0.02)                 # sim step (s)
        self.declare_parameter('realtime_factor', 1.0)      # simulated seconds per wall second
        self.declare_parameter('mass', 1.0)                # kg
        self.declare_parameter('gravity', 9.81)            # m/s^2
        self.declare_parameter('drag_coeff', 0.3)          # N per (m/s)
        self.declare_parameter('thrust_max', 20.0)         # N
        self.declare_parameter('max_time', 12.0)           # s
        self.declare_parameter('max_altitude', 50.0)       # m (UI ceiling & plot range)
        self.declare_parameter('setpoint', 20.0)           # m

        # PID gains (optional via params; otherwise prompted via Tk dialog)
        self.declare_parameter('Kp', None)
        self.declare_parameter('Ki', None)
        self.declare_parameter('Kd', None)

        p = self.get_parameters([
            'dt', 'realtime_factor', 'mass', 'gravity', 'drag_coeff', 'thrust_max', 'max_time',
            'max_altitude', 'setpoint', 'Kp', 'Ki', 'Kd'
        ])

        self.dt           = p[0].get_parameter_value().double_value
        self.realtime_factor = p[1].get_parameter_value().double_value
        self.mass         = p[2].get_parameter_value().double_value
        self.g            = p[3].get_parameter_value().double_value
        self.c_drag       = p[4].get_parameter_value().double_value
        self.thrust_max   = p[5].get_parameter_value().double_value
        self.max_time     = p[6].get_parameter_value().double_value
        self.max_alt      = p[7].get_parameter_value().double_value
        self.setpoint     = p[8].get_parameter_value().double_value
        self.Kp           = p[9].get_parameter_value().double_value if p[9].get_parameter_value().type != 0 else None
        self.Ki           = p[10].get_parameter_value().double_value if p[10].get_parameter_value().type != 0 else None
        self.Kd           = p[11].get_parameter_value().double_value if p[11].get_parameter_value().type != 0 else None

        # Validate parameters
        if self.dt <= 0 or self.realtime_factor <= 0 or self.max_time <= 0:
            raise ValueError('dt, realtime_factor, and max_time must be greater than zero')
        if self.mass <= 0 or self.max_alt <= 0:
            raise ValueError('mass and max_altitude must be greater than zero')

        # State 
        self.z = 0.0        # altitude (m)
        self.vz = 0.0       # vertical velocity (m/s)
        self.sim_time = 0.0
        self.thrust_cmd = 0.0
        self.integral = 0.0
        self.previous_error = 0.0

        # light anti-windup to keep it stable while tuning
        self.integral_min = -50.0
        self.integral_max = 50.0

        self.time_data = [0.0]
        self.alt_data  = [0.0]
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.next_plot_time = 0.0
        self.simulation_running = True

        # ROS Publisher 
        self.alt_pub = self.create_publisher(Float32, 'altitude', 10)

        # Tkinter UI 
        self.root = tk.Tk()
        self.root.title("PID Drone Altitude Simulation (ROS 2)")
        self.root.geometry("1080x640")

        # Prompt for PID if not provided as ROS params
        if self.Kp is None or self.Ki is None or self.Kd is None:
            if not self.get_pid_values():
                self.get_logger().info("PID input cancelled. Exiting.")
                self.root.destroy()
                return

        # Left: Scene canvas (drone + ground + setpoint)
        self.scene = tk.Canvas(self.root, width=460, height=540, bg="#e6f3ff")
        self.scene.pack(side=tk.LEFT, padx=20, pady=20)

        # Draw ground
        self.ground_y = 500
        self.scene.create_rectangle(0, self.ground_y, 460, 540, fill="#6aa84f", outline="")

        # Vertical scale (0 at ground, max_alt at top)
        self.scene.create_rectangle(60, 80, 380, 500, outline="#444", width=2)
        self.scene.create_text(40, 500, text="0 m", anchor="e")
        self.scene.create_text(40, 80, text=f"{int(self.max_alt)} m", anchor="e")

        # Drone body (simple oval) + thrust flame
        self.drone = self.scene.create_oval(200, 200, 240, 240, fill="#333", outline="")
        self.flame = self.scene.create_polygon(0, 0, 0, 0, 0, 0, fill="#ff9900", outline="")

        # Setpoint line
        sp_y = self.alt_to_canvas_y(self.setpoint)
        self.setpoint_line = self.scene.create_line(60, sp_y, 380, sp_y, fill="red", dash=(4, 3), width=2)
        self.setpoint_label = self.scene.create_text(390, sp_y, text="Setpoint", anchor="w",
                                                     fill="red", font=("Arial", 10, "bold"))

        # Right: Matplotlib figure (Altitude vs Time)
        self.fig, self.ax = plt.subplots(figsize=(5.8, 3.5))
        self.ax.set_xlim(0, self.max_time)
        self.ax.set_ylim(0, self.max_alt)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Altitude (m)")
        self.ax.set_title("Altitude vs Time")
        (self.line,) = self.ax.plot(self.time_data, self.alt_data, "b-", label="Altitude")
        self.ax.axhline(y=self.setpoint, color="r", linestyle="--", label="Setpoint")
        self.ax.legend()
        self.canvas_graph = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_graph.get_tk_widget().pack(side=tk.RIGHT, padx=20, pady=20)

        # Close hook -> shutdown ROS cleanly
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # Start the update loop
        self.last_wall_time = time.perf_counter()
        self.root.after(16, self.update_system)

    # Helpers 
    def alt_to_canvas_y(self, z_m):
        """Map altitude (m) to canvas Y (px). 0 m -> ground_y; max_alt -> near 80 px."""
        z_clamped = max(0.0, min(z_m, self.max_alt))
        usable_px = (self.ground_y - 80)  # top border is 80 px
        return self.ground_y - (z_clamped / self.max_alt) * usable_px

    def get_pid_values(self):
        """Pop simpledialogs to get PID gains."""
        try:
            self.Kp = simpledialog.askfloat("Enter Kp", "Enter Proportional Gain (Kp):", minvalue=0, maxvalue=50)
            self.Ki = simpledialog.askfloat("Enter Ki", "Enter Integral Gain (Ki):", minvalue=0, maxvalue=10)
            self.Kd = simpledialog.askfloat("Enter Kd", "Enter Derivative Gain (Kd):", minvalue=0, maxvalue=20)
        except Exception as e:
            messagebox.showerror("Error", f"Error reading PID values: {e}")
            return False
        if self.Kp is None or self.Ki is None or self.Kd is None:
            return False
        return True

    def step_simulation(self, step_dt):
        """Advance the fixed-step PID simulation and publish the resulting altitude."""
        # PID Control
        error = self.setpoint - self.z
        self.integral += error * step_dt
        # anti-windup clamp
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))
        derivative = (error - self.previous_error) / step_dt

        self.thrust_cmd = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # type: ignore
        # Thrust cannot be negative; saturate at [0, thrust_max]
        self.thrust_cmd = max(0.0, min(self.thrust_cmd, self.thrust_max))
        self.previous_error = error

        # Plant Dynamics 
        # m * a = T - m g - c_drag * v
        acc = (self.thrust_cmd - self.mass * self.g - self.c_drag * self.vz) / self.mass
        self.vz += acc * step_dt
        self.z  += self.vz * step_dt

        # Bound altitude and simple bounce off ground (zero velocity if hits ground)
        if self.z < 0.0:
            self.z = 0.0
            if self.vz < 0:
                self.vz = 0.0
        if self.z > self.max_alt:
            self.z = self.max_alt
            if self.vz > 0:
                self.vz = 0.0

        self.sim_time += step_dt
        self.time_data.append(self.sim_time)
        self.alt_data.append(self.z)
        if len(self.time_data) > 2000:
            del self.time_data[:-2000]
            del self.alt_data[:-2000]

        msg = Float32()
        msg.data = float(self.z)
        self.alt_pub.publish(msg)

    def update_scene(self):
        """Render the current simulation state in the Tk canvas."""
        # Update UI (drone position + flame)
        cy = self.alt_to_canvas_y(self.z)
        # drone is a 40x40 oval; center at (220, cy)
        x0, y0, x1, y1 = 200, cy - 20, 240, cy + 20
        self.scene.coords(self.drone, x0, y0, x1, y1)

        # flame polygon under the drone; length proportional to thrust
        flame_len = 5 + 25 * (self.thrust_cmd / max(1e-6, self.thrust_max))
        # triangle points: bottom center, bottom-left, bottom-right
        self.scene.coords(self.flame,
                          220, cy + 20,        # tip under center
                          212, cy + 20 + flame_len,
                          228, cy + 20 + flame_len)
        # hide flame if very low thrust
        if self.thrust_cmd < 0.2:
            self.scene.itemconfig(self.flame, state="hidden")
        else:
            self.scene.itemconfig(self.flame, state="normal")

        # Move setpoint line if setpoint changed externally
        sp_y = self.alt_to_canvas_y(self.setpoint)
        self.scene.coords(self.setpoint_line, 60, sp_y, 380, sp_y)
        self.scene.coords(self.setpoint_label, 390, sp_y)

    def update_plot(self):
        """Refresh the plot at a lower rate than the physics simulation."""
        self.line.set_data(self.time_data, self.alt_data)
        self.canvas_graph.draw_idle()

    def update_system(self):
        """Run fixed-step physics, then update the UI and process ROS callbacks."""
        now = time.perf_counter()
        elapsed = now - self.last_wall_time
        self.last_wall_time = now
        self.accumulator += elapsed * self.realtime_factor

        while self.simulation_running and self.sim_time < self.max_time:
            step_dt = min(self.dt, self.max_time - self.sim_time)
            if self.accumulator < step_dt:
                break
            self.step_simulation(step_dt)
            self.accumulator -= step_dt

        if self.simulation_running:
            self.update_scene()
            if self.sim_time >= self.next_plot_time or self.sim_time >= self.max_time:
                self.update_plot()
                self.next_plot_time = self.sim_time + 0.1
            if self.sim_time >= self.max_time:
                self.simulation_running = False
                self.get_logger().info("Simulation complete. Final result is displayed.")

        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            self.root.after(16, self.update_system)

    def on_close(self):
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass


def main():
    rclpy.init()
    node = PIDDroneAltSimNode()
    try:
        node.root.mainloop()
    except Exception:
        pass
    finally:
        node.on_close()


if __name__ == "__main__":
    main()