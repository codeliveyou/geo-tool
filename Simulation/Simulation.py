#!/usr/bin/env python3
"""
A Simple Proportional Navigation Simulation with Actuator Dynamics

Setup:
  1. Click on the canvas:
      - First click sets the rocket’s starting position (red circle)
      - Second click sets the target’s starting position (blue circle)
  2. Enter the following parameters:
      - Rocket Speed (m/s) and Rocket Angle (deg)
      - Target Speed (m/s) and Target Angle (deg)
      - PN Navigation Constant (dimensionless)
      - Maximum allowed Vertical Acceleration (m/s²)
      - Actuator Time Constant (sec) -- time required for the launcher to achieve the ideal PN command.
  3. Enter the Scale: number of meters per pixel.
  4. When "Start Simulation" is pressed:
      - A dashed line is drawn between the rocket and target showing their initial separation.
      - The target moves in a constant straight-line path while the rocket uses a PN law.
      - The rocket’s vertical (lateral) acceleration is not applied instantly but follows a
        first-order dynamic defined by the actuator time constant.
      - The trajectories are drawn (rocket: red, target: blue) and the current and maximum
        vertical acceleration values are displayed.
      - Every 1 second a purple LOS line is drawn between the current positions of the rocket and target.
      - When the rocket comes within 10 meters (or less than 2 m in this code) of the target, a collision is marked in green.
  5. Press "Reset Simulation" to clear the canvas and start over.
  
Note: The canvas is expanded.
"""

import tkinter as tk
import math

class SimulationApp:
    def __init__(self, master):
        self.master = master
        master.title("Simple Proportional Navigation Simulation")
        self.canvas_width = 1200
        self.canvas_height = 800
        self.canvas = tk.Canvas(master, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4, padx=5, pady=5)
        self.info_label = tk.Label(master, text="Click: first sets Rocket position, second sets Target position.")
        self.info_label.grid(row=1, column=0, columnspan=4, sticky="w", padx=5)
        tk.Label(master, text="Rocket Speed (m/s):").grid(row=2, column=0, sticky=tk.E)
        self.rocket_speed_entry = tk.Entry(master, width=10)
        self.rocket_speed_entry.insert(0, "25")
        self.rocket_speed_entry.grid(row=2, column=1, sticky=tk.W)
        tk.Label(master, text="Rocket Angle (deg):").grid(row=2, column=2, sticky=tk.E)
        self.rocket_angle_entry = tk.Entry(master, width=10)
        self.rocket_angle_entry.insert(0, "270")
        self.rocket_angle_entry.grid(row=2, column=3, sticky=tk.W)
        tk.Label(master, text="Target Speed (m/s):").grid(row=3, column=0, sticky=tk.E)
        self.target_speed_entry = tk.Entry(master, width=10)
        self.target_speed_entry.insert(0, "15")
        self.target_speed_entry.grid(row=3, column=1, sticky=tk.W)
        tk.Label(master, text="Target Angle (deg):").grid(row=3, column=2, sticky=tk.E)
        self.target_angle_entry = tk.Entry(master, width=10)
        self.target_angle_entry.insert(0, "0")
        self.target_angle_entry.grid(row=3, column=3, sticky=tk.W)
        tk.Label(master, text="PN Navigation Constant:").grid(row=4, column=0, sticky=tk.E)
        self.pn_constant_entry = tk.Entry(master, width=10)
        self.pn_constant_entry.insert(0, "3.0")
        self.pn_constant_entry.grid(row=4, column=1, sticky=tk.W)
        tk.Label(master, text="Max Vertical Acc (m/s²):").grid(row=4, column=2, sticky=tk.E)
        self.max_allowed_acc_entry = tk.Entry(master, width=10)
        self.max_allowed_acc_entry.insert(0, "5")
        self.max_allowed_acc_entry.grid(row=4, column=3, sticky=tk.W)
        tk.Label(master, text="Actuator Time Constant (sec):").grid(row=5, column=0, sticky=tk.E)
        self.time_constant_entry = tk.Entry(master, width=10)
        self.time_constant_entry.insert(0, "2.0")
        self.time_constant_entry.grid(row=5, column=1, sticky=tk.W)
        tk.Label(master, text="Scale (m/px):").grid(row=6, column=0, sticky=tk.E)
        self.scale_entry = tk.Entry(master, width=10)
        self.scale_entry.insert(0, "0.1")
        self.scale_entry.grid(row=6, column=1, sticky=tk.W)
        self.start_button = tk.Button(master, text="Start Simulation", command=self.start_simulation)
        self.start_button.grid(row=7, column=0, columnspan=2, pady=5)
        self.reset_button = tk.Button(master, text="Reset Simulation", command=self.reset_simulation)
        self.reset_button.grid(row=7, column=2, columnspan=2, pady=5)
        self.curr_acc_label = tk.Label(master, text="Current Vertical Acc: 0.0 m/s²")
        self.curr_acc_label.grid(row=8, column=0, columnspan=2, sticky="w", padx=5)
        self.max_acc_label = tk.Label(master, text="Max Vertical Acc: 0.0 m/s²")
        self.max_acc_label.grid(row=8, column=2, columnspan=2, sticky="w", padx=5)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.reset_simulation_variables()
        self.dt = 0.02

    def reset_simulation_variables(self):
        self.rocket_set = False
        self.target_set = False
        self.rocket_start_px = None
        self.target_start_px = None
        self.simulation_running = False
        self.simulation_time = 0.0
        self.rocket_traj = []
        self.target_traj = []
        self.initial_distance_line = None
        self.initial_distance_text = None
        self.a_actual = 0.0
        self.next_los_time = 1.0

    def on_canvas_click(self, event):
        if not self.rocket_set:
            self.rocket_start_px = (event.x, event.y)
            self.rocket_set = True
            r = 4
            self.canvas.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="red")
            self.info_label.config(text="Rocket position set. Click to set Target position.")
        elif not self.target_set:
            self.target_start_px = (event.x, event.y)
            self.target_set = True
            r = 4
            self.canvas.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="blue")
            self.info_label.config(text="Both positions set. Press 'Start Simulation' to begin.")
        else:
            pass

    def start_simulation(self):
        if not (self.rocket_set and self.target_set):
            self.info_label.config(text="Please click to set both Rocket and Target positions!")
            return
        try:
            self.rocket_speed = float(self.rocket_speed_entry.get())
            self.rocket_angle = math.radians(float(self.rocket_angle_entry.get()))
            self.target_speed = float(self.target_speed_entry.get())
            self.target_angle = math.radians(float(self.target_angle_entry.get()))
            self.pn_constant = float(self.pn_constant_entry.get())
            self.max_allowed_acc = float(self.max_allowed_acc_entry.get())
            self.actuator_time_constant = float(self.time_constant_entry.get())
            self.scale = float(self.scale_entry.get())
        except ValueError:
            self.info_label.config(text="Invalid parameters! Please enter numeric values.")
            return
        self.rocket_pos = [self.rocket_start_px[0] * self.scale, self.rocket_start_px[1] * self.scale]
        self.target_pos = [self.target_start_px[0] * self.scale, self.target_start_px[1] * self.scale]
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        initial_distance = math.hypot(dx, dy)
        self.initial_distance_line = self.canvas.create_line(
            self.rocket_pos[0] / self.scale, self.rocket_pos[1] / self.scale,
            self.target_pos[0] / self.scale, self.target_pos[1] / self.scale,
            dash=(4,2), fill="gray"
        )
        mid_x = (self.rocket_pos[0] + self.target_pos[0]) / (2 * self.scale)
        mid_y = (self.rocket_pos[1] + self.target_pos[1]) / (2 * self.scale)
        self.initial_distance_text = self.canvas.create_text(
            mid_x, mid_y-10,
            text=f"Initial Distance: {initial_distance:.1f} m",
            fill="gray", font=("Arial", 10, "italic")
        )
        self.rocket_vel = [self.rocket_speed * math.cos(self.rocket_angle),
                           self.rocket_speed * math.sin(self.rocket_angle)]
        self.target_vel = [self.target_speed * math.cos(self.target_angle),
                           self.target_speed * math.sin(self.target_angle)]
        self.prev_LOS = math.atan2(dy, dx)
        self.rocket_traj = [tuple(self.rocket_pos)]
        self.target_traj = [tuple(self.target_pos)]
        self.a_actual = 0.0
        self.next_los_time = 1.0
        self.simulation_running = True
        self.simulation_time = 0.0
        self.info_label.config(text="Simulation running...")
        self.start_button.config(state=tk.DISABLED)
        self.update_simulation()

    def update_simulation(self):
        if not self.simulation_running:
            return
        dt = self.dt
        self.simulation_time += dt
        self.target_pos[0] += self.target_vel[0] * dt
        self.target_pos[1] += self.target_vel[1] * dt
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        current_LOS = math.atan2(dy, dx)
        def angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff
        LOS_rate = angle_diff(current_LOS, self.prev_LOS) / dt
        rel_vel = [self.target_vel[0] - self.rocket_vel[0],
                   self.target_vel[1] - self.rocket_vel[1]]
        r = math.hypot(dx, dy)
        rhat = [dx/r, dy/r] if r != 0 else [0, 0]
        closing_velocity = - (rel_vel[0]*rhat[0] + rel_vel[1]*rhat[1])
        a_command_desired = self.pn_constant * closing_velocity * LOS_rate
        if abs(a_command_desired) > self.max_allowed_acc:
            a_command_desired = self.max_allowed_acc * (1 if a_command_desired > 0 else -1)
        self.a_actual += dt * (a_command_desired - self.a_actual) / self.actuator_time_constant
        current_vertical_acc = abs(self.a_actual)
        try:
            current_max = float(self.max_acc_label.cget("text").split()[-2])
        except:
            current_max = 0.0
        max_encountered = max(current_vertical_acc, current_max)
        self.curr_acc_label.config(text=f"Current Vertical Acc: {current_vertical_acc:.2f} m/s²")
        self.max_acc_label.config(text=f"Max Vertical Acc: {max_encountered:.2f} m/s²")
        turn_rate = self.a_actual / self.rocket_speed
        self.rocket_angle += turn_rate * dt
        self.rocket_vel[0] = self.rocket_speed * math.cos(self.rocket_angle)
        self.rocket_vel[1] = self.rocket_speed * math.sin(self.rocket_angle)
        self.rocket_pos[0] += self.rocket_vel[0] * dt
        self.rocket_pos[1] += self.rocket_vel[1] * dt
        self.rocket_traj.append((self.rocket_pos[0], self.rocket_pos[1]))
        self.target_traj.append((self.target_pos[0], self.target_pos[1]))
        if len(self.rocket_traj) >= 2:
            self.canvas.create_line(self.rocket_traj[-2][0] / self.scale, self.rocket_traj[-2][1] / self.scale,
                                    self.rocket_traj[-1][0] / self.scale, self.rocket_traj[-1][1] / self.scale,
                                    fill="red")
        if len(self.target_traj) >= 2:
            self.canvas.create_line(self.target_traj[-2][0] / self.scale, self.target_traj[-2][1] / self.scale,
                                    self.target_traj[-1][0] / self.scale, self.target_traj[-1][1] / self.scale,
                                    fill="blue")
        if self.simulation_time >= self.next_los_time:
            self.canvas.create_line(self.rocket_pos[0] / self.scale, self.rocket_pos[1] / self.scale,
                                    self.target_pos[0] / self.scale, self.target_pos[1] / self.scale,
                                    fill="purple", dash=(2,2))
            self.next_los_time += 1.0
        self.prev_LOS = current_LOS
        if r < 2:
            self.info_label.config(text="Impact! Rocket hit the target.")
            self.simulation_running = False
            r_collide = 6
            self.canvas.create_oval((self.rocket_pos[0]-r_collide)/self.scale, (self.rocket_pos[1]-r_collide)/self.scale,
                                    (self.rocket_pos[0]+r_collide)/self.scale, (self.rocket_pos[1]+r_collide)/self.scale,
                                    fill="green")
            self.start_button.config(state=tk.NORMAL)
            return
        if self.simulation_time > 30:
            self.info_label.config(text="Simulation time exceeded. Stopping simulation.")
            self.simulation_running = False
            self.start_button.config(state=tk.NORMAL)
            return
        self.master.after(int(dt*1000), self.update_simulation)

    def reset_simulation(self):
        self.simulation_running = False
        self.canvas.delete("all")
        self.reset_simulation_variables()
        self.info_label.config(text="Click: first sets Rocket position, second sets Target position.")
        self.curr_acc_label.config(text="Current Vertical Acc: 0.0 m/s²")
        self.max_acc_label.config(text="Max Vertical Acc: 0.0 m/s²")
        self.start_button.config(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    app = SimulationApp(root)
    root.mainloop()