#!/usr/bin/env python3
"""
A Simple Proportional Navigation Simulation with Actuator Dynamics

Setup:
  1. Click on the canvas:
      - First click sets the rocket’s starting marker (red circle)
      - Second click sets the target’s starting marker (blue circle)
  2. Enter the following parameters:
      - Rocket Speed (m/s) and Rocket Angle (deg)
      - Target Speed (m/s) and Target Angle (deg)
      - PN Navigation Constant (dimensionless)
      - Rocket Max Vertical Acc (m/s²)
      - Actuator Time Constant (sec) – time for the rocket to achieve the ideal PN command.
      - Target Max Vertical Acc (m/s²) – if zero, target moves straight; if nonzero, target will change its direction randomly (without exceeding this value)
  3. Enter the Scale (m/px).
  4. When the target marker is set, the simulation automatically starts:
      - A dashed gray line is drawn showing the initial separation.
      - The target moves along a constant straight-line path (or turns randomly if its max vertical acc > 0)
        while the rocket uses a PN law.
      - The rocket’s vertical acceleration is applied with first-order dynamics.
      - Trajectories are drawn (rocket: red, target: blue) and LOS lines (purple dashed every 1 sec) are drawn.
      - When the rocket comes within 2 m of the target, a collision is marked in green.
      - After impact, the simulation automatically restarts (simulation drawings are cleared, but markers remain).
  5. The "Stop Simulation" button halts the simulation.
  6. The "Reset Simulation" button clears everything – including markers – so new ones can be set.
  
Note: The canvas is expanded.
"""

import tkinter as tk
import math
import random

class SimulationApp:
    def __init__(self, master):
        self.master = master
        master.title("Proportional Navigation Guidance Simulation")
        self.canvas_width = 1200
        self.canvas_height = 800
        self.canvas = tk.Canvas(master, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4, padx=5, pady=5)

        self.info_label = tk.Label(master, text="Click: first sets Rocket marker, second sets Target marker.")
        self.info_label.grid(row=1, column=0, columnspan=4, sticky="w", padx=5)

        # Row 2: Rocket parameters.
        tk.Label(master, text="Rocket Speed (m/s):").grid(row=2, column=0, sticky=tk.E)
        self.rocket_speed_entry = tk.Entry(master, width=10)
        self.rocket_speed_entry.insert(0, "25")
        self.rocket_speed_entry.grid(row=2, column=1, sticky=tk.W)
        
        tk.Label(master, text="Rocket Angle (deg):").grid(row=2, column=2, sticky=tk.E)
        self.rocket_angle_entry = tk.Entry(master, width=10)
        self.rocket_angle_entry.insert(0, "270")
        self.rocket_angle_entry.grid(row=2, column=3, sticky=tk.W)

        # Row 3: Target parameters.
        tk.Label(master, text="Target Speed (m/s):").grid(row=3, column=0, sticky=tk.E)
        self.target_speed_entry = tk.Entry(master, width=10)
        self.target_speed_entry.insert(0, "15")
        self.target_speed_entry.grid(row=3, column=1, sticky=tk.W)
        
        tk.Label(master, text="Target Angle (deg):").grid(row=3, column=2, sticky=tk.E)
        self.target_angle_entry = tk.Entry(master, width=10)
        self.target_angle_entry.insert(0, "0")
        self.target_angle_entry.grid(row=3, column=3, sticky=tk.W)

        # Row 4: PN constant & Rocket max vertical acc.
        tk.Label(master, text="PN Navigation Constant:").grid(row=4, column=0, sticky=tk.E)
        self.pn_constant_entry = tk.Entry(master, width=10)
        self.pn_constant_entry.insert(0, "3.0")
        self.pn_constant_entry.grid(row=4, column=1, sticky=tk.W)
        
        tk.Label(master, text="Rocket Max Vertical Acc (m/s²):").grid(row=4, column=2, sticky=tk.E)
        self.max_allowed_acc_entry = tk.Entry(master, width=10)
        self.max_allowed_acc_entry.insert(0, "5")
        self.max_allowed_acc_entry.grid(row=4, column=3, sticky=tk.W)

        # Row 5: Actuator time constant.
        tk.Label(master, text="Actuator Time Constant (sec):").grid(row=5, column=0, sticky=tk.E)
        self.time_constant_entry = tk.Entry(master, width=10)
        self.time_constant_entry.insert(0, "2.0")
        self.time_constant_entry.grid(row=5, column=1, sticky=tk.W)
        
        # Row 6: Target max vertical acc.
        tk.Label(master, text="Target Max Vertical Acc (m/s²):").grid(row=6, column=0, sticky=tk.E)
        self.target_max_acc_entry = tk.Entry(master, width=10)
        self.target_max_acc_entry.insert(0, "0")  # 0 means constant path
        self.target_max_acc_entry.grid(row=6, column=1, sticky=tk.W)
        
        # Row 7: Scale.
        tk.Label(master, text="Scale (m/px):").grid(row=7, column=0, sticky=tk.E)
        self.scale_entry = tk.Entry(master, width=10)
        self.scale_entry.insert(0, "0.5")
        self.scale_entry.grid(row=7, column=1, sticky=tk.W)

        # Row 8: Buttons.
        self.stop_button = tk.Button(master, text="Stop Simulation", command=self.stop_simulation)
        self.stop_button.grid(row=8, column=0, columnspan=2, pady=5)
        self.reset_button = tk.Button(master, text="Reset Simulation", command=self.full_reset)
        self.reset_button.grid(row=8, column=2, columnspan=2, pady=5)

        # Row 9: Acceleration display.
        self.curr_acc_label = tk.Label(master, text="Current Vertical Acc: 0.0 m/s²")
        self.curr_acc_label.grid(row=9, column=0, columnspan=2, sticky="w", padx=5)
        self.max_acc_label = tk.Label(master, text="Max Vertical Acc: 0.0 m/s²")
        self.max_acc_label.grid(row=9, column=2, columnspan=2, sticky="w", padx=5)

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.reset_simulation_variables(full=True)
        self.dt = 0.02

        # For target turning dynamics:
        self.target_turn_rate = 0.0
        self.last_target_update = 0.0

    def reset_simulation_variables(self, full=False):
        if full:
            self.canvas.delete("all")
            self.rocket_set = False
            self.target_set = False
            self.rocket_start_px = None
            self.target_start_px = None
        self.simulation_running = False
        self.simulation_time = 0.0
        self.rocket_traj = []
        self.target_traj = []
        self.canvas.delete("sim")
        self.initial_distance_line = None
        self.initial_distance_text = None
        self.a_actual = 0.0
        self.next_los_time = 1.0
        self.last_target_update = 0.0
        self.target_turn_rate = 0.0

    def on_canvas_click(self, event):
        if not self.rocket_set:
            self.rocket_start_px = (event.x, event.y)
            self.rocket_set = True
            r = 4
            self.canvas.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="red", tags="marker")
            self.info_label.config(text="Rocket marker set. Click to set Target marker.")
        elif not self.target_set:
            self.target_start_px = (event.x, event.y)
            self.target_set = True
            r = 4
            self.canvas.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="blue", tags="marker")
            self.info_label.config(text="Target marker set. Starting simulation...")
            self.start_simulation()
        else:
            pass

    def start_simulation(self):
        try:
            self.rocket_speed = float(self.rocket_speed_entry.get())
            self.rocket_angle = math.radians(float(self.rocket_angle_entry.get()))
            self.target_speed = float(self.target_speed_entry.get())
            self.target_angle = math.radians(float(self.target_angle_entry.get()))
            self.pn_constant = float(self.pn_constant_entry.get())
            self.max_allowed_acc = float(self.max_allowed_acc_entry.get())  # Rocket max vertical
            self.actuator_time_constant = float(self.time_constant_entry.get())
            self.target_max_acc = float(self.target_max_acc_entry.get())
            self.scale = float(self.scale_entry.get())
        except ValueError:
            self.info_label.config(text="Invalid parameters! Enter numeric values.")
            return

        self.rocket_pos = [self.rocket_start_px[0] * self.scale, self.rocket_start_px[1] * self.scale]
        self.target_pos = [self.target_start_px[0] * self.scale, self.target_start_px[1] * self.scale]
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        initial_distance = math.hypot(dx, dy)
        self.initial_distance_line = self.canvas.create_line(
            self.rocket_pos[0] / self.scale, self.rocket_pos[1] / self.scale,
            self.target_pos[0] / self.scale, self.target_pos[1] / self.scale,
            dash=(4,2), fill="gray", tags="sim"
        )
        mid_x = (self.rocket_pos[0] + self.target_pos[0]) / (2 * self.scale)
        mid_y = (self.rocket_pos[1] + self.target_pos[1]) / (2 * self.scale)
        self.initial_distance_text = self.canvas.create_text(
            mid_x, mid_y-10,
            text=f"Initial Distance: {initial_distance:.1f} m",
            fill="gray", font=("Arial", 10, "italic"), tags="sim"
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
        self.last_target_update = 0.0
        self.target_turn_rate = 0.0
        self.simulation_running = True
        self.simulation_time = 0.0
        self.info_label.config(text="Simulation running...")
        self.stop_button.config(state=tk.NORMAL)
        self.update_simulation()

    def update_simulation(self):
        if not self.simulation_running:
            return
        dt = self.dt
        self.simulation_time += dt

        # --- Update target dynamics ---
        # If target_max_acc > 0, then randomly update its turn rate every second.
        if self.target_max_acc > 0:
            if self.simulation_time - self.last_target_update >= 1.0:
                # Desired turn rate (rad/s) is random, limited by (max_acc/target_speed)
                self.target_turn_rate = random.uniform(-self.target_max_acc/self.target_speed,
                                                         self.target_max_acc/self.target_speed)
                self.last_target_update = self.simulation_time
            self.target_angle += self.target_turn_rate * dt
            self.target_vel[0] = self.target_speed * math.cos(self.target_angle)
            self.target_vel[1] = self.target_speed * math.sin(self.target_angle)
        # Else, target moves with constant direction

        # Update target position
        self.target_pos[0] += self.target_vel[0] * dt
        self.target_pos[1] += self.target_vel[1] * dt

        # --- Update rocket dynamics (PN guidance) ---
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        current_LOS = math.atan2(dy, dx)
        def angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2*math.pi
            while diff < -math.pi:
                diff += 2*math.pi
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
            self.canvas.create_line(
                self.rocket_traj[-2][0] / self.scale, self.rocket_traj[-2][1] / self.scale,
                self.rocket_traj[-1][0] / self.scale, self.rocket_traj[-1][1] / self.scale,
                fill="red", tags="sim"
            )
        if len(self.target_traj) >= 2:
            self.canvas.create_line(
                self.target_traj[-2][0] / self.scale, self.target_traj[-2][1] / self.scale,
                self.target_traj[-1][0] / self.scale, self.target_traj[-1][1] / self.scale,
                fill="blue", tags="sim"
            )
        if self.simulation_time >= self.next_los_time:
            self.canvas.create_line(
                self.rocket_pos[0] / self.scale, self.rocket_pos[1] / self.scale,
                self.target_pos[0] / self.scale, self.target_pos[1] / self.scale,
                fill="purple", dash=(2,2), tags="sim"
            )
            self.next_los_time += 1.0

        self.prev_LOS = current_LOS

        if r <= 2:
            self.info_label.config(text="Impact! Rocket hit the target. Restarting simulation...")
            self.simulation_running = False
            r_collide = 2
            self.canvas.create_oval(
                (self.rocket_pos[0]-r_collide) / self.scale, (self.rocket_pos[1]-r_collide) / self.scale,
                (self.rocket_pos[0]+r_collide) / self.scale, (self.rocket_pos[1]+r_collide) / self.scale,
                fill="green", tags="sim"
            )
            self.stop_button.config(state=tk.DISABLED)
            self.master.after(2000, self.restart_simulation)
            return

        if self.simulation_time > 30:
            self.info_label.config(text="Simulation time exceeded. Stopping simulation.")
            self.simulation_running = False
            self.stop_button.config(state=tk.DISABLED)
            return

        self.master.after(int(dt*1000), self.update_simulation)

    def restart_simulation(self):
        # Restart simulation using the same markers.
        self.canvas.delete("sim")
        self.rocket_pos = [self.rocket_start_px[0] * self.scale, self.rocket_start_px[1] * self.scale]
        self.target_pos = [self.target_start_px[0] * self.scale, self.target_start_px[1] * self.scale]
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        self.rocket_traj = [tuple(self.rocket_pos)]
        self.target_traj = [tuple(self.target_pos)]
        self.a_actual = 0.0
        self.simulation_time = 0.0
        self.next_los_time = 1.0
        self.last_target_update = 0.0
        self.target_turn_rate = 0.0
        self.prev_LOS = math.atan2(dy, dx)
        self.info_label.config(text="Simulation restarting...")
        self.simulation_running = True
        self.stop_button.config(state=tk.NORMAL)
        self.update_simulation()

    def stop_simulation(self):
        self.simulation_running = False
        self.info_label.config(text="Simulation stopped by user.")
        self.stop_button.config(state=tk.DISABLED)

    def full_reset(self):
        self.canvas.delete("all")
        self.reset_simulation_variables(full=True)
        self.info_label.config(text="Click: first sets Rocket marker, second sets Target marker.")
        self.curr_acc_label.config(text="Current Vertical Acc: 0.0 m/s²")
        self.max_acc_label.config(text="Max Vertical Acc: 0.0 m/s²")
        self.stop_button.config(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    app = SimulationApp(root)
    root.mainloop()