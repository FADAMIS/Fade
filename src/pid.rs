#[derive(Debug, Clone, Copy)]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// Internal controller for a single axis
struct PidController {
    // Gains
    kp: f32,
    ki: f32,
    kd: f32,
    kf: f32,

    // State
    integral: f32,
    prev_act_pos: f32,
    prev_derivative: f32,
    prev_set_pos: f32,
    prev_feedforward: f32,

    // Configuration
    integral_limit: f32,
    max_output: f32,
    d_filter_weight: f32,
    ff_filter_weight: f32,
}

// Main public struct that holds all axis controllers
pub struct PidControllers {
    roll: PidController,
    pitch: PidController,
    yaw: PidController,
}

impl PidController {
    fn new() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            kf: 0.0,
            integral: 0.0,
            prev_act_pos: 0.0,
            prev_derivative: 0.0,
            prev_set_pos: 0.0,
            prev_feedforward: 0.0,
            integral_limit: 1000.0,
            max_output: 500.0, // dps
            d_filter_weight: 0.5,
            ff_filter_weight: 0.5,
        }
    }

    fn set_gains(&mut self, p: i32, i: i32, d: i32, f: i32) {
        self.kp = p as f32 / 10.0;
        self.ki = i as f32 / 100.0;
        self.kd = d as f32 / 100.0;
        self.kf = f as f32 / 10.0;
    }

    fn calculate(&mut self, set_pos: f32, act_pos: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }

        let mut pos_err = set_pos - act_pos;
        if pos_err > 180.0 {
            pos_err -= 360.0;
        } else if pos_err < -180.0 {
            pos_err += 360.0;
        }

        let p_term = self.kp * pos_err;

        self.integral += pos_err * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        let mut derivative_angle = act_pos - self.prev_act_pos;
        if derivative_angle > 180.0 {
            derivative_angle -= 360.0;
        } else if derivative_angle < -180.0 {
            derivative_angle += 360.0;
        }
        let derivative = derivative_angle / dt;
        let filtered_derivative =
            self.prev_derivative * (1.0 - self.d_filter_weight) + derivative * self.d_filter_weight;
        self.prev_derivative = filtered_derivative;
        let d_term = self.kd * filtered_derivative;

        let setpoint_derivative = (set_pos - self.prev_set_pos) / dt;
        let filtered_feedforward = self.prev_feedforward * (1.0 - self.ff_filter_weight)
            + setpoint_derivative * self.ff_filter_weight;
        self.prev_feedforward = filtered_feedforward;
        let f_term = self.kf * filtered_feedforward;

        self.prev_act_pos = act_pos;
        self.prev_set_pos = set_pos;

        let output = p_term + i_term - d_term + f_term;
        output.clamp(-self.max_output, self.max_output)
    }
}

impl PidControllers {
    pub fn new() -> Self {
        Self {
            roll: PidController::new(),
            pitch: PidController::new(),
            yaw: PidController::new(),
        }
    }

    pub fn update_gains(&mut self, axis: Axis, p: i32, i: i32, d: i32, f: i32) {
        let controller = match axis {
            Axis::Roll => &mut self.roll,
            Axis::Pitch => &mut self.pitch,
            Axis::Yaw => &mut self.yaw,
        };
        controller.set_gains(p, i, d, f);
    }

    pub fn calculate(&mut self, setpoints: &Vector3, actuals: &Vector3, dt: f32) -> Vector3 {
        Vector3 {
            x: self.roll.calculate(setpoints.x, actuals.x, dt),
            y: self.pitch.calculate(setpoints.y, actuals.y, dt),
            z: self.yaw.calculate(setpoints.z, actuals.z, dt),
        }
    }
}
