use oort_api::prelude::*;

const BULLET_SPEED: f64 = 1000.0; // m/s

const BOOST_ACCURACY: f64 = 0.005; // radians within which we boost towards a target
const FIRE_ACCURACY: f64 = 0.05; // radians within which we fire at a target

pub struct Ship {
    last_target_vel: Option<Vec2>,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {last_target_vel: None}
    }

    pub fn tick(&mut self) {
        if self.target_is_moving_towards_ship(target(), target_velocity()) {
            self.zero_vel_no_turning();
            self.fire_on_moving_target(target(), target_velocity())
        } else {
            self.move_to_and_fire_on_moving_target(target(), target_velocity());
        }        
    }

    fn target_is_moving_towards_ship(&self, target_pos: Vec2, target_vel: Vec2) -> bool {
        target_vel.dot(target_pos - position()) < 0.0
    }

    fn relative_radial_speed_to_target(&self, target_pos: Vec2, target_vel: Vec2) -> f64 {
        let radial_vector = (position() - target_pos).normalize();
        velocity().dot(radial_vector) - target_vel.dot(radial_vector)
    }

    fn move_to_and_fire_on_moving_target(&mut self, target_pos: Vec2, target_vel: Vec2) {
        let future_bullet_target_pos = self.intercept_coords(target_pos, target_vel, BULLET_SPEED);
        let ship_speed = 150.0; // How fast we can get in 30 ticks/0.5s
        let future_ship_target_pos = self.intercept_coords(target_pos, target_vel, ship_speed);

        draw_diamond(future_bullet_target_pos, 20.0, rgb(255, 0, 0));

        self.goto_aiming_at_target(future_bullet_target_pos);

        if angle_diff(heading(), (future_bullet_target_pos - position()).angle()).abs() < FIRE_ACCURACY {
            fire(0);
        }
    }

    fn fire_on_moving_target(&mut self, target_pos: Vec2, target_vel: Vec2) {
        let future_bullet_target_pos = self.intercept_coords(target_pos, target_vel, BULLET_SPEED);

        draw_diamond(future_bullet_target_pos, 20.0, rgb(255, 255, 0));

        // TODO: a proper algorithm for compensating aim.. hardcoded at 0.01 for now (?!), this doesn't generalise well
        let compensate = 0.01;

        self.turn((future_bullet_target_pos - position()).angle() + compensate);

        if angle_diff(heading(), (future_bullet_target_pos - position()).angle()).abs() < FIRE_ACCURACY {
            fire(0);
        }
    }

    fn intercept_coords(&mut self, target_pos: Vec2, target_vel: Vec2, intercept_speed: f64) -> Vec2 {
        let a: f64 = (target_vel.length() * target_vel.length()) - (intercept_speed * intercept_speed);
        let b: f64 = 2.0 * target_vel.dot(target_pos - position());
        let c: f64 = (target_pos - position()).length() * (target_pos - position()).length();

        let t_to_intercept = (-b - (b * b - 4.0 * a * c).sqrt()) / (2.0 * a);

        let a = if let Some(last_vel) = self.last_target_vel {
            target_vel - last_vel
        } else {
            vec2(0.0, 0.0)
        };
        self.last_target_vel = Some(target_vel);

        target() + t_to_intercept * (target_velocity()  + 0.5 * a * t_to_intercept + 0.008333333333 * a)
    }

    fn turn(&self, target_heading: f64) {
        let theta = angle_diff(heading(), target_heading);
        let omega = angular_velocity();
        let alpha_max = max_angular_acceleration();
        let direction = theta.signum();

        // How much time it would take to stop if we deccelerate by max amount
        let t_to_stop = omega.abs() / alpha_max;
        // How many ticks it takes us to stop if we deccelerate by the max amount
        let ticks_to_stop = t_to_stop / TICK_LENGTH;
        // How fast we're rotating if we accelerate by the max amount for just one tick
        let omega_after_accel = omega + direction * alpha_max * TICK_LENGTH;
        // How much distance we travel if we accelerate for one tick and then deccelerate as much as possible
        let theta_to_stop_after_accel = (omega_after_accel - direction * t_to_stop * 0.5 * alpha_max) * (t_to_stop + TICK_LENGTH);

        if theta.abs() < 0.01 && ticks_to_stop < 0.975 {
            // We're going to stop THIS tick and we're close to the target
            torque(theta - omega);
        } else if theta.signum() == omega.signum() && theta_to_stop_after_accel.abs() > theta.abs() {
            // we've run out of space to accelerate further, so deccelerate
            torque(-1.0 * direction * alpha_max);
        } else {
            torque(1.0 * direction * alpha_max);
        }
    }

    fn goto(&self, target: Vec2) {
        let v0 = velocity();
        let displ = target - position();
        let orth = displ.normalize().rotate(PI / 2.0);
        let v_lateral = orth.dot(v0);

        let mut new_heading = displ.angle();

        debug!("v_lat: {}, thresh: {}", v_lateral, 2.0 * max_lateral_acceleration() * TICK_LENGTH);

        draw_line(position(), position() + orth * v_lateral, rgb(u8::MAX, 0, 200));
        draw_line(position(), position() + displ.normalize() * displ.normalize().dot(v0), rgb(u8::MAX, 100, 0));
        draw_line(position(), target, rgb(0, 0, u8::MAX));

        if v_lateral.abs() > 10.0 * max_lateral_acceleration() * TICK_LENGTH {
            debug!("Overcorrecting!");
            new_heading += -0.5 * v_lateral.signum();
        }
        self.turn(new_heading);

        let radial_acceleration = if {
            let t_left = v0.length() / max_backward_acceleration();
            let distance = v0.length() * t_left - 0.5 * max_backward_acceleration() * (t_left * t_left + TICK_LENGTH * t_left);
            displ.length() <= distance
        } {
            let required_acc = (displ.length() - v0.length() * TICK_LENGTH).abs() / (TICK_LENGTH * TICK_LENGTH);
            deactivate_ability(Ability::Boost);
            if required_acc < max_backward_acceleration() {
                -1.0 * displ.normalize() * required_acc
            } else {
                -1.0 * displ.normalize() * max_backward_acceleration()
            }
        } else {
            if angle_diff(heading(), displ.angle()).abs() < BOOST_ACCURACY {
                activate_ability(Ability::Boost);
            } else {
                deactivate_ability(Ability::Boost);
            }
            displ.normalize() * max_forward_acceleration()
        };

        let lateral_acceleration = -1.0 * v_lateral * orth;

        accelerate(radial_acceleration + lateral_acceleration);
    }

    fn zero_vel_no_turning(&self) {
        accelerate(-1.0 * velocity());
    }

    fn goto_aiming_at_target(&self, target: Vec2) {
        let v0 = velocity();
        let displ = target - position();
        let orth = displ.normalize().rotate(PI / 2.0);
        let v_lateral = orth.dot(v0);

        self.turn(displ.angle());

        let radial_acceleration = if {
            let t_left = v0.length() / max_backward_acceleration();
            let distance = v0.length() * t_left - 0.5 * max_backward_acceleration() * (t_left * t_left + TICK_LENGTH * t_left);
            displ.length() <= distance
        } {
            let required_acc = (displ.length() - v0.length() * TICK_LENGTH).abs() / (TICK_LENGTH * TICK_LENGTH);
            deactivate_ability(Ability::Boost);
            if required_acc < max_backward_acceleration() {
                -1.0 * displ.normalize() * required_acc
            } else {
                -1.0 * displ.normalize() * max_backward_acceleration()
            }
        } else {
            if angle_diff(heading(), displ.angle()).abs() < BOOST_ACCURACY {
                activate_ability(Ability::Boost);
            } else {
                deactivate_ability(Ability::Boost);
            }
            displ.normalize() * max_forward_acceleration()
        };

        let lateral_acceleration = -1.0 * v_lateral * orth;

        accelerate(radial_acceleration + lateral_acceleration);
    }

    fn goto_aiming_at_other(&self, target: Vec2, other: Vec2) {
        let v0 = velocity();
        let displ = target - position();
        let orth = displ.normalize().rotate(PI / 2.0);
        let v_lateral = orth.dot(v0);

        self.turn((other - position()).angle());

        debug!("velocity: {}", v0);

        let radial_acceleration = if {
            let t_left = v0.length() / max_backward_acceleration();
            let distance = v0.length() * t_left - 0.5 * max_backward_acceleration() * (t_left * t_left + TICK_LENGTH * t_left);
            debug!("dist_required: {}", distance);
            displ.length() <= distance
        } {
            let required_acc = (displ.length() - v0.length() * TICK_LENGTH).abs() / (TICK_LENGTH * TICK_LENGTH);
            deactivate_ability(Ability::Boost);
            if required_acc < max_backward_acceleration() {
                -1.0 * displ.normalize() * required_acc
            } else {
                -1.0 * displ.normalize() * max_backward_acceleration()
            }
        } else {
            if angle_diff(heading(), displ.angle()).abs() < BOOST_ACCURACY {
                activate_ability(Ability::Boost);
            } else {
                deactivate_ability(Ability::Boost);
            }
            debug!("displ: {}", displ);
            debug!("suggested radial acc: {}", displ.normalize() * max_forward_acceleration());
            displ.normalize() * max_forward_acceleration()
        };

        let lateral_acceleration = -1.0 * v_lateral * orth;

        accelerate(radial_acceleration + lateral_acceleration);
    }

    fn dash_through(&self, target: Vec2) {
        let displ = target - position();
        self.turn(displ.angle());
        if angle_diff(heading(), displ.angle()).abs() < BOOST_ACCURACY {
            activate_ability(Ability::Boost);
        } else {
            deactivate_ability(Ability::Boost);
        }
        accelerate(displ);
    }   
}