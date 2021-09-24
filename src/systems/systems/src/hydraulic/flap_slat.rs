use super::brake_circuit::Actuator;
use crate::simulation::{SimulationElement, SimulatorWriter, UpdateContext, Write};
use uom::si::{
    angle::{degree, radian},
    angular_velocity::{radian_per_second, revolution_per_minute},
    f64::*,
    pressure::psi,
    ratio::ratio,
    volume::{cubic_inch, gallon},
    volume_rate::{gallon_per_minute, gallon_per_second},
};

pub struct FlapSlatHydraulicMotor {
    speed: AngularVelocity,
    displacement: Volume,
    current_flow: VolumeRate,

    total_volume_to_actuator: Volume,
    total_volume_returned_to_reservoir: Volume,
}
impl FlapSlatHydraulicMotor {
    // Simulates rpm transients.
    const LOW_PASS_RPM_TRANSIENT_TIME_CONSTANT_S: f64 = 0.5;

    fn new(displacement: Volume) -> Self {
        Self {
            speed: AngularVelocity::new::<radian_per_second>(0.),
            displacement,
            current_flow: VolumeRate::new::<gallon_per_second>(0.),
            total_volume_to_actuator: Volume::new::<gallon>(0.),
            total_volume_returned_to_reservoir: Volume::new::<gallon>(0.),
        }
    }

    fn update_speed(&mut self, speed: AngularVelocity, context: &UpdateContext) {
        // Low pass filter to simulate motors spool up and down. Will ease pressure impact on transients
        self.speed = self.speed
            + (speed - self.speed)
                * (1.
                    - std::f64::consts::E.powf(
                        -context.delta_as_secs_f64() / Self::LOW_PASS_RPM_TRANSIENT_TIME_CONSTANT_S,
                    ));

        if self.speed.get::<revolution_per_minute>() < 20. {
            self.speed = AngularVelocity::new::<revolution_per_minute>(0.);
        }

        self.current_flow = VolumeRate::new::<gallon_per_minute>(
            self.speed.get::<revolution_per_minute>() * self.displacement.get::<cubic_inch>()
                / 231.,
        );

        self.total_volume_to_actuator += self.current_flow * context.delta_as_time();
        self.total_volume_returned_to_reservoir += self.current_flow * context.delta_as_time();
    }

    fn torque_nm(&self, pressure: Pressure) -> f64 {
        0.113 * pressure.get::<psi>() * self.displacement.get::<cubic_inch>() / 6.28
    }

    fn reset_accumulators(&mut self) {
        self.total_volume_to_actuator = Volume::new::<gallon>(0.);
        self.total_volume_returned_to_reservoir = Volume::new::<gallon>(0.);
    }

    fn _flow(&self) -> VolumeRate {
        self.current_flow
    }
}
impl Actuator for FlapSlatHydraulicMotor {
    fn used_volume(&self) -> Volume {
        self.total_volume_to_actuator
    }
    fn reservoir_return(&self) -> Volume {
        self.total_volume_returned_to_reservoir
    }
}

pub struct FlapSlatAssembly {
    position_id: String,

    flap_control_arm_position: Angle,

    max_synchro_gear_position: Angle,
    max_flap_control_arm_position: Angle,

    current_speed: AngularVelocity,
    current_max_speed: AngularVelocity,
    full_pressure_max_speed: AngularVelocity,

    gearbox_ratio: Ratio,
    flap_to_synchro_gear_ratio: Ratio,
    flap_gear_ratio: Ratio,

    left_motor: FlapSlatHydraulicMotor,
    right_motor: FlapSlatHydraulicMotor,
}
impl FlapSlatAssembly {
    const BRAKE_PRESSURE_MIN_TO_ALLOW_MOVEMENT_PSI: f64 = 800.;
    const MAX_CIRCUIT_PRESSURE_PSI: f64 = 3000.;
    const ANGLE_THRESHOLD_FOR_REDUCED_SPEED_DEGREES: f64 = 6.69;
    const ANGULAR_SPEED_LIMIT_WHEN_APROACHING_POSITION_RAD_S: f64 = 0.05;

    pub fn new(
        id: &str,
        motor_displacement: Volume,
        full_pressure_max_speed: AngularVelocity,
        max_synchro_gear_position: Angle,
        synchro_gear_ratio: Ratio,
        gearbox_ratio: Ratio,
        flap_gear_ratio: Ratio,
    ) -> Self {
        let mut new_obj = Self {
            position_id: format!("HYD_{}_POSITION", id),
            flap_control_arm_position: Angle::new::<radian>(0.),
            max_synchro_gear_position,
            max_flap_control_arm_position: Angle::new::<radian>(0.),
            current_speed: AngularVelocity::new::<radian_per_second>(0.),
            current_max_speed: AngularVelocity::new::<radian_per_second>(0.),
            full_pressure_max_speed,
            gearbox_ratio,
            flap_to_synchro_gear_ratio: flap_gear_ratio / synchro_gear_ratio,
            flap_gear_ratio,
            left_motor: FlapSlatHydraulicMotor::new(motor_displacement),
            right_motor: FlapSlatHydraulicMotor::new(motor_displacement),
        };
        new_obj.max_flap_control_arm_position =
            new_obj.synchro_angle_to_flap_angle(max_synchro_gear_position);
        new_obj
    }

    fn synchro_angle_to_flap_angle(&self, synchro_gear_angle: Angle) -> Angle {
        synchro_gear_angle / self.flap_to_synchro_gear_ratio.get::<ratio>()
    }

    pub fn update(
        &mut self,
        synchro_gear_angle_request: Angle,
        left_pressure: Pressure,
        right_pressure: Pressure,
        context: &UpdateContext,
    ) {
        //limiting input value
        let synchro_gear_angle_request = synchro_gear_angle_request
            .max(Angle::new::<radian>(0.))
            .min(self.max_synchro_gear_position);

        self.update_current_max_speed(synchro_gear_angle_request, left_pressure, right_pressure);

        self.update_speed_and_position(synchro_gear_angle_request, context);

        self.update_motor_flows(left_pressure, right_pressure, context);
    }

    fn update_speed_and_position(
        &mut self,
        synchro_gear_angle_request: Angle,
        context: &UpdateContext,
    ) {
        if synchro_gear_angle_request > self.position_feedback() {
            self.flap_control_arm_position += Angle::new::<radian>(
                self.current_max_speed.get::<radian_per_second>() * context.delta_as_secs_f64(),
            );
            self.current_speed = self.current_max_speed;
        } else if synchro_gear_angle_request < self.position_feedback() {
            self.flap_control_arm_position -= Angle::new::<radian>(
                self.current_max_speed.get::<radian_per_second>() * context.delta_as_secs_f64(),
            );
            self.current_speed = -self.current_max_speed;
        } else {
            self.current_speed = AngularVelocity::new::<radian_per_second>(0.);
        }

        if self.current_speed > AngularVelocity::new::<radian_per_second>(0.)
            && synchro_gear_angle_request < self.position_feedback()
            || self.current_speed < AngularVelocity::new::<radian_per_second>(0.)
                && synchro_gear_angle_request > self.position_feedback()
        {
            self.flap_control_arm_position =
                self.synchro_angle_to_flap_angle(synchro_gear_angle_request);
        }

        self.flap_control_arm_position = self
            .flap_control_arm_position
            .max(Angle::new::<radian>(0.))
            .min(self.synchro_angle_to_flap_angle(self.max_synchro_gear_position));
    }

    fn update_current_max_speed(
        &mut self,
        synchro_gear_angle_request: Angle,
        left_pressure: Pressure,
        right_pressure: Pressure,
    ) {
        if left_pressure + right_pressure
            > Pressure::new::<psi>(Self::BRAKE_PRESSURE_MIN_TO_ALLOW_MOVEMENT_PSI)
        {
            self.current_max_speed = AngularVelocity::new::<radian_per_second>(
                self.full_pressure_max_speed.get::<radian_per_second>()
                    * (left_pressure.get::<psi>() + right_pressure.get::<psi>())
                    / (Self::MAX_CIRCUIT_PRESSURE_PSI * 2.),
            );

            if self.is_approaching_requested_position(synchro_gear_angle_request) {
                self.current_max_speed = self.current_max_speed.min(AngularVelocity::new::<
                    radian_per_second,
                >(
                    Self::ANGULAR_SPEED_LIMIT_WHEN_APROACHING_POSITION_RAD_S,
                ))
            }
        } else {
            self.current_max_speed = AngularVelocity::new::<radian_per_second>(0.);
        }
    }

    fn update_motor_flows(
        &mut self,
        left_pressure: Pressure,
        right_pressure: Pressure,
        context: &UpdateContext,
    ) {
        let torque_shaft_speed = AngularVelocity::new::<radian_per_second>(
            self.current_speed.get::<radian_per_second>() * self.flap_gear_ratio.get::<ratio>(),
        );

        let left_torque_nm = self.left_motor.torque_nm(left_pressure);
        let right_torque_nm = self.right_motor.torque_nm(right_pressure);

        let total_motor_torque = left_torque_nm + right_torque_nm;

        let left_torque_ratio = left_torque_nm / total_motor_torque;
        let right_torque_ratio = right_torque_nm / total_motor_torque;

        self.left_motor.update_speed(
            AngularVelocity::new::<radian_per_second>(
                torque_shaft_speed.get::<radian_per_second>()
                    * left_torque_ratio
                    * self.gearbox_ratio.get::<ratio>(),
            ),
            context,
        );
        self.right_motor.update_speed(
            AngularVelocity::new::<radian_per_second>(
                torque_shaft_speed.get::<radian_per_second>()
                    * right_torque_ratio
                    * self.gearbox_ratio.get::<ratio>(),
            ),
            context,
        );
    }

    fn is_approaching_requested_position(&self, synchro_gear_angle_request: Angle) -> bool {
        self.current_speed.get::<radian_per_second>() > 0.
            && synchro_gear_angle_request - self.position_feedback()
                < Angle::new::<degree>(Self::ANGLE_THRESHOLD_FOR_REDUCED_SPEED_DEGREES)
            || self.current_speed.get::<radian_per_second>() < 0.
                && self.position_feedback() - synchro_gear_angle_request
                    < Angle::new::<degree>(Self::ANGLE_THRESHOLD_FOR_REDUCED_SPEED_DEGREES)
    }

    pub fn position_feedback(&self) -> Angle {
        self.flap_control_arm_position * self.flap_to_synchro_gear_ratio.get::<ratio>()
    }

    pub fn left_motor(&mut self) -> &impl Actuator {
        &self.left_motor
    }

    pub fn right_motor(&mut self) -> &impl Actuator {
        &self.right_motor
    }

    // Reset of accumulators will be moved to Actuator trait in other hydraulic overhaul PR
    pub fn reset_left_accumulators(&mut self) {
        self.left_motor.reset_accumulators();
    }

    pub fn reset_right_accumulators(&mut self) {
        self.right_motor.reset_accumulators();
    }
}
impl SimulationElement for FlapSlatAssembly {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(
            &self.position_id,
            self.position_feedback().get::<degree>()
                / self.max_synchro_gear_position.get::<degree>(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::Duration;
    use uom::si::angle::degree;

    use uom::si::{
        acceleration::foot_per_second_squared, length::foot, pressure::psi,
        thermodynamic_temperature::degree_celsius, velocity::knot,
    };

    #[test]
    fn flap_slat_assembly_init() {
        let flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);
        assert!(flap_system.left_motor.current_flow == VolumeRate::new::<gallon_per_minute>(0.));
        assert!(flap_system.right_motor.current_flow == VolumeRate::new::<gallon_per_minute>(0.));

        assert!(flap_system.left_motor.speed == AngularVelocity::new::<revolution_per_minute>(0.));
        assert!(flap_system.right_motor.speed == AngularVelocity::new::<revolution_per_minute>(0.));
    }

    #[test]
    fn flap_slat_assembly_full_pressure() {
        let mut flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);
        assert!(flap_system.current_speed.get::<radian_per_second>() == 0.);

        flap_system.update(
            Angle::new::<degree>(150.),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            &context(Duration::from_millis(2000)),
        );

        assert!(
            (flap_system.current_speed - flap_system.full_pressure_max_speed).abs()
                <= AngularVelocity::new::<radian_per_second>(0.1)
        );

        assert!(
            flap_system.left_motor.speed >= AngularVelocity::new::<revolution_per_minute>(2000.)
                && flap_system.left_motor.speed
                    <= AngularVelocity::new::<revolution_per_minute>(6000.)
        );
        assert!(
            flap_system.right_motor.speed >= AngularVelocity::new::<revolution_per_minute>(2000.)
                && flap_system.right_motor.speed
                    <= AngularVelocity::new::<revolution_per_minute>(6000.)
        );

        assert!(
            flap_system.left_motor._flow() >= VolumeRate::new::<gallon_per_minute>(3.)
                && flap_system.left_motor._flow() <= VolumeRate::new::<gallon_per_minute>(8.)
        );
        assert!(
            flap_system.right_motor._flow() >= VolumeRate::new::<gallon_per_minute>(3.)
                && flap_system.right_motor._flow() <= VolumeRate::new::<gallon_per_minute>(8.)
        );
    }

    #[test]
    fn flap_slat_assembly_half_pressure_right() {
        let mut flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);

        flap_system.update(
            Angle::new::<degree>(150.),
            Pressure::new::<psi>(0.),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            &context(Duration::from_millis(2000)),
        );

        assert!(flap_system.left_motor.speed == AngularVelocity::new::<revolution_per_minute>(0.));
        assert!(
            flap_system.right_motor.speed >= AngularVelocity::new::<revolution_per_minute>(2000.)
                && flap_system.right_motor.speed
                    <= AngularVelocity::new::<revolution_per_minute>(6000.)
        );

        assert!(flap_system.left_motor._flow() == VolumeRate::new::<gallon_per_minute>(0.));
        assert!(
            flap_system.right_motor._flow() >= VolumeRate::new::<gallon_per_minute>(3.)
                && flap_system.right_motor._flow() <= VolumeRate::new::<gallon_per_minute>(8.)
        );
    }

    #[test]
    fn flap_slat_assembly_half_pressure_left() {
        let mut flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);

        flap_system.update(
            Angle::new::<degree>(150.),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            Pressure::new::<psi>(0.),
            &context(Duration::from_millis(2000)),
        );

        assert!(flap_system.right_motor.speed == AngularVelocity::new::<revolution_per_minute>(0.));
        assert!(
            flap_system.left_motor.speed >= AngularVelocity::new::<revolution_per_minute>(2000.)
                && flap_system.left_motor.speed
                    <= AngularVelocity::new::<revolution_per_minute>(6000.)
        );

        assert!(flap_system.right_motor._flow() == VolumeRate::new::<gallon_per_minute>(0.));
        assert!(
            flap_system.left_motor._flow() >= VolumeRate::new::<gallon_per_minute>(3.)
                && flap_system.left_motor._flow() <= VolumeRate::new::<gallon_per_minute>(8.)
        );
    }

    #[test]
    fn flap_slat_assembly_goes_to_req_position() {
        let mut flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);

        flap_system.update(
            Angle::new::<degree>(150.),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            &context(Duration::from_millis(100)),
        );

        let mut time = Duration::from_millis(0);
        let mut last_position = flap_system.position_feedback();

        let synchro_gear_angle_request = Angle::new::<degree>(150.);
        for _ in 0..200 {
            flap_system.update(
                Angle::new::<degree>(150.),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                &context(Duration::from_millis(100)),
            );
            if flap_system.position_feedback() < synchro_gear_angle_request {
                assert!(flap_system.position_feedback() > last_position);
            }
            assert!(flap_system.position_feedback() <= flap_system.max_synchro_gear_position);

            last_position = flap_system.position_feedback();
            println!(
                "Time: {:.1}s  -> Position {:.2}/{} speed{:.3} RightMotor rpm {:.0}",
                time.as_secs_f64(),
                last_position.get::<degree>(),
                synchro_gear_angle_request.get::<degree>(),
                flap_system.current_speed.get::<radian_per_second>(),
                flap_system.right_motor.speed.get::<revolution_per_minute>()
            );
            time += Duration::from_millis(100);
        }

        assert!(flap_system.position_feedback() == synchro_gear_angle_request);
    }

    #[test]
    fn flap_slat_assembly_stops_at_max_position_on_half_pressure_less_than_37_seconds() {
        let mut flap_system = flap_system();

        assert!(flap_system.position_feedback().get::<degree>() == 0.);

        flap_system.update(
            Angle::new::<degree>(800.),
            Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
            Pressure::new::<psi>(0.),
            &context(Duration::from_millis(100)),
        );

        let mut time = Duration::from_millis(0);
        let mut last_position = flap_system.position_feedback();
        for _ in 0..370 {
            flap_system.update(
                Angle::new::<degree>(800.),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                Pressure::new::<psi>(0.),
                &context(Duration::from_millis(100)),
            );

            assert!(flap_system.position_feedback() <= flap_system.max_synchro_gear_position);

            last_position = flap_system.position_feedback();
            println!(
                "Time: {:.1}s  -> Position {:.2}/{}-> speed {:.3}",
                time.as_secs_f64(),
                last_position.get::<degree>(),
                flap_system.max_synchro_gear_position.get::<degree>(),
                flap_system.current_speed.get::<radian_per_second>()
            );
            time += Duration::from_millis(100);
        }
    }

    #[test]
    fn flap_slat_assembly_goes_back_from_max_position() {
        let mut flap_system = flap_system();

        let mut time = Duration::from_millis(0);
        let mut last_position = flap_system.position_feedback();
        for _ in 0..300 {
            flap_system.update(
                Angle::new::<degree>(800.),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                &context(Duration::from_millis(100)),
            );

            assert!(flap_system.position_feedback() <= flap_system.max_synchro_gear_position);

            last_position = flap_system.position_feedback();
            println!(
                "Time: {:.1}s  -> Position {:.2}/{}",
                time.as_secs_f64(),
                last_position.get::<degree>(),
                flap_system.max_synchro_gear_position.get::<degree>()
            );
            time += Duration::from_millis(100);
        }

        assert!(flap_system.position_feedback() == flap_system.max_synchro_gear_position);

        for _ in 0..300 {
            flap_system.update(
                Angle::new::<degree>(-100.),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                Pressure::new::<psi>(FlapSlatAssembly::MAX_CIRCUIT_PRESSURE_PSI),
                &context(Duration::from_millis(100)),
            );

            assert!(flap_system.position_feedback() <= flap_system.max_synchro_gear_position);

            last_position = flap_system.position_feedback();
            println!(
                "Time: {:.1}s  -> Position {:.1}/{}",
                time.as_secs_f64(),
                last_position.get::<degree>(),
                flap_system.max_synchro_gear_position.get::<degree>()
            );
            time += Duration::from_millis(100);
        }

        assert!(flap_system.position_feedback() == Angle::new::<degree>(0.));
    }

    fn context(delta_time: Duration) -> UpdateContext {
        UpdateContext::new(
            delta_time,
            Velocity::new::<knot>(250.),
            Length::new::<foot>(5000.),
            ThermodynamicTemperature::new::<degree_celsius>(25.0),
            true,
            Acceleration::new::<foot_per_second_squared>(0.),
        )
    }

    fn flap_system() -> FlapSlatAssembly {
        FlapSlatAssembly::new(
            "FLAPS",
            Volume::new::<cubic_inch>(0.32),
            AngularVelocity::new::<radian_per_second>(0.11),
            Angle::new::<degree>(251.97),
            Ratio::new::<ratio>(140.),
            Ratio::new::<ratio>(16.632),
            Ratio::new::<ratio>(314.98),
        )
    }
}
