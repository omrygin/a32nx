use systems::simulation::{
    Read, SimulationElement, SimulationElementVisitor, SimulatorReader, SimulatorWriter,
    UpdateContext, Write,
};

use uom::si::{angle::degree, f64::*, velocity::knot};

//The different flaps configurations
#[derive(Debug, Copy, Clone, PartialEq)]
enum FlapsConf {
    Conf0 = 0,
    Conf1 = 1,
    Conf1F = 2,
    Conf2 = 3,
    Conf3 = 4,
    ConfFull = 5,
}

impl From<usize> for FlapsConf {
    fn from(value: usize) -> Self {
        match value {
            0 => FlapsConf::Conf0,
            1 => FlapsConf::Conf1,
            2 => FlapsConf::Conf1F,
            3 => FlapsConf::Conf2,
            4 => FlapsConf::Conf3,
            _ => FlapsConf::ConfFull,
        }
    }
}

//A struct to read the handle position
struct FlapsHandle {
    handle_position: usize,
    old_handle_position: usize,
}

impl FlapsHandle {
    fn new() -> Self {
        Self {
            handle_position: 0,
            old_handle_position: 0,
        }
    }

    fn signal_new_position(&self) -> Option<(usize, usize)> {
        println!("{}, {}",self.handle_position , self.old_handle_position );
        if self.handle_position == self.old_handle_position {
            match self.handle_position {
                1 => Some((1, 1)),
                _ => None,
            }
        } else {
            Some((self.old_handle_position, self.handle_position))
        }
    }

    fn equilibrate_old_and_current(&mut self) {
        self.old_handle_position = self.handle_position;
    }
}

impl SimulationElement for FlapsHandle {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.handle_position = reader.read("FLAPS_HANDLE_INDEX");
    }
}

//This is the basis of what will become
//the SFCC. For now it just applies the simple
//flaps logic implemented before.
pub struct SlatFlapControlComputer {
    flaps_demanded_angle: Angle,
    slats_demanded_angle: Angle,
    flaps_conf: FlapsConf,
    indicated_airspeed: Velocity,
}

impl SlatFlapControlComputer {
    const EQUAL_ANGLE_DELTA_DEGREE: f64 = 0.01;
    const HANDLE_ONE_CONF_AIRSPEED_THRESHOLD_KNOTS: f64 = 100.;
    const CONF1F_TO_CONF1_AIRSPEED_THRESHOLD_KNOTS: f64 = 210.;

    fn new() -> Self {
        Self {
            flaps_demanded_angle: Angle::new::<degree>(0.),
            slats_demanded_angle: Angle::new::<degree>(0.),
            flaps_conf: FlapsConf::Conf0,
            indicated_airspeed: Velocity::new::<knot>(0.),
        }
    }

    fn set_target_flaps_angle(&mut self, angle: Angle) {
        self.flaps_demanded_angle = angle;
    }

    fn set_target_slats_angle(&mut self, angle: Angle) {
        self.slats_demanded_angle = angle;
    }

    fn target_flaps_angle_from_state(flap_state: FlapsConf) -> Angle {
        match flap_state {
            FlapsConf::Conf0 => Angle::new::<degree>(0.),
            FlapsConf::Conf1 => Angle::new::<degree>(0.),
            FlapsConf::Conf1F => Angle::new::<degree>(10.),
            FlapsConf::Conf2 => Angle::new::<degree>(15.),
            FlapsConf::Conf3 => Angle::new::<degree>(20.),
            FlapsConf::ConfFull => Angle::new::<degree>(40.),
        }
    }

    fn target_slats_angle_from_state(flap_state: FlapsConf) -> Angle {
        match flap_state {
            FlapsConf::Conf0 => Angle::new::<degree>(0.),
            FlapsConf::Conf1 => Angle::new::<degree>(18.),
            FlapsConf::Conf1F => Angle::new::<degree>(18.),
            FlapsConf::Conf2 => Angle::new::<degree>(22.),
            FlapsConf::Conf3 => Angle::new::<degree>(22.),
            FlapsConf::ConfFull => Angle::new::<degree>(27.),
        }
    }

    fn generate_configuration(
        &self,
        handle_transition: Option<(usize, usize)>,
    ) -> Option<FlapsConf> {
        if let Some((from, to)) = handle_transition {
            match from {
                0 => match to {
                    1 => {
                        if self.indicated_airspeed.get::<knot>()
                            <= Self::HANDLE_ONE_CONF_AIRSPEED_THRESHOLD_KNOTS
                        {
                            Some(FlapsConf::Conf1F)
                        } else {
                            Some(FlapsConf::Conf1)
                        }
                    }
                    0 => Some(FlapsConf::from(0)),
                    _ => Some(FlapsConf::from(to + 1)),
                },
                1 => match to {
                    1 => {
                        if self.indicated_airspeed.get::<knot>()
                            > Self::CONF1F_TO_CONF1_AIRSPEED_THRESHOLD_KNOTS
                        {
                            Some(FlapsConf::Conf1)
                        } else {
                            None
                        }
                    }
                    0 => Some(FlapsConf::from(0)),
                    _ => Some(FlapsConf::from(to + 1)),
                },
                _ => match to {
                    1 => {
                        if self.indicated_airspeed.get::<knot>()
                            <= Self::CONF1F_TO_CONF1_AIRSPEED_THRESHOLD_KNOTS
                        {
                            Some(FlapsConf::Conf1F)
                        } else {
                            Some(FlapsConf::Conf1)
                        }
                    }
                    0 => Some(FlapsConf::from(0)),
                    _ => Some(FlapsConf::from(to + 1)),
                },
            }
        } else {
            None
        }
    }

    pub fn update(&mut self, context: &UpdateContext, handle_transition: Option<(usize, usize)>) {
        self.indicated_airspeed = context.indicated_airspeed();

        if let Some(new_config) = self.generate_configuration(handle_transition) {
            self.flaps_conf = new_config;
            println!("NEW CONFIG {:?}", new_config);
        }

        //If the system is not pressurized, remain in the same configuration.

        //Update target angle based on handle position
        self.set_target_flaps_angle(Self::target_flaps_angle_from_state(self.flaps_conf));
        self.set_target_slats_angle(Self::target_slats_angle_from_state(self.flaps_conf));
    }

    pub fn signal_flap_movement(&self, position_feedback: Angle) -> Option<Angle> {
        if (self.flaps_demanded_angle - position_feedback)
            .get::<degree>()
            .abs()
            > Self::EQUAL_ANGLE_DELTA_DEGREE
        {
            Some(self.flaps_demanded_angle)
        } else {
            None
        }
    }

    fn signal_slat_movement(&self, position_feedback: Angle) -> Option<Angle> {
        if (self.slats_demanded_angle - position_feedback)
            .get::<degree>()
            .abs()
            > Self::EQUAL_ANGLE_DELTA_DEGREE
        {
            Some(self.slats_demanded_angle)
        } else {
            None
        }
    }
}

impl SimulationElement for SlatFlapControlComputer {
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(
            "LEFT_FLAPS_TARGET_ANGLE",
            self.flaps_demanded_angle.get::<degree>(),
        );
        writer.write(
            "RIGHT_FLAPS_TARGET_ANGLE",
            self.flaps_demanded_angle.get::<degree>(),
        );

        writer.write(
            "LEFT_SLATS_TARGET_ANGLE",
            self.slats_demanded_angle.get::<degree>(),
        );
        writer.write(
            "RIGHT_SLATS_TARGET_ANGLE",
            self.slats_demanded_angle.get::<degree>(),
        );

        writer.write(
            "FLAPS_CONF_HANDLE_INDEX_HELPER",
            self.flaps_conf as u8 as f64,
        );
    }
}

pub struct SlatFlapControlComplex {
    sfcc: SlatFlapControlComputer,
    flaps_handle: FlapsHandle,
}

impl SlatFlapControlComplex {
    pub fn new() -> Self {
        Self {
            sfcc: SlatFlapControlComputer::new(),
            flaps_handle: FlapsHandle::new(),
        }
    }

    pub fn update(&mut self, context: &UpdateContext) {
        self.sfcc.update(context, self.flaps_handle.signal_new_position());
        self.flaps_handle.equilibrate_old_and_current();
    }

    pub fn sfcc(&self) -> &SlatFlapControlComputer {
        &self.sfcc
    }
}

impl SimulationElement for SlatFlapControlComplex {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where
        Self: Sized,
    {
        self.flaps_handle.accept(visitor);
        self.sfcc.accept(visitor);
        visitor.visit(self);
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use std::time::Duration;
//     use systems::simulation::test::TestBed;
//     use systems::simulation::{test::SimulationTestBed, Aircraft};

//     struct A320FlapsTestAircraft {
//         slat_flap_complex: SlatFlapComplex,
//     }

//     impl A320FlapsTestAircraft {
//         fn new() -> Self {
//             Self {
//                 slat_flap_complex: SlatFlapComplex::new(),
//             }
//         }
//     }

//     impl Aircraft for A320FlapsTestAircraft {
//         fn update_after_power_distribution(&mut self, context: &UpdateContext) {
//             self.slat_flap_complex.update(context);
//         }
//     }

//     impl SimulationElement for A320FlapsTestAircraft {
//         fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
//             self.slat_flap_complex.accept(visitor);
//             visitor.visit(self);
//         }
//     }

//     struct A320FlapsTestBed {
//         test_bed: SimulationTestBed<A320FlapsTestAircraft>,
//     }

//     impl A320FlapsTestBed {
//         const HYD_TIME_STEP_MILI: u64 = 100;
//         fn new() -> Self {
//             Self {
//                 test_bed: SimulationTestBed::new(|_a| A320FlapsTestAircraft::new()),
//             }
//         }

//         fn run_one_tick(mut self) -> Self {
//             self.test_bed
//                 .run_with_delta(Duration::from_millis(Self::HYD_TIME_STEP_MILI));
//             self
//         }

//         fn run_waiting_for(mut self, delta: Duration) -> Self {
//             self.test_bed.run_multiple_frames(delta);
//             self
//         }

//         fn set_flaps_handle_position(mut self, pos: usize) -> Self {
//             self.write("FLAPS_HANDLE_INDEX", pos as f64);
//             self
//         }

//         fn read_flaps_handle_position(&mut self) -> f64 {
//             self.read("FLAPS_HANDLE_INDEX")
//         }

//         fn set_indicated_airspeed(mut self, indicated_airspeed: f64) -> Self {
//             self.write("AIRSPEED INDICATED", indicated_airspeed);
//             self
//         }

//         fn set_hyd_pressure(mut self) -> Self {
//             self.write("HYD_GREEN_PRESSURE", 2500.);
//             self
//         }

//         fn get_flaps_demanded_angle(&self) -> f64 {
//             self.query(|a| {
//                 a.slat_flap_complex
//                     .sfcc
//                     .flaps_demanded_angle
//                     .get::<degree>()
//             })
//         }

//         fn get_slats_demanded_angle(&self) -> f64 {
//             self.query(|a| {
//                 a.slat_flap_complex
//                     .sfcc
//                     .slats_demanded_angle
//                     .get::<degree>()
//             })
//         }

//         fn get_flaps_conf(&self) -> FlapsConf {
//             self.query(|a| a.slat_flap_complex.sfcc.flaps_conf)
//         }

//         fn get_flaps_angle(&self) -> f64 {
//             self.query(|a| a.slat_flap_complex.flap_gear.current_angle.get::<degree>())
//         }

//         fn get_slats_angle(&self) -> f64 {
//             self.query(|a| a.slat_flap_complex.slat_gear.current_angle.get::<degree>())
//         }
//     }
//     impl TestBed for A320FlapsTestBed {
//         type Aircraft = A320FlapsTestAircraft;

//         fn test_bed(&self) -> &SimulationTestBed<A320FlapsTestAircraft> {
//             &self.test_bed
//         }

//         fn test_bed_mut(&mut self) -> &mut SimulationTestBed<A320FlapsTestAircraft> {
//             &mut self.test_bed
//         }
//     }

//     fn test_bed() -> A320FlapsTestBed {
//         A320FlapsTestBed::new()
//     }

//     fn test_bed_with() -> A320FlapsTestBed {
//         test_bed()
//     }

//     #[test]
//     fn flaps_simvars() {
//         let test_bed = test_bed_with().run_one_tick();

//         assert!(test_bed.contains_key("LEFT_FLAPS_ANGLE"));
//         assert!(test_bed.contains_key("RIGHT_FLAPS_ANGLE"));
//         assert!(test_bed.contains_key("LEFT_FLAPS_POSITION_PERCENT"));
//         assert!(test_bed.contains_key("RIGHT_FLAPS_POSITION_PERCENT"));
//         assert!(test_bed.contains_key("LEFT_FLAPS_TARGET_ANGLE"));
//         assert!(test_bed.contains_key("RIGHT_FLAPS_TARGET_ANGLE"));

//         assert!(test_bed.contains_key("LEFT_SLATS_ANGLE"));
//         assert!(test_bed.contains_key("RIGHT_SLATS_ANGLE"));
//         assert!(test_bed.contains_key("LEFT_SLATS_POSITION_PERCENT"));
//         assert!(test_bed.contains_key("RIGHT_SLATS_POSITION_PERCENT"));
//         assert!(test_bed.contains_key("LEFT_SLATS_TARGET_ANGLE"));
//         assert!(test_bed.contains_key("RIGHT_SLATS_TARGET_ANGLE"));

//         assert!(test_bed.contains_key("FLAPS_CONF_HANDLE_INDEX_HELPER"));
//     }

//     // Tests flaps configuration and angles for regular
//     // increasing handle transitions, i.e 0->1->2->3->4 in sequence
//     // below 100 knots
//     #[test]
//     fn flaps_test_regular_handle_increase_transitions_flaps_target_airspeed_below_100() {
//         let angle_delta: f64 = 0.1;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(50.)
//             .run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 0);
//         assert!(test_bed.get_flaps_demanded_angle() == 0.);
//         assert!(test_bed.get_slats_demanded_angle() == 0.);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed
//             .set_flaps_handle_position(1)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 1);
//         assert!((test_bed.get_flaps_demanded_angle() - 10.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 18.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed
//             .set_flaps_handle_position(2)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 2);
//         assert!((test_bed.get_flaps_demanded_angle() - 15.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed
//             .set_flaps_handle_position(3)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 3);
//         assert!((test_bed.get_flaps_demanded_angle() - 20.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed
//             .set_flaps_handle_position(4)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 4);
//         assert!((test_bed.get_flaps_demanded_angle() - 40.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 27.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);
//     }

//     // Tests flaps configuration and angles for regular
//     // increasing handle transitions, i.e 0->1->2->3->4 in sequence
//     // above 100 knots
//     #[test]
//     fn flaps_test_regular_handle_increase_transitions_flaps_target_airspeed_above_100() {
//         let angle_delta: f64 = 0.1;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(150.)
//             .run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 0);
//         assert!(test_bed.get_flaps_demanded_angle() == 0.);
//         assert!(test_bed.get_slats_demanded_angle() == 0.);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed
//             .set_flaps_handle_position(1)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 1);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);
//         assert!(test_bed.get_flaps_demanded_angle() == 0.);
//         assert!((test_bed.get_slats_demanded_angle() - 18.).abs() < angle_delta);

//         test_bed = test_bed
//             .set_flaps_handle_position(2)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 2);
//         assert!((test_bed.get_flaps_demanded_angle() - 15.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed
//             .set_flaps_handle_position(3)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 3);
//         assert!((test_bed.get_flaps_demanded_angle() - 20.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed
//             .set_flaps_handle_position(4)
//             .run_waiting_for(Duration::from_millis(500));

//         assert!(test_bed.read_flaps_handle_position() as u8 == 4);
//         assert!((test_bed.get_flaps_demanded_angle() - 40.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 27.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);
//     }

//     //Tests regular transition 2->1 below and above 210 knots
//     #[test]
//     fn flaps_test_regular_handle_transition_pos_2_to_1() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(150.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);
//     }

//     //Tests transition between Conf1F to Conf1 above 210 knots
//     #[test]
//     fn flaps_test_regular_handle_transition_pos_1_to_1() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(50.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_indicated_airspeed(150.);

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_indicated_airspeed(220.).run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);
//     }

//     // Tests flaps configuration and angles for regular
//     // decreasing handle transitions, i.e 0->1->2->3->4 in sequence
//     // below 210 knots
//     #[test]
//     fn flaps_test_regular_decrease_handle_transition_flaps_target_airspeed_below_210() {
//         let angle_delta: f64 = 0.1;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(150.)
//             .run_one_tick();

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 4);
//         assert!((test_bed.get_flaps_demanded_angle() - 40.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 27.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 3);
//         assert!((test_bed.get_flaps_demanded_angle() - 20.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 2);
//         assert!((test_bed.get_flaps_demanded_angle() - 15.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 1);
//         assert!((test_bed.get_flaps_demanded_angle() - 10.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 18.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 0);
//         assert!((test_bed.get_flaps_demanded_angle() - 0.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 0.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);
//     }

//     // Tests flaps configuration and angles for regular
//     // decreasing handle transitions, i.e 0->1->2->3->4 in sequence
//     // above 210 knots
//     #[test]
//     fn flaps_test_regular_decrease_handle_transition_flaps_target_airspeed_above_210() {
//         let angle_delta: f64 = 0.1;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .run_one_tick();

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 4);
//         assert!((test_bed.get_flaps_demanded_angle() - 40.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 27.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 3);
//         assert!((test_bed.get_flaps_demanded_angle() - 20.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 2);
//         assert!((test_bed.get_flaps_demanded_angle() - 15.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 22.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 1);
//         assert!((test_bed.get_flaps_demanded_angle() - 0.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 18.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();

//         assert!(test_bed.read_flaps_handle_position() as u8 == 0);
//         assert!((test_bed.get_flaps_demanded_angle() - 0.).abs() < angle_delta);
//         assert!((test_bed.get_slats_demanded_angle() - 0.).abs() < angle_delta);
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);
//     }

//     //All the tests that follow test irregular transitions
//     //i.e. direct from 0 to 3 or direct from 4 to 0.
//     //This is possible in the simulator, but obviously
//     //not possible in real life. An irregular transition from x = 2,3,4
//     // to y = 0,1 should behave like a sequential transition.
//     #[test]
//     fn flaps_test_irregular_handle_transition_init_pos_0() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed
//             .set_indicated_airspeed(110.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);
//     }

//     #[test]
//     fn flaps_test_irregular_handle_transition_init_pos_1() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(110.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(110.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);
//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);
//     }

//     #[test]
//     fn flaps_test_irregular_handle_transition_init_pos_2() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);
//     }

//     #[test]
//     fn flaps_test_irregular_handle_transition_init_pos_3() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(150.)
//             .set_flaps_handle_position(3)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(3)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(3)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(3).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);
//     }

//     #[test]
//     fn flaps_test_irregular_handle_transition_init_pos_4() {
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(150.)
//             .set_flaps_handle_position(4)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(4)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(1).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(4)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(0).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);

//         test_bed = test_bed.set_flaps_handle_position(2).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(4).run_one_tick();
//         assert!(test_bed.get_flaps_conf() == FlapsConf::ConfFull);
//     }

//     #[test]
//     fn flaps_test_movement_0_to_1f() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(1);

//         let mut previous_angle: f64 = test_bed.get_flaps_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             println!(
//                 "{}, {}, {}",
//                 test_bed.get_flaps_angle(),
//                 test_bed.get_flaps_demanded_angle(),
//                 previous_angle
//             );
//             if (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_flaps_angle());
//             }
//             previous_angle = test_bed.get_flaps_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn flaps_test_movement_1f_to_2() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(2);

//         let mut previous_angle: f64 = test_bed.get_flaps_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_flaps_angle());
//             }
//             previous_angle = test_bed.get_flaps_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn flaps_test_movement_2_to_3() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(3);

//         let mut previous_angle: f64 = test_bed.get_flaps_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_flaps_angle());
//             }
//             previous_angle = test_bed.get_flaps_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn flaps_test_movement_3_to_full() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(3)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(4);

//         let mut previous_angle: f64 = test_bed.get_flaps_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_flaps_angle());
//             }
//             previous_angle = test_bed.get_flaps_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_flaps_angle() - test_bed.get_flaps_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn slats_test_movement_0_to_1f() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(1);

//         let mut previous_angle: f64 = test_bed.get_slats_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_slats_angle());
//             }
//             previous_angle = test_bed.get_slats_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn slats_and_flaps_test_movement_0_to_1() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(220.)
//             .set_flaps_handle_position(0)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf0);

//         test_bed = test_bed.set_flaps_handle_position(1);

//         let mut previous_angle: f64 = test_bed.get_slats_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_slats_angle());
//             }
//             previous_angle = test_bed.get_slats_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs() <= angle_delta
//         );
//         assert!(test_bed.get_flaps_angle() == 0.);
//     }

//     #[test]
//     fn slats_test_movement_1f_to_2() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(1)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf1F);

//         test_bed = test_bed.set_flaps_handle_position(2);

//         let mut previous_angle: f64 = test_bed.get_slats_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_slats_angle());
//             }
//             previous_angle = test_bed.get_slats_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn slats_test_movement_2_to_3() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(2)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf2);

//         test_bed = test_bed.set_flaps_handle_position(3);

//         let mut previous_angle: f64 = test_bed.get_slats_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_slats_angle());
//             }
//             previous_angle = test_bed.get_slats_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs() <= angle_delta
//         );
//     }

//     #[test]
//     fn slats_test_movement_3_to_full() {
//         let angle_delta = 0.01;
//         let mut test_bed = test_bed_with()
//             .set_hyd_pressure()
//             .set_indicated_airspeed(0.)
//             .set_flaps_handle_position(3)
//             .run_one_tick();

//         assert!(test_bed.get_flaps_conf() == FlapsConf::Conf3);

//         test_bed = test_bed.set_flaps_handle_position(4);

//         let mut previous_angle: f64 = test_bed.get_slats_angle();
//         test_bed = test_bed.run_one_tick();
//         for _ in 0..1200 {
//             if (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                 <= angle_delta
//             {
//                 test_bed = test_bed.run_waiting_for(Duration::from_secs(5));
//                 assert!(
//                     (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs()
//                         <= angle_delta
//                 );
//                 break;
//             } else {
//                 assert!(previous_angle < test_bed.get_slats_angle());
//             }
//             previous_angle = test_bed.get_slats_angle();
//             test_bed = test_bed.run_one_tick();
//         }
//         assert!(
//             (test_bed.get_slats_angle() - test_bed.get_slats_demanded_angle()).abs() <= angle_delta
//         );
//     }
// }
