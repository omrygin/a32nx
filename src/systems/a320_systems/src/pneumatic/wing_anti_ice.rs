use core::panic;

use crate::UpdateContext;

use uom::si::{
    f64::*,
    pressure::{pascal, psi},
    ratio::{percent, ratio},
    thermodynamic_temperature::degree_celsius,
    volume::cubic_meter,
    volume_rate::cubic_meter_per_second,
};

use systems::{hydraulic::Fluid, overhead::{AutoOffFaultPushButton, OnOffFaultPushButton}, pneumatic::{ApuCompressionChamberController, CompressionChamber, ConstantConsumerController, ControllablePneumaticValve, ControlledPneumaticValveSignal, CrossBleedValveSelectorKnob, CrossBleedValveSelectorMode, DefaultConsumer, DefaultPipe, DefaultValve, EngineCompressionChamberController, EngineState, PneumaticContainer, TargetPressureSignal, WingAntiIcePushButtonMode}, shared::{
        ControllerSignal, EngineCorrectedN1, EngineCorrectedN2, EngineFirePushButtons,
        PneumaticValve,
    }, simulation::{
        Read, SimulationElement, SimulationElementVisitor, SimulatorReader, SimulatorWriter, Write,
    }};

use pid::Pid;

use super::*;


struct WingAntiIceValveSignal {
    target_open_amount: Ratio,
}


impl WingAntiIceValveSignal {
    pub fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    pub fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }

    pub fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }

}


impl ControlledPneumaticValveSignal for WingAntiIceValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}




pub struct WingAntiIceValveController {
    wing_anti_ice_button_pos: WingAntiIcePushButtonMode,
}

impl WingAntiIceValveController {
    pub fn new() -> Self {
        Self {
            wing_anti_ice_button_pos: WingAntiIcePushButtonMode::Off,
       }
    
    }
    
    pub fn update (
        &mut self,
        wing_anti_ice_button_pos: WingAntiIcePushButtonMode,
    ) {
        self.wing_anti_ice_button_pos = wing_anti_ice_button_pos;
    }
}


impl ControllerSignal<WingAntiIceValveSignal> for WingAntiIceValveController {
    fn signal(&self) -> Option<WingAntiIceValveSignal> {
        match self.wing_anti_ice_button_pos {
            WingAntiIcePushButtonMode::Off => Some(WingAntiIceValveSignal::new_closed()),
            WingAntiIcePushButtonMode::On => Some(WingAntiIceValveSignal::new_open()),
        }
    }
}

pub struct StaticExhaust {
    open_amount: Ratio,
}

//A static exhaust is a valve that has a fixed open amount
//that cannot change, and it moves air from one container to 
//the ambient atmosphere. It draws air from a consumer.
impl StaticExhaust {
    const TRANSFER_SPEED: f64 = 3.;
    
    
    pub fn new(open_amount: Ratio) -> Self {
        Self { open_amount }
    }
    
    pub fn open_amount(&self) -> Ratio {
        self.open_amount
    }

    pub fn update_move_fluid(
        &self,
        context: &UpdateContext,
        from: &mut impl PneumaticContainer,
        ambient_pressure: Pressure,
    ) {

        let equalization_volume = (from.pressure()-ambient_pressure) * from.volume()
            / Pressure::new::<pascal>(142000.);
        self.exhaust_volume(
            from,
            self.open_amount()
                * equalization_volume
                * (1. - (-Self::TRANSFER_SPEED * context.delta_as_secs_f64()).exp()),
        );
    } 
                        
    fn exhaust_volume(  
            &self,
            from: &mut impl PneumaticContainer,
            volume: Volume,
    ) {
        from.change_volume(-volume);
    }
}


pub struct WingAntiIceConsumer {
    pipe: DefaultPipe,
}

impl PneumaticContainer for WingAntiIceConsumer {
    fn pressure(&self) -> Pressure {
        self.pipe.pressure()
    }

    fn volume(&self) -> Volume {
        self.pipe.volume()
    }

    fn temperature(&self) -> ThermodynamicTemperature {
        self.pipe.temperature()
    }

    fn change_volume(&mut self, volume: Volume) {
        self.pipe.change_volume(volume)
    }

}

impl WingAntiIceConsumer {
    pub fn new(volume: Volume) -> Self {
        Self {
            pipe: DefaultPipe::new(volume,
                Fluid::new(Pressure::new::<pascal>(142000.)),
                Pressure::new::<psi>(14.7),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
                ),
        }
    }

}

pub struct WingAntiIceComplex {
    left_wai_exhaust: StaticExhaust,
    left_wai_valve: DefaultValve,
    left_wai_consumer: WingAntiIceConsumer,

    right_wai_exhaust: StaticExhaust,
    right_wai_valve: DefaultValve,
    right_wai_consumer: WingAntiIceConsumer,

    valve_controller: WingAntiIceValveController,

}

impl WingAntiIceComplex {
    pub fn new() -> Self {
        Self {
            left_wai_exhaust: StaticExhaust::new(Ratio::new::<percent>(100.)),
            left_wai_valve: DefaultValve::new_closed(),
            left_wai_consumer: WingAntiIceConsumer::new(Volume::new::<cubic_meter>(1.)),

            right_wai_exhaust: StaticExhaust::new(Ratio::new::<percent>(100.)),
            right_wai_valve: DefaultValve::new_closed(),
            right_wai_consumer: WingAntiIceConsumer::new(Volume::new::<cubic_meter>(1.)),

            valve_controller: WingAntiIceValveController::new(),
         }
    }

    pub fn update_controller(&mut self,wai_mode: WingAntiIcePushButtonMode) {
        self.valve_controller.update(wai_mode);
    }

    pub fn is_left_wai_valve_open(&self) -> bool {
        self.left_wai_valve.is_open()
    }
    pub fn is_right_wai_valve_open(&self) -> bool {
        self.right_wai_valve.is_open()
    }

    pub fn left_wai_consumer_pressure(&self) -> Pressure {
        self.left_wai_consumer.pressure()
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        engine_systems: &mut [EngineBleedAirSystem; 2],
    ) {
        self.left_wai_valve.update_open_amount(&self.valve_controller);
        self.right_wai_valve.update_open_amount(&self.valve_controller);

        self.left_wai_exhaust.update_move_fluid(
            context, 
            &mut self.left_wai_consumer, 
            context.ambient_pressure());
        self.right_wai_exhaust.update_move_fluid(
            context, 
            &mut self.right_wai_consumer, 
            context.ambient_pressure());
     
        self.left_wai_valve.update_move_fluid(
            context, 
            &mut engine_systems[0].regulated_pressure_pipe, 
            &mut self.left_wai_consumer);

        self.right_wai_valve.update_move_fluid(
            context, 
            &mut engine_systems[1].regulated_pressure_pipe, 
            &mut self.right_wai_consumer);
    }

}



#[cfg(test)]
mod tests {
    use super::*;
    use systems::{
        engine::leap_engine::LeapEngine,
        pneumatic::{EngineState, PneumaticContainer},
        shared::{ApuBleedAirValveSignal, MachNumber, ISA},
        simulation::{
            test::{SimulationTestBed, TestBed},
            Aircraft, SimulationElement, Write,
        },
    };

    use std::{fs::File, time::Duration};

    use uom::si::{length::foot, pressure::pascal, thermodynamic_temperature::degree_celsius};


    #[test]
    fn dummy_test() {
        assert_eq!(1,1);
    }
//    fn exhaust_equalizes_pressure() {
//        let mut consumer_a: WingAntiIceConsumer = WingAntiIceConsumer::new(Volume::new::<cubic_meter>(1.));
//        let exhaust_a: StaticExhaust = StaticExhaust::new(Ratio::new::<percent>(1.));
//        let ambient_pressure: Pressure = Pressure::new::<psi>(12.); 
//        let pressure_epsilon: Pressure = Pressure::new::<psi>(0.01);
//        let dt: f64 =0.1;
//        let mut time: f64 = 0.;
//
//        let mut pressure_at_t = Vec::new();
//        let mut time_list = Vec::new();
//
//        for _ in 0..1000 {   
//            exhaust_a.update_move_fluid(time,&mut consumer_a,ambient_pressure);
//            time += dt;
//            pressure_at_t.push(consumer_a.pressure().get::<psi>());
//
//            time_list.push(time);
//            println!("Consumar pressure = {}, Ambient pressure = {}, t = {}",
//                     consumer_a.pressure().get::<psi>(),
//                     ambient_pressure.get::<psi>(),
//                     dt)
//        }
//
//        let mut file = File::create("pressure_test_2.txt").expect("Could not create file");
//        use std::io::Write;
//        writeln!(file,"{:?}",pressure_at_t).expect("Could not write file");
//        assert!((consumer_a.pressure()-ambient_pressure).abs() < pressure_epsilon);
//
//    }

    struct TestApu {
        bleed_air_valve_signal: ApuBleedAirValveSignal,
    }
    impl TestApu {
        fn new() -> Self {
            Self {
                bleed_air_valve_signal: ApuBleedAirValveSignal::Close,
            }
        }

        fn update(&self, bleed_valve: &mut impl ControllablePneumaticValve) {
            bleed_valve.update_open_amount(self);
        }

        fn set_bleed_air_valve_signal(&mut self, signal: ApuBleedAirValveSignal) {
            self.bleed_air_valve_signal = signal;
        }
    }
    impl ControllerSignal<ApuBleedAirValveSignal> for TestApu {
        fn signal(&self) -> Option<ApuBleedAirValveSignal> {
            Some(self.bleed_air_valve_signal)
        }
    }

    struct TestEngineFirePushButtons {
        is_released: [bool; 2],
    }
    impl TestEngineFirePushButtons {
        fn new() -> Self {
            Self {
                is_released: [false, false],
            }
        }

        fn release(&mut self, engine_number: usize) {
            self.is_released[engine_number - 1] = true;
        }
    }
    impl EngineFirePushButtons for TestEngineFirePushButtons {
        fn is_released(&self, engine_number: usize) -> bool {
            self.is_released[engine_number - 1]
        }
    }
    struct PneumaticTestAircraft {
        pneumatic: A320Pneumatic,
        apu: TestApu,
        engine_1: LeapEngine,
        engine_2: LeapEngine,
        overhead_panel: A320PneumaticOverheadPanel,
        fire_pushbuttons: TestEngineFirePushButtons,
    }
    impl PneumaticTestAircraft {
        fn new() -> Self {
            Self {
                pneumatic: A320Pneumatic::new(),
                apu: TestApu::new(),
                engine_1: LeapEngine::new(1),
                engine_2: LeapEngine::new(2),
                overhead_panel: A320PneumaticOverheadPanel::new(),
                fire_pushbuttons: TestEngineFirePushButtons::new(),
            }
        }
    }
    impl Aircraft for PneumaticTestAircraft {
        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.apu.update(self.pneumatic.apu_bleed_air_valve());
            self.pneumatic.update(
                context,
                [&self.engine_1, &self.engine_2],
                &self.overhead_panel,
                &self.fire_pushbuttons,
            );
        }
    }
    impl SimulationElement for PneumaticTestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
        where
            Self: Sized,
        {
            self.pneumatic.accept(visitor);
            self.engine_1.accept(visitor);
            self.engine_2.accept(visitor);
            self.overhead_panel.accept(visitor);

            visitor.visit(self);
        }
    }
    struct PneumaticTestBed {
        test_bed: SimulationTestBed<PneumaticTestAircraft>,
    }
    impl TestBed for PneumaticTestBed {
        type Aircraft = PneumaticTestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<PneumaticTestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<PneumaticTestAircraft> {
            &mut self.test_bed
        }
    }
    impl PneumaticTestBed {
        fn new() -> Self {
            Self {
                test_bed: SimulationTestBed::<PneumaticTestAircraft>::new(|_| {
                    PneumaticTestAircraft::new()
                }),
            }
        }

        fn and_run(mut self) -> Self {
            self.run();

            self
        }

        fn and_stabilize(mut self) -> Self {
            for _ in 1..1000 {
                self.run_with_delta(Duration::from_millis(16));
            }

            self
        }

        fn mach_number(mut self, mach: MachNumber) -> Self {
            self.write("AIRSPEED MACH", mach);

            self
        }

        fn in_isa_atmosphere(mut self, altitude: Length) -> Self {
            self.set_ambient_pressure(ISA::pressure_at_altitude(altitude));
            self.set_ambient_temperature(ISA::temperature_at_altitude(altitude));

            self
        }

        fn idle_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", true);
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(0.55));
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(0.2));
            self.write("ENGINE_STATE:1", EngineState::On);

            self
        }

        fn idle_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", true);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(0.55));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(0.2));
            self.write("ENGINE_STATE:2", EngineState::On);

            self
        }

        fn stop_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", false);
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(0.));

            self
        }

        fn stop_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", false);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(0.));

            self
        }

        fn start_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", true);
            self.write("ENGINE_STATE:1", EngineState::Starting);

            self
        }

        fn start_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", true);
            self.write("ENGINE_STATE:2", EngineState::Starting);

            self
        }

        fn cross_bleed_valve_selector_knob(mut self, mode: CrossBleedValveSelectorMode) -> Self {
            self.write("KNOB_OVHD_AIRCOND_XBLEED_Position", mode);

            self
        }

        fn wing_anti_ice_push_button(mut self, mode: WingAntiIcePushButtonMode) -> Self {
            self.write("BUTTON_OVHD_ANTI_ICE_WING_Position",mode);

            self
        }

        fn for_both_engine_systems<T: Fn(&EngineBleedAirSystem) -> ()>(&self, func: T) {
            self.query(|a| a.pneumatic.engine_systems.iter().for_each(|sys| func(sys)));
        }

        fn ip_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_pressure())
        }

        fn hp_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_pressure())
        }

        fn transfer_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].transfer_pressure())
        }

        fn regulated_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].regulated_pressure())
        }

        fn ip_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_temperature())
        }

        fn hp_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_temperature())
        }

        fn transfer_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].transfer_temperature())
        }

        fn regulated_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].regulated_temperature())
        }

        fn ip_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_valve.is_open())
        }

        fn hp_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_valve.is_open())
        }

        fn pr_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].pr_valve.is_open())
        }

        fn es_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].esv_is_open())
        }

        fn apu_bleed_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.apu_bleed_air_valve.is_open())
        }

        fn left_wai_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.wing_anti_ice.is_left_wai_valve_open())
        }

        fn right_wai_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.wing_anti_ice.is_right_wai_valve_open())
        }

        fn set_engine_bleed_push_button_off(mut self, number: usize) -> Self {
            self.write(&format!("OVHD_PNEU_ENG_{}_BLEED_PB_IS_AUTO", number), false);

            self
        }

        fn set_engine_bleed_push_button_has_fault(
            mut self,
            number: usize,
            has_fault: bool,
        ) -> Self {
            self.write(
                &format!("OVHD_PNEU_ENG_{}_BLEED_PB_HAS_FAULT", number),
                has_fault,
            );

            self
        }

        fn set_apu_bleed_valve_signal(mut self, signal: ApuBleedAirValveSignal) -> Self {
            self.command(|a| a.apu.set_bleed_air_valve_signal(signal));

            self
        }

        fn set_bleed_air_pb(mut self, is_on: bool) -> Self {
            self.write("OVHD_APU_BLEED_PB_IS_ON", is_on);

            self
        }

        fn set_bleed_air_running(mut self) -> Self {
            self.write("APU_BLEED_AIR_PRESSURE", Pressure::new::<psi>(35.));
            self.set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
                .set_bleed_air_pb(true)
        }

        fn release_fire_pushbutton(mut self, number: usize) -> Self {
            self.command(|a| a.fire_pushbuttons.release(number));

            self
        }

        fn set_engine_state(mut self, number: usize, engine_state: EngineState) -> Self {
            self.write(&format!("ENGINE_STATE:{}", number), engine_state);

            self
        }

        fn engine_state(&self, number: usize) -> EngineState {
            self.query(|a| a.pneumatic.fadec.engine_state(number))
        }

        fn is_fire_pushbutton_released(&self, number: usize) -> bool {
            self.query(|a| a.fire_pushbuttons.is_released(number))
        }

        fn cross_bleed_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.cross_bleed_valve.is_open())
        }

        fn cross_bleed_valve_selector(&self) -> CrossBleedValveSelectorMode {
            self.query(|a| a.overhead_panel.cross_bleed_mode())
        }

        fn engine_bleed_push_button_is_auto(&self, number: usize) -> bool {
            self.query(|a| a.overhead_panel.engine_bleed_pb_is_auto(number))
        }

        fn engine_bleed_push_button_has_fault(&self, number: usize) -> bool {
            self.query(|a| a.overhead_panel.engine_bleed_pb_has_fault(number))
        }
    }

    fn test_bed() -> PneumaticTestBed {
        PneumaticTestBed::new()
    }

    fn test_bed_with() -> PneumaticTestBed {
        test_bed()
    }

    fn pressure_tolerance() -> Pressure {
        Pressure::new::<pascal>(100.)
    }
}
