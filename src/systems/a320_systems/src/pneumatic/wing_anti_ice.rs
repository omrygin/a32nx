use core::panic;
use std::time::Duration;

use crate::{
    hydraulic::{A320Hydraulic, FakeHydraulicReservoir},
    UpdateContext,
};

use uom::si::{
    f64::*,
    pressure::{pascal, psi},
    ratio::{percent, ratio},
    thermodynamic_temperature::degree_celsius,
    temperature_interval,
    volume::cubic_meter,
    volume_rate::cubic_meter_per_second,
};

use systems::{hydraulic::Fluid, overhead::{AutoOffFaultPushButton, OnOffFaultPushButton}, 
    pneumatic::{ApuCompressionChamberController, CompressionChamber, ConstantConsumerController, 
        ControllablePneumaticValve, ControlledPneumaticValveSignal, CrossBleedValveSelectorKnob, 
        CrossBleedValveSelectorMode, DefaultConsumer, DefaultPipe, DefaultValve, 
        EngineCompressionChamberController, EngineState, PneumaticContainer, TargetPressureSignal, 
        WingAntiIcePushButtonMode}, shared::{
        ControllerSignal, EngineCorrectedN1, EngineCorrectedN2, EngineFirePushButtons,
        PneumaticValve,
    }, simulation::{Read, SimulationElement, SimulationElementVisitor, SimulatorReader, SimulatorWriter, Write, test::SimulationTestBed}};

use pid::Pid;

use super::*;

/* This is actually not specific for the wing anti ice,
 * and should be attached to any consumer that exhausts 
 * air to the outside of the plane.
 * This is actually a statis valve, that keeps its open amount
 * without a controller. Unlike `DefaultValve`, the `update_move_fluid`
 * here moves air from a container to the ambient atmosphere, which
 * is an infinite pressure bath.
 * */

// Just holds how much the exhaust is open.
pub struct StaticExhaust {
    open_amount: Ratio,
}

impl StaticExhaust {
    const MASS_TRANSFER_SPEED: f64 = 1.;
    
    // Unlike `DefaultValve`, we need to specify the 
    // initial open amount everytime we initiate a new exhaust
    pub fn new(open_amount: Ratio) -> Self {
        Self { open_amount }
    }
    
    // Returns the open amount 
    pub fn open_amount(&self) -> Ratio {
        self.open_amount
    }

    // Compute the amount of air that is exhausted
    // given the pressure gradient between the container
    // and the ambient atmosphere.
    pub fn update_move_fluid(
        &self,
        context: &UpdateContext,
        from: &mut impl PneumaticContainer,
    ) {

        let equalization_volume = (from.pressure()-context.ambient_pressure()) * from.volume()
            / Pressure::new::<pascal>(142000.);
        self.exhaust_volume(
            context,
            from,
            self.open_amount()
                * equalization_volume
                * (1. - (-Self::MASS_TRANSFER_SPEED * context.delta_as_secs_f64()).exp()),
        );
    } 
    
    // Exhaust a certain amount of volume
    fn exhaust_volume(  
            &self,
            context: &UpdateContext,
            from: &mut impl PneumaticContainer,
            volume: Volume,
    ) {
        from.change_volume(-volume);    
    }
}

/* The valve itself is a DefaultValve. The only thing
 * we need to re-implement is the controller, that sets
 * whether or not the valve should be open.
 *
 * The controller works using signals. A signal basically
 * tells the controlloer what fraction of the valve should
 * be open. Each valve has an `update_open_amount` method,
 * that accepts a controller that implements the 
 * `ControllerSignal` trait. This trait has a single method,
 * which returns an option for the signal type (e.g. wing anti ice signal)
 * depending on the button/selector position.
 **/

// A WAI valve signal, just indicates what fraction
// of the valve should be open
struct WingAntiIceValveSignal {
    target_open_amount: Ratio,
}
// We can create a signal to be 100% open,
// totally closed, or potentially something in between
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

// A controlled valve signal. This just lets us access the target amount
impl ControlledPneumaticValveSignal for WingAntiIceValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

// This is the actual controller. It holds the push button status.
// - After 30 seconds, the ON light would turn off.
// - After takeoff, it should be turned on again.
pub struct WingAntiIceValveController {
    wing_anti_ice_button_pos: WingAntiIcePushButtonMode, //The position of the button
    valve_pid: Pid<f64>, //PID controller for the valve - to regulate pressure
    valve_pid_output: f64, //Output of the PID controller - open_amount
    is_on_ground: bool, //Needed for the 30 seconds test logic
    system_test_timer: Duration, //Timer to count up to 30 seconds
    system_test_done: bool, //Timer reached 30 seconds while on the ground
    controller_signals_on: bool, //If button is pushed and the test is finished
                                        //the ON light should turn off.
    supplier_pressurized: bool,
}

impl WingAntiIceValveController {
    const WAI_TEST_TIME: Duration = Duration::from_secs(30);
    pub fn new() -> Self {
        Self {
            wing_anti_ice_button_pos: WingAntiIcePushButtonMode::Off,
            valve_pid: Pid::new(0.5,0.0,0.5,1.,1.,1.,1.,22.),
            valve_pid_output: 0.,
            is_on_ground: true,
            system_test_timer: Duration::from_secs(0),
            system_test_done: false,
            controller_signals_on: false,
            supplier_pressurized: false,
       }
    
    }

    pub fn controller_signals_on(&self) -> bool {
        self.controller_signals_on
    }
    
    pub fn update (
        &mut self,
        context: &UpdateContext,
        wing_anti_ice_button_pos: WingAntiIcePushButtonMode,
        supplier_pressurized: bool,
    ) {
        self.wing_anti_ice_button_pos = wing_anti_ice_button_pos;
        self.supplier_pressurized = supplier_pressurized;

        if self.wing_anti_ice_button_pos == WingAntiIcePushButtonMode::On {
            if self.is_on_ground && self.system_test_done == false {
                if self.supplier_pressurized {
                    self.system_test_timer += context.delta();
                }
                self.system_test_timer = self.system_test_timer.min(Self::WAI_TEST_TIME);
                if self.system_test_timer == Self::WAI_TEST_TIME {
                    self.system_test_done = true;
                    self.controller_signals_on = false;
                } else {
                    self.controller_signals_on = true;
                }
            } else if self.is_on_ground == false {
                self.controller_signals_on = true;
            }
        } else {
            self.controller_signals_on = false;
        }

        
        //If the plane has took off, we reset the timer
        //and set test_done to false in order for the 
        //mechanism to work when landing.
        if self.is_on_ground == false && self.system_test_timer > Duration::from_secs(0) {
            self.system_test_timer = Duration::from_secs(0);
            self.system_test_done = false;
        }
    }


    pub fn get_timer(&self) -> Duration {
        self.system_test_timer
    }
}
impl SimulationElement for WingAntiIceValveController {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.is_on_ground = reader.read(&"SIM ON GROUND");
    }
}
// This is the part that interacts with the valve, via DefaultValve.update_open_amount.
// That method has if let Some(signal) = controller.signal(). The right hand side
// is what is returned from this implementation.
impl ControllerSignal<WingAntiIceValveSignal> for WingAntiIceValveController {
    fn signal(&self) -> Option<WingAntiIceValveSignal> {
        match self.wing_anti_ice_button_pos {
            WingAntiIcePushButtonMode::Off => Some(WingAntiIceValveSignal::new_closed()),
            WingAntiIcePushButtonMode::On => {
                // Even if the button is pushed, we need to check if either
                // the plane is airborne or it is within the 30 second timeframe.
                // Also, we need to check if the supplier is pressurized
                // since the valve is pneumatically operated.
                if self.supplier_pressurized && (
                    !self.is_on_ground || (self.is_on_ground && !self.system_test_done)) {
                    Some(WingAntiIceValveSignal::new(Ratio::new::<ratio>(
                                self.valve_pid_output.max(0.).min(1.))))
                } else {
                    Some(WingAntiIceValveSignal::new_closed())
                }
            },
        }
    }
}

/* The wing anti ice is a consumer,
 * meaning it is a simple container that consumes
 * air from the bleed system, and exhausts it to the
 * ambient atmosphere. This is just the implementation 
 * of a regular container
 * */

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

    fn update_temperature(&mut self, temperature: TemperatureInterval) {
        self.pipe.update_temperature(temperature);
    }

    fn update_pressure_only(&mut self, volume: Volume) { 
        self.pipe.update_pressure_only(volume);
    }
}

impl WingAntiIceConsumer {
    const CONDUCTION_RATE: f64 = 0.1;
    pub fn new(volume: Volume) -> Self {
        Self {
            pipe: DefaultPipe::new(volume,
                Fluid::new(Pressure::new::<pascal>(142000.)),
                Pressure::new::<psi>(14.7),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
        }
    }
    // Radiate heat to the ambient atmosphere
    // according to Newton's law of cooling
    // dT/dt = -(T-T_atmo) / tau
    pub fn radiate_heat_to_ambient(&mut self,context: &UpdateContext) {
        let delta_t: TemperatureInterval = TemperatureInterval::new::<temperature_interval::degree_celsius>(
            self.temperature().get::<degree_celsius>() - context.ambient_temperature().get::<degree_celsius>()
        );

        self.update_temperature(-delta_t*context.delta_as_secs_f64()*Self::CONDUCTION_RATE);
    }
}
/* The entire WAI system could have been hard coded
 * into A320Pneumatic, however I think this is cleaner.
 * The complex includes both WAI parts. Each part contains 
 * a consumer, a valve and an exhaust.
 * 
 * There are two valve controllers, one for each.
 * Still need to figure out whether this is how it works.
 * */
pub struct WingAntiIceComplex {
    wai_exhaust: [StaticExhaust; 2],
    wai_valve: [DefaultValve; 2],
    wai_consumer: [WingAntiIceConsumer; 2],
    valve_controller: [WingAntiIceValveController; 2],
    wai_system_has_fault: bool,
    wai_system_on: bool,
}
impl WingAntiIceComplex {
    const NUM_OF_WAI: usize = 2;
    pub fn new() -> Self {
        Self {
            wai_exhaust: [StaticExhaust::new(Ratio::new::<percent>(10.)),
                        StaticExhaust::new(Ratio::new::<percent>(10.))],
            wai_valve: [DefaultValve::new_closed(),
                        DefaultValve::new_closed()],
            wai_consumer: [WingAntiIceConsumer::new(Volume::new::<cubic_meter>(1.)),
                        WingAntiIceConsumer::new(Volume::new::<cubic_meter>(1.))],
            valve_controller: [WingAntiIceValveController::new(),
                                WingAntiIceValveController::new()],

            wai_system_has_fault: false,
            wai_system_on: false,
         }
    }

    fn update_valve_controller(&mut self, context: &UpdateContext, wai_mode: WingAntiIcePushButtonMode, number: usize,
                                supplier_pressurized: bool) {
        self.valve_controller[number].update(context, wai_mode,supplier_pressurized);
    }

    pub fn is_wai_valve_open(&self, number: usize) -> bool {
        self.wai_valve[number].is_open()
    }

    pub fn wai_consumer_pressure(&self, number: usize) -> Pressure {
        self.wai_consumer[number].pressure()
    }

    pub fn wai_consumer_temperature(&self, number: usize) -> ThermodynamicTemperature {
        self.wai_consumer[number].temperature()
    }
    
    // This is where the action happens
    pub fn update(
        &mut self,
        context: &UpdateContext,
        engine_systems: &mut [EngineBleedAirSystem; 2],
        wai_mode: WingAntiIcePushButtonMode,
    ) {
        let mut has_fault: bool = false; //Tracks if the system has a fault
        let mut num_of_on: usize = 0; //Number of controllers that signal `on`

        for n in 0..Self::NUM_OF_WAI {
            self.valve_controller[n].valve_pid_output = 
                self.valve_controller[n].valve_pid.next_control_output(
                   self.wai_consumer_pressure(n).get::<psi>(),
                )
                .output;
            
            // First, we see if the valve's open amount changes this update,
            // as a result of a change in the ovhd panel push button.
            // If the precooler is not pressurized, a FAULT should light.
            self.update_valve_controller(context,wai_mode,n,
                engine_systems[n].precooler_outlet_pressure().get::<psi>() > 1.05*context.ambient_pressure().get::<psi>());
            self.wai_valve[n].update_open_amount(&self.valve_controller[n]);
            
            //We need both controllers to signal `on` for the 
            //system to be considered on without a fault.
            if self.valve_controller[n].controller_signals_on() {
                num_of_on +=1;
                //If a controller signals `on` while its corresponding valve is closed
                //this means the system has a fault.
                if self.is_wai_valve_open(n)  == false {
                    has_fault = true;
                }
            }

            //An exhaust tick always happens, no matter what 
            //the valve's state is
            //
            self.wai_exhaust[n].update_move_fluid(
                context, 
                &mut self.wai_consumer[n], 
                );

            //The heated slats radiate energy to the ambient atmosphere.
            self.wai_consumer[n].radiate_heat_to_ambient(context);
        
            //This only changes the volume if open_amount is not zero.
            self.wai_valve[n].update_move_fluid_with_temperature(
                context, 
                &mut engine_systems[n].precooler_outlet_pipe, 
                &mut self.wai_consumer[n]);

        }

        self.wai_system_has_fault = has_fault;
        
        if num_of_on < 2 {
            self.wai_system_on = false;
        } else {
            if !has_fault {
                self.wai_system_on = true;
            }
        }

    }

}



impl SimulationElement for WingAntiIceComplex {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where Self: Sized, {
        for n in 0..Self::NUM_OF_WAI {
            self.valve_controller[n].accept(visitor);
        }
        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write("PNEU_WING_ANTI_ICE_SYSTEM_ON", self.wai_system_on);
        writer.write("PNEU_WING_ANTI_ICE_HAS_FAULT",self.wai_system_has_fault);
        writer.write("PNEU_LEFT_WING_ANTI_ICE_CONSUMER_PRESSURE", self.wai_consumer[0].pressure());
        writer.write("PNEU_RIGHT_WING_ANTI_ICE_CONSUMER_PRESSURE", self.wai_consumer[1].pressure());
        writer.write("PNEU_LEFT_WING_ANTI_ICE_CONSUMER_TEMPERATURE", self.wai_consumer[0].temperature());
        writer.write("PNEU_RIGHT_WING_ANTI_ICE_CONSUMER_TEMPERATURE", self.wai_consumer[1].temperature());
        writer.write("PNEU_LEFT_WING_ANTI_ICE_VALVE_OPEN",self.is_wai_valve_open(0));
        writer.write("PNEU_RIGHT_WING_ANTI_ICE_VALVE_OPEN",self.is_wai_valve_open(1));

    }
}
//End WAI Complex block


//Begin simple tests

mod tests {

    #[test]
    fn dummy_test() {
        assert!(1 == 1);
    }
    
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
        hydraulic: A320Hydraulic,
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
                hydraulic: A320Hydraulic::new(),
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
                &self.hydraulic,
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

        fn and_stabilize_steps(mut self, n: usize) -> Self {
            for _ in 1..n {
                for _ in 1..1000 {
                    self.run_with_delta(Duration::from_millis(16));
                }
            }
           self 
        }

        fn in_isa_atmosphere(mut self, altitude: Length) -> Self {
            self.set_ambient_pressure(ISA::pressure_at_altitude(altitude));
            self.set_ambient_temperature(ISA::temperature_at_altitude(altitude));
            
            self
        }

        fn stop_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", false);
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(0.));

            self
        }
        fn stop_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", false);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(0.));

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
        
        fn power_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", true);
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(1.));
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(1.));
            self.write("ENGINE_STATE:1", EngineState::On);

            self

        }

        fn power_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", true);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(1.));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(1.));
            self.write("ENGINE_STATE:2", EngineState::On);

            self

        }

        fn wing_anti_ice_push_button(mut self, mode: WingAntiIcePushButtonMode) -> Self {
            match mode {
                WingAntiIcePushButtonMode::On => 
                    self.write("BUTTON_OVHD_ANTI_ICE_WING_Position", true),
                _ => self.write("BUTTON_OVHD_ANTI_ICE_WING_Position", false),
            };

            self
        }

        fn wing_anti_ice_system_on(&mut self) -> bool {
            self.read("PNEU_WING_ANTI_ICE_SYSTEM_ON")
        }

        fn is_sim_on_ground(&mut self) -> bool {
            self.read("SIM ON GROUND")
        }

        fn wing_anti_ice_has_fault(&mut self) -> bool {
            self.read("PNEU_WING_ANTI_ICE_HAS_FAULT")
        }

        //Utility functions to get info from the test bed
        fn left_wai_pressure(&self) -> Pressure {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_consumer_pressure(0))
        }
        fn right_wai_pressure(&self) -> Pressure {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_consumer_pressure(1))
        }

        fn left_wai_temperature(&self) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_consumer_temperature(0))
        }

        fn right_wai_temperature(&self) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_consumer_temperature(1))
        }

        fn precooler_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number-1].precooler_outlet_pressure())
        }

        fn precooler_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number-1].precooler_outlet_temperature())
        }

        fn left_valve_open_amount(&self) -> f64 {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_valve[0].open_amount().get::<ratio>())
        }

        fn right_valve_open_amount(&self) -> f64 {
            self.query(|a| a.pneumatic.wing_anti_ice.wai_valve[1].open_amount().get::<ratio>())
        }

        fn left_valve_controller_timer(&self) -> Duration {
            self.query(|a| a.pneumatic.wing_anti_ice.valve_controller[0].get_timer())
        }

        fn right_valve_controller_timer(&self) -> Duration {
            self.query(|a| a.pneumatic.wing_anti_ice.valve_controller[1].get_timer())
        }

        fn left_valve_open(&self) -> bool {
            self.query(|a| a.pneumatic.wing_anti_ice.is_wai_valve_open(0))
        }
        
        fn right_valve_open(&self) -> bool {
            self.query(|a| a.pneumatic.wing_anti_ice.is_wai_valve_open(1))
        }
 


    }

    fn test_bed() -> PneumaticTestBed{
        PneumaticTestBed::new()
    }


    #[test]
    fn wing_anti_ice_simvars() {
        let test_bed = test_bed();

        assert!(test_bed.contains_key("PNEU_WING_ANTI_ICE_SYSTEM_ON"));
        assert!(test_bed.contains_key("PNEU_WING_ANTI_ICE_HAS_FAULT"));
        assert!(test_bed.contains_key("PNEU_LEFT_WING_ANTI_ICE_CONSUMER_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_RIGHT_WING_ANTI_ICE_CONSUMER_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_LEFT_WING_ANTI_ICE_CONSUMER_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_RIGHT_WING_ANTI_ICE_CONSUMER_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_LEFT_WING_ANTI_ICE_VALVE_OPEN"));
        assert!(test_bed.contains_key("PNEU_RIGHT_WING_ANTI_ICE_VALVE_OPEN"));
        assert!(test_bed.contains_key("BUTTON_OVHD_ANTI_ICE_WING_Position"));
    }
    #[test]
    fn wing_anti_ice_cold_and_dark() {
        let altitude = Length::new::<foot>(500.);
        let ambient_pressure = ISA::pressure_at_altitude(altitude);
        let ambient_temperature = ISA::temperature_at_altitude(altitude);

        let temperature_epsilon = ThermodynamicTemperature::new::<degree_celsius>(0.05);
        let pressure_epsilon = Pressure::new::<psi>(0.01);

        let mut test_bed = test_bed()
            .stop_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude);
        test_bed.set_on_ground(true);
        test_bed = test_bed.and_stabilize_steps(7);

        println!("left press = {}", test_bed.left_wai_pressure().get::<psi>());
        println!("right press = {}", test_bed.right_wai_pressure().get::<psi>());
        println!("ambient press = {}", ambient_pressure.get::<psi>());
        println!("left temp = {}", test_bed.left_wai_temperature().get::<degree_celsius>());
        println!("right temp = {}", test_bed.right_wai_temperature().get::<degree_celsius>());
        println!("ambient temp = {}", ambient_temperature.get::<degree_celsius>());

        assert!((test_bed.left_wai_pressure() - ambient_pressure).abs() < pressure_epsilon);
        assert!((test_bed.right_wai_pressure() - ambient_pressure).abs() < pressure_epsilon);
        assert!((test_bed.left_wai_temperature().get::<degree_celsius>()
                     - ambient_temperature.get::<degree_celsius>()).abs() < temperature_epsilon.get::<degree_celsius>());
        assert!((test_bed.right_wai_temperature().get::<degree_celsius>()
                     - ambient_temperature.get::<degree_celsius>()).abs() < temperature_epsilon.get::<degree_celsius>());
        assert!(test_bed.left_valve_open() == false);
        assert!(test_bed.right_valve_open() == false);
        assert!(test_bed.wing_anti_ice_system_on() == false);
        assert!(test_bed.wing_anti_ice_has_fault() == false);

    }

    #[test]
    fn wing_anti_ice_has_fault_when_precooler_not_pressurized() {
        let altitude = Length::new::<foot>(500.);

        let mut test_bed = test_bed()
            .stop_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude);
        test_bed.set_on_ground(true);
        test_bed = test_bed.and_stabilize();

        test_bed = test_bed
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On)
            .and_stabilize();
        assert!(test_bed.wing_anti_ice_has_fault());
    }

    #[test]
    fn wing_anti_ice_no_fault_after_starting_engine() {
        let altitude = Length::new::<foot>(500.);
        let wai_pressure: Pressure = Pressure::new::<psi>(22.);
        let pressure_epsilon: Pressure = Pressure::new::<psi>(0.1);



        let mut test_bed = test_bed()
            .stop_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude);
        test_bed.set_on_ground(true);
        test_bed = test_bed.and_stabilize();

        test_bed = test_bed
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On)
            .and_stabilize();
        assert!(test_bed.wing_anti_ice_has_fault());

        test_bed = test_bed
                .idle_eng1()
                .idle_eng2()
                .and_stabilize();
        assert!(test_bed.wing_anti_ice_has_fault() == false);
        assert!(test_bed.wing_anti_ice_system_on());
    }

    #[test]
    fn wing_anti_ice_timer_doesnt_start_if_precooler_not_pressurized() {
        let altitude = Length::new::<foot>(500.);

        let mut test_bed = test_bed()
            .stop_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude);
        test_bed.set_on_ground(true);
        test_bed = test_bed.and_stabilize();
        
        test_bed = test_bed
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On)
            .and_stabilize();
        assert!(test_bed.wing_anti_ice_has_fault());
        assert!(test_bed.left_valve_controller_timer() == Duration::from_secs(0));
        assert!(test_bed.right_valve_controller_timer() == Duration::from_secs(0));

    }

    #[test]
    fn wing_anti_ice_tweak_pid() {
        let altitude = Length::new::<foot>(0.);
        let wai_pressure: Pressure = Pressure::new::<psi>(22.);
        let pressure_epsilon: Pressure = Pressure::new::<psi>(0.1);
        let mut test_bed = test_bed()
            .in_isa_atmosphere(altitude)
            .idle_eng1()
            .idle_eng2()
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On);

        let mut openamount = Vec::new();
        for _ in 0..2000 {
            openamount.push(test_bed.left_valve_open_amount());
            // openamount.push(test_bed.left_wai_pressure().get::<psi>());
            println!("left pressure = {}",test_bed.left_wai_pressure().get::<psi>());
            println!("left open amount = {}", test_bed.left_valve_open_amount());
            test_bed.run_with_delta(Duration::from_millis(16));
        }

        // let mut file = File::create("DO NOT COMMIT.txt").expect("Could not create file");

        // use std::io::Write;

        // writeln!(file, "{:?}", openamount).expect("Could not write file");
        // assert!(1>1);
    }

    #[test]
    fn wing_anti_ice_pressure_regulated() {
        let altitude = Length::new::<foot>(0.);
        let wai_pressure: Pressure = Pressure::new::<psi>(22.);
        let pressure_epsilon: Pressure = Pressure::new::<psi>(0.1);

        let mut test_bed = test_bed()
            .in_isa_atmosphere(altitude)
            .idle_eng1()
            .idle_eng2()
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On)
            .and_stabilize_steps(10);

        println!("left pressure = {}",test_bed.left_wai_pressure().get::<psi>());
        println!("right pressure = {}",test_bed.right_wai_pressure().get::<psi>());


        assert!((test_bed.left_wai_pressure() - wai_pressure).abs() < pressure_epsilon);
        assert!((test_bed.right_wai_pressure() - wai_pressure).abs() < pressure_epsilon);

    }


    #[test]
    fn wing_anti_ice_valve_close_after_30_seconds_on_ground () {
        let mut test_bed = test_bed()
            .idle_eng1()
            .idle_eng2()
            .in_isa_atmosphere(Length::new::<foot>(0.))
            .and_stabilize();
        test_bed.set_on_ground(true);
        
        assert!(test_bed.left_valve_controller_timer() == Duration::from_secs(0));
        assert!(test_bed.right_valve_controller_timer() == Duration::from_secs(0));

        test_bed = test_bed.wing_anti_ice_push_button(WingAntiIcePushButtonMode::On);
        test_bed.run_with_delta(Duration::from_millis(16));
        assert!(test_bed.wing_anti_ice_system_on() == true);
        test_bed.run_with_delta(Duration::from_secs(1));
        
        assert!(test_bed.left_valve_open_amount()>0.);
        assert!(test_bed.left_valve_controller_timer() <  Duration::from_secs(2));
        assert!(test_bed.right_valve_open_amount()>0.);
        assert!(test_bed.right_valve_controller_timer() <  Duration::from_secs(2));

        test_bed.run_with_delta(Duration::from_secs(30));
        assert!(test_bed.left_valve_controller_timer() == Duration::from_secs(30));
        assert!(test_bed.left_valve_open_amount() == 0.);
        assert!(test_bed.right_valve_controller_timer() == Duration::from_secs(30));
        assert!(test_bed.right_valve_open_amount() == 0.);
        assert!(test_bed.wing_anti_ice_system_on() == false);


    }

    #[test]
    fn wing_anti_ice_valve_open_after_leaving_ground_after_test() {
        let altitude = Length::new::<foot>(500.);
        let mut test_bed = test_bed()
            .idle_eng1()
            .idle_eng2()
            .in_isa_atmosphere(Length::new::<foot>(0.))
            .and_stabilize();
        test_bed.set_on_ground(true);

        test_bed = test_bed.wing_anti_ice_push_button(WingAntiIcePushButtonMode::On);
        test_bed.run_with_delta(Duration::from_secs(31));
        
        assert!(test_bed.wing_anti_ice_system_on() == false);
        assert!(test_bed.left_valve_open_amount() == 0.);
        assert!(test_bed.right_valve_open_amount() == 0.);

        test_bed = test_bed
                .in_isa_atmosphere(altitude)
                .power_eng1()
                .power_eng2()
                .and_stabilize();
        test_bed.set_on_ground(false);
        test_bed.run_with_delta(Duration::from_secs(1));
        
        assert!(test_bed.wing_anti_ice_system_on() == true);
        assert!(test_bed.left_valve_open_amount() > 0.);
        assert!(test_bed.right_valve_open_amount() > 0.);
        assert!(test_bed.left_valve_controller_timer() == Duration::from_secs(0));
        assert!(test_bed.right_valve_controller_timer() == Duration::from_secs(0));

    }

    #[test]
    fn wing_anti_ice_valve_controller_timer_starts_after_landing() {
        let altitude = Length::new::<foot>(500.);
        let mut test_bed = test_bed()
            .idle_eng1()
            .idle_eng2()
            .in_isa_atmosphere(altitude)
            .wing_anti_ice_push_button(WingAntiIcePushButtonMode::On)
            .and_stabilize();
        
        test_bed.set_on_ground(true);
        test_bed.run_with_delta(Duration::from_secs(30));
        assert!(test_bed.left_valve_open() == false);
        assert!(test_bed.right_valve_open() == false);

        test_bed.set_on_ground(false);
        test_bed.run_with_delta(Duration::from_millis(16));

        assert!(test_bed.left_valve_open_amount() > 0.);
        assert!(test_bed.right_valve_open_amount() > 0.);
        assert!(test_bed.left_valve_controller_timer() == Duration::from_secs(0));
        assert!(test_bed.right_valve_controller_timer() == Duration::from_secs(0));

        test_bed.set_on_ground(true);
        test_bed.run_with_delta(Duration::from_millis(500));

        assert!(test_bed.left_valve_open_amount() > 0.);
        assert!(test_bed.right_valve_open_amount() > 0.);
        assert!(test_bed.left_valve_controller_timer() > Duration::from_secs(0));
        assert!(test_bed.right_valve_controller_timer() > Duration::from_secs(0));
        
        test_bed.run_with_delta(Duration::from_secs(30));
        assert!(test_bed.left_valve_open_amount() == 0.);
        assert!(test_bed.right_valve_open_amount() == 0.);


    }

}
