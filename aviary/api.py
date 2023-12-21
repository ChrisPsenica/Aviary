'''
This module is the API for Aviary aircraft analysis code

For users: All built-in Aviary functions, code, and objects 
should be imported from this file.

For developers: All Aviary code which is intended to be
user-facing should be imported to this file.
'''

# TODO: don't rename things here, do it in the entire codebase
# TODO: when documenting methods and classes, make sure to include documentation (printing of docstrings) for everything that's imported in this API
# TODO: remove overload prototype
# TODO: import examples once we settle on those
# TODO: import this in all user-facing files


###################
# General Imports #
###################

from aviary.variable_info.variables import Aircraft, Mission, Dynamic
from aviary.variable_info.options import get_option_defaults, is_option
from aviary.utils.develop_metadata import add_meta_data, update_meta_data
from aviary.variable_info.variable_meta_data import CoreMetaData
from aviary.variable_info.functions import add_aviary_input, add_aviary_output, get_units, override_aviary_vars, setup_trajectory_params
from aviary.utils.merge_hierarchies import merge_hierarchies
from aviary.utils.merge_variable_metadata import merge_meta_data
from aviary.utils.named_values import NamedValues, get_keys, get_items, get_values
from aviary.utils.aviary_values import AviaryValues
from aviary.utils.csv_data_file import read_data_file, write_data_file
from aviary.utils.data_interpolator_builder import build_data_interpolator
from aviary.variable_info.enums import AlphaModes, AnalysisScheme, ProblemType, SpeedType, GASP_Engine_Type, Flap_Type
from aviary.interface.default_phase_info.gasp import phase_info as default_2DOF_phase_info
from aviary.interface.default_phase_info.flops import phase_info as default_height_energy_phase_info
from aviary.interface.default_phase_info.gasp_fiti import create_gasp_based_ascent_phases, create_gasp_based_descent_phases
from aviary.interface.default_phase_info.solved import phase_info as default_solved_phase_info
from aviary.interface.default_phase_info.simple import phase_info as default_simple_phase_info
from aviary.interface.methods_for_level1 import run_level_1
from aviary.interface.methods_for_level1 import run_aviary
from aviary.interface.methods_for_level2 import AviaryProblem
from aviary.interface.utils.check_phase_info import check_phase_info
from aviary.utils.engine_deck_conversion import EngineDeckConverter
from aviary.utils.Fortran_to_Aviary import create_aviary_deck
from aviary.utils.functions import set_aviary_initial_values, get_path
from aviary.utils.options import list_options
from aviary.constants import GRAV_METRIC_GASP, GRAV_ENGLISH_GASP, GRAV_METRIC_FLOPS, GRAV_ENGLISH_FLOPS, GRAV_ENGLISH_LBM, RHO_SEA_LEVEL_ENGLISH, RHO_SEA_LEVEL_METRIC, MU_TAKEOFF, MU_LANDING, PSLS_PSF, TSLS_DEGR, RADIUS_EARTH_METRIC
from aviary.subsystems.test.subsystem_tester import TestSubsystemBuilderBase
from aviary.interface.default_phase_info.flops import default_premission_subsystems, default_mission_subsystems

###################
# Level 3 Imports #
###################

# Miscellaneous
from aviary.interface.methods_for_level2 import PreMissionGroup, PostMissionGroup
from aviary.mission.gasp_based.flight_conditions import FlightConditions
from aviary.subsystems.premission import CorePreMission
from aviary.subsystems.subsystem_builder_base import SubsystemBuilderBase
from aviary.utils.preprocessors import preprocess_options, preprocess_propulsion
from aviary.utils.process_input_decks import create_vehicle
from aviary.utils.functions import create_opts2vals, add_opts2vals, Null
from aviary.variable_info.variables_in import VariablesIn
from aviary.utils.preprocessors import preprocess_crewpayload

# ODEs
# TODO: check and see if this works with both sides, or just GASP
from aviary.mission.gasp_based.ode.base_ode import BaseODE
from aviary.mission.flops_based.ode.landing_ode import LandingODE as DetailedLandingODE
from aviary.mission.flops_based.ode.landing_ode import FlareODE as DetailedFlareODE
from aviary.mission.flops_based.ode.mission_ODE import MissionODE as HeightEnergyMissionODE
from aviary.mission.flops_based.ode.takeoff_ode import TakeoffODE as DetailedTakeoffODE
from aviary.mission.gasp_based.ode.accel_ode import AccelODE as TwoDOFAccelerationODE
from aviary.mission.gasp_based.ode.ascent_ode import AscentODE as TwoDOFAscentODE
from aviary.mission.gasp_based.ode.breguet_cruise_ode import BreguetCruiseODESolution
from aviary.mission.gasp_based.ode.climb_ode import ClimbODE as TwoDOFClimbODE
from aviary.mission.gasp_based.ode.descent_ode import DescentODE as TwoDOFDescentODE
from aviary.mission.gasp_based.ode.flight_path_ode import FlightPathODE as TwoDOFFlightPathODE
from aviary.mission.gasp_based.ode.groundroll_ode import GroundrollODE as TwoDOFGroundrollODE
from aviary.mission.gasp_based.ode.rotation_ode import RotationODE as TwoDOFRotationODE
from aviary.mission.gasp_based.phases.landing_group import LandingSegment as TwoDOFSimplifiedLanding
from aviary.mission.gasp_based.phases.taxi_group import TaxiSegment as AnalyticTaxi
from aviary.mission.flops_based.phases.simplified_takeoff import TakeoffGroup as HeightEnergySimplifiedTakeoff
from aviary.mission.flops_based.phases.simplified_landing import LandingGroup as HeightEnergySimplifiedLanding


# Phase builders
from aviary.mission.flops_based.phases.phase_builder_base import PhaseBuilderBase
# note that this is only for simplified right now
from aviary.mission.flops_based.phases.build_landing import Landing as HeightEnergyLandingPhaseBuilder
# note that this is only for simplified right now
from aviary.mission.flops_based.phases.build_takeoff import Takeoff as HeightEnergyTakeoffPhaseBuilder
from aviary.mission.flops_based.phases.climb_phase import Climb as HeightEnergyClimbPhaseBuilder
from aviary.mission.flops_based.phases.cruise_phase import Cruise as HeightEnergyCruisePhaseBuilder
from aviary.mission.flops_based.phases.descent_phase import Descent as HeightEnergyDescentPhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingApproachToMicP3 as DetailedLandingApproachToMicP3PhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingMicP3ToObstacle as DetailedLandingMicP3ToObstaclePhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingObstacleToFlare as DetailedLandingObstacleToFlarePhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingFlareToTouchdown as DetailedLandingFlareToTouchdownPhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingTouchdownToNoseDown as DetailedLandingTouchdownToNoseDownPhaseBuilder
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingNoseDownToStop as DetailedLandingNoseDownToStopPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffBrakeReleaseToDecisionSpeed as DetailedTakeoffBrakeReleaseToDecisionSpeedPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffDecisionSpeedToRotate as DetailedTakeoffDecisionSpeedToRotatePhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffDecisionSpeedBrakeDelay as DetailedTakeoffDecisionSpeedBrakeDelayPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffRotateToLiftoff as DetailedTakeoffRotateToLiftoffPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffLiftoffToObstacle as DetailedTakeoffLiftoffToObstaclePhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffObstacleToMicP2 as DetailedTakeoffObstacleToMicP2PhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffMicP2ToEngineCutback as DetailedTakeoffMicP2ToEngineCutbackPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffEngineCutback as DetailedTakeoffEngineCutbackPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffEngineCutbackToMicP1 as DetailedTakeoffEngineCutbackToMicP1PhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffMicP1ToClimb as DetailedTakeoffMicP1ToClimbPhaseBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffBrakeToAbort as DetailedTakeoffBrakeToAbortPhaseBuilder

# Phase getters  # TODO these should be going away in favor of phase builders
from aviary.mission.gasp_based.phases.accel_phase import get_accel as get_2DOF_acceleration_phase
from aviary.mission.gasp_based.phases.ascent_phase import get_ascent as get_2DOF_ascent_phase
from aviary.mission.gasp_based.phases.climb_phase import get_climb as get_2DOF_climb_phase
from aviary.mission.gasp_based.phases.desc_phase import get_descent as get_2DOF_descent_phase
from aviary.mission.gasp_based.phases.groundroll_phase import get_groundroll as get_2DOF_groundroll_phase
from aviary.mission.gasp_based.phases.rotation_phase import get_rotation as get_2DOF_rotation_phase


# Trajectory builders
from aviary.mission.flops_based.phases.detailed_landing_phases import LandingTrajectory as DetailedLandingTrajectoryBuilder
from aviary.mission.flops_based.phases.detailed_takeoff_phases import TakeoffTrajectory as DetailedTakeoffTrajectoryBuilder

# SimuPy
from aviary.mission.gasp_based.ode.time_integration_base_classes import SimuPyProblem
from aviary.mission.gasp_based.phases.time_integration_phases import SGMGroundroll, SGMRotation, SGMAscent, SGMAscentCombined, SGMAccel, SGMClimb, SGMCruise, SGMDescent
from aviary.mission.gasp_based.phases.time_integration_traj import TimeIntegrationTrajBase, FlexibleTraj

# Aerodynamics
from aviary.subsystems.aerodynamics.aerodynamics_builder import AerodynamicsBuilderBase
from aviary.subsystems.aerodynamics.aerodynamics_builder import CoreAerodynamicsBuilder
from aviary.subsystems.aerodynamics.flops_based.tabular_aero_group import TabularAeroGroup

# Geometry
from aviary.subsystems.geometry.geometry_builder import GeometryBuilderBase
from aviary.subsystems.geometry.geometry_builder import CoreGeometryBuilder

# Mass
from aviary.subsystems.mass.mass_builder import MassBuilderBase
from aviary.subsystems.mass.mass_builder import CoreMassBuilder

# Propulsion
from aviary.subsystems.propulsion.engine_deck import EngineDeck
from aviary.subsystems.propulsion.engine_model import EngineModel
from aviary.subsystems.propulsion.propulsion_builder import PropulsionBuilderBase
from aviary.subsystems.propulsion.propulsion_builder import CorePropulsionBuilder

# Testing
from aviary.validation_cases.validation_tests import get_flops_inputs, get_flops_outputs
from aviary.validation_cases.validation_data.flops_data.FLOPS_Test_Data import FLOPS_Test_Data
