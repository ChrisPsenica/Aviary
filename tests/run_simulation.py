#Chris Psenica
#Aviary Script For Testing Drag Calculations
#Level 2

#---------- Imports ----------
import openmdao.api as om
import aviary.api as av
import pandas as pd
import os
from aviary.examples.external_subsystems.extract_val_drag_subsystem.drag_subsystem_builder import CoreAerodynamicsBuilder , AerodynamicsBuilderBase

#---------- Time, Drag, and Mach Matrices ----------
Time_ml = []
Test_ml = []
current_mach = []

#---------- Run Simulation Function ----------
def run_simulation(initial_mach , final_mach , lower_mach_limit , upper_mach_limit , report , phase = str , test = str , Opt_mach = False , Opt_alt = False , Opt_mass = True , takeoff = False , landing = False):

    #---------- Phase Info ----------
    
    phase_info_cessna = {
        "pre_mission": {"include_takeoff": takeoff, "optimize_mass": Opt_mass},
        "climb_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 3,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 3,
                "solve_for_distance": True,
                "initial_mach": (initial_mach, "unitless"),
                "final_mach": (0.5, "unitless"),
                "mach_bounds": ((lower_mach_limit, 0.52), "unitless"),
                "initial_altitude": (0.0, "ft"),
                "final_altitude": (35000.0, "ft"),
                "altitude_bounds": ((0.0, 35500.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": True,
                "constrain_final": False,
                "fix_duration": False,
                "no_descent": True,
                "initial_bounds": ((0.0, 0.0), "min"),
                "duration_bounds": ((16.5, 49.5), "min"),
            },
            "initial_guesses": {"time": ([0.0, 33.0], "min")},
        },
        "cruise_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 3,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 3,
                "solve_for_distance": True,
                "initial_mach": (0.5, "unitless"),
                "final_mach": (0.5, "unitless"),
                "mach_bounds": ((0.48, 0.52), "unitless"),
                "initial_altitude": (35000.0, "ft"),
                "final_altitude": (35000.0, "ft"),
                "altitude_bounds": ((34500.0, 35500.0), "ft"),
                "throttle_enforcement": "boundary_constraint",
                "fix_initial": False,
                "constrain_final": False,
                "fix_duration": False,
                "initial_bounds": ((16.5, 49.5), "min"),
                "duration_bounds": ((34.5, 103.5), "min"),
            },
            "initial_guesses": {"time": ([33.0, 69.0], "min")},
        },
        "descent_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 3,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 3,
                "solve_for_distance": True,
                "initial_mach": (0.5, "unitless"),
                "final_mach": (final_mach, "unitless"),
                "mach_bounds": ((upper_mach_limit, 0.52), "unitless"),
                "initial_altitude": (35000.0, "ft"),
                "final_altitude": (500.0, "ft"),
                "altitude_bounds": ((0.0, 35500.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": False,
                "constrain_final": True,
                "fix_duration": False,
                "no_climb": True,
                "initial_bounds": ((51.0, 153.0), "min"),
                "duration_bounds": ((24.0, 72.0), "min"),
            },
            "initial_guesses": {"time": ([102.0, 48.0], "min")},
        },
        "post_mission": {
            "include_landing": landing,
            "constrain_range": True,
            "target_range": (720.91, "nmi"),
        },
    }

    phase_info_c40 = {
    "pre_mission": {"include_takeoff": False, "optimize_mass": True},
    "climb_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "use_polynomial_control": True,
            "num_segments": 3,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.3, "unitless"),
            "final_mach": (0.78, "unitless"),
            "mach_bounds": ((0.27999999999999997, 0.8), "unitless"),
            "initial_altitude": (0.0, "ft"),
            "final_altitude": (32000.0, "ft"),
            "altitude_bounds": ((0.0, 32500.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": True,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((0.0, 0.0), "min"),
            "duration_bounds": ((22.5, 67.5), "min"),
        },
        "initial_guesses": {"time": ([0.0, 45.0], "min")},
    },
    "cruise_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "use_polynomial_control": True,
            "num_segments": 3,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.78, "unitless"),
            "final_mach": (0.78, "unitless"),
            "mach_bounds": ((0.76, 0.8), "unitless"),
            "initial_altitude": (32000.0, "ft"),
            "final_altitude": (32000.0, "ft"),
            "altitude_bounds": ((31500.0, 32500.0), "ft"),
            "throttle_enforcement": "boundary_constraint",
            "fix_initial": False,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((22.5, 67.5), "min"),
            "duration_bounds": ((159.0, 477.0), "min"),
        },
        "initial_guesses": {"time": ([45.0, 318.0], "min")},
    },
    "descent_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "use_polynomial_control": True,
            "num_segments": 3,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.78, "unitless"),
            "final_mach": (0.3, "unitless"),
            "mach_bounds": ((0.27999999999999997, 0.8), "unitless"),
            "initial_altitude": (32000.0, "ft"),
            "final_altitude": (0.0, "ft"),
            "altitude_bounds": ((0.0, 32500.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": False,
            "constrain_final": True,
            "fix_duration": False,
            "initial_bounds": ((181.5, 544.5), "min"),
            "duration_bounds": ((14.5, 43.5), "min"),
        },
        "initial_guesses": {"time": ([363.0, 29.0], "min")},
    },
    "post_mission": {
        "include_landing": False,
        "constrain_range": True,
        "target_range": (3200.35, "nmi"),
    },
}

    phase_info_c5 = {
        "pre_mission": {"include_takeoff": takeoff, "optimize_mass": Opt_mass},
        "climb_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 2,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 5,
                "solve_for_distance": False,
                "initial_mach": (initial_mach, "unitless"),
                "final_mach": (0.77, "unitless"),
                "mach_bounds": ((lower_mach_limit, 0.79), "unitless"),
                "initial_altitude": (35.0, "ft"),
                "final_altitude": (34000.0, "ft"),
                "altitude_bounds": ((25.0, 34500.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": True,
                "constrain_final": False,
                "fix_duration": False,
                "initial_bounds": ((0.0, 0.0), "min"),
                "duration_bounds": ((49.5, 148.5), "min"),
            },
            "initial_guesses": {"time": ([0, 99], "min")},
        },
        "cruise_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 2,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 5,
                "solve_for_distance": False,
                "initial_mach": (0.77, "unitless"),
                "final_mach": (0.77, "unitless"),
                "mach_bounds": ((0.75, 0.79), "unitless"),
                "initial_altitude": (34000.0, "ft"),
                "final_altitude": (34000.0, "ft"),
                "altitude_bounds": ((33500.0, 34500.0), "ft"),
                "throttle_enforcement": "boundary_constraint",
                "fix_initial": False,
                "constrain_final": False,
                "fix_duration": False,
                "initial_bounds": ((49.5, 148.5), "min"),
                "duration_bounds": ((51.0, 153.0), "min"),
            },
            "initial_guesses": {"time": ([99, 102], "min")},
        },
        "descent_1": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 2,
                "use_polynomial_control": True,
                "num_segments": 3,
                "order": 5,
                "solve_for_distance": False,
                "initial_mach": (0.77, "unitless"),
                "final_mach": (final_mach, "unitless"),
                "mach_bounds": ((upper_mach_limit, 0.79), "unitless"),
                "initial_altitude": (34000.0, "ft"),
                "final_altitude": (500.0, "ft"),
                "altitude_bounds": ((0.0, 34500.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": False,
                "constrain_final": True,
                "fix_duration": False,
                "initial_bounds": ((100.5, 301.5), "min"),
                "duration_bounds": ((49.0, 147.0), "min"),
            },
            "initial_guesses": {"time": ([201, 98], "min")},
        },
        "post_mission": {
            "include_landing": landing,
            "constrain_range": True,
            "target_range": (1934, "nmi"),
        },
    }

    phase_info_FwFm = {
        "pre_mission": {"include_takeoff": takeoff, "optimize_mass": Opt_mass},
        "climb": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 1,
                "num_segments": 5,
                "order": 3,
                "solve_for_distance": False,
                "initial_mach": (initial_mach, "unitless"),
                "final_mach": (0.72, "unitless"),
                "mach_bounds": ((lower_mach_limit, 0.74), "unitless"),
                "initial_altitude": (0.0, "ft"),
                "final_altitude": (32000.0, "ft"),
                "altitude_bounds": ((0.0, 34000.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": True,
                "constrain_final": False,
                "fix_duration": False,
                "initial_bounds": ((0.0, 0.0), "min"),
                "duration_bounds": ((64.0, 192.0), "min"),
            },
            "initial_guesses": {"time": ([0, 128], "min")},
        },
        "cruise": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 1,
                "num_segments": 5,
                "order": 3,
                "solve_for_distance": False,
                "initial_mach": (0.72, "unitless"),
                "final_mach": (0.72, "unitless"),
                "mach_bounds": ((0.7, 0.74), "unitless"),
                "initial_altitude": (32000.0, "ft"),
                "final_altitude": (34000.0, "ft"),
                "altitude_bounds": ((23000.0, 38000.0), "ft"),
                "throttle_enforcement": "boundary_constraint",
                "fix_initial": False,
                "constrain_final": False,
                "fix_duration": False,
                "initial_bounds": ((64.0, 192.0), "min"),
                "duration_bounds": ((56.5, 169.5), "min"),
            },
            "initial_guesses": {"time": ([128, 113], "min")},
        },
        "descent": {
            "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
            "user_options": {
                "optimize_mach": Opt_mach,
                "optimize_altitude": Opt_alt,
                "polynomial_control_order": 1,
                "num_segments": 5,
                "order": 3,
                "solve_for_distance": False,
                "initial_mach": (0.72, "unitless"),
                "final_mach": (final_mach, "unitless"),
                "mach_bounds": ((upper_mach_limit, 0.74), "unitless"),
                "initial_altitude": (34000.0, "ft"),
                "final_altitude": (500.0, "ft"),
                "altitude_bounds": ((0.0, 38000.0), "ft"),
                "throttle_enforcement": "path_constraint",
                "fix_initial": False,
                "constrain_final": True,
                "fix_duration": False,
                "initial_bounds": ((120.5, 361.5), "min"),
                "duration_bounds": ((29.0, 87.0), "min"),
            },
            "initial_guesses": {"time": ([241, 58], "min")},
        },
        "post_mission": {
            "include_landing": landing,
            "target_range": (1906, "nmi"),
        },
    }

    #---------- Lift & Drag Parameters ----------
    subsystem_options = {'core_aerodynamics':
                        {'method': 'low_speed',
                        'ground_altitude': 0.,  # units='ft'
                        'angles_of_attack': [
                            -5.0, -4.0, -3.0, -2.0, -1.0,
                            0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
                            6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
                            12.0, 13.0, 14.0, 15.0],  # units='deg'
                        'lift_coefficients': [
                            0.01, 0.1, 0.2, 0.3, 0.4,
                            0.5178, 0.6, 0.75, 0.85, 0.95, 1.05,
                            1.15, 1.25, 1.35, 1.5, 1.6, 1.7,
                            1.8, 1.85, 1.9, 1.95],
                        'drag_coefficients': [
                            0.04, 0.02, 0.01, 0.02, 0.04,
                            0.0674, 0.065, 0.065, 0.07, 0.072, 0.076,
                            0.084, 0.09, 0.10, 0.11, 0.12, 0.13,
                            0.15, 0.16, 0.18, 0.20],
                        'lift_coefficient_factor': 1.,
                        'drag_coefficient_factor': 1.}}

    phase_info_takeoff = {
        "pre_mission": {"include_takeoff": takeoff, "optimize_mass": Opt_mass},
        'AB': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': True,
                'ground_roll': True,
                'duration_ref': (100., 'kn'),
                'duration_bounds': ((100., 500.), 'kn'),
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(0., 2.e3), 'ft'],
                'time': [(0., 20.), 's'],
                'velocity': [(1., 120.), 'kn'],
                'mass': [(175.e3, 174.85e3), 'lbm'],
            },
        },
        'rotate': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': True,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((1.e3, 3.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((200., 2.e3), 'ft'),
                'mach_bounds': ((lower_mach_limit, 0.2), 'unitless'),
                'polynomial_control_order': 1,
                'throttle_enforcement': 'boundary_constraint',
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'rotation': True,
                'initial_mach': (initial_mach, 'unitless'),
                'final_mach': (0.2, 'unitless'),
                'initial_altitude': (0., 'ft'),
                'final_altitude': (0., 'ft'),
                'constraints': {
                    'normal_force': {
                        'equals': 0.,
                        'loc': 'final',
                        'units': 'lbf',
                        'type': 'boundary',
                        'ref': 10.e5,
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(2.e3, 1.e3), 'ft'],
                'time': [(20., 25.), 's'],
                'mass': [(174.85e3, 174.84e3), 'lbm'],
                'alpha': [(0., 12.), 'deg'],
            },
        },
        'BC': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((1., 16.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((500., 1500.), 'ft'),
                'mach_bounds': ((0.2, 0.22), 'unitless'),
                'altitude_bounds': ((0., 250.), 'ft'),
                'initial_mach': (0.2, 'unitless'),
                'final_mach': (0.22, 'unitless'),
                'initial_altitude': (0., 'ft'),
                'final_altitude': (50., 'ft'),
                'polynomial_control_order': 1,
                'throttle_enforcement': 'boundary_constraint',
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'rotation': False,
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(3.e3, 1.e3), 'ft'],
                'time': [(25., 35.), 's'],
                'mass': [(174.84e3, 174.82e3), 'lbm'],
            },
        },
        'CD_to_P2': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((1.e3, 20.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((3.e3, 20.e3), 'ft'),
                'mach_bounds': ((0.22, 0.3), 'unitless'),
                'altitude_bounds': ((0., 985.), 'ft'),
                'initial_mach': (0.22, 'unitless'),
                'final_mach': (0.3, 'unitless'),
                'initial_altitude': (50., 'ft'),
                'final_altitude': (985., 'ft'),
                'polynomial_control_order': 1,
                'throttle_enforcement': 'boundary_constraint',
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'constraints': {
                    'altitude': {
                        'equals': 985.,
                        'loc': 'final',
                        'units': 'ft',
                        'type': 'boundary',
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(4.e3, 10.e3), 'ft'],
                'time': [(35., 60.), 's'],
                'mass': [(174.82e3, 174.8e3), 'lbm'],
            },
        },
        'P2_to_DE': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((1.e3, 20.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((3.e3, 20.e3), 'ft'),
                'mach_bounds': ((0.24, 0.32), 'unitless'),
                'altitude_bounds': ((985., 1100.), 'ft'),
                'initial_mach': (0.3, 'unitless'),
                'final_mach': (0.3, 'unitless'),
                'initial_altitude': (985., 'ft'),
                'final_altitude': (1100., 'ft'),
                'polynomial_control_order': 1,
                'throttle_enforcement': 'path_constraint',
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'constraints': {
                    'distance': {
                        'upper': 19.e3,
                        'ref': 20.e3,
                        'loc': 'final',
                        'units': 'ft',
                        'type': 'boundary',
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(10.e3, 14.e3), 'ft'],
                'time': [(60., 80.), 's'],
                'mass': [(174.8e3, 174.5e3), 'lbm'],
            },
        },
        'DE': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((500., 30.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((50., 5000.), 'ft'),
                'mach_bounds': ((0.24, 0.32), 'unitless'),
                'altitude_bounds': ((985., 1.5e3), 'ft'),
                'initial_mach': (0.3, 'unitless'),
                'final_mach': (0.3, 'unitless'),
                'initial_altitude': (1100., 'ft'),
                'final_altitude': (1200., 'ft'),
                'polynomial_control_order': 2,
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'throttle_enforcement': 'path_constraint',
                'constraints': {
                    'flight_path_angle': {
                        'equals': 4.,
                        'loc': 'final',
                        'units': 'deg',
                        'type': 'boundary',
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(18.e3, 2.e3), 'ft'],
                'mass': [(174.5e3, 174.4e3), 'lbm'],
                'time': [(80., 85.), 's'],
            },
        },
        'EF_to_P1': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((500., 50.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((1.e3, 20.e3), 'ft'),
                'mach_bounds': ((0.24, 0.32), 'unitless'),
                'altitude_bounds': ((1.1e3, 1.2e3), 'ft'),
                'initial_mach': (0.3, 'unitless'),
                'final_mach': (0.3, 'unitless'),
                'initial_altitude': (1100., 'ft'),
                'final_altitude': (1200., 'ft'),
                'polynomial_control_order': 1,
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'throttle_enforcement': 'bounded',
                'constraints': {
                    'distance': {
                        'equals': 21325.,
                        'units': 'ft',
                        'type': 'boundary',
                        'loc': 'final',
                        'ref': 30.e3,
                    },
                    'flight_path_angle': {
                        'equals': 4.,
                        'loc': 'final',
                        'units': 'deg',
                        'type': 'boundary',
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(20.e3, 1325.), 'ft'],
                'mass': [(174.4e3, 174.3e3), 'lbm'],
                'time': [(85., 90.), 's'],
            },
        },
        'EF_past_P1': {
            'user_options': {
                'num_segments': 3,
                'order': 3,
                'fix_initial': False,
                'ground_roll': False,
                'clean': False,
                'initial_ref': (1.e3, 'ft'),
                'initial_bounds': ((20.e3, 50.e3), 'ft'),
                'duration_ref': (1.e3, 'ft'),
                'duration_bounds': ((100., 50.e3), 'ft'),
                'mach_bounds': ((upper_mach_limit, 0.3), 'unitless'),
                'altitude_bounds': ((1.e3, 3.e3), 'ft'),
                'initial_mach': (0.3, 'unitless'),
                'final_mach': (final_mach, 'unitless'),
                'initial_altitude': (1200., 'ft'),
                'final_altitude': (2000., 'ft'),
                'polynomial_control_order': 1,
                'optimize_mach': Opt_mach,
                'optimize_altitude': Opt_alt,
                'throttle_enforcement': 'boundary_constraint',
                'constraints': {
                    'flight_path_angle': {
                        'equals': 4.,
                        'loc': 'final',
                        'units': 'deg',
                        'type': 'boundary',
                    },
                    'distance': {
                        'equals': 30.e3,
                        'units': 'ft',
                        'type': 'boundary',
                        'loc': 'final',
                        'ref': 30.e3,
                    },
                },
            },
            'subsystem_options': subsystem_options,
            'initial_guesses': {
                'distance': [(21325., 50.e3), 'ft'],
                'mass': [(174.3e3, 174.2e3), 'lbm'],
                'time': [(90., 180.), 's'],
            },
        },
        "post_mission": {
            "include_landing": landing,
            "constrain_range": False,
        },
    }

    if phase == "phase_info_cessna":
        phase_info = phase_info_cessna
        csv = "CSV_files/Cessna-500.csv"

    elif phase == "phase_info_c5":
        phase_info = phase_info_c5
        csv = "CSV_files/c5.csv"

    elif phase == "phase_info_FwFm":
        phase_info = phase_info_FwFm
        csv = "CSV_files/FwFm.csv"
    else:
        phase_info == "phase_info_c40"
        csv = "CSV_files/c40.csv"

    #---------- Run Simulation ----------
    extract_val = AerodynamicsBuilderBase()
    phase_info['climb']['external_subsystems'] = [extract_val]
    phase_info['cruise']['external_subsystems'] = [extract_val]
    phase_info['descent']['external_subsystems'] = [extract_val]
    prob = av.AviaryProblem()
    prob.load_inputs(csv , phase_info)
    prob.check_and_preprocess_inputs()
    prob.add_pre_mission_systems()
    prob.add_phases()
    prob.add_post_mission_systems()
    prob.link_phases()
    prob.add_driver("SLSQP" , max_iter = 5)
    prob.add_design_variables()
    prob.add_objective('mass')
    prob.setup()
    prob.set_initial_guesses()
    prob.run_aviary_problem(record_filename = 'mission.db' , suppress_solver_print = True , make_plots = True)
    #prob.check_partials()

    if report == 1:
        file_path = os.path.join("reports" , test , "mission_timeseries_data.csv")
    else:
        file_path = os.path.join("reports" , test + str(report) , "mission_timeseries_data.csv")
    data = pd.read_csv(file_path)
    
    current_mach.append(str(initial_mach))
    Time_ml.append(data['time (s)'].tolist())
    if test == "test_drag":
        Test_ml.append(data['drag (lbf)'].tolist())
    elif test == "test_mass":
        Test_ml.append(data['mass (kg)'].tolist())

    return Test_ml , Time_ml , current_mach