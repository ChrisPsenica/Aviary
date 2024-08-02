#Chris Psenica
#Aviary Test For A C5 Galaxy
#Level 2

#---------- Imports ----------
import aviary.api as av

#---------- Opt Parameters ----------
Opt_mach = False
Opt_alt = False
Opt_mass = True
takeoff = False
landing = True
polynomial_control_order = 2
num_segments = 3

#---------- Phase Info ----------
phase_info = {
    "pre_mission": {"include_takeoff": takeoff, "optimize_mass": Opt_mass},
    "climb_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": Opt_mach,
            "optimize_altitude": Opt_alt,
            "polynomial_control_order": polynomial_control_order,
            "use_polynomial_control": True,
            "num_segments": num_segments,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.2, "unitless"),
            "final_mach": (0.5, "unitless"),
            "mach_bounds": ((0.18, 0.52), "unitless"),
            "initial_altitude": (35.0, "ft"),
            "final_altitude": (20000.0, "ft"),
            "altitude_bounds": ((25.0, 20500.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": True,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((0.0, 0.0), "min"),
            "duration_bounds": ((49.0, 147.0), "min"),
        },
        "initial_guesses": {"time": ([0, 98], "min")},
    },
    "cruise_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": Opt_mach,
            "optimize_altitude": Opt_alt,
            "polynomial_control_order": polynomial_control_order,
            "use_polynomial_control": True,
            "num_segments": num_segments,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.5, "unitless"),
            "final_mach": (0.5, "unitless"),
            "mach_bounds": ((0.48, 0.52), "unitless"),
            "initial_altitude": (20000.0, "ft"),
            "final_altitude": (20000.0, "ft"),
            "altitude_bounds": ((19500.0, 20500.0), "ft"),
            "throttle_enforcement": "boundary_constraint",
            "fix_initial": False,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((49.0, 147.0), "min"),
            "duration_bounds": ((26.0, 78.0), "min"),
        },
        "initial_guesses": {"time": ([98, 52], "min")},
    },
    "climb_2": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": Opt_mach,
            "optimize_altitude": Opt_alt,
            "polynomial_control_order": polynomial_control_order,
            "use_polynomial_control": True,
            "num_segments": num_segments,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.5, "unitless"),
            "final_mach": (0.77, "unitless"),
            "mach_bounds": ((0.48, 0.79), "unitless"),
            "initial_altitude": (20000.0, "ft"),
            "final_altitude": (34000.0, "ft"),
            "altitude_bounds": ((19500.0, 34500.0), "ft"),
            "throttle_enforcement": "boundary_constraint",
            "fix_initial": False,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((75.0, 225.0), "min"),
            "duration_bounds": ((24.0, 72.0), "min"),
        },
        "initial_guesses": {"time": ([150, 48], "min")},
    },
    "cruise_2": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": Opt_mach,
            "optimize_altitude": Opt_alt,
            "polynomial_control_order": polynomial_control_order,
            "use_polynomial_control": True,
            "num_segments": num_segments,
            "order": 3,
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
            "initial_bounds": ((99.0, 297.0), "min"),
            "duration_bounds": ((24.5, 73.5), "min"),
        },
        "initial_guesses": {"time": ([198, 49], "min")},
    },
    "descent_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": Opt_mach,
            "optimize_altitude": Opt_alt,
            "polynomial_control_order": polynomial_control_order,
            "use_polynomial_control": True,
            "num_segments": num_segments,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.77, "unitless"),
            "final_mach": (0.2, "unitless"),
            "mach_bounds": ((0.18, 0.79), "unitless"),
            "initial_altitude": (34000.0, "ft"),
            "final_altitude": (500.0, "ft"),
            "altitude_bounds": ((0.0, 34500.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": False,
            "constrain_final": True,
            "fix_duration": False,
            "initial_bounds": ((123.5, 370.5), "min"),
            "duration_bounds": ((51.5, 154.5), "min"),
        },
        "initial_guesses": {"time": ([247, 103], "min")},
    },
    "post_mission": {
        "include_landing": landing,
        "constrain_range": True,
        "target_range": (1983, "nmi"),
    },
}

#---------- Run Aviary ----------
C5_csv = "define_C5.csv"

prob = av.AviaryProblem()
prob.load_inputs(C5_csv , phase_info)
prob.check_and_preprocess_inputs()
prob.add_pre_mission_systems()
prob.add_phases()
prob.add_post_mission_systems()
prob.link_phases()
prob.add_driver("SLSQP" , max_iter = 1500)
prob.add_design_variables()
prob.add_objective(objective_type = "mass", ref=-1e5)
prob.setup()
prob.set_initial_guesses()
prob.run_aviary_problem(record_filename = 'Level2_C5.db' , suppress_solver_print = True , make_plots = True)
prob.check_partials()