#Aviary Script To Test Takeoff Partial Derivative Accuracy Against Complex Step Method
#FLOPS Based Test

#---------- Imports ----------
import unittest
import openmdao.api as om
from aviary.mission.flops_based.ode.takeoff_eom import (TakeoffEOM , StallSpeed)
from aviary.subsystems.mass.flops_based.fuel_capacity import WingFuelCapacity
from aviary.models.N3CC.N3CC_data import (detailed_takeoff_climbing, detailed_takeoff_ground, inputs)
from aviary.validation_cases.validation_tests import do_validation_test
from aviary.variable_info.variables import Dynamic, Mission , Aircraft

#---------- User Option ----------
print()
print("OPTIONS")
print("[1] For Stall Velocity (Takeoff)")
print("[2] For Wing Fuel Capacity")
print("[3] All")
print() 

deriv_choice = int(input("Option: "))

#---------- Partial Derivatives ----------
class PartialsCheck(unittest.TestCase):
    
    if deriv_choice == 1:
        #---------- Stall Speed ----------
        def test_stallspeed_derivs(self):
            prob = om.Problem()
            model = prob.model

            time, _ = detailed_takeoff_climbing.get_item('time')
            nn = len(time)
            aviary_options = inputs

            model.add_subsystem('StallSpeed' , StallSpeed(num_nodes = nn) , promotes_inputs = ['*'] , promotes_outputs = ['*'])
            prob.setup(force_alloc_complex = True)

            prob.set_val('area' , val = 576.0) 
            prob.set_val('mass' , val = 300000.0)
            prob.set_val('lift_coefficient_max' , val = 3.5)
            prob.set_val('density' , val = 1.225)

            prob.run_model()

            derivs = prob.check_partials(out_stream = None , method = "cs")
            prob.check_partials(compact_print = True)

    if deriv_choice == 2:
        #---------- Wing Capacity ----------
        def test_wing_capacity(self):
            prob = om.Problem()
            model = prob.model

            model.add_subsystem('WingCapacity' , WingFuelCapacity() , promotes_inputs = ['*'] , promotes_outputs = ['*'])
            prob.setup(force_alloc_complex = True)

            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_TERM_A , val = 0.0) 
            prob.set_val(Aircraft.Wing.AREA , val = 1370.0)
            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_AREA , val = 1200.0)
            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_TERM_B , val = 1.2)
            prob.set_val(Aircraft.Fuel.DENSITY_RATIO , val = 1.0) 
            prob.set_val(Aircraft.Fuel.CAPACITY_FACTOR , val = 23) 
            prob.set_val(Aircraft.Wing.SPAN , val = 117.83) 
            prob.set_val(Aircraft.Wing.TAPER_RATIO , val = 10.0) 
            prob.set_val(Aircraft.Wing.THICKNESS_TO_CHORD , val = 0.13) 

            prob.run_model()

            derivs = prob.check_partials(out_stream = None , method = "cs")
            prob.check_partials(compact_print = True)

    if deriv_choice == 3:
        def test_stallspeed_derivs(self):
            prob = om.Problem()
            model = prob.model

            time, _ = detailed_takeoff_climbing.get_item('time')
            nn = len(time)
            aviary_options = inputs

            model.add_subsystem('StallSpeed' , StallSpeed(num_nodes = nn) , promotes_inputs = ['*'] , promotes_outputs = ['*'])
            prob.setup(force_alloc_complex = True)

            prob.set_val('area' , val = 576.0) 
            prob.set_val('mass' , val = 300000.0)
            prob.set_val('lift_coefficient_max' , val = 3.5)
            prob.set_val('density' , val = 1.225)

            prob.run_model()

            derivs = prob.check_partials(out_stream = None , method = "cs")
            prob.check_partials(compact_print = True)

        def test_wing_capacity(self):
            prob = om.Problem()
            model = prob.model

            model.add_subsystem('WingCapacity' , WingFuelCapacity() , promotes_inputs = ['*'] , promotes_outputs = ['*'])
            prob.setup(force_alloc_complex = True)

            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_TERM_A , val = 0.0) 
            prob.set_val(Aircraft.Wing.AREA , val = 1370.0)
            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_AREA , val = 1200.0)
            prob.set_val(Aircraft.Fuel.WING_REF_CAPACITY_TERM_B , val = 1.2)
            prob.set_val(Aircraft.Fuel.DENSITY_RATIO , val = 1.0) 
            prob.set_val(Aircraft.Fuel.CAPACITY_FACTOR , val = 23) 
            prob.set_val(Aircraft.Wing.SPAN , val = 117.83) 
            prob.set_val(Aircraft.Wing.TAPER_RATIO , val = 10.0) 
            prob.set_val(Aircraft.Wing.THICKNESS_TO_CHORD , val = 0.13) 

            prob.run_model()

            derivs = prob.check_partials(out_stream = None , method = "cs")
            prob.check_partials(compact_print = True)

#---------- Output ----------
if __name__ == "__main__":
    unittest.main()