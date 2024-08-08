# Drag subsystem to more accurately compute drag 
# 8/6/2024
# Chris P.

"""
Define subsystem builder for Aviary core aerodynamics using a new interpolation/extrapolation scheme.

Classes
-------
AerodynamicsBuilderBase : the interface for an aerodynamics subsystem builder.

CoreAerodynamicsBuilder : the interface for Aviary's core aerodynamics subsystem builder
"""

#---------- imports ----------
import numpy as np
import openmdao.api as om

from aviary.variable_info.variables import Aircraft, Dynamic
from aviary.subsystems.subsystem_builder_base import SubsystemBuilderBase
from aviary.subsystems.aerodynamics.flops_based.aero_report import AeroReport
from aviary.subsystems.aerodynamics.flops_based.design import Design
from aviary.examples.external_subsystems.drag_MK2.computed_aero_group import ComputedAeroGroup
from aviary.variable_info.enums import LegacyCode



FLOPS = LegacyCode.FLOPS

_default_name = 'aerodynamics'


class AerodynamicsBuilderBase(SubsystemBuilderBase):
    def __init__(self, name=None, meta_data=None):
        if name is None:
            name = _default_name

        super().__init__(name=name, meta_data=meta_data)

    def mission_inputs(self, **kwargs):
        return ['*']

    def mission_outputs(self, **kwargs):
        return ['*']


class CoreAerodynamicsBuilder(AerodynamicsBuilderBase):
    def __init__(self, name=None, meta_data=None, code_origin=None):
        if name is None:
            name = 'custom_aero'

        self.code_origin = code_origin

        super().__init__(name=name, meta_data=meta_data)

    def build_pre_mission(self, aviary_inputs):
        aero_group = om.Group()
        aero_group.add_subsystem(
            'design', Design(aviary_options=aviary_inputs),
            promotes_inputs=['*'],
            promotes_outputs=['*'])

        aero_group.add_subsystem(
            'aero_report', AeroReport(aviary_options=aviary_inputs),
            promotes_inputs=['*'],
            promotes_outputs=['*'])

        return aero_group

    def build_mission(self, num_nodes, aviary_inputs, **kwargs):
        aero_group = ComputedAeroGroup(num_nodes=num_nodes,
                                        aviary_options=aviary_inputs,
                                        **kwargs)

        return aero_group

    def mission_inputs(self, **kwargs):
        promotes = [Dynamic.Mission.STATIC_PRESSURE,
                        Dynamic.Mission.MACH,
                        Dynamic.Mission.TEMPERATURE,
                        Dynamic.Mission.MASS,
                        'aircraft:*', 'mission:*']

        return promotes

    def mission_outputs(self, **kwargs):
        promotes = [Dynamic.Mission.DRAG, Dynamic.Mission.LIFT]


        return promotes

    def get_parameters(self, aviary_inputs=None, phase_info=None):
        """
        Return a dictionary of fixed values for the subsystem.

        Optional, used if subsystems have fixed values.

        Used in the phase builders (e.g. cruise_phase.py) when other parameters are added to the phase.

        This is distinct from `get_design_vars` in a nuanced way. Design variables
        are variables that are optimized by the problem that are not at the phase level.
        An example would be something that occurs in the pre-mission level of the problem.
        Parameters are fixed values that are held constant throughout a phase, but if
        `opt=True`, they are able to change during the optimization.

        Parameters
        ----------
        phase_info : dict
            The phase_info subdict for this phase.

        Returns
        -------
        fixed_values : dict
            A dictionary where the keys are the names of the fixed variables
            and the values are dictionaries with the following keys:

            - 'value': float or array
                The fixed value for the variable.
            - 'units': str
                The units for the fixed value (optional).
            - any additional keyword arguments required by OpenMDAO for the fixed
              variable.
        """
        num_engine_type = len(aviary_inputs.get_val(Aircraft.Engine.NUM_ENGINES))
        params = {}

        param_vars = [Aircraft.Nacelle.CHARACTERISTIC_LENGTH,
                        Aircraft.Nacelle.FINENESS,
                        Aircraft.Nacelle.LAMINAR_FLOW_LOWER,
                        Aircraft.Nacelle.LAMINAR_FLOW_UPPER,
                        Aircraft.Nacelle.WETTED_AREA]
        for var in param_vars:
            params[var] = {'shape': (num_engine_type, ), 'static_target': True}

        return params