# 1. FLOPS-BASED ALTERNATE DRAG SUBSYSTEM (MK2) - How To Setup In Aviary

## Adding MK2 To A Mission
1.) After downloading MK2, place the drag_MK2 folder in: 

    aviary/examples/external_subsystems

2.) To implement the subsystem into an Aviary mission add the following imports to the run script file (where you create and run the Aviary problem):

    from aviary.variable_info.enums import LegacyCode
    from aviary.examples.external_subsystems.drag_MK2.drag_subsystem_builder import CoreAerodynamicsBuilder

3.) Lastly, to the same file as step 2, add the following argument to the Aviary problem after running `prob.check_and_preprocess_inputs()`:

    prob.core_subsystems["aerodynamics"] = CoreAerodynamicsBuilder('custom_aero' , code_origin = LegacyCode.FLOPS)


The new MK2 drag subsystem will now be the default drag subsystem for the Aviary mission that you specified.

## Making MK2 The Default Drag Subsystem In Aviary
If you wish to permanently make MK2 the default drag subsystem for all missions within Aviary then after the first step from section 1 (and disregarding steps 2 and 3 from section 1) change the following import in `aviary/subsystems/aerodynamics/aerodynamics_builder.py` from:

    from aviary.subsystems.aerodynamics.flops_based.computed_aero_group import ComputedAeroGroup

to

    from aviary.examples.external_subsystems.drag_MK2.computed_aero_group import ComputedAeroGroup

Aviary will now use MK2 as its default, meaning you will no longer have to specify in the problem setup which drag subsystem to use.

# 2. MK2 Drag Subsystem General Information

## What MK2 Can Be Used For
MK2 was designed specifically for the 22 aerodynamic tables taken from FLOPS and should only be used with these tables. These 22 tables are included in `interpolate.py`. Using MK2 with any other data table may produce unrealistic results and be an unstable solution.

## How MK2 Handles Derivatives
Currently, MK2 does not support any direct derivative computation. MK2 operates by approximating the drag derivatives via finite difference by default. Please note that the complex step method for derivative computations is also supported but this method tends to slow the optimizer down more in comparison to the finite difference default. The remaining derivatives for all aerodynamic calculations follows Aviarys norm of using analytical derivatives. 

Future developments for MK2 include implementing automatic differentiation which will help with derivative accuracy and ultimately allow the optimizer to converge more quickly.
\
\
\
\
\
\
\
\
\
copyright 2024 \
release version 1.1 