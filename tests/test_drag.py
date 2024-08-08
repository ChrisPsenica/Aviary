#Plots of timeseries data to compare drag for different mach numbers

#---------- Imports ----------
import matplotlib.pyplot as plt
import random as rd
from run_simulation import run_simulation , Test_ml , Time_ml , current_mach

#---------- Parameters Of Mission ----------
initial_mach = 0.2
go_to_mach = 0.3
mach_step = 0.02
final_mach = initial_mach
lower_mach_limit = initial_mach - 0.02
upper_mach_limit = initial_mach + 0.02
report = 1
run_count = (go_to_mach - initial_mach) / mach_step + 2


phase = "phase_info_FwFm"
test = "test_drag"

Opt_mach = False 
Opt_alt = False 
Opt_mass = True 
takeoff = False
landing = False

#---------- Get Data ----------
for i in range(int(run_count)):
    Test_ml , Time_ml , current_mach = run_simulation(initial_mach , final_mach , lower_mach_limit , upper_mach_limit , 
                                                      report , phase, test, Opt_mach , Opt_alt, 
                                                      Opt_mass , takeoff , landing)
    initial_mach = initial_mach + mach_step
    lower_mach_limit = lower_mach_limit + mach_step
    upper_mach_limit = upper_mach_limit + 0.02
    report = report + 1

#---------- Plot Data ----------
colors = ['r' , 'g' , 'b' , 'c' , 'm' , 'y']
plt.figure()
for i in range(int(run_count)):
    plt.plot(Time_ml[i] , Test_ml[i] , label = current_mach[i] , color = colors[rd.randint(0 , 5)])
plt.xlabel("Time (seconds)")
plt.ylabel("Drag (Lbf)")
plt.title("Drag Vs. Time")
plt.legend()
plt.grid()
plt.show()