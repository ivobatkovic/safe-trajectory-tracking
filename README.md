# Safe Trajectory Tracking in Uncertain Environments
<p align="center">
    <img src="https://img.shields.io/badge/Developed%20and%20tested%20on-macOS%2011.0%20Matlab%202020b%7C%20Windows%2010%20Matlab%202021a-informational"/>
    <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-informational"/></a>
</p>

This repository contains all Matlab code that generates the simulation results presented in [Safe Trajectory Tracking in Uncertain Environments](https://arxiv.org/abs/2001.11602)

Repository structure:
```bash
safe_trajectory_tracking     # root
├── code                     # simulation code
│   ├──run_all_experiments.m
│   ├──utils                 # utility functions
│   ├──vehicle               # simulation in Section IV-A
│   ├──double_integrator     # simulation in Section IV-B
│   └──robotic_manipulator   # simulation in Section IV-C
└── writing                  # paper preprint
```

## Matlab (2020b/2021a)
### Dependencies
The simulations are entirely written in Matlab and require the following packages/toolboxes to be included in your Matlab path:
> [Signal Processing Toolbox](https://se.mathworks.com/products/signal.html)  
> [CasADi v.3.5.5](https://web.casadi.org/get/)  
> [YALMIP (latest)](https://yalmip.github.io/download/)  
> [Multi-Parametric Toolbox 3](https://www.mpt3.org/)  
> [tight_subplot.m (optional)](https://se.mathworks.com/matlabcentral/fileexchange/27991-tight_subplot-nh-nw-gap-marg_h-marg_w)  
> [tightfig.m (optional)](https://se.mathworks.com/matlabcentral/fileexchange/34055-tightfig-hfig)

## Simulation
To generate the simulation data and to plot the results, set the current folder in Matlab to be `code`, and then run the following Matlab scripts for each simulation. The results will be stored in the `data` folder that will be generated

### Vehicle Example 
```bash 
# Generate simulation data
run_vehicle_experiments.m

# Generate Figs. 1 and 2 in the paper
plot_vehicle_results.m
```

### Double Integrator Example 
```bash 
# Generate simulation data
run_double_integrator_experiments.m

# Generate Fig. 3 in the paper
plot_double_integrator_results.m

# Generate Fig. 4 in the paper
plot_terminal_safe_set.m
```

### Robotic Manipulator Example
```bash 
# Generate simulation data
run_robotic_manipulator_examples.m

# Generate Figs. 5 and 6 in the paper
plot_robotic_manipulator_results.m
```

### All examples
```bash
# Generate data for all simulations
run_all_examples.m
```

## Paper
[Safe Trajectory Tracking in Uncertain Environments](https://arxiv.org/abs/2001.11602)  
Ivo Batkovic - ivo.batkovic@zenseact.com  
Mohammad Ali - mohammad.ali@zenseact.com  
Paolo Falcone - paolo.falcone@unimore.it  
Mario Zanon -  mario.zanon@imtlucca.it  
Submitted to Transactions on Automatic Control (2020).