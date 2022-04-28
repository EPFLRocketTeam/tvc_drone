# pid_gnc
A template package for GNC projects, following the architecture defined in the package [real_time_simulator](https://github.com/EPFLRocketTeam/real_time_simulator).

This template should serve as a starting point for every new GNC project. It already contains all the proper interfaces between the GNC nodes and the simulator, as well as some basic implementations of each GNC algorithm.

You can directly use this package as is, or modify each algorithm as you need it.

## Installation
1. Install the simulator

Visit https://github.com/EPFLRocketTeam/real_time_simulator to install the simulator, and check the [documentation](http://wiki.ros.org/real_time_simulator).

2. Create your first GNC package

Clone this package into your catkin repository:
```bash
cd ~/catkin_ws/src
git clone git@github.com:EPFLRocketTeam/pid_gnc.git
```

Then rename the package with the name of your GNC project using the provided script. For example if you want to name your project `test_rocket_gnc`, you would do:
```bash
cd ~/catkin_ws/src/pid_gnc
bash_scripts/create_GNC_package.sh test_rocket
```
Note that the script will automatically add `_gnc` at the end to respect the convention. The name is also converted to lower case.

You can use this script as many times as you want until you're happy with the name of your project.

3. Build and setup
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


## Authors

Albéric de Lajarte (albericlajarte@gmail.com)
