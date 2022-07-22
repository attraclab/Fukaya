# Fukaya robot

Autonomous drive robot for Fukaya greenhouse.

## Dependencies

- [jmoab-ros](https://github.com/rasheeddo/jmoab-ros)
- [greenhouse_dyn_params](https://github.com/attraclab/greenhouse_dyn_params)
- [atcart_greenhouse_gazebo](https://github.com/attraclab/atcart_greenhouse_gazebo) for simulation only

## Run

For Simulation, run [fives termnials](https://github.com/attraclab/atcart_greenhouse_gazebo#run) as explained on atcart_greenhouse_gazebo
```sh
# Terminal1
rosrun greenhouse_dyn_params greenhouse_nav_params_server.py --params_file ~/Fukaya/GreenhouseNavParams.yaml

# Terminal2
rosrun rqt_reconfigure rqt_reconfigure
# make sure the rqt_reconfigure is loaded correct paramters as in GreenhouseNavParams.yaml

# Those two terminals above are for tuning purpose
# Terminal3
cd ~/Fukaya
python3 greenhouse_navigation2.py --params_file GreenhouseNavParams.yaml

```