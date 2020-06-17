# Polaris Vega Interface

This package is used to obtain the pose of passive markers compatible with Polaris Vega. 

## Pre-requisites

This code depends on pyserial.

```shell script
python3 -m pip install pyserial
```

## Configuration

Adjust the configuration file to have the correct information for your environment.

Configuration file: `cfg/config.yml`.

|Variable|Meaning|
|---|---|
|polaris_vega:ip|The ip of the Polaris Vega|
|polaris_vega:port|The port you want to connect to. Not necessary to change it.|
|polaris_vega:fps|Set the tracking frequency. It can be 20, 30, or 60 frames per second.|
|polaris_vega:debug|Set it to true to have all commands and replies printed.|
|passive_tools:|A list of paths for the passive tools definition files.|
|passive_tools:path| The path to the definition file of that specific passive tool.|

## Usage

`rosrun polaris_vega_interface polaris_driver_ros.py`

The marker pose data will be published in the topic `/polaris_vega_interface/tools/getpose`.

# Acknowledgements
- Created by: Murilo Marques Marinho (murilo@nml.t.u-tokyo.ac.jp)