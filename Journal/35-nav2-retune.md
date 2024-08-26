some of the default setting come with nav2 is not going to work well.


## New planner

https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html#summary

Since I made the foot print non-circular, it might need a better planner

`Smac Lattice Planner` Seems to be the recomanded one.


The catch for using this planner is it needs a specific config file. It can be generated here https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner/lattice_primitives

However, I don't know how to set a relative path in the yaml config, so I directly set the abs path to it.

the config.json used by `generate_motion_primitives.py` to generate it is:

```
{
    "motion_model": "diff",
    "turning_radius": 0.1,
    "grid_resolution": 0.025,
    "stopping_threshold": 5,
    "num_of_headings": 16
}
```

Which I try to make turning radius small, but seems it doesn't like it.