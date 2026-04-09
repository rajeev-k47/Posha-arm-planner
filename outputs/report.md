# Posha Robotic Sub Report

## Objective

This project models and plans the four required macro and micro dispense tasks from the Posha robotics assignment.

## Tasks Covered

- `macro_container_5_to_pan_2`: container_5 -> pan_2
- `macro_container_1_to_pan_1`: container_1 -> pan_1
- `micro_pod_1_to_pan_2`: pod_1 -> pan_2
- `micro_pod_19_to_pan_1`: pod_19 -> pan_1

## Planning Summary

The planner generates a multi step task sequence including home, approach, grasp, lift, park, target approach, dispense, retreat, return, place, and home return.

## Kinematics Summary

A simple geometric inverse kinematics model is used. It is more realistic than direct pose-to-joint copying, but it is still a simplified arm model rather than full Piper URDF-based kinematics.

## Collision Summary

Collision checks are performed against simplified box obstacles representing the kitchen base, robot base keepout, macro area, pod wall zones, pans, and rear wall.

### macro_container_5_to_pan_2

- `home`: collisions=['rear_wall'], transition_collisions=[]
- `source_above`: collisions=['macro_zone'], transition_collisions=['macro_zone', 'rear_wall']
- `grasp`: collisions=['macro_zone'], transition_collisions=['macro_zone']
- `lift`: collisions=['macro_zone'], transition_collisions=['macro_zone']
- `park`: collisions=[], transition_collisions=['macro_zone']
- `target_above`: collisions=[], transition_collisions=['macro_zone']
- `dispense`: collisions=[], transition_collisions=[]
- `target_retreat`: collisions=[], transition_collisions=[]
- `park_return`: collisions=[], transition_collisions=['macro_zone']
- `source_return`: collisions=['macro_zone'], transition_collisions=['macro_zone']
- `place`: collisions=['macro_zone'], transition_collisions=['macro_zone']
- `home_return`: collisions=['rear_wall'], transition_collisions=['macro_zone', 'rear_wall']

### macro_container_1_to_pan_1

- `home`: collisions=['rear_wall'], transition_collisions=[]
- `source_above`: collisions=['robot_base_keepout', 'macro_zone'], transition_collisions=['macro_zone', 'rear_wall', 'robot_base_keepout']
- `grasp`: collisions=['robot_base_keepout', 'macro_zone'], transition_collisions=['macro_zone', 'robot_base_keepout']
- `lift`: collisions=['robot_base_keepout', 'macro_zone'], transition_collisions=['macro_zone', 'robot_base_keepout']
- `park`: collisions=[], transition_collisions=['macro_zone', 'robot_base_keepout']
- `target_above`: collisions=[], transition_collisions=['macro_zone']
- `dispense`: collisions=[], transition_collisions=[]
- `target_retreat`: collisions=[], transition_collisions=[]
- `park_return`: collisions=[], transition_collisions=['macro_zone']
- `source_return`: collisions=['robot_base_keepout', 'macro_zone'], transition_collisions=['macro_zone', 'robot_base_keepout']
- `place`: collisions=['robot_base_keepout', 'macro_zone'], transition_collisions=['macro_zone', 'robot_base_keepout']
- `home_return`: collisions=['rear_wall'], transition_collisions=['macro_zone', 'rear_wall', 'robot_base_keepout']

### micro_pod_1_to_pan_2

- `home`: collisions=['rear_wall'], transition_collisions=[]
- `source_above`: collisions=['rear_wall'], transition_collisions=['pod_wall_negative_y', 'rear_wall']
- `grasp`: collisions=['pod_wall_negative_y', 'rear_wall'], transition_collisions=['pod_wall_negative_y', 'rear_wall']
- `lift`: collisions=['rear_wall'], transition_collisions=['pod_wall_negative_y', 'rear_wall']
- `park`: collisions=[], transition_collisions=['rear_wall']
- `target_above`: collisions=[], transition_collisions=[]
- `dispense`: collisions=[], transition_collisions=[]
- `target_retreat`: collisions=[], transition_collisions=[]
- `park_return`: collisions=[], transition_collisions=[]
- `source_return`: collisions=['rear_wall'], transition_collisions=['rear_wall']
- `place`: collisions=['pod_wall_negative_y', 'rear_wall'], transition_collisions=['pod_wall_negative_y', 'rear_wall']
- `home_return`: collisions=['rear_wall'], transition_collisions=['pod_wall_negative_y', 'rear_wall']

### micro_pod_19_to_pan_1

- `home`: collisions=['rear_wall'], transition_collisions=[]
- `source_above`: collisions=['pod_wall_positive_y', 'rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `grasp`: collisions=['pod_wall_positive_y', 'rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `lift`: collisions=['pod_wall_positive_y', 'rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `park`: collisions=[], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `target_above`: collisions=[], transition_collisions=['macro_zone']
- `dispense`: collisions=[], transition_collisions=[]
- `target_retreat`: collisions=[], transition_collisions=[]
- `park_return`: collisions=[], transition_collisions=['macro_zone']
- `source_return`: collisions=['pod_wall_positive_y', 'rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `place`: collisions=['pod_wall_positive_y', 'rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']
- `home_return`: collisions=['rear_wall'], transition_collisions=['pod_wall_positive_y', 'rear_wall']

## Assumptions

- Workspace coordinates are currently modeled manually.
- Collision geometry is simplified using box envelopes.
- The arm model is a learning stage approximation, not a full production robot model.

## Next Improvements

- Implementation of automatic obstacle avoiding algorithm it would be similar to tracking nearing obstacle and try to avoid it according to the given trajectory.
- Replace simplified collision boxes with more realistic environment geometry.
- Upgrade kinematics toward a full Piper-based model.

