---
name: ros-launch-audit
description: Use this skill when analyzing ROS launch files, startup order, parameters, remaps, node names, namespaces, and included launch files.
---

Analyze ROS launch files safely.

Rules:
- Do not run roslaunch.
- Only read launch/xml/yaml files.
- For each launch file, identify:
  1. included launch files
  2. nodes started
  3. package and executable name
  4. parameters loaded
  5. remapped topics
  6. namespace/group
  7. required hardware or sensors
- Explain the startup chain from top-level launch to each node.
- Identify which launch file is safest for simulation or offline reading.