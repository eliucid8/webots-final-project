# UC3M Robotics Final Project
---
Authors: Bill Athanassiou, Eric Liu

## TODO:
- [ ] Modify length of distance sensors (check writeup for lab 1 or 2 to find the distance we should modify to, maybe 0.6)
- [ ] Implement bug2 algorithm
- [ ] Implement P control for turning
- [ ] Implement kalman filter for odometry and GPS?
    - [ ] Figure out error of odometry
- [ ] Combine camera with distance sensors
- [ ] Break code into multiple files/classes?
- [ ] Detect humans at end of maze
    - [ ] Experiment with methods for distinguishing human green from wall cyan (lack of blue components in color? Gradient on cylinders?)
- [ ] Implement human finding mode at end of maze

- [x] Divide responsibilities
- [x] ~~Research Braitenburg constants (use all 16 distance sensors)~~ (not needed)
- [x] Implement P control for forward movement
- [x] Improve front obstacle detection (front_avg doesn't work well for slanted surfaces)


## Notes:
**Scenario 1:**
- Normal: cyan walls, grey floor.
**Scenario 2:**
- Extra debris
- Different lighting conditions.

**Scenario 3:**
- Unevenness in lighting conditions, debris.
- Gets stuck between debris and wall.

**Scenario 4:**
- Very foggy
- Gets into loop near end in sharp corner and debris.

**Scenario 5:**
- Slightly foggy
- Gets stuck in loop involving sharp front corner

**Scenario 6:**
- Very foggy
- Has trouble detecting thin wall directly in front
- Gets stuck in sharp corner dead end near end

**Scenario 7:**
- Very foggy
- Different layout
- Gets stuck for a bit at beginning, takes too long.

**Scenario 8:**
- Foggy, mild amounts of debris.

**Scenario 9:**
- Weird lighting conditions, light source shining on far side of walls
- Makes contact/gets stuck on thin wall, maybe fault with distance sensors?

**Scenario 10:**
- Debris blocking many routes, but distance sensors/wall following works fine.
