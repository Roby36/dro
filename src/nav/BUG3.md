# "Bug3" Algorithm

## Aim of the algorithm

Travel between two arbitrary points `(xs, ys, zs)` `(xg, yg, zg)` in the `map` reference frame, avoiding obstacles and minimizing total distance traveled and direction changes. The “Bug3” algorithm is an extension of the “Bug2” algorithm visited in class.

## Inputs and parameters

### Minimum height of drone relative to the ground

This input requires a “rangefinder” sensor located at the bottom of the drone.
At a simulation level, this piece of data is published to the `mavros/rangefinder/rangefinder` topic and provides the relative perpendicular height from the ground, up to a height of 50 meters. In reality, this range will depend on the sensor employed. Using this topic, we can set a parameter `min_ground_height` which ensures the drone does not get too close to the ground, except when in landing mode.

### Maximum height of drone relative to the ground

This parameter is also very important for safety reasons, and must be taken into account to ensure that the drone doesn’t reach dangerous altitudes. However, beyond 50 meters (in simulation) the rangefinder sensor does not provide useful data anymore. Many manufacturers (e.g. DJI) have dealt with this issue simply by limiting the altitude relative to the takeoff point (which would be available in the current mavros topics). Yet, this approach presents significant pitfalls, especially when operating in extreme environments such as high mountains, where altitude relative to the ground and altitude relative to the takeoff point differ drastically. A possible solution to this problem is to convert the barometer readings from the topic `mavros/global_position/global`  to mean meters above sea level and then use a map of the operating environment’s altitudes to compute the height above ground of the drone. Alternatively, we may use a long-range rangefinder to directly measure the altitude above ground level (AGL) of the drone, although even the more costly of these sensors are limited to a range of 100-200m. In the first stages of development, we may omit this parameter or use the maximum offered reading of 50 meters for simplicity.

### Minimum distance of drone relative to the closest horizontal obstacles
To identify the obstacles on the drone’s path we are planning to use a 360° lidar sensor, similar to the one on the turtlebot. Depending on the drone’s velocity, there will be a minimum distance `min_obst_distance` at which an obstacle is considered “dangerously close,” similarly to the random-walk situation in PA0. At a simulation level, we still need to find or add the relevant topic for this sensor to the  `/mavros`node.

## Output
The output of the algorithm is simply a sequence of velocity commands published to
`/mavros/setpoint_velocity/cmd_vel` to bring the drone to the goal state,
computed and executed dynamically as the drone gathers data about the environment.

## Motion preferences considerations
Since the drone is not operating in 2 dimensions like in Bug2 and has also
significantly more degrees of freedom, it needs to consider whether to travel around an obstacle, or attempt to surpass it by increasing its altitude. More critically, the three parameters defined above do not provide sufficient restraints to fully define this choice.

On the one hand, the navigation strategy could be entirely *z*-preferential, increasing altitude until no obstacle is close and then continuing on the path to the goal (some DJI drones seem to employ this startegy when executing the Return to Home). However, very tall buildings (e.g. very high antenna tower, skyscrapers) may lead the drone to unnecessarily increase its altitude to dangerous levels. 
On the other hand, a navigation strategy entirely *x,y*-preferential, acting exactly like "Bug2", would clearly fail to surpass very large obstacles of undefined width such as mountains. The consideration of these two extreme cases motivates the adoption of an obstacle avoidance strategy exploiting all the three dimensions.

Exploring these edge cases, we can make the assumption that the entirely *z*-preferential strategy fails almost exclusively for artificial constructions, where the vertical walls force the drone upwards without gaining any horizontal distance, thereby increasing its AGL. However, these artificial obstacles also have a very limited width, and thus could be easily circumvented by the standard Bug2 algorithm at a safe height. This motivates the implementation of the algorithm below, where we apply a *z*-preferential strategy until we exhaust the `max_ground_height` allowed, and then use an *x,y*-preferential strategy to try circumvent the obstacle.

## “Bug3” V1
### Parameters
#### Contants
    (xs, ys, zs)        // starting coordinates (map)
    (xg, yg, zg)        // goal coordinates (map)
    min_ground_height   // minimum height allowed above ground level (AGL)
    max_ground_height   // maximum height allowed above ground level (AGL)
    min_obst_distance   // minimum distance allowed from closest object
    buffer_height       // extra altitude gain after avoiding an obstacle,
                           before proceeding again towards goal
    
#### Variables
    (x,y,z)             // coordinates of drone (map)
    roll, pitch, yaw    // orientation of drone (map)
    curr_ground_height  // current height above ground level (AGL)
    curr_obst_distance  // current distance to closest object,
                           when heading directly towards the goal
    avoiding_obstacle   // flag-variable indicating whetether we are 
                           in the obstacle avoidance process
    
### Pseudocode
    define isClose = (curr_obst_distance < min_obst_distance)
    define isLow   = (curr_ground_height < min_ground_height)
    define isHigh  = (curr_ground_height > max_ground_height)
    set avoiding_obstacle = false
    compute linear path between (xs, ys) and (xg, yg)
    rotate yaw toward goal
    while ((x, y) != (xg, yg))
        follow linear path toward (xg, yg)
        while (isClose || isLow)
            if (!isHigh)
                set avoiding_obstacle = true
                increase altitude
            else /*** Bug2 ***/
                while linear path not intersected at lower distance to goal
                    follow obstacle anti-clockwise at min_obst_distance (PID)
                rotate yaw toward goal
        if (avoiding_obstacle)
            increase altitude by buffer_height
            set avoiding_obstacle = false
    while (z != zg)
        increase / decrease altitude 

Notice that the algorithm prioritizes the achievement of the *x,y* goal coordinates, and reaches the goal's *z*-coordinate only at the end. Whenever a close obstacle is detected, the first action is to try to "out-high" it. As soon as the lidar signals that the obstacle has left the close range, the altitude is further increased by `buffer_height` to ensure the obstacle is sufficiently far and the drone proceeds to the goal without the need to adjust its yaw. If, on the contrary, the drone reaches the maximum AGL allowed (without ever moving forwards in the process), it assumes that it is likely in front of a vertical wall and attempts to circumvent it by applying the Bug2 algorithm. This method only considers the distances on the *x,y*-plane and uses a PID system (similarly to PA2) to maintain the drone at 
`min_obst_distance` from the current obstacle. After circumventing the obstacle, the drone adjusts its yaw and proceeds toward the goal.

### Limitations
* As stated before, the current rangefinder sensor would allow the `max_ground_height` parameter to be set only up to 50 meters, since above that measure its data would be unreliable. While this can be good to test more intensively the Bug2 section of the algotrithm in simulation, especially when testing with the real drone we will need to come up with a reliable method of computing AGL.
* The algorithm is not complete: if the obstacle is too high and the drone applies Bug2, it is not guaranteed to intersect the path to the goal at a lower distance, since very large three-dimensional obstacles may be impossible to circumvent. The algorithm relies on the fundamental assumption that **either** an obstacle is too large to circumvent, **or** it is too sharp and vertical to force the drone above the maxiumum AGL. \
The algorithm could be made slightly more robust by allowing Bug2 to execute for a maximum distance, say `max_circ_dist`, and then check the `isHigh`condition at the start of the loop again to try to revert to the primary obstacle avoidance method. However, this technique would still not make the algorithm complete and add an unnecessary layer of complexity at a point where the algorithm still needs to be tested in simulation. 
* We still need to account for the possibility of the drone hitting the ceiling. For now, we are under the assumption that the drone will operate at sufficiently high altitudes AGL to not worry about the ceiling. However, in the future we will need an additional rangefinder sensor on the top of the drone and parameters `curr_ceiling_distance` and `max_ceiling_distance` to make the algorithm more robust.
        
    