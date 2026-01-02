IF sensor_part_detected THEN
    robot_pick()
    move_to("assembly")
    robot_place()
END_IF
