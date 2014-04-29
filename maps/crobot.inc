define crobot position(
    size [0.33 0.33 0.25]
    origin [0 0 0 0]
    gui_nose 1
    drive "diff"

    # This block approximates a circular shape of a Robot
    block( points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0 1]
    )
    
    hokuyo( pose [0.15 0 -0.1 0] )

    # Report error-free position in world coordinates
    #localization "gps"
    #localization_origin [ 0 0 0 0 ]

    # Some more realistic localization error
    localization "odom"
    odom_error [ 0.01 0.01 0.0 0.1 ]
)
