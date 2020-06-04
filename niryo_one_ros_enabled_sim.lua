-- 
-- Joint State Information: The move_group monitors the /joint_states topic for determining 
-- where each joint of the robot is. 
--  -> This program has to implement a joint State publisher. 
-- 
-- Transform Information: The move_group monitors transform information using the ROS library. 
-- this allows the node to get global information about the robot's pose among other things. 
-- The move_group will listen to TF. To publish TF information, we need to have a 
-- robot_state_publisher node running on the simulation script, or at a higher level. 
-- 
-- Controller Interface: The move_group talks to the controllers on the robot using the 
-- FollowJointTrajectoryAction interface. This is a ROS action interface. A server on the 
-- robot needs to service this action and move_it will only instantiate a client to talk to 
-- this controller action server on your robot. 
-- 

------------------------------------------------------------------
-- Niryo One Threaded ROS simulation controller.
------------------------------------------------------------------
-- This is a non-theraded script linked to the Niryo One robot on the simulation.  

function close_gripper()
    sim.setIntegerSignal(gripperName..'_close',1)
end

function open_gripper()
    sim.clearIntegerSignal(gripperName..'_close')
end

--------------------------
-- SUBSCRIBER CALLBACKS --
--------------------------

function setMaxVelConstantCallback(msg) 
    vel = msg.data
end 

function setMaxAccelConstantCallback(msg) 
    accel = msg.data
end 

function setMaxJerkConstantCallback(msg) 
    jerk = msg.data
end 

function setMaxVelVectorCallback(msg) 
    local newValues = msg.data -- float64[] 
    for i=1,#newValues do
        maxVel[i] = newValues[i] 
    end
end

function setMaxAccelVectorCallback(msg) 
    local newValues = msg.data -- float64[] 
    for i=1,#newValues do
        maxAccel[i] = newValues[i] 
    end
end

function setMaxJerkVectorCallback(msg) 
    local newValues = msg.data -- float64[] 
    for i=1,#newValues do
        maxJerk[i] = newValues[i] 
    end
end

function gripperCommandCallback(msg)    
    if msg.data then 
        if isGripperOpen then
            print("Gripper already open.")
        else 
            print("Opening gripper.")
            open_gripper()
            isGripperOpen = true
        end
    else
        if isGripperOpen then 
            print("Closing gripper.")
            close_gripper()
            isGripperOpen = false
        else
            print("Gripper already closed.")
        end
    end
end

function targetJointAngularPositionCallback(msg)
    local newValues = msg.data -- float64[]
    local concat_val = "" 
    for i=1, #newValues do
        concat_val = concat_val.." "..newValues[i]
    end
    print(robotID..": Received new target position: ")

    local newValues = msg.data -- float64[] 
    for i=1,#newValues do
        targetJointPosition[i] = newValues[i]
        if sim.setJointTargetPosition(jointHandles[i], targetJointPosition[i]) == -1 then
            print("<font color='#F00'>Cannot set joint to target position:  ".. targetJointPosition[i] .." </font>@html")
        end 
    end

    -- Call Movement library to move the robot to the target location.

    --proc = coroutine.wrap(function () 
    --    sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetJointPosition,targetVel)
    --end)
    --coroutine.resume(proc)
end

--------------------------
-- SIMULATION LOOP      -- 
--------------------------

function sysCall_actuation() 
    if simROS then 
        -- Publish hyperparameter values.
        simROS.publish(velConstantPub, {data=vel})
        simROS.publish(accelConstantPub, {data=accel})
        simROS.publish(jerkConstantPub, {data=jerk})
        simROS.publish(maxVelJointVectorPub, {data=maxVel})
        simROS.publish(maxAccelJointVectorPub, {data=maxAccel})
        simROS.publish(maxJerkJointVectorPub, {data=maxJerk})

        -- Robot simulation state.
        simROS.publish(gripperStatePublisher, {data=isGripperOpen})
        simROS.publish(currentJointVelPub, {data=currentVel})
        simROS.publish(currentJointAccelPub, {data=currentAccel})
        simROS.publish(targetJointPosPub, {data=targetJointPosition})
        simROS.publish(currentJointPositionPub, {data=currentJointPosition})
        simROS.publish(simTimePub, {data=sim.getSimulationTime()})
    end
end


function sysCall_init()
    -- Robot ID: To be set to different IDs in case we have a scene with different Niryo One robots. 
    --      This is then used to differentiate the topics on a ROS distributed system. 
    robotID = 0

    -- Generate the handles of the joints to actuate on the robot.
    -- TODO: This might cause a career condition. 
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end

    -- Get a connection to the gripper of the robot, which is initially open.
    connection=sim.getObjectHandle('NiryoOne_connection')
    gripper=sim.getObjectChild(connection,0)
    gripperName="NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end

    if simROS then 
        -- Not initalized. 
        if initialized_robot == nil then
            -- First robot to initialize.
            initialized_robot = {}
            initialized_robot[robotID] = false
        end
        if initialized_robot[robotID] then 
            print("<font color='#F00'>Existing robot with same ID. Cannot run.</font>@html")
            return
        else 
            -- Not the first robot to initialize, clear robot ID
            initialized_robot[robotID] = true
        end

        print("<font color='#0F0'>ROS interface was found.</font>@html")

        simulationTopicRoot = "/coppeliaSIM/NiryoOne_"..robotID
        simulationTimePublisher = simulationTopicRoot.."/simulationTime" --TODO
        -- Constant Hyperparameters Topic Names
        maxVelocityConstantSub = simulationTopicRoot.."/maximumVelocityConstantSub"
        maxVelocityConstantPub = simulationTopicRoot.."/maximumVelocityConstantPub"
        maxAccelerationConstantSub = simulationTopicRoot.."/maximumAccelerationConstantSub"
        maxAccelerationConstantPub = simulationTopicRoot.."/maximumAccelerationConstantPub"
        maxJerkConstantSub = simulationTopicRoot.."/maxJerkConstantSub"
        maxJerkConstantPub = simulationTopicRoot.."/maxJerkConstantPub"
        -- Vector Hyperparameters Topic Names 
        maxJointAngularVelocityVectorSub = simulationTopicRoot.."/maxJointAngularVelocityVectorSub"
        maxJointAngularVelocityVectorPub = simulationTopicRoot.."/maxJointAngularVelocityVectorPub"
        maxJointAngularAccelerationVectorSub = simulationTopicRoot.."/maxJointAngularAccelerationyVectorSub"
        maxJointAngularAccelerationVectorPub = simulationTopicRoot.."/maxJointAngularAccelerationVectorPub"
        maxJointAngularJerkVectorSub = simulationTopicRoot.."/maxJointAngularJerkVectorSub"
        maxJointAngularJerkVectorPub = simulationTopicRoot.."/maxJointAngularJerkVectorPub"
        -- Robot State Topic Names 
        currentJointAngularVelocityPub = simulationTopicRoot.."/currentJointAngularVelocityPub"
        currentJointAngularAccelPub = simulationTopicRoot.."/currentJointAngularAccelerationPub"
        currentJointAngularPosition = simulationTopicRoot.."/currentJointAngularPosition"
        targetPositionReachedPub = simulationTopicRoot.."/targetPositionIsCurrentPositionPub"
        targetJointAngularPositionPub = simulationTopicRoot.."/targetJointAngularPositionPub"
        targetJointAngularPositionSub = simulationTopicRoot.."/targetJointAngularPositionSub"
        -- Gripper Control Topic Names 
        gripperStatePub = simulationTopicRoot.."/isGripperOpenPub" 
        openGripperSub = simulationTopicRoot.."/GripperCommandSub"

        -- Set-up some of the RML vectors:
        -- Remember that variables are globally accessible in LUA by default.

        --------------------------------
        -- Simulation Hyperparameters --
        --------------------------------

        -- These are the hyperparameters of the simulation, in theory they should be adapted to the values in
        -- from the real robot. Otherwise, the final movement trayectories will be different.
        --  PROBLEM: We do not know the real value of these parameters on the physical robot. 
        --      IDEA: We could setup a machine learning model to minimize the difference between the assigned values
        --            for the simulation, and the real behaviour of the physical twin. 
        
        -- We need topics to set and subscribe to these values. 
        vel=20      -- Maximum velocity.
        velConstantPub = simROS.advertise(maxVelocityConstantPub, 'std_msgs/Int64')
        velConstantSub = simROS.subscribe(maxVelocityConstantSub, 'std_msgs/Int64', 'setMaxVelConstantCallback')

        accel=40    -- Maximum acceleration.
        accelConstantPub = simROS.advertise(maxAccelerationConstantPub, 'std_msgs/Int64')
        accelConstantSub = simROS.subscribe(maxAccelerationConstantSub, 'std_msgs/Int64', 'setMaxAccelConstantCallback')

        jerk=80     -- Maximum jerk.
        jerkConstantPub = simROS.advertise(maxJerkConstantPub, 'std_msgs/Int64')
        jerkConstantSub = simROS.subscribe(maxJerkConstantSub, 'std_msgs/Int64', 'setMaxJerkConstantCallback')
      

        -- PROBLEM: These vectors assume that all axis have the same physical properties. This may not be true.
        --      See previous IDEA.

        -- The maximum allowed angular velocity of the joints.
        --      We need a topic to set it and another to read it. 
        maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
        maxVelJointVectorPub = simROS.advertise(maxJointAngularVelocityVectorPub, 'std_msgs/Float64MultiArray')
        maxVelJointVectorSub = simROS.subscribe(maxJointAngularVelocityVectorSub, 'std_msgs/Float64MultiArray', 'setMaxVelVectorCallback')

        -- The maximum allowed angular acceleration of the joints.
        --      We need a topic to set it and another to read it.
        maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
        maxAccelJointVectorPub = simROS.advertise(maxJointAngularAccelerationVectorPub, 'std_msgs/Float64MultiArray')
        maxAccelJointVectorSub = simROS.subscribe(maxJointAngularAccelerationVectorSub, 'std_msgs/Float64MultiArray', 'setMaxAccelVectorCallback')

        -- The maximum allowed angular jerk of the joints.
        --      We need a topic to set it and another to read it.
        maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
        maxJerkJointVectorPub = simROS.advertise(maxJointAngularJerkVectorPub, 'std_msgs/Float64MultiArray')
        maxJerkJointVectorSub = simROS.subscribe(maxJointAngularJerkVectorSub, 'std_msgs/Float64MultiArray', 'setMaxJerkVectorCallback')

        ---------------------------
        -- Simulation Variables  --
        ---------------------------

        -- The current velocity of the joints. Can be nil in which case a velocity vector of 0 is used.
        --      This vector needs a publisher. 
        currentVel={0,0,0,0,0,0,0}
        currentJointVelPub = simROS.advertise(currentJointAngularVelocityPub, 'std_msgs/Float64MultiArray')

        -- The current acceleration of the joints. Can be nil in which case an acceleration vector of 0 is used.
        --      This vector needs a publisher. 
        currentAccel={0,0,0,0,0,0,0}
        currentJointAccelPub = simROS.advertise(currentJointAngularAccelPub, 'std_msgs/Float64MultiArray')
        
        -- The desired velocity of the joints at the target. Can be nil in which case a velocity vector of 0 is used.
        --      In our case, we want to use a motion library, so this could be nil. We will ignore it. 
        targetVel={0,0,0,0,0,0}

        -- Simulation time.
        simTimePub = simROS.advertise(simulationTimePublisher, 'std_msgs/Float32')

        -- The target angular position we want the joints in. TODO: Still need to figure out how to model this.
        --      This needs a publisher and a subscriber.
        targetJointPosition = {90*math.pi/180,-54*math.pi/180,0*math.pi/180,0*math.pi/180,-36*math.pi/180,-90*math.pi/180}
        targetJointPosPub = simROS.advertise(targetJointAngularPositionPub, 'std_msgs/Float64MultiArray')
        targetJointPosSub = simROS.subscribe(targetJointAngularPositionSub, 'std_msgs/Float64MultiArray', 'targetJointAngularPositionCallback')
        --targetJointPositionIsCurrentPosition = true -- Indicates if the target position has been reached.

        currentJointPosition = {0, 0, 0, 0, 0, 0}
        currentJointPositionPub = simROS.advertise(currentJointAngularPosition, 'std_msgs/Float64MultiArray')
        -- GripperController
        isGripperOpen = true -- Open at startup.
        gripperStatePublisher = simROS.advertise(gripperStatePub, 'std_msgs/Bool')
        openGripperCommand = simROS.subscribe(openGripperSub, 'std_msgs/Bool', 'gripperCommandCallback')

    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_sensing()
    -- Update the data structures of the simulation.
end

function sysCall_cleanup()
    -- do some clean-up here
end


