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

function setTargetJointPositions(targetVector)
    for i=1,#targetVector do
        if sim.setJointTargetPosition(jointHandles[i], targetVector[i]) == -1 then
            print("<font color='#F00'>Cannot set joint to target position:  ".. targetVector[i] .." </font>@html")
        end 
    end
end

function setTargetJointVelocity(targetVector)
    for i=1,#targetVector do
        if sim.setJointTargetVelocity(jointHandles[i], targetVector[i]) then
            print("<font color='#F00'>Cannot set joint to target velocity:  ".. targetVector[i] .." </font>@html")
        end 
    end
end

function setTargetJointForce(targetVector)
    for i=1,#targetVector do
        if sim.setJointTargetForce(jointHandles[i], targetVector[i]) then
            print("<font color='#F00'>Cannot set joint to target force:  ".. targetVector[i] .." </font>@html")
        end 
    end
end

function updateJointVelocityVector() 
    for i=1,#targetVel do
         targetVel[i] = sim.getJointTargetVelocity(jointHandles[i])
    end
end 

function updateJointPositionVector() 
    for i=1,#targetJointPosition do
         targetJointPosition[i] = sim.getJointTargetPosition(jointHandles[i])
    end
end 

function updateJointForceVector()
    for i=1,#targetJointForce do
        targetJointForce[i] = sim.getJointForce(jointHandles[i])
    end
end

--------------------------
-- SUBSCRIBER CALLBACKS --
--------------------------

function setTargetJointAngularVelVectorCallback(msg) 
    setTargetJointVelocity(msg.data)
end

function targetJointAngularPositionCallback(msg)
    local newValues = msg.data -- float64[]
    local concat_val = "" 
    for i=1, #newValues do
        concat_val = concat_val.." "..newValues[i]
    end
    print(robotID..": Received new target position: "..concat_val)
    setTargetJointPositions(newValues)   
end

function targetJointAngularForceCallback(msg)
    setTargetJointForce(msg.data)
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


--------------------------
-- SIMULATION LOOP      -- 
--------------------------

function sysCall_actuation() 
    if simROS then 
        simROS.publish(gripperStatePublisher,   {data=isGripperOpen})
        simROS.publish(targetJointPosPub,       {data=targetJointPosition})
        simROS.publish(simTimePub,              {data=sim.getSimulationTime()})
        simROS.publish(targetJointVelPub,       {data=targetVel})
        simROS.publish(targetForcePub,          {data=targetJointForce})
    end
end


function sysCall_init()
    -- Robot ID: To be set to different IDs in case we have a scene with different Niryo One robots. 
    --      This is then used to differentiate the topics on a ROS distributed system. 
    robotID = 0

    -- Generate the handles of the joints to actuate on the robot.
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
        simulationTimePublisher = simulationTopicRoot.."/simulationTime" 

        -- Robot State Topic Names 
        targetJointAngularVelocityPub = simulationTopicRoot.."/targetJointAngularVelocityPub"
        targetJointAngularVelocitySub = simulationTopicRoot.."/targetJointAngularVelocitySub"
        targetJointAngularPositionPub  = simulationTopicRoot.."/targetJointAngularPositionPub"
        targetJointAngularPositionSub  = simulationTopicRoot.."/targetJointAngularPositionSub"
        targetJointAngularForcePub = simulationTopicRoot.."/targetJointAngularForcePub"
        targetJointAngularForceSub = simulationTopicRoot.."/targetJointAngularForceSub"
        -- Gripper Control Topic Names 
        gripperStatePub = simulationTopicRoot.."/isGripperOpenPub" 
        openGripperSub  = simulationTopicRoot.."/GripperCommandSub"

        ---------------------------
        -- Simulation Variables  --
        ---------------------------

        -- The target velocity of the joints.
        targetVel={0,0,0,0,0,0}
        targetJointVelPub = simROS.advertise(targetJointAngularVelocityPub, 'std_msgs/Float64MultiArray')
        targetJointVelSub = simROS.subscribe(targetJointAngularVelocitySub, 'std_msgs/Float64MultiArray', 'setTargetJointAngularVelVectorCallback')

        -- Simulation time.
        simTimePub = simROS.advertise(simulationTimePublisher, 'std_msgs/Float32')

        -- The target angular position we want the joints in.
        targetJointPosition = {0,0,0,0,0,0}
        setTargetJointPositions(targetJointPosition)
        targetJointPosPub = simROS.advertise(targetJointAngularPositionPub, 'std_msgs/Float64MultiArray')
        targetJointPosSub = simROS.subscribe(targetJointAngularPositionSub, 'std_msgs/Float64MultiArray', 'targetJointAngularPositionCallback')

        -- The target force of the joints. 
        targetJointForce = {0,0,0,0,0,0}
        targetForcePub = simROS.advertise(targetJointAngularForcePub, 'std_msgs/Float64MultiArray')
        targetForceSub = simROS.subscribe(targetJointAngularForceSub, 'std_msgs/Float64MultiArray', "targetJointAngularForceCallback")

        -- GripperController
        isGripperOpen = true -- Open at startup.
        gripperStatePublisher   = simROS.advertise(gripperStatePub, 'std_msgs/Bool')
        openGripperCommand      = simROS.subscribe(openGripperSub, 'std_msgs/Bool', 'gripperCommandCallback')

    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_sensing()
    -- Update additional data structures of the simulation.
    updateJointForceVector()
    updateJointVelocityVector()
    updateJointPositionVector()
end

function sysCall_cleanup()
    simROS.shutdownPublisher(targetJointVelPub)
    simROS.shutdownSubscriber(targetJointVelSub)

    simROS.shutdownPublisher(targetJointPosPub)
    simROS.shutdownSubscriber(targetJointPosSub)

    simROS.shutdownPublisher(targetForcePub)
    simROS.shutdownSubscriber(targetForceSub)

    simROS.shutdownPublisher(simTimePub)

    simROS.shutdownPublisher(gripperStatePublisher)
    simROS.shutdownSubscriber(openGripperCommand)
end


