--
--    This program is free software: you can redistribute it and/or modify
--    it under the terms of the GNU General Public License as published by
--    the Free Software Foundation, either version 3 of the License, or
--    (at your option) any later version.
--
--    This program is distributed in the hope that it will be useful,
--    but WITHOUT ANY WARRANTY; without even the implied warranty of
--    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--    GNU General Public License for more details.
--
--    You should have received a copy of the GNU General Public License
--    along with this program.  If not, see <http://www.gnu.org/licenses/>.
-----------------------------------------------------------------------------------------------
--
------------------------------------------------------------------
-- Niryo One Threaded ROS simulation controller.
------------------------------------------------------------------
-- This is a non-theraded script linked to the Niryo One robot on the simulation.  

------------------------------------------------------------------
-- Helper Functions
------------------------------------------------------------------

function closeGripper()
    sim.setIntegerSignal(gripperName..'_close',1)
end

function openGripper()
    sim.clearIntegerSignal(gripperName..'_close')
end

function setTargetJointPositions(remoteTargetVector)
    local targetVector = remoteTargetVector
    for i=1,#targetVector do
        if sim.setJointTargetPosition(JointHandles[i], targetVector[i]) == -1 then
            print("<font color='#F00'>Cannot set joint to target position:  ".. targetVector[i] .." </font>@html")
        end 
    end 
end

function setTargetJointVelocity(targetVector)
    for i=1,#targetVector do
        if sim.setJointTargetVelocity(JointHandles[i], targetVector[i]) then
            print("<font color='#F00'>Cannot set joint to target velocity:  ".. targetVector[i] .." </font>@html")
        end 
    end
end

function setTargetJointForce(targetVector)
    for i=1,#targetVector do
        if(targetVector[i] ~= 0) then 
            if sim.setJointForce(JointHandles[i], targetVector[i]) then
                print("<font color='#F00'>Cannot set joint to target force:  ".. targetVector[i] .." </font>@html")
            end 
        end
    end
end

function getJointVelocityVector(targetVel) 
    for i=1,6 do
        targetVel[i] = sim.getJointTargetVelocity(JointHandles[i])
    end
end 

function getJointPositionVector(targetJointPosition) 
    for i=1,6 do
        targetJointPosition[i] = sim.getJointPosition(JointHandles[i])
    end
end 

function getJointForceVector(targetJointForce)
    for i=1,6 do
        targetJointForce[i] = sim.getJointForce(JointHandles[i])
    end
end

--------------------------
-- SUBSCRIBER CALLBACKS --
--------------------------

function phyJointStateUpdateCallback(msg)
    -- Received a change on the state of the robot joints. 
    -- Update the model and propagate the state variables. 
    if subscribe_mode then 
        setTargetJointPositions(msg.position);
    end
end 

function digJointStateUpdateCallback(msg)
    -- Received a change on the state of the robot joints. 
    -- Update the model and propagate the state variables.
    if (is_pure_subscriber == false) then 
        setTargetJointPositions(msg.data);
    end
end 

function setSubModeCallback(msg)
    is_pure_subscriber = msg.data;
end

function gripperCommandCallback(msg)    
    if msg.data then 
        if isGripperOpen then
            print("Gripper already open.")
        else 
            print("Opening gripper.")
            openGripper()
            isGripperOpen = true
        end
    else
        if isGripperOpen then 
            print("Closing gripper.")
            closeGripper()
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
        simROS.publish(simTimePub,              {data=sim.getSimulationTime()})
        
        pos = {0, 0, 0, 0, 0, 0}
        vel = {0, 0, 0, 0, 0, 0}
        eff = {0, 0, 0, 0, 0, 0}
        getJointPositionVector(pos)
        getJointVelocityVector(vel)
        getJointForceVector(eff)
        simROS.publish(JointStatePubDig, {
            position = pos, 
            velocity = vel, 
            effort   = eff 
        })

    end
end

function sysCall_init()
    -- Generate the handles of the joints to actuate on the robot.
    JointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        JointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
        -- Set torque/force mode on the joint. 
        --  In this mode, the joint is simulated by the dynamics module, 
        --  if and only if it is dynamically enabled
        sim.setJointMode(JointHandles[i], sim.jointmode_force, 0)
    end

    -- Get a connection to the gripper of the robot, which is initially open.
    connection=sim.getObjectHandle('NiryoOne_connection')
    gripper=sim.getObjectChild(connection,0)
    gripperName="NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end

    if simROS then 
        print("<font color='#0F0'>ROS interface was found.</font>@html")

        -- Digital Twin Topics
        SIM_TOPIC_ROOT = "/coppeliaSIM/NiryoOne"
        SIM_TIME_TOPIC = SIM_TOPIC_ROOT.."/simulationTime" 
        SIM_JOINT_STATE_TOPIC = SIM_TOPIC_ROOT.."/joint_states"
        SIM_JOINT_STATE_ORDER = SIM_TOPIC_ROOT.."/joint_states_order"
        SIM_SUBS_MODE = SIM_TOPIC_ROOT.."/set_subscriber_mode"

        -- Physical Twin Topics TODO: Gripper
        JOINT_STATE_TOPIC = "/joint_states"

        -- Gripper Control Topic Names 
        gripperStatePub = SIM_TOPIC_ROOT.."/isGripperOpenPub" 
        openGripperSub  = SIM_TOPIC_ROOT.."/GripperCommandSub"

        ---------------------------
        -- Simulation Variables  --
        ---------------------------
        -- Simulation time.
        simTimePub = simROS.advertise(SIM_TIME_TOPIC, 'std_msgs/Float32')

        -- The target angular position we want the joints in.
        DEFAULT_JOINTS_POSITION = {0.0, 0.640187, -1.397485, 0.0, 0.0, 0.0}
        targetJointPosition = {0,0,0,0,0,0}
        setTargetJointPositions(DEFAULT_JOINTS_POSITION) 

        -- TODO: We need to publish the current state of the digital twin as a desired state 
        --  for the physical. We will assume that the control comes from the LUA script.

        -- Physical Twin Mirror --> Mirror state published by the physical twin.
        JointStateSubPhy = simROS.subscribe(JOINT_STATE_TOPIC, 'sensor_msgs/JointState', 'phyJointStateUpdateCallback')

        -- Digital Twin Mirror --> Receive desired state from a simulation client and send back 
        --                         joint state.
        JointStatePubDig = simROS.advertise(SIM_JOINT_STATE_TOPIC,'sensor_msgs/JointState')
        JointStateSubDig = simROS.subscribe(SIM_JOINT_STATE_ORDER, 'std_msgs/Float64MultiArray', 'digJointStateUpdateCallback')

        setPureSubscriberSub = simROS.subscribe(SIM_SUBS_MODE, 'std_msgs/Bool', 'setSubModeCallback')
        is_pure_subscriber = false;

        -- GripperController --> Receive desired state from a simulation client.
        isGripperOpen = true -- Open at startup. TODO: Get from physical twin.
        gripperStatePublisher   = simROS.advertise(gripperStatePub, 'std_msgs/Bool')
        openGripperCommand      = simROS.subscribe(openGripperSub, 'std_msgs/Bool', 'gripperCommandCallback')
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_sensing()
    -- Update additional data structures of the simulation.
    -- TODO: For now we want a full simulation subscriber of the physical robot.
    --      We need to send a signal to the JOINT_TRAJECTORY_ACTION_TOPIC.
end

function sysCall_cleanup()
    simROS.shutdownPublisher(simTimePub)
    simROS.shutdownSubscriber(JointStateSubPhy)
    simROS.shutdownPublisher(JointStatePubDig)
    simROS.shutdownSubscriber(setPureSubscriberSub)
    simROS.shutdownSubscriber(JointStateSubDig)
    simROS.shutdownPublisher(gripperStatePublisher)
    simROS.shutdownSubscriber(openGripperCommand)
end


