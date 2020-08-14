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
    print("Closing gripper.")
    sim.setIntegerSignal(gripperName..'_close',1)
end

function openGripper()
    print("Opening gripper.")
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
    print("Received joint state update from the physical twin.")  
    table.insert(state_sequence_buffer, msg.position)
end 

function gripperJoystickControlCallback(msg)
    open_button     = msg.buttons[3]
    close_button    = msg.buttons[2]
    open_gripper    = (open_button == 1)
    close_gripper   = (close_button == 1)
end

function gripperCommandCallback(msg)
    open = msg.data 
    if isGripperOpen and open then
        print("Gripper already open.")
        return
    else 
        open_gripper = true
        return 
    end
    if isGripperOpen and not open then 
        close_gripper = true
        return 
    else
        print("Gripper already closed.")
        return
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
        if #state_sequence_buffer>0 then
            print("Setting joint positions on digital twin.")
            new_joint_state = state_sequence_buffer[1]  -- Get the first element.
            table.remove(state_sequence_buffer, 1)      -- consume it.
            setTargetJointPositions(new_joint_state);
        end

        if open_gripper then 
            open_gripper = false
            print("Opening gripper")
            openGripper()
        end
        if close_gripper then
            close_gripper = false
            print("Closing gripper")
            closeGripper()
        end
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

        -- Simulated Twin Topics
        SIM_TOPIC_ROOT          = "/coppeliaSIM/NiryoOne"
        SIM_TIME_TOPIC          = SIM_TOPIC_ROOT.."/simulationTime" 
        SIM_JOINT_STATE_TOPIC   = SIM_TOPIC_ROOT.."/joint_states"
        SIM_JOINT_STATE_ORDER   = SIM_TOPIC_ROOT.."/joint_states_order"
        SIM_SUBS_MODE           = SIM_TOPIC_ROOT.."/set_subscriber_mode"
        -- Gripper Control Topic Names 
        OPEN_SIM_GRIPPER  = SIM_TOPIC_ROOT.."/GripperCommandSub"
        SIM_GRIPPER_STATE = SIM_TOPIC_ROOT.."/isGripperOpenPub" 
        -- Physical Twin Topics
        JOINT_STATE_TOPIC   = "/joint_states"
        JOY_TOPIC           = "/joy"
        TOOL_STATUS         = "/niryo_one/tool_action/status"

        -- The target angular position we want the joints in is a global variable to be updated
        -- on callback.
        targetJointPosition     = {0,0,0,0,0,0} 
        setTargetJointPositions({0.0, 0.640187, -1.397485, 0.0, 0.0, 0.0}) 
        -- This is a queue to temporarily store the states arriving to the digital twin
        state_sequence_buffer = {}

        -- Physical Twin Mirror --> Mirror state published by the physical twin.
        JointStateSubPhy = simROS.subscribe(JOINT_STATE_TOPIC, 'sensor_msgs/JointState', 'phyJointStateUpdateCallback')

        -- Digital Twin Mirror --> Receive desired state from a simulation client and send back 
        --                         joint state of the simulation model.
        JointStatePubDig        = simROS.advertise(SIM_JOINT_STATE_TOPIC,'sensor_msgs/JointState')
        gripperStatePublisher   = simROS.advertise(SIM_GRIPPER_STATE,   'std_msgs/Bool')
        openGripperCommand      = simROS.subscribe(OPEN_SIM_GRIPPER,    'std_msgs/Bool',    'gripperCommandCallback')
        joystickGripperCommand  = simROS.subscribe(JOY_TOPIC,           'sensor_msgs/Joy',  'gripperJoystickControlCallback')
        simTimePub              = simROS.advertise(SIM_TIME_TOPIC, 'std_msgs/Float32')

        -- GripperController --> Receive desired state from a simulation client.
        -- We will assume gripper is open at startup. 
        isGripperOpen = true 
        open_gripper  = false
        close_gripper = false

    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_sensing()
    -- Update additional data structures of the simulation.
end

function sysCall_cleanup()
    simROS.shutdownPublisher(simTimePub)
    simROS.shutdownSubscriber(JointStateSubPhy)
    simROS.shutdownPublisher(gripperStatePublisher)
    simROS.shutdownSubscriber(openGripperCommand)
    simROS.shutdownSubscriber(joystickGripperCommand)
end


