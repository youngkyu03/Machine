-- 통합 버전: 원본 픽앤플레이스 시퀀스에 향상된 가짜 진공 그리퍼 로직을 결합
-- 기존 상부 스크립트의 흐름(코루틴/IK/FK)은 유지하고, 하부 스크립트에서 동작 확인된
-- 그리퍼 검색·부착 방식을 이식했다.

function sysCall_init()
    corout = coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout) ~= 'dead' then
        local ok, err = coroutine.resume(corout)
        if err then
            error(debug.traceback(corout, err), 2)
        end
    end
end

----------------------------------------------------------
-- Fake Vacuum Grip Functions (향상된 근접 검색)
----------------------------------------------------------
local function isDescendantOfRobot(objHandle, robotBase)
    local parent = objHandle
    while parent ~= -1 do
        if parent == robotBase then
            return true
        end
        parent = sim.getObjectParent(parent)
    end
    return false
end

function fake_grab(gripperHandle)
    local robotBase = sim.getObject('.')
    local gripperPos = sim.getObjectPosition(gripperHandle, sim.handle_world)

    -- 장면 내 모든 shape 후보를 수집하되, 로봇 계통은 제외한다.
    local objects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    local minDist = math.huge
    local targetObj = nil

    for i = 1, #objects do
        local obj = objects[i]
        if obj ~= gripperHandle and not isDescendantOfRobot(obj, robotBase) then
            local objPos = sim.getObjectPosition(obj, sim.handle_world)
            local dx = gripperPos[1] - objPos[1]
            local dy = gripperPos[2] - objPos[2]
            local dz = gripperPos[3] - objPos[3]
            local dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            -- 0.10m 이하에서 가장 가까운 오브젝트를 선택
            if dist < 0.10 and dist < minDist then
                minDist = dist
                targetObj = obj
            end
        end
    end

    if targetObj then
        print('Grabbed object: ' .. sim.getObjectName(targetObj))
        sim.setObjectInt32Param(targetObj, sim.shapeintparam_static, 1)
        sim.resetDynamicObject(targetObj)
        sim.setObjectParent(targetObj, gripperHandle, true)
        return targetObj
    end

    print('No object to grab')
    return nil
end

function fake_release(objHandle)
    if objHandle and sim.isHandle(objHandle) then
        print('Released object')
        sim.setObjectParent(objHandle, -1, true)
        sim.setObjectInt32Param(objHandle, sim.shapeintparam_static, 0)
        sim.resetDynamicObject(objHandle)
    end
end

----------------------------------------------------------
-- IK MOVE
----------------------------------------------------------
function moveToPoseCallback(q, vel, accel, aux)
    sim.setObjectPose(aux.target, sim.handle_world, q)
    simIK.applyIkEnvironmentToScene(aux.env, aux.group)
end

function moveToPose_viaIK(maxVel, maxAccel, maxJerk, targetPose, aux)
    local cur = sim.getObjectPose(aux.tip, sim.handle_world)
    sim.moveToPose(-1, cur, maxVel, maxAccel, maxJerk, targetPose, moveToPoseCallback, aux, nil)
end

----------------------------------------------------------
-- FK MOVE
----------------------------------------------------------
function FK_move(joints, targetConfig)
    for i = 1, #joints do
        sim.setJointTargetPosition(joints[i], targetConfig[i])
    end
end

----------------------------------------------------------
-- MAIN
----------------------------------------------------------
function coroutineMain()
    ------------------------------------------------------
    -- Handles
    ------------------------------------------------------
    local joints = {}
    for i = 1, 6 do
        joints[i] = sim.getObject('./joint', { index = i - 1 })
    end

    local simTip    = sim.getObject('./ikTip')
    local simTarget = sim.getObject('./ikTarget')
    local modelBase = sim.getObject('.')

    local sensor     = sim.getObject('/_sensor')
    local pickDum    = sim.getObject('/pickDum')
    local tableDum   = sim.getObject('/TableDummy')

    local gripper    = sim.getObject('./BaxterVacuumCup')

    ------------------------------------------------------
    -- IK Environment
    ------------------------------------------------------
    local env = simIK.createEnvironment()
    local group = simIK.createIkGroup(env)
    simIK.addIkElementFromScene(env, group, modelBase, simTip, simTarget, simIK.constraint_pose)
    local aux = { env = env, group = group, tip = simTip, target = simTarget }

    ------------------------------------------------------
    -- Speed
    ------------------------------------------------------
    local ikMaxVel   = { 0.3, 0.3, 0.3, 1.0 }
    local ikMaxAccel = { 0.5, 0.5, 0.5, 1.0 }
    local ikMaxJerk  = { 0.3, 0.3, 0.3, 1.0 }

    local FK_target = {
        -150 * math.pi / 180,
        -21.5 * math.pi / 180,
        -31.9 * math.pi / 180,
        52.7 * math.pi / 180,
        81.2 * math.pi / 180,
        -90 * math.pi / 180
    }
    local FK_home = { 0, 0, 0, 0, 0, 0 }

    ------------------------------------------------------
    -- 1) FK로 Pick 자세 이동
    ------------------------------------------------------
    print('Start: FK -> Pick Config')
    FK_move(joints, FK_target)
    sim.wait(1.0)

    ------------------------------------------------------
    -- 2) 센서 대기
    ------------------------------------------------------
    print('Waiting for Sensor...')
    while sim.readProximitySensor(sensor) <= 0 do
        sim.wait(0.02)
    end
    print('Sensor Triggered!')

    ------------------------------------------------------
    -- 3) PickDummy 기준 IK 이동 (Up/Down)
    ------------------------------------------------------
    local pickPose = sim.getObjectPose(pickDum, sim.handle_world)

    local pickUp   = { unpack(pickPose) }
    pickUp[3] = pickUp[3] + 0.01

    local pickDown = { unpack(pickPose) }
    pickDown[3] = pickDown[3] - 0.015

    moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pickUp, aux)
    sim.wait(0.2)

    moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pickDown, aux)
    sim.wait(0.2)

    ------------------------------------------------------
    -- 4) Fake Grip (향상된 근접 검색 사용)
    ------------------------------------------------------
    local grabbed = fake_grab(gripper)
    sim.wait(0.3)

    -- 상향 이동
    moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pickUp, aux)
    sim.wait(0.3)

    ------------------------------------------------------
    -- 5) TableDummy 기준 IK 이동 (Place)
    ------------------------------------------------------
    if grabbed then
        local tablePose = sim.getObjectPose(tableDum, sim.handle_world)

        local tableUp   = { unpack(tablePose) }
        tableUp[3] = tableUp[3] + 0.01

        local tableDown = { unpack(tablePose) }
        tableDown[3] = tableDown[3] - 0.015

        moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, tableUp, aux)
        sim.wait(0.2)

        moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, tableDown, aux)
        sim.wait(0.2)

        -- 필요시 해제 동작 활성화
        -- fake_release(grabbed)
        -- sim.wait(0.2)
    else
        print('집을 대상이 없습니다')
    end

    ------------------------------------------------------
    -- 6) FK Home
    ------------------------------------------------------
    print('Returning to Home (FK)...')
    FK_move(joints, FK_home)
    sim.wait(1.0)

    print('Done.')
    sim.stopSimulation()
end
