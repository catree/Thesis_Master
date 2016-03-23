ts=simGetSimulationTimeStep()

if (simGetScriptExecutionCount()==0) then
	-- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
	v=simGetIntegerParameter(sim_intparam_program_version)
	if (v<20413) then
		simDisplayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim_dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
	end

	-- Detatch the manipulation sphere:
	targetObj=simGetObjectHandle('Quadricopter_target')
	simSetObjectParent(targetObj,-1,true)

	-- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

	d=simGetObjectHandle('Quadricopter_base')

	particlesAreVisible=simGetScriptSimulationParameter(sim_handle_self,'particlesAreVisible')
	simSetScriptSimulationParameter(sim_handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
	
	simulateParticles=simGetScriptSimulationParameter(sim_handle_self,'simulateParticles')
	simSetScriptSimulationParameter(sim_handle_tree,'simulateParticles',tostring(simulateParticles))

	propellerScripts={-1,-1,-1,-1}
	for i=1,4,1 do
		propellerScripts[i]=simGetScriptHandle('Quadricopter_propeller_respondable'..i)
	end
	heli=simGetObjectAssociatedWithScript(sim_handle_self)

	propForces={0,0,0,0}
	force={0,0,0}
	tForce={0,0,0}
	propCentres={{0,0,0}
				,{0,0,0}
				,{0,0,0}
				,{0,0,0}}

	desiredAngle={0,0,0}
	heliAngle={0,0,0}

	fakeShadow=simGetScriptSimulationParameter(sim_handle_self,'fakeShadow')
	if (fakeShadow) then
		shadowCont=simAddDrawingObject(sim_drawing_discpoints+sim_drawing_cyclic+sim_drawing_25percenttransparency+sim_drawing_50percenttransparency+sim_drawing_itemsizes,0.2,0,-1,1)
	end

	-- Prepare 2 floating views with the camera views:
	floorCam=simGetObjectHandle('Quadricopter_floorCamera')
	frontCam=simGetObjectHandle('Quadricopter_frontCamera')
	floorView=simFloatingViewAdd(0.9,0.9,0.2,0.2,0)
	frontView=simFloatingViewAdd(0.7,0.9,0.2,0.2,0)
	simAdjustView(floorView,floorCam,64)
	simAdjustView(frontView,frontCam,64)

	heliQuaternion=simGetObjectQuaternion(heli, -1)
	simSetObjectQuaternion(targetObj, -1, heliQuaternion)

	done=false
end

s=simGetObjectSizeFactor(d)

pos=simGetObjectPosition(d,-1)
if (fakeShadow) then
	itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
	simAddDrawingObjectItem(shadowCont,itemData)
end

mass,_,_ = simGetShapeMassAndInertia(heli)

gravity = simGetArrayParameter(sim_arrayparam_gravity)

print("Mass:")
print(mass)

for i=1,4,1 do
	prop=simGetObjectAssociatedWithScript(propellerScripts[i])
	tmpMass,_,propCentres[i]=simGetShapeMassAndInertia(prop, simGetObjectMatrix(heli, -1))
	mass=mass+tmpMass
	print(mass)
end

length=math.abs(propCentres[1][1]-propCentres[2][1])
width=math.abs(propCentres[1][2]-propCentres[4][2])
print("Width:")
print(width)

print("Length:")
print(length)

heliQuaternion=simGetObjectQuaternion(heli, -1)

mag=math.sqrt(heliQuaternion[3]*heliQuaternion[3] + heliQuaternion[4]*heliQuaternion[4])
targetQuaternion={0,0,heliQuaternion[3]/mag,heliQuaternion[4]/mag}

simSetObjectQuaternion(targetObj, -1, targetQuaternion)

print("Orientation:")
print(heliQuaternion[1])
print(heliQuaternion[2])
print(heliQuaternion[3])
print(heliQuaternion[4])

heliQuaternion=simGetObjectQuaternion(heli, targetObj)

print("Adjusted orientation:")
print(heliQuaternion[1])
print(heliQuaternion[2])
print(heliQuaternion[3])
print(heliQuaternion[4])

heliAngle={0,0,0}

mag=math.sqrt(heliQuaternion[1]*heliQuaternion[1] + heliQuaternion[4]*heliQuaternion[4])
if(mag~=0) then
	sideQuaternion={heliQuaternion[1]/mag,0,0,heliQuaternion[4]/mag}
	heliAngle[1]=2*math.acos(sideQuaternion[4])
	if(heliAngle[1]>math.pi/2) then
		heliAngle[1]=heliAngle[1]-math.pi
	end
end

force[1]=2*mass*(desiredAngle[1]-heliAngle[1])/width
force[1]=force[1]-tForce[1]
tForce[1]=tForce[1]+force[1]


mag=math.sqrt(heliQuaternion[2]*heliQuaternion[2] + heliQuaternion[4]*heliQuaternion[4])
if(mag~=0) then
	forwardQuaternion={0,heliQuaternion[2]/mag,0,heliQuaternion[4]/mag}
	heliAngle[2]=2*math.acos(forwardQuaternion[4])
	if(heliAngle[2]>math.pi/2) then
		heliAngle[2]=heliAngle[2]-math.pi
	end
end

force[2]=2*mass*(desiredAngle[2]-heliAngle[2])/length
force[2]=force[2]-tForce[2]
tForce[2]=tForce[2]+force[2]

--QC is facing along positive X

vertAngle=2*math.acos((math.cos(heliAngle[2])+math.cos(heliAngle[1]))/(math.sqrt(2+2*math.cos(heliAngle[2])*math.cos(heliAngle[1]))))

print("vertAngle:")
print(vertAngle)

v0=simGetObjectVelocity(heli)

desiredVelocity=0.1

force[3] = (-(gravity[3]) + desiredVelocity - v0[3]) * mass / math.cos(vertAngle)

print("velocity:")
print(v0[1])
print(v0[2])
print(v0[3])

print("angle:")
print(heliAngle[1])
print(heliAngle[2])
print(heliAngle[3])

print("Force:")
print(force[1])
print(force[2])
print(force[3])

print("tForce:")
print(tForce[1])
print(tForce[2])
print(tForce[3])

propForces[1]=force[3]/4 - force[2] / 4 + force[1] / 4
propForces[2]=force[3]/4 + force[2] / 4 + force[1] / 4
propForces[3]=force[3]/4 + force[2] / 4 - force[1] / 4
propForces[4]=force[3]/4 - force[2] / 4 - force[1] / 4

-- Send the desired motor velocities to the 4 rotors:
for i=1,4,1 do
	simSetScriptSimulationParameter(propellerScripts[i],'totalForce',propForces[i])
end
simHandleChildScript(sim_handle_all_except_explicit)


if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
	-- Now reset the manipulation sphere:
	simSetObjectParent(targetObj,d,true)
	simSetObjectPosition(targetObj,sim_handle_parent,{0,0,0})
	simSetObjectOrientation(targetObj,sim_handle_parent,{0,0,0})
end
