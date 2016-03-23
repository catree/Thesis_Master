------------------------------------------------------------------------------ 
-- The main script was automatically adjusted by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later. You will find the original main script 
-- commented out below 
if (sim_call_type==sim_childscriptcall_initialization) then 
	simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,-1) 
	local theTxt='The main script of this scene was customized (which is anyway not recommended),&&nand probably not compatible with V-REP release 3.1.3 and later. For that reason,&&nthe main script was replaced with the default main script. If your scene does not run&&nas expected, have a look at the main script code. '
	local h=simDisplayDialog('Compatibility issue',theTxt,sim_dlgstyle_ok,false,'',{0.8,0,0,0,0,0},{0.5,0,0,1,1,1}) 
end 
------------------------------------------------------------------------------ 
 
 
-- This is the main script. The main script is not supposed to be modified,
-- unless there is a very good reason to do it.
-- The main script is called at each simulation pass. Without main script,
-- there is no real simulation (child scripts are not called either in that case).
-- A main script marked as "default" (this is the default case) will use the
-- content of following file: system/dltmscpt.txt. This allows your old simulation
-- scenes to be automatically also using newer features, without explicitely coding
-- them. If you modify the main script, it will be marked as "customized", and you
-- won't benefit of that automatic forward compatibility mechanism. 

-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

-- Initialization part (executed just once, at simulation start) ---------
if (sim_call_type==sim_mainscriptcall_initialization) then
	simOpenModule(sim_handle_all)
	simHandleGraph(sim_handle_all_except_explicit,0)
end
--------------------------------------------------------------------------

-- Regular part (executed at each simulation step) -----------------------
if (sim_call_type==sim_mainscriptcall_regular) then
	-- "Actuation"-part --
	simResumeThreads(sim_scriptthreadresume_default)
	simResumeThreads(sim_scriptthreadresume_actuation_first)
	simLaunchThreadedChildScripts()
	simHandleChildScripts(sim_childscriptcall_actuation)
	simResumeThreads(sim_scriptthreadresume_actuation_last)
	simHandleCustomizationScripts(sim_customizationscriptcall_simulationactuation)
	simHandleModule(sim_handle_all,false)
	simHandleJoint(sim_handle_all_except_explicit,simGetSimulationTimeStep()) -- DEPRECATED
	simHandlePath(sim_handle_all_except_explicit,simGetSimulationTimeStep()) -- DEPRECATED
	simHandleMechanism(sim_handle_all_except_explicit)
	simHandleIkGroup(sim_handle_all_except_explicit)
	simHandleDynamics(simGetSimulationTimeStep())
	simHandleVarious()
	simHandleMill(sim_handle_all_except_explicit)

	-- "Sensing"-part --
	workThreadCount=simGetInt32Parameter(sim_intparam_work_thread_count)
	coreCount=simGetInt32Parameter(sim_intparam_core_count)
	if (workThreadCount<0) then
		workThreadCount=coreCount -- auto setting: thread count=core count
		if (workThreadCount<2) then
			workThreadCount=0 -- turn work threads off if less than 2 cores
		end
	end
	simEnableWorkThreads(workThreadCount) -- thread count can be changed on-the-fly.
	startTime1=simGetSystemTimeInMilliseconds()
	simHandleCollision(sim_handle_all_except_explicit)
	simHandleDistance(sim_handle_all_except_explicit)
	simHandleProximitySensor(sim_handle_all_except_explicit)
	startTime2=simGetSystemTimeInMilliseconds()
	simHandleVisionSensor(sim_handle_all_except_explicit)
	timeDiff2=simGetSystemTimeInMilliseconds(startTime2)
	simWaitForWorkThreads()
	timeDiff=simGetSystemTimeInMilliseconds(startTime1)-timeDiff2
	if (workThreadCount==0) then
		timeDiff=0
	end
	simSetInt32Parameter(sim_intparam_work_thread_calc_time_ms,timeDiff)
	simResumeThreads(sim_scriptthreadresume_sensing_first)
	simHandleChildScripts(sim_childscriptcall_sensing)
	simResumeThreads(sim_scriptthreadresume_sensing_last)
	simHandleCustomizationScripts(sim_customizationscriptcall_simulationsensing)
	simHandleModule(sim_handle_all,true)
	simResumeThreads(sim_scriptthreadresume_allnotyetresumed)
	simHandleGraph(sim_handle_all_except_explicit,simGetSimulationTime()+simGetSimulationTimeStep())
end
--------------------------------------------------------------------------

-- Clean-up part (executed just once, before simulation ends) ------------
if (sim_call_type==sim_mainscriptcall_cleanup) then
	simEnableWorkThreads(0)
	simResetMilling(sim_handle_all)
	simResetMill(sim_handle_all_except_explicit)
	simResetCollision(sim_handle_all_except_explicit)
	simResetDistance(sim_handle_all_except_explicit)
	simResetProximitySensor(sim_handle_all_except_explicit)
	simResetVisionSensor(sim_handle_all_except_explicit)
	simCloseModule(sim_handle_all)
end


--------------------------------------------------------------------------

-- By default threaded child scripts switch back to the main thread after 2 ms. The main
-- thread switches back to a threaded child script at one of above's "simResumeThreads"
-- location
 
 
------------------------------------------------------------------------------ 
-- Following main script automatically commented out by V-REP to guarantee 
-- compatibility with V-REP 3.1.3 and later: 
 
--[[ 
 
-- This is the main script. The main script is not supposed to be modified,
-- unless there is a very good reason to do it.
-- The main script is called at each simulation pass. Without main script,
-- there is no real simulation (child scripts are not called either in that case).
-- A main script marked as "default" (this is the default case) will use the
-- content of following file: system/dltmscpt.txt. This allows your old simulation
-- scenes to be automatically also using newer features, without explicitely coding
-- them. If you modify the main script, it will be marked as "customized", and you
-- won't benefit of that automatic forward compatibility mechanism. 


-- Initialization part (executed just once, at simulation start) ---------
if (simGetSimulationState()==sim_simulation_advancing_firstafterstop) then
	simOpenModule(sim_handle_all)
	simHandleMechanism(sim_handle_all_except_explicit)
	simHandleIkGroup(sim_handle_all_except_explicit)
	simHandleMill(sim_handle_all_except_explicit)
	simHandleCollision(sim_handle_all_except_explicit)
	simHandleDistance(sim_handle_all_except_explicit)
	simHandleProximitySensor(sim_handle_all_except_explicit)
	simHandleVisionSensor(sim_handle_all_except_explicit)
	simHandleGraph(sim_handle_all_except_explicit,simGetSimulationTime())
end
--------------------------------------------------------------------------

-- "Actuation"-part ------------------------------------------------------
simResumeThreads(1)
-- Following code line will execute all "regular" child scripts that
-- appear as first child scripts in the scene hierarchy, and only those
-- that are not marked as "explicit handling":
simHandleChildScript(sim_handle_all_except_explicit)
simHandleModule(sim_handle_all,false)
simResumeThreads(2)
simHandleJoint(sim_handle_all_except_explicit,simGetSimulationTimeStep())
simHandlePath(sim_handle_all_except_explicit,simGetSimulationTimeStep())
simHandleMechanism(sim_handle_all_except_explicit)
simHandleIkGroup(sim_handle_all_except_explicit)
simHandleDynamics(simGetSimulationTimeStep())
simHandleVarious()
simHandleMill(sim_handle_all_except_explicit)
--------------------------------------------------------------------------

-- "Sensing"-part --------------------------------------------------------
workThreadCount=simGetInt32Parameter(sim_intparam_work_thread_count)
coreCount=simGetInt32Parameter(sim_intparam_core_count)
if (workThreadCount<0) then
	workThreadCount=coreCount -- auto setting: thread count=core count
	if (workThreadCount<2) then
		workThreadCount=0 -- turn work threads off if less than 2 cores
	end
end
simEnableWorkThreads(workThreadCount) -- thread count can be changed on-the-fly.
startTime1=simGetSystemTimeInMilliseconds()
simHandleCollision(sim_handle_all_except_explicit)
simHandleDistance(sim_handle_all_except_explicit)
simHandleProximitySensor(sim_handle_all_except_explicit)
startTime2=simGetSystemTimeInMilliseconds()
simHandleVisionSensor(sim_handle_all_except_explicit)
timeDiff2=simGetSystemTimeInMilliseconds(startTime2)
simWaitForWorkThreads()
timeDiff=simGetSystemTimeInMilliseconds(startTime1)-timeDiff2
if (workThreadCount==0) then
	timeDiff=0
end
simSetInt32Parameter(sim_intparam_work_thread_calc_time_ms,timeDiff)
simResumeThreads(3)
-- Following code line will execute all "sensing" child scripts that
-- appear as first child scripts in the scene hierarchy, and only those
-- that are not marked as "explicit handling":
simHandleSensingChildScripts()
simHandleModule(sim_handle_all,true)
simResumeThreads(4)
simHandleGraph(sim_handle_all_except_explicit,simGetSimulationTime()+simGetSimulationTimeStep())
--------------------------------------------------------------------------

-- Clean-up part (executed just once, before simulation ends) ------------
if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
	simEnableWorkThreads(0)
	simResetMilling(sim_handle_all)
	simResetMill(sim_handle_all)
	simResetCollision(sim_handle_all)
	simResetDistance(sim_handle_all)
	simResetProximitySensor(sim_handle_all)
	simResetVisionSensor(sim_handle_all)
	simCloseModule(sim_handle_all)
end
--------------------------------------------------------------------------

-- Threaded child scripts are launched in the same way as non-threaded child scripts.
-- By default they switch back to the main thread after 2 ms. The main thread switches
-- back to a threaded child script just before the main script is called (by default),
-- or at one of above's "simResumeThreads" location (if simSetThreadResumeLocation was
-- called)
 
 
--]] 
------------------------------------------------------------------------------ 

