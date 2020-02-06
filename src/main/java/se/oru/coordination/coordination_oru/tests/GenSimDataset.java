package se.oru.coordination.coordination_oru.tests;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

// import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
// import se.oru.coordination.coordination_oru.util.RVizVisualization;
import se.oru.coordination.coordination_oru.util.Missions;

import org.metacsp.multi.spatioTemporal.paths.Pose;
// import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

@DemoDescription(desc = "Demo of Zed.")
public abstract class GenSimDataset {

	public static void main(String[] args) throws InterruptedException, FileNotFoundException {

		// *********************** Customize 1 ***********************
		double MAX_ACCEL = 10.0; // 1.0
		double MAX_VEL = 20.0; // 4.0

		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings 
		// --thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec
							 = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())
				       -(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		}); // tec实例的addComparator方法传入一个Comparator类实例并复写compare方法
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()
				       -o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});
		
		// *********************** Customize 2 ***********************
		boolean readPathFromFile = false;
		boolean saveImg = false;
		boolean showEnvelope = false;
		boolean showNumber = false;
		String pathFile = "paths/aTest_path.txt"; // "pathPoint+robotID"
		String yamlFile = "maps/mapTest1.yaml"; // map name and properties
		Integer[] robotIDs = new Integer[] {1};
		int max_iter = 10; // if -1, then infinite

		char[] pathPoints = {'a','b'}; // if read path from file
		Pose startRobot1 = new Pose( 5.5, 42.5, Math.PI/4);
		// Pose goalRobot1  = new Pose(32.0, 24.0, 5*Math.PI/4);

		Missions.loadLocationAndPathData(pathFile); // test_poses
		String mapFile = "maps/"+Missions.getProperty("image", yamlFile);
		double res = Double.parseDouble(Missions.getProperty("resolution", yamlFile));
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization(yamlFile,showNumber,saveImg);
		viz.setMinimumVisibleFrame(0, 0, 64, 48); // forbid slipping screen
		tec.setVisualization(viz);
		tec.showEnvelope = showEnvelope;

		//Create a shape representing the robot
		Coordinate fp1 = new Coordinate(-1.0,0.5);
		Coordinate fp2 = new Coordinate(1.0,0.3);
		Coordinate fp3 = new Coordinate(1.0,-0.3);
		Coordinate fp4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(fp1, fp2, fp3, fp4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
 
		tec.setUseInternalCriticalPoints(false);
		tec.setYieldIfParking(false);

		//Path planner: ReedsShepp
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename(mapFile);
		rsp.setMapResolution(res);
		rsp.setRadius(0.2);
		rsp.setFootprint(fp1, fp2, fp3, fp4);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		
		//Set and place robots
		for (Integer robotID : robotIDs) {
			tec.setForwardModel(robotID, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));	
			if (readPathFromFile) tec.placeRobot(robotID, Missions.getLocation("a"+robotID));
			else tec.placeRobot(robotID, startRobot1);
		}
		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final Integer robotID : robotIDs) {
			//For each robot, create a thread 
			// --that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				int onMission = 0;
				boolean exit = false;
				@Override
				public void run() {
					while (!exit) {

						if (onMission==0) {
							if (!readPathFromFile) {
								Pose goalRobot1  = new Pose(32.0, 24.0, 5*Math.PI/4);
								rsp.setStart(startRobot1);
								rsp.setGoals(goalRobot1);
								rsp.plan();
								Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
							}
							else {
								for (int i=0; i<pathPoints.length-1; i++) { 
									rsp.setStart(Missions.getLocation(pathPoints[i]+Integer.toString(robotID)));
									rsp.setGoals(Missions.getLocation(pathPoints[i+1]+Integer.toString(robotID)));
									rsp.plan();
									Missions.enqueueMission(new Mission(robotID,rsp.getPath()));
								}
							}
							onMission = 1;
						} // end planning path
						
						Mission m = Missions.getMission(robotID, iteration%Missions.getMissions(robotID).size());
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								viz.pathCNT = viz.pathCNT+1;
								tec.computeCriticalSections();
								tec.startTrackingAddedMissions();
								iteration++;
								onMission = 0;
							}
						}
						//Sleep for a little (original 2 sec)
						try { Thread.sleep(500); }
						catch (InterruptedException e) { e.printStackTrace(); }
						System.out.println("*****"+iteration+"/"+max_iter+"*****");
						if (iteration==max_iter & max_iter!=-1) {
							exit = true;
						}
					}
				} // end run
			}; 
			//Start the thread!
			t.start();
		} // end for loop of robots

	} // end main function

} // end class demo "aTest_map"