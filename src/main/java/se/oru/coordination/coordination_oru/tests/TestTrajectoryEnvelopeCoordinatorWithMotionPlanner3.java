package se.oru.coordination.coordination_oru.tests;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.logging.MetaCSPLogging;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;

public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner3 {

	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner3.class);
	public static HashMap<Integer,ArrayList<Mission>> missions = new HashMap<Integer, ArrayList<Mission>>();

	//Get property from YAML file
	public static String getProperty(String property, String yamlFile) {
		String ret = null;
		try {
			File file = new File(yamlFile);
			BufferedReader br = new BufferedReader(new FileReader(file));
			String st;
			while((st=br.readLine()) != null){
				String key = st.substring(0, st.indexOf(":")).trim();
				String value = st.substring(st.indexOf(":")+1).trim();
				if (key.equals(property)) {
					ret = value;
					break;
				}
			}
			br.close();
		}
		catch (IOException e) { e.printStackTrace(); }
		return ret;
	}

	//Convenience method to put a mission into a global hashmap
	private static void putMission(Mission m) {
		if (!missions.containsKey(m.getRobotID())) missions.put(m.getRobotID(), new ArrayList<Mission>());
		missions.get(m.getRobotID()).add(m);
	}

	//Convenience method to get the i-th mission for a given robotID from the global hashmap	
	private static Mission getMission(int robotID, int missionNumber) {
		return missions.get(robotID).get(missionNumber);
	}

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker (also abstract)
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add a comparator to determine robot orderings thru critical sections
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));

		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map-empty.yaml";
		tec.setupGUI(yamlFile);
		//tec.setupGUI(null);

		tec.setUseInternalCriticalPoints(false);

		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+getProperty("image", yamlFile));
		double res = Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setNumInterpolationPoints(100);

		ArrayList<Pose> posesRobot1 = new ArrayList<Pose>();
		posesRobot1.add(new Pose(2.0,10.0,0.0));
		posesRobot1.add(new Pose(10.0,13.0,0.0));
		posesRobot1.add(new Pose(18.0,10.0,0.0));
		posesRobot1.add(new Pose(26.0,13.0,0.0));
		posesRobot1.add(new Pose(34.0,10.0,0.0));
		posesRobot1.add(new Pose(42.0,13.0,0.0));
		posesRobot1.add(new Pose(50.0,10.0,0.0));

		ArrayList<Pose> posesRobot2 = new ArrayList<Pose>();
		//Robot 1 and robot 2 in opposing directions
		posesRobot2.add(new Pose(50.0,13.0,Math.PI));
		posesRobot2.add(new Pose(42.0,10.0,Math.PI));
		posesRobot2.add(new Pose(34.0,13.0,Math.PI));
		posesRobot2.add(new Pose(26.0,10.0,Math.PI));
		posesRobot2.add(new Pose(18.0,13.0,Math.PI));
		posesRobot2.add(new Pose(10.0,10.0,Math.PI));
		posesRobot2.add(new Pose(2.0,13.0,Math.PI));
		
//		//Robot 1 and Robot 2 in same direction
//		posesRobot2.add(new Pose(2.0,13.0,0.0));
//		posesRobot2.add(new Pose(10.0,10.0,0.0));
//		posesRobot2.add(new Pose(18.0,13.0,0.0));
//		posesRobot2.add(new Pose(26.0,10.0,0.0));
//		posesRobot2.add(new Pose(34.0,13.0,0.0));
//		posesRobot2.add(new Pose(42.0,10.0,0.0));
//		posesRobot2.add(new Pose(50.0,13.0,0.0));
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, posesRobot1.get(0));
		tec.placeRobot(2, posesRobot2.get(0));

		ArrayList<PoseSteering> pathsRobot1 = new ArrayList<PoseSteering>();
		for (int i = 0; i < posesRobot1.size()-1; i++) {
			rsp.setStart(posesRobot1.get(i));
			rsp.setGoal(posesRobot1.get(i+1));
			rsp.clearObstacles();
			Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
			rsp.addObstacles(fpGeom, posesRobot2.get(0), posesRobot2.get(posesRobot2.size()-1));
			if (!rsp.plan()) throw new Error ("No path between " + posesRobot1.get(i) + " and " + posesRobot1.get(i+1));			
			PoseSteering[] path = rsp.getPath();
			if (i == 0) pathsRobot1.add(path[0]);
			for (int j = 1; j < path.length; j++) pathsRobot1.add(path[j]);
		}
		ArrayList<PoseSteering> pathsRobot1Inv = new ArrayList<PoseSteering>();
		pathsRobot1Inv.addAll(pathsRobot1);
		Collections.reverse(pathsRobot1Inv);

		ArrayList<PoseSteering> pathsRobot2 = new ArrayList<PoseSteering>();
		for (int i = 0; i < posesRobot2.size()-1; i++) {
			rsp.setStart(posesRobot2.get(i));
			rsp.setGoal(posesRobot2.get(i+1));
			rsp.clearObstacles();
			Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(footprint1, footprint2, footprint3, footprint4);
			rsp.addObstacles(fpGeom, posesRobot1.get(0), posesRobot1.get(posesRobot1.size()-1));
			if (!rsp.plan()) throw new Error ("No path between " + posesRobot2.get(i) + " and " + posesRobot2.get(i+1));			
			PoseSteering[] path = rsp.getPath();
			if (i == 0) pathsRobot2.add(path[0]);
			for (int j = 1; j < path.length; j++) pathsRobot2.add(path[j]);
		}
		ArrayList<PoseSteering> pathsRobot2Inv = new ArrayList<PoseSteering>();
		pathsRobot2Inv.addAll(pathsRobot2);
		Collections.reverse(pathsRobot2Inv);

		putMission(new Mission(1, pathsRobot1.toArray(new PoseSteering[pathsRobot1.size()])));
		putMission(new Mission(1, pathsRobot1Inv.toArray(new PoseSteering[pathsRobot1Inv.size()])));
		putMission(new Mission(2, pathsRobot2.toArray(new PoseSteering[pathsRobot2.size()])));
		putMission(new Mission(2, pathsRobot2Inv.toArray(new PoseSteering[pathsRobot2Inv.size()])));

		metaCSPLogger.info("Added missions " + missions);

		final Random rand = new Random(Calendar.getInstance().getTimeInMillis());
		final int minDelay = 500;
		final int maxDelay = 3000;
		//Start a mission dispatching thread for each robot, which will run forever
		for (int i = 1; i <= 2; i++) {
			final int robotID = i;
			//For each robot, create a thread that dispatches the "next" mission when the robot is free 
			Thread t = new Thread() {
				int iteration = 0;
				public void run() {
					while (true) {
						//Mission to dispatch alternates between (rip -> desti) and (desti -> rip)
						Mission m = getMission(robotID, iteration%2);
						synchronized(tec) {
							//addMission returns true iff the robot was free to accept a new mission
							if (tec.addMissions(m)) {
								tec.computeCriticalSections();
								if (minDelay > 0) {
									long delay = minDelay+rand.nextInt(maxDelay-minDelay);
									//Sleep for a random delay in [minDelay,maxDelay]
									try { Thread.sleep(delay); }
									catch (InterruptedException e) { e.printStackTrace(); }
								}
								tec.startTrackingAddedMissions();
								iteration++;
							}
						}
						//Sleep for a little (2 sec)
						try { Thread.sleep(2000); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
				}
			};
			//Start the thread!
			t.start();
		}

	}

}
