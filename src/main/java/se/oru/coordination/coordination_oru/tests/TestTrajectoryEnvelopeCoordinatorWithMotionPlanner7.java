package se.oru.coordination.coordination_oru.tests;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.logging.Logger;

import javax.imageio.ImageIO;

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

public abstract class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner7 {
	
	private static Logger metaCSPLogger = MetaCSPLogging.getLogger(TestTrajectoryEnvelopeCoordinatorWithMotionPlanner7.class);
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
		tec.setForwardModel(3, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		
		Coordinate[] ret = new Coordinate[6];		
		ret[0] = new Coordinate(0.36, 0.0);
		ret[1] = new Coordinate(0.18, 0.36);
		ret[2] = new Coordinate(-0.18, 0.36);
		ret[3] = new Coordinate(-0.36, 0.0);
		ret[4] = new Coordinate(-0.18, -0.36);
		ret[5] = new Coordinate(0.18, -0.36);
		
		tec.setFootprint(ret);

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
		String mapFile = "maps"+File.separator+getProperty("image", yamlFile);
		rsp.setMapFilename(mapFile);
		double res = Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setNumInterpolationPoints(100);
	
		Pose startPoseRobot1 = new Pose(3.0,3.0,Math.PI/4);
		Pose goalPoseRobot1 = new Pose(10.0,10.0,Math.PI/4);
		Pose startPoseRobot2 = new Pose(10.0,3.0,3*Math.PI/4);
		Pose goalPoseRobot2 = new Pose(3.0,10.0,3*Math.PI/4);
		Pose startPoseRobot3 = new Pose(3.0,6.5,0.0);
		Pose goalPoseRobot3 = new Pose(10.0,6.5,0.0);
		
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		tec.placeRobot(1, startPoseRobot1);
		tec.placeRobot(2, startPoseRobot2);
		tec.placeRobot(3, startPoseRobot3);

		ArrayList<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();
		
		rsp.setStart(startPoseRobot1);
		rsp.setGoal(goalPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot1 + " and " + goalPoseRobot1);
		PoseSteering[] pss1 = rsp.getPath();
		paths.add(pss1);
		
		rsp.setStart(startPoseRobot2);
		rsp.setGoal(goalPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot2 + " and " + goalPoseRobot2);
		PoseSteering[] pss2 = rsp.getPath();
		paths.add(pss2);
		
		rsp.setStart(startPoseRobot3);
		rsp.setGoal(goalPoseRobot3);
		if (!rsp.plan()) throw new Error ("No path between " + startPoseRobot3 + " and " + goalPoseRobot3);
		PoseSteering[] pss3 = rsp.getPath();
		paths.add(pss3);

		rsp.setStart(goalPoseRobot1);
		rsp.setGoal(startPoseRobot1);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot1 + " and " + startPoseRobot1);
		PoseSteering[] pssInv1 = rsp.getPath();
		paths.add(pssInv1);
				
		rsp.setStart(goalPoseRobot2);
		rsp.setGoal(startPoseRobot2);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot2 + " and " + startPoseRobot2);
		PoseSteering[] pssInv2 = rsp.getPath();
		paths.add(pssInv2);
		
		rsp.setStart(goalPoseRobot3);
		rsp.setGoal(startPoseRobot3);
		if (!rsp.plan()) throw new Error ("No path between " + goalPoseRobot3 + " and " + startPoseRobot3);
		PoseSteering[] pssInv3 = rsp.getPath();
		paths.add(pssInv3);

		//Start a mission dispatching thread for each robot, which will run forever
		int iteration = 0;
		while(true) {
			for (int i = 0; i < 3; i++) {
				Mission m = null;
				if (iteration%2==0) m = new Mission(i+1,paths.get(i));
				else m = new Mission(i+1,paths.get(i+3));
				while(!tec.addMissions(m)) {
					//Sleep for a little (2 sec)
					try { Thread.sleep(500); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
			}			
			tec.computeCriticalSections();
			tec.startTrackingAddedMissions();
			iteration++;
		}				
	}
	
}
