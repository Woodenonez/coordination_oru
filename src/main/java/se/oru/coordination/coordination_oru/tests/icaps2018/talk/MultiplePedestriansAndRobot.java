package se.oru.coordination.coordination_oru.tests.icaps2018.talk;

import java.io.File;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Vector;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.PedestrianForwardModel;
import se.oru.coordination.coordination_oru.simulation2D.PedestrianTrajectory;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "One-shot navigation of several pedestrians and a robot coordinating on static paths that overlap in a straight portion.")
public class MultiplePedestriansAndRobot {

	public static void main(String[] args) throws InterruptedException {

		double MAX_ACCEL = 1.0;
		double MAX_VEL = 1.0;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec = new TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(MAX_VEL, MAX_ACCEL);
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
		
		// Pedestrian Footprints
		Coordinate footprint1 = new Coordinate(-0.1,0.1);
		Coordinate footprint2 = new Coordinate(0.1,0.1);
		Coordinate footprint3 = new Coordinate(0.1,-0.1);
		Coordinate footprint4 = new Coordinate(-0.1,-0.1);
		
		// Robot Footprints
		Coordinate f1 = new Coordinate(0.3, 0.3);
		Coordinate f2 = new Coordinate(0.3, -0.3);
		Coordinate f3 = new Coordinate(-0.3, -0.3);
		Coordinate f4 = new Coordinate(-0.3, 0.3);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);

		String fileNamePrefix = "paths_pedsim/person";
		int nums[] = {112, 146, 160, 29, 30, 55, 5, 77, 97};
		
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(nums);
		BrowserVisualization viz = new BrowserVisualization();
		viz.setInitialTransform(39, -1.8, 1.4);
		tec.setVisualization(viz);

		for(int i = 0; i < nums.length; i++) {
			
			// Two robots. Others behave as pedestrians.
			if (i != 1) {
				tec.setFootprint(nums[i], footprint1, footprint2, footprint3, footprint4);
				tec.addUncontrollableRobots(nums[i]);	
				tec.setForwardModel(nums[i], new PedestrianForwardModel());
			}
			else {
				tec.setFootprint(nums[i], f1, f2, f3, f4);
				tec.setForwardModel(nums[i], new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
			}
			
			//PoseSteering[] path1 = Missions.loadPathFromFile(filename_prefix + Integer.toString(nums[i]) + ".txt");
			PedestrianTrajectory p1 = new PedestrianTrajectory(fileNamePrefix + Integer.toString(nums[i]) + ".txt");
			tec.addPedestrianTrajectory(nums[i], p1);
			tec.placeRobot(nums[i], p1.getPose(0));
			Mission m1 = new Mission(nums[i], p1.getPoseSteeringAsArray());

			//Place robots in their initial locations (looked up in the data file that was loaded above)
			// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
			// -- each trajectory envelope has a path of one pose (the pose of the location)
			// -- each trajectory envelope is the footprint of the corresponding robot in that pose

			tec.addMissions(m1);
			Thread.sleep(200);
		}
		
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();
	}
}
