package se.oru.coordination.coordination_oru.simulation2D;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;

import com.vividsolutions.jts.geom.Geometry;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;

public abstract class TrajectoryEnvelopeTrackerRK4 extends AbstractTrajectoryEnvelopeTracker implements Runnable {

	protected static final long WAIT_AMOUNT_AT_END = 3000;
	protected static final double EPSILON = 0.01;
	protected final double MAX_VELOCITY;
	protected final double MAX_ACCELERATION;
	protected double overallDistance = 0.0;
	protected double totalDistance = 0.0;
	protected double positionToSlowDown = -1.0;
	protected double elapsedTrackingTime = 0.0;
	private Thread th = null;
	protected State state = null;
	protected double[] curvatureDampening = null;
	private ArrayList<Integer> internalCriticalPoints = new ArrayList<Integer>();
	private int maxDelayInMilis = 0;
	private Random rand = new Random();
	private TreeMap<Double,Double> slowDownProfile = null;
	private boolean slowingDown = false;
	private boolean useInternalCPs = true;

	public void setUseInternalCriticalPoints(boolean value) {
		this.useInternalCPs = value;
	}
	
	public TrajectoryEnvelopeTrackerRK4(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeSolver solver, TrackingCallback cb) {
		this(te, timeStep, temporalResolution, 1.0, 0.1, solver, cb);
	}
	
	private void computeInternalCriticalPoints() {
		this.curvatureDampening = new double[te.getTrajectory().getPose().length];
		this.curvatureDampening[0] = 1.0;
		Pose[] poses = this.traj.getPose();
		double prevTheta = poses[0].getTheta();
		if (poses.length > 1) prevTheta = Math.atan2(poses[1].getY() - poses[0].getY(), poses[1].getX() - poses[0].getX());
		for (int i = 0; i < poses.length-1; i++) {
			double theta = Math.atan2(poses[i+1].getY() - poses[i].getY(), poses[i+1].getX() - poses[i].getX());
			double deltaTheta = (theta-prevTheta);
			prevTheta = theta;
			if (Math.abs(deltaTheta) > Math.PI/2 && Math.abs(deltaTheta) < 1.9*Math.PI) {
				internalCriticalPoints.add(i);
				metaCSPLogger.info("Found internal critical point (" + te.getComponent() + "): " + (i));
			}
			this.curvatureDampening[i+1] = 1.0;
		}
	}
	
	private double getCurvatureDampening(int index, boolean backwards) {
		if (!backwards) return curvatureDampening[index];
		return curvatureDampening[this.traj.getPose().length-1-index];
	}
	
	public TrajectoryEnvelopeTrackerRK4(TrajectoryEnvelope te, int timeStep, double temporalResolution, double maxVelocity, double maxAcceleration, TrajectoryEnvelopeSolver solver, TrackingCallback cb) {
		super(te, temporalResolution, solver, timeStep, cb);
		this.MAX_VELOCITY = maxVelocity;
		this.MAX_ACCELERATION = maxAcceleration;
		this.te = te;
		this.traj = te.getTrajectory();
		this.temporalResolution = temporalResolution;
		this.state = new State(0.0, 0.0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
		this.computeInternalCriticalPoints();
		this.slowDownProfile = this.getSlowdownProfile();
		this.positionToSlowDown = this.computePositionToSlowDown();
		this.th = new Thread(this, "RK4 tracker " + te.getComponent());
	}
	
	public int getCriticalPoint(TrajectoryEnvelope te1, TrajectoryEnvelope te2, int currentPIR1, int te1Start, int te1End, int te2Start) {
		
		//Number of additional path points robot 2 should stay behind robot 1
		int TRAILING_PATH_POINTS = 3;
		
		//How far ahead in robot 1's trajectory should we look ahead for collisions
		//(all the way to end of critical section if robots can drive in opposing directions)
		int LOOKAHEAD = te1End; //5;
		
		//How far into the critical section has robot 1 reached 
		int depthRobot1 = currentPIR1-te1Start;
		
		//If robot 1 not in critical section yet, return critical section start as waiting point for robot 2
		if (depthRobot1 < 0) return Math.max(0, te2Start-TRAILING_PATH_POINTS);	
		
		//Compute sweep of robot 1's footprint from current position to LOOKAHEAD
		Pose robot1Pose = te1.getTrajectory().getPose()[currentPIR1];
		Geometry robot1InPose = TrajectoryEnvelope.getFootprint(te1.getFootprint(), robot1Pose.getX(), robot1Pose.getY(), robot1Pose.getTheta());
		for (int i = 1; currentPIR1+i < LOOKAHEAD && currentPIR1+i < te1.getTrajectory().getPose().length; i++) {
			Pose robot1NextPose = te1.getTrajectory().getPose()[currentPIR1+i];
			robot1InPose = robot1InPose.union(TrajectoryEnvelope.getFootprint(te1.getFootprint(), robot1NextPose.getX(), robot1NextPose.getY(), robot1NextPose.getTheta()));			
		}
		
		//Starting from the same depth into the critical section as that of robot 1,
		//and decreasing the pose index backwards down to te2Start,
		//return the pose index as soon as robot 2's footprint does not intersect with robot 1's sweep
		for (int i = Math.min(te2.getTrajectory().getPose().length-1, te2Start+depthRobot1); i > te2Start-1; i--) {
			Pose robot2Pose = te2.getTrajectory().getPose()[i];
			Geometry robot2InPose = TrajectoryEnvelope.getFootprint(te2.getFootprint(), robot2Pose.getX(), robot2Pose.getY(), robot2Pose.getTheta());
			if (!robot1InPose.intersects(robot2InPose)) {
				return Math.max(0, i-TRAILING_PATH_POINTS);
			}
		}

		//The only situation where the above has not returned is when robot 2 should
		//stay "parked", therefore wait at index 0
		return Math.max(0, te2Start-TRAILING_PATH_POINTS);
		
//		//The above should have returned, as i is decremented until robot 2 is before the critical section
//		System.out.println("Robot" + te1.getRobotID() + ": start=" +te1Start + " depth=" + depthRobot1 + " Robot" + te2.getRobotID() + ": start=" + te2Start);
//		metaCSPLogger.severe("Could not determine CP for " + te2);
//		throw new Error("Could not determine CP for " + te2);
	}
	
	public void startTracking() {
		while (this.th == null) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		this.th.start();
		if (useInternalCPs) this.startInternalCPThread();
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < endIndex; i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}

	private void startInternalCPThread() {
		Thread t = new Thread() {
			private HashMap<Integer,Integer> userCPReplacements = new HashMap<Integer, Integer>();
			public void run() {
				while (th.isAlive()) {
					ArrayList<Integer> toRemove = new ArrayList<Integer>();
					for (Integer i : internalCriticalPoints) {
						if (getRobotReport().getPathIndex() >= i) {
							toRemove.add(i);
							setCriticalPoint(userCPReplacements.get(i));
							metaCSPLogger.info("Restored critical point (" + te.getComponent() + "): " + userCPReplacements.get(i) + " which was masked by internal critical point " + i);
							break;
						}
						else {
							if (criticalPoint == -1 || criticalPoint > i) {
								userCPReplacements.put(i, criticalPoint);
								metaCSPLogger.info("Set internal critical point (" + te.getComponent() + "): " + i + " replacing critical point " + criticalPoint);
								setCriticalPoint(i);
								break;					
							}
						}
					}
					for (Integer i : toRemove) {
						internalCriticalPoints.remove(i);
					}
					try { Thread.sleep(trackingPeriodInMillis); }
					catch (InterruptedException e) { e.printStackTrace(); }
				}
			}
		};
		t.start();
	}
	
	private TreeMap<Double,Double> getSlowdownProfile() {
		TreeMap<Double,Double> ret = new TreeMap<Double, Double>(Collections.reverseOrder());
		State tempStateBW = new State(0.0, 0.0);
		ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());

		double time = 0.0;
		double deltaTime = 0.5*(this.trackingPeriodInMillis/this.temporalResolution);
		//Compute where to slow down (can do forward here for both states...)
		while (tempStateBW.getVelocity() < MAX_VELOCITY*1.1) {
			double dampeningBW = getCurvatureDampening(getRobotReport(tempStateBW).getPathIndex(), true);
			integrateRK4(tempStateBW, time, deltaTime, false, MAX_VELOCITY*1.1, dampeningBW, MAX_ACCELERATION);
			time += deltaTime;
			ret.put(tempStateBW.getVelocity(), tempStateBW.getPosition());
		}
		
		//for (Double speed : ret.keySet()) System.out.println("@speed " + speed + " --> " + ret.get(speed));
		return ret;
	}
	
	private double computePositionToSlowDown() {
		State tempStateFW = new State(state.getPosition(), state.getVelocity());
		double time = 0.0;
		double deltaTime = 0.5*(this.trackingPeriodInMillis/this.temporalResolution);

		//Compute where to slow down (can do forward here, we have the slowdown profile...)
		while (tempStateFW.getPosition() < this.totalDistance) {
			double prevSpeed = -1.0;
			for (Double speed : this.slowDownProfile.keySet()) {
				//Find your speed in the table (table is ordered w/ highest speed first)...
				if (tempStateFW.getVelocity() > speed) {
					//If this speed lands you after total dist you are OK (you've checked at lower speeds and either returned or breaked...) 
					double landingPosition = tempStateFW.getPosition()+slowDownProfile.get(prevSpeed);
					if (landingPosition > totalDistance) {
						//System.out.println("Found: speed = " + tempStateFW.getVelocity() + " space needed = " + slowDownProfile.get(prevSpeed) + " (delta = " + Math.abs(totalDistance-landingPosition) + ")");
						//System.out.println("Position to slow down = " + tempStateFW.getPosition());
						return tempStateFW.getPosition();
					}
					//System.out.println("Found: speed = " + tempStateFW.getVelocity() + " space needed = " + slowDownProfile.get(speed) + " (undershoot by " + (totalDistance-tempStateFW.getPosition()+slowDownProfile.get(speed)) + ")");
					break;
				}
				prevSpeed = speed;
			}
			
			double dampeningFW = getCurvatureDampening(getRobotReport(tempStateFW).getPathIndex(), true);
			integrateRK4(tempStateFW, time, deltaTime, false, MAX_VELOCITY, dampeningFW, MAX_ACCELERATION);

			time += deltaTime;
		}
		return -this.totalDistance;
	}
	
	public static void integrateRK4(State state, double time, double deltaTime, boolean slowDown, double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION) {
		synchronized(state) {
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
	
			double dxdt = (1.0f / 6.0f) * ( a.getVelocity() + 2.0f*(b.getVelocity() + c.getVelocity()) + d.getVelocity() ); 
		    double dvdt = (1.0f / 6.0f) * ( a.getAcceleration() + 2.0f*(b.getAcceleration() + c.getAcceleration()) + d.getAcceleration() );
			
		    state.setPosition(state.getPosition()+dxdt*deltaTime);
		    state.setVelocity(state.getVelocity()+dvdt*deltaTime);
		}
	}
			
	@Override
	public void setCriticalPoint(int criticalPointToSet) {

		if (this.criticalPoint != criticalPointToSet) {
			
			//A new intermediate index to stop at has been given
			if (criticalPointToSet != -1 && criticalPointToSet > getRobotReport().getPathIndex()) {				
				//Store backups in case we are too late for critical point
				double totalDistanceBKP = this.totalDistance;
				int criticalPointBKP = this.criticalPoint;
				double positionToSlowDownBKP = this.positionToSlowDown;
	
				this.criticalPoint = criticalPointToSet;
				//TOTDIST: ---(state.getPosition)--->x--(computeDist)--->CP
				this.totalDistance = computeDistance(0, criticalPointToSet);
				this.positionToSlowDown = computePositionToSlowDown();
				
				//We are too late for critical point, restore everything
				if (this.positionToSlowDown < state.getPosition()) {
					metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because slowdown distance (" + this.positionToSlowDown +") < current distance (" + state.getPosition() + ")");
					this.criticalPoint = criticalPointBKP;
					this.totalDistance = totalDistanceBKP;
					this.positionToSlowDown = positionToSlowDownBKP;
				}
				else {
					metaCSPLogger.info("Set critical point (" + te.getComponent() + "): " + criticalPointToSet + ", currently at distance " + state.getPosition() + ", will slow down at distance " + this.positionToSlowDown);
				}
			}

			//Critical point = current position, ignore
			else if (criticalPointToSet != -1 && criticalPointToSet <= getRobotReport().getPathIndex()) {
				metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because robot is already at or past that point (and current CP is " + this.criticalPoint + ")");
			}
			
			//The critical point has been reset, go to the end
			else if (criticalPointToSet == -1) {
				this.criticalPoint = criticalPointToSet;
				this.totalDistance = computeDistance(0, traj.getPose().length-1);
				this.positionToSlowDown = computePositionToSlowDown();
				metaCSPLogger.info("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			}
		}
		
		//Same critical point was already set
		else {
			metaCSPLogger.warning("Critical point (" + te.getComponent() + ") " + criticalPointToSet + " was already set!");
		}
		
	}
	
	@Override
	public RobotReport getRobotReport() {
		if (state == null) return null;
		if (!this.th.isAlive()) return new RobotReport(traj.getPose()[0], -1, 0.0, 0.0, -1);
		synchronized(state) {
			Pose pose = null;
			int currentPathIndex = -1;
			double accumulatedDist = 0.0;
			Pose[] poses = traj.getPose();
			for (int i = 0; i < poses.length-1; i++) {
				double deltaS = poses[i].distanceTo(poses[i+1]);
				accumulatedDist += deltaS;
				if (accumulatedDist > state.getPosition()) {
					double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
					pose = poses[i].interpolate(poses[i+1], ratio);
					currentPathIndex = i;
					break;
				}
			}
			if (currentPathIndex == -1) {
				currentPathIndex = poses.length-1;
				pose = poses[currentPathIndex];
			}
			return new RobotReport(pose, currentPathIndex, state.getVelocity(), state.getPosition(), this.criticalPoint);
		}
	}

	private static RobotReport getRobotReport(Trajectory traj, State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), -1);
	}

	public RobotReport getRobotReport(State auxState) {
		if (auxState == null) return null;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		Pose[] poses = traj.getPose();
		for (int i = 0; i < poses.length-1; i++) {
			double deltaS = poses[i].distanceTo(poses[i+1]);
			accumulatedDist += deltaS;
			if (accumulatedDist > auxState.getPosition()) {
				double ratio = 1.0-(accumulatedDist-auxState.getPosition())/deltaS;
				pose = poses[i].interpolate(poses[i+1], ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = poses.length-1;
			pose = poses[currentPathIndex];
		}
		return new RobotReport(pose, currentPathIndex, auxState.getVelocity(), auxState.getPosition(), -1);
	}

	@Override
	public abstract void onPositionUpdate();

	public void delayIntegrationThread(int maxDelayInmillis) {
		this.maxDelayInMilis = maxDelayInmillis;
	}
	
	@Override
	public void run() {
		this.elapsedTrackingTime = 0.0;
		double deltaTime = 0.0;
		boolean atCP = false;
		int myRobotID = te.getRobotID();
		int myTEID = te.getID();
		
		while (true) {
						
			//End condition: passed the middle AND velocity < 0 AND no criticalPoint 			
			boolean skipIntegration = false;
			//if (state.getPosition() >= totalDistance/2.0 && state.getVelocity() < 0.0) {
			if (state.getPosition() >= this.positionToSlowDown && state.getVelocity() < 0.0) {
				if (criticalPoint == -1 && !atCP) {
					//set state to final position, just in case it didn't quite get there (it's certainly close enough)
					state = new State(totalDistance, 0.0);
					onPositionUpdate();
					break;
				}
								
				//Vel < 0 hence we are at CP, thus we need to skip integration
				if (!atCP /*&& getRobotReport().getPathIndex() == criticalPoint*/) {
					metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint);
					atCP = true;
				}
				
				skipIntegration = true;
				
			}

			//Compute deltaTime
			long timeStart = Calendar.getInstance().getTimeInMillis();
			
			//Update the robot's state via RK4 numerical integration
			if (!skipIntegration) {
				if (atCP) {
					metaCSPLogger.info("Resuming from critical point (" + te.getComponent() + ")");
					atCP = false;
				}
				slowingDown = false;
				if (state.getPosition() >= positionToSlowDown) slowingDown = true;
				double dampening = getCurvatureDampening(getRobotReport().getPathIndex(), false);
				integrateRK4(state, elapsedTrackingTime, deltaTime, slowingDown, MAX_VELOCITY, dampening, MAX_ACCELERATION);

			}
			
			//Do some user function on position update
			onPositionUpdate();
						
			//Sleep for tracking period
			int delay = trackingPeriodInMillis;
			if (maxDelayInMilis > 0) delay += rand.nextInt(maxDelayInMilis);
			try { Thread.sleep(delay); }
			catch (InterruptedException e) { e.printStackTrace(); }
			
			//Advance time to reflect how much we have slept (~ trackingPeriod)
			long deltaTimeInMillis = Calendar.getInstance().getTimeInMillis()-timeStart;
			deltaTime = deltaTimeInMillis/this.temporalResolution;
			elapsedTrackingTime += deltaTime;
		}
		
		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
			//System.out.println("Waiting " + te.getComponent());
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
	}

	public static double[] computeDTs(Trajectory traj, double maxVel, double maxAccel) {
		double distance = computeDistance(traj, 0, traj.getPose().length-1);
		State state = new State(0.0, 0.0);
		double time = 0.0;
		double deltaTime = 0.0001;
		
		ArrayList<Double> dts = new ArrayList<Double>();
		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
		dts.add(0.0);
		times.put(0, 0.0);
		
		//First compute time to stop (can do FW here...)
		while (state.getPosition() < distance/2.0 && state.getVelocity() < maxVel) {
			integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		double positionToSlowDown = distance-state.getPosition();
		//System.out.println("Position to slow down is: " + MetaCSPLogging.printDouble(positionToSlowDown,4));

		state = new State(0.0, 0.0);
		time = 0.0;
		while (true) {
			if (state.getPosition() >= distance/2.0 && state.getVelocity() < 0.0) break;
			if (state.getPosition() >= positionToSlowDown) {
				integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			}
			else {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);				
			}
			//System.out.println("Time: " + time + " " + rr);
			//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
			time += deltaTime;
			RobotReport rr = getRobotReport(traj, state);
			if (!times.containsKey(rr.getPathIndex())) {
				times.put(rr.getPathIndex(), time);
				dts.add(time-times.get(rr.getPathIndex()-1));
			}
		}
		if (dts.size() < traj.getPose().length) {
			times.put(traj.getPose().length-1, time);		
			dts.add(time-times.get(traj.getPose().length-2));
		}
		
		//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
		
		double[] ret = new double[dts.size()];
		for (int i = 0; i < dts.size(); i++) ret[i] = dts.get(i);
		return ret;

	}

	

}