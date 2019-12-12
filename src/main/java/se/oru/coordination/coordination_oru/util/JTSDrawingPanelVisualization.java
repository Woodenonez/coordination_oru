package se.oru.coordination.coordination_oru.util;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.metacsp.multi.spatial.DE9IM.GeometricShapeDomain;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.utility.UI.JTSDrawingPanel;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
// import com.vividsolutions.jts.geom.Point;
// import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

// used for save images (Zed, CTH)
import java.io.File;
import javax.imageio.ImageIO;
import javax.swing.JPanel;
import java.awt.image.BufferedImage;
import java.awt.Robot;
import java.awt.Graphics2D;
import java.awt.AWTException;

public class JTSDrawingPanelVisualization implements FleetVisualization {

	private JTSDrawingPanel panel = null;
	private boolean saveImage = false; // (Zed, CTH)
	private String imgType = "png"; // (Zed, CTH)

	public int pathCNT = 0;

	/**
	 * This function is used to save the screenshot of the panel.
	 * @param panel
	 * @author Zed, CTH
	 */
	protected void saveImage(JPanel panel) {
		String imgtype = "png";
		this.saveImage(panel, imgtype, "test."+imgtype);
	}
	private void saveImage(JPanel panel, String imgType, String imgName) {
		BufferedImage imageBuf = null;
		try {
			imageBuf = new Robot().createScreenCapture(panel.getBounds());
		} catch (AWTException e1) {
			e1.printStackTrace();
		}
		Graphics2D graphics2d = imageBuf.createGraphics();
		panel.paint(graphics2d);
		try {
			ImageIO.write(imageBuf, imgType, new File(imgName+'.'+imgType));
		} catch (Exception e) {
			System.out.println("Error saving!");
		}
	} // end save images

	public JTSDrawingPanelVisualization() {
		this(null);
	}
	public JTSDrawingPanelVisualization(String mapYAMLFile) {
		this.setupGUI(mapYAMLFile, true); // (Zed, CTH)
	}
	public JTSDrawingPanelVisualization(String mapYAMLFile, boolean showText) {
		this.setupGUI(mapYAMLFile, showText);
	} // (Zed, CTH)
	public JTSDrawingPanelVisualization(String mapYAMLFile, boolean showText, boolean saveImg) {
		this.saveImage = saveImg;
		this.setupGUI(mapYAMLFile, showText);
	} // (Zed, CTH)
	
	public JTSDrawingPanel getPanel() {
		return this.panel;
	}
	
	public void setSize(int width, int height) {
		JFrame topFrame = (JFrame) SwingUtilities.getWindowAncestor(this.getPanel());
		topFrame.setSize(width,height);
	}

	@Override
	public void displayRobotState(TrajectoryEnvelope te, RobotReport rr, String ... extraStatusInfo) {
		double x = rr.getPose().getX();
		double y = rr.getPose().getY();
		double theta = rr.getPose().getTheta();
		String name = "R"+te.getRobotID();
		int idx = rr.getPathIndex(); // (Zed, CTH)
		double dist = rr.getDistanceTraveled(); // (Zed, CTH)
		String color = "#FF0000"; // red - for buzy/target
		if (extraStatusInfo != null) {
			for (String st : extraStatusInfo) {
				name += ("\\"+st);
			}
		}
		if (idx == -1) {
			x = te.getTrajectory().getPose()[0].getX();
			y = te.getTrajectory().getPose()[0].getY();
			theta = te.getTrajectory().getPose()[0].getTheta();
			color = "#4286F4"; // blue - for idle/obstacle
		}
		panel.addGeometry(name,
						  TrajectoryEnvelope.getFootprint(te.getFootprint(), x, y, theta), 
						  false, true, false, color); // add the robot

		if (idx != -1 && this.saveImage) { // (Zed, CTH)
			String pathName = new SimpleDateFormat("dd").format(new Date());
			pathName = "/home/Append/Data/CORUSD/"+pathName+'_'+name+'_'+Integer.toString(this.pathCNT)+'/';
			// pathName = "record/"+pathName+'_'+name+'_'+Integer.toString(this.pathCNT)+'/';
			String imgName = name+"_"+Double.toString(dist)
								 +"_x"+Double.toString(x)+"_y"+Double.toString(y);
			if (!new File(pathName).exists()) {
				new File(pathName).mkdirs();
			}
			this.saveImage(this.panel, this.imgType, pathName+imgName);
		}
		
	}

	@Override
	public void displayDependency(RobotReport rrWaiting, RobotReport rrDriving, String dependencyDescriptor) {
		panel.addArrow(dependencyDescriptor, rrWaiting.getPose(), rrDriving.getPose());
	}
	
	private void setupGUI(String mapYAMLFile, boolean showText) {
		//Show everything in a GUI (vehicle positions are updated in real time by the trackers, see below)
		this.panel = JTSDrawingPanel.makeEmpty("Current status of robots");
		//setPriorityOfEDT(Thread.MIN_PRIORITY);
		//setPriorityOfEDT(Thread.MAX_PRIORITY);
		panel.setSmoothTransitions(false);
		panel.setZoomIntensity(0.01);
		panel.setArrowHeadSizeInMeters(0.6*TrajectoryEnvelopeCoordinator.MAX_DEFAULT_FOOTPRINT_DIMENSION);
		if (!showText) panel.setTextSizeInMeters(0); // (Zed, CTH)
		else panel.setTextSizeInMeters(0.8*TrajectoryEnvelopeCoordinator.MAX_DEFAULT_FOOTPRINT_DIMENSION);
		//System.out.println("TEXT SIZE IN METERS IS " + 0.5*getMaxFootprintDimension(1));
		if (mapYAMLFile != null) panel.setMap(mapYAMLFile);
		panel.addKeyListener(new KeyListener() {

			@Override
			public void keyTyped(KeyEvent e) {
				String fileName = new SimpleDateFormat("yyyy-MM-dd-HH:mm:ss:SSS").format(new Date());
				if (e.getKeyChar() == 's') {
					fileName += ".svg";
					dumpSVG(fileName);
					System.out.println("Saved screenshot " + fileName);
				}
				else if (e.getKeyChar() == 'p') {
					fileName += ".pdf";
					dumpPDF(fileName);
					System.out.println("Saved screenshot " + fileName);
				}
				else if (e.getKeyChar() == 'e') {
					fileName += ".eps";
					dumpEPS(fileName);
					System.out.println("Saved screenshot " + fileName);
				}
//				else if (e.getKeyChar() == 'm') {
//					System.out.println("Muted robots: " + Arrays.toString(tec.getMuted()));
//				}
//				else {
//					try {
//						int robotID = Integer.parseInt(""+e.getKeyChar());
//						tec.toggleMute(robotID);
//					}
//					catch(NumberFormatException e1) {}
//				}

			}

			@Override
			public void keyReleased(KeyEvent e) { }

			@Override
			public void keyPressed(KeyEvent e) { }
		});
		panel.setFocusable(true);
	}
	
	/**
	 * Dump a vector graphics file with the contents of the GUI. Note: this operation is slow, so this method
	 * should not be called within the control loop.
	 * @param fileName Name of the PDF file to write to.
	 */
	public void dumpPDF(String fileName) {
		panel.writePDF(fileName);
	}

	/**
	 * Dump a vector graphics file with the contents of the GUI. Note: this operation is slow, so this method
	 * should not be called within the control loop.
	 * @param fileName Name of the SVG file to write to.
	 */
	public void dumpSVG(String fileName) {
		panel.writeSVG(fileName);
	}

	/**
	 * Dump a vector graphics file with the contents of the GUI. Note: this operation is slow, so this method
	 * should not be called within the control loop.
	 * @param fileName Name of the EPS file to write to.
	 */
	public void dumpEPS(String fileName) {
		panel.writeEPS(fileName);
	}

	/**
	 * Scale and translate the view of the GUI to include all geometries. 
	 */
	public void centerView() {
		//		this.panel.centerView();
		this.panel.reinitVisualization();
	}
	

	@Override
	public void updateVisualization() {
		panel.removeOldGeometries(1000);
		panel.updatePanel();
	}

	@Override
	public void addEnvelope(TrajectoryEnvelope te) {
		GeometricShapeDomain dom = (GeometricShapeDomain)te.getEnvelopeVariable().getDomain();
		panel.addGeometry("_"+te.getID(), dom.getGeometry(), true, false);
		panel.setPermanent("_"+te.getID());
	}

	@Override
	public void removeEnvelope(TrajectoryEnvelope te) {
		panel.removeGeometry("_"+te.getID());
	}
	
	public void setMinimumVisibleFrame(double minX, double minY, double maxX, double maxY) {
		GeometryFactory gf = new GeometryFactory();
		Geometry frame = gf.createPolygon(new Coordinate[] {
				new Coordinate(minX,minY),
				new Coordinate(minX,maxY),
				new Coordinate(maxX,maxY),
				new Coordinate(maxX,minY),
				new Coordinate(minX,minY)
		});	
		panel.removeGeometry("_frame");
		panel.addGeometry("_frame", frame, true, false, true, "#000000");
		panel.setPermanent("_frame");
	}

	@Override
	public void setMap(String yamlFile) {
		panel.setMap(yamlFile);
	}

	@Override
	public int periodicEnvelopeRefreshInMillis() {
		// TODO Auto-generated method stub
		return 0;
	}

}
