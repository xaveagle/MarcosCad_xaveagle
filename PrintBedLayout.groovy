import java.lang.reflect.Type

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import com.neuronrobotics.bowlerkernel.Bezier3d.IInteractiveUIElementProvider
import com.neuronrobotics.bowlerkernel.Bezier3d.manipulation
import com.neuronrobotics.bowlerkernel.Bezier3d.manipulation.DragState
import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.BowlerStudioController
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import javafx.event.EventHandler;
import javafx.event.EventType;
import javafx.scene.input.MouseEvent;
import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Vector3d
import javafx.application.Platform
import javafx.scene.input.MouseEvent
import javafx.scene.paint.Color
import javafx.scene.paint.PhongMaterial
import javafx.scene.transform.Affine

double bedX=250
double bedY=250

def colors =[
	Color.WHITE,
	Color.GREY,
	Color.BLUE,
	Color.TAN
]

class PrintBedObject{
	HashMap<EventType<MouseEvent>, EventHandler<MouseEvent>> map = new HashMap<>();
	double startx = 0;
	double starty = 0;
	double newx = 0;
	double newy = 0;
	double newz = 0;
	TransformNR camFrame = null;
	boolean dragging = false;
	double depth = 0;

	private ArrayList<Runnable> eventListeners = new ArrayList<>();
	private ArrayList<Runnable> saveListeners = new ArrayList<>();
	private Vector3d orintation=new Vector3d(1, 1, 0);
	private TransformNR globalPose = new TransformNR();
	TransformNR currentPose;
	private PhongMaterial color;// = new PhongMaterial(getColor());
	private PhongMaterial highlight = new PhongMaterial(Color.GOLD);
	private String name;
	private CSG part;
	private File source;
	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
	private String name2;
	private double xMax;
	private double xMin;
	private double yMax;
	private double yMin;
	private Affine manipulator =new Affine()
	IInteractiveUIElementProvider ui =new IInteractiveUIElementProvider() {
		public void runLater(Runnable r) {
			BowlerStudio.runLater(r);
		}

		public TransformNR getCamerFrame() {
			return BowlerStudio.getCamerFrame();
		}

		public double getCamerDepth() {
			return BowlerStudio.getCamerDepth();
		}
	}
	private enum DragState{
		IDLE,
		Dragging
	}
	private DragState state = DragState.IDLE;

	public IInteractiveUIElementProvider getUi() {
		return ui;
	}
	private void fireMove( TransformNR trans, TransformNR camFrame2) {
		for (Runnable R : eventListeners) {
			R.run();
		}
	}
	private void fireSave() {
		new Thread( {
			save();
		}).start();
	}

	public void set(double newX, double newY, double newZ) {
		newx=newX;
		newy=newY;
		newz=newZ;
		globalPose.setX(newX);
		globalPose.setY(newY);
		globalPose.setZ(newZ);
		setGlobal(new TransformNR(newX,newY,newZ,new RotationNR()));
		for (Runnable R : eventListeners) {
			R.run();
		}

	}
	public PrintBedObject(String name, CSG part, double xMax, double xMin, double yMax, double yMin){
		this.part =  part;
		this.name =  name;
		this.xMax =  xMax;
		this.xMin =  xMin;
		this.yMax =  yMax;
		this.yMin =  yMin;
		source=new File(ScriptingEngine.getRepositoryCloneDirectory("https://github.com/OperationSmallKat/Marcos.git").getAbsolutePath()+"/print_bed_location_"+name+".json")
		if(source.exists()) {
			//println "Loading location from "+source.getAbsolutePath()
			Type TT_mapStringString = new TypeToken<ArrayList<TransformNR>>() {
					}.getType();

			def sourceText = source.text
			//println " text "+sourceText
			ArrayList<TransformNR> l = gson.fromJson(sourceText, TT_mapStringString);
			if(l!=null&& l.size()>0)
				this.globalPose =l.get(0)
		}
		if(globalPose==null) {
			println "Loading new pose for "+name
			globalPose = new TransformNR();
		}
		currentPose=globalPose.copy();
		part.setManipulator(manipulator)
		save()
		color=new PhongMaterial(part.getColor());
		getUi().runLater( {
			TransformFactory.nrToAffine(globalPose, manipulator);
		});

		map.put(MouseEvent.ANY,  new EventHandler<MouseEvent>() {
					@Override
					public void handle(MouseEvent event) {
						switch(event.getEventType().getName()) {
							case "MOUSE_PRESSED":
								pressed(event);
								break;
							case "MOUSE_DRAGGED":
								dragged(event);
								break;
							case "MOUSE_RELEASED":
								release(event);
								break;
							case "MOUSE_MOVED":
							// ignore
								break;
							case "MOUSE_ENTERED":
								part.getMesh().setMaterial(highlight);
								break;
							case "MOUSE_EXITED":
								if(state==DragState.IDLE)
									part.getMesh().setMaterial(color);
								break;
							default:
							//System.out.println("UNKNOWN! Mouse event "+name);
								break;
						}
					}
				});
		// manipulator added to UI via the parts storage, transported to the UI when added
		part.getStorage().set("manipulator", map);
		BowlerStudioController.addCsg(part)

	}
	public void save() {
		update()
		source.text = gson.toJson([currentPose])
	}
	public void update() {
		double minYTest = part.getMinY()-yMin+globalPose.getY()
		double maxYTest = part.getMaxY()-yMax+globalPose.getY()
		double minXTest = part.getMinX()-xMin+globalPose.getX()
		double maxXTest = part.getMaxX()-xMax+globalPose.getX()
		if(minYTest<0)
			globalPose.translateY(-minYTest)
		if(minXTest<0)
			globalPose.translateX(-minXTest)
		if(maxYTest>0)
			globalPose.translateY(-maxYTest)
		if(maxXTest>0)
			globalPose.translateX(-maxXTest)
		currentPose=globalPose.copy()
		println "Update "+name+" to "+((int)globalPose.getX())+" : "+((int)globalPose.getY())
		Platform.runLater({
			TransformFactory.nrToAffine(globalPose,manipulator)
		})
	}
	private void pressed(MouseEvent event) {
		state = DragState.Dragging;
		new Thread( {
			camFrame = getUi().getCamerFrame();
			depth = -1600 / getUi().getCamerDepth();
			event.consume();
			dragging = false;
		}).start();
	}
	private void release(MouseEvent event) {
		mouseRelease(event);
		state = DragState.IDLE;
		part.getMesh().setMaterial(color);
	}
	private void dragged(MouseEvent event) {
		getUi().runLater( {
			setDraggingEvent(event);
			double deltx = (startx - event.getScreenX());
			double delty = (starty - event.getScreenY());
			TransformNR trans = new TransformNR(deltx / depth, delty / depth, 0, new RotationNR());

			performMove( trans,camFrame);
		});
		event.consume();
	}
	public boolean isMoving() {
		return state==DragState.Dragging;
	}
	private void mouseRelease(MouseEvent event) {
		if (dragging) {
			dragging = false;
			globalPose.setX(newx);
			globalPose.setY(newy);
			globalPose.setZ(newz);
			event.consume();
			fireSave();
		}
	}
	private void setDraggingEvent(MouseEvent event) {
		if (dragging == false) {
			startx = event.getScreenX();
			starty = event.getScreenY();
		}
		dragging = true;
	}
	private void performMove( TransformNR trans, TransformNR camFrame2) {
		TransformNR globalTMP = camFrame2.copy();
		globalTMP.setX(0);
		globalTMP.setY(0);
		globalTMP.setZ(0);
		TransformNR global = globalTMP.times(trans);
		newx = (global.getX() * orintation.x + globalPose.getX());
		newy = (global.getY() * orintation.y + globalPose.getY());
		newz = (global.getZ() * orintation.z + globalPose.getZ());
		global.setX(newx);
		global.setY(newy);
		global.setZ(newz);

		global.setRotation(new RotationNR());
		setGlobal(global);
		// System.out.println(" drag "+global.getX()+" , "+global.getY()+" ,
		// "+global.getZ()+" "+deltx+" "+delty);
		fireMove(trans,camFrame2);
	}
	private void setGlobal(TransformNR global) {

		currentPose.setX(newx);
		currentPose.setY(newy);
		currentPose.setZ(newz);
		getUi().runLater( {
			TransformFactory.nrToAffine(global, manipulator);
		});
	}

}

MobileBase marcos=DeviceManager.getSpecificDevice( "Marcos",{
	return ScriptingEngine.gitScriptRun(	"https://github.com/OperationSmallKat/Marcos.git","Marcos.xml",null)
})

IgenerateBed bedGen = (IgenerateBed)MobileBaseCadManager.get(marcos).getIgenerateBed()
ArrayList<CSG> cache = bedGen.cache
if(cache==null)
	throw new RuntimeException("This model has no bed generator!")
HashMap<String,CSG> mapOfNameToMFGPart = new HashMap<>()
HashMap<String,ArrayList<CSG>> bedNames =new HashSet<>();
BowlerStudioController.clearCSG()
for(CSG c:cache) {
	String name =c.getName()
	def bitGetStorageGetValue = c.getStorage().getValue("bedType")
	if(bitGetStorageGetValue.present) {
		def bedName = bitGetStorageGetValue.get().toString()
		if(bedNames.get(bedName)==null)
			bedNames.put(bedName, new ArrayList<>())
		CSG tmp = c.prepForManufacturing()
		mapOfNameToMFGPart.put(name, tmp)
		bedNames.get(bedName).add(tmp)
	}
}
int bedIndex=0;
HashMap<String,PrintBedObject> pbo = new HashMap<>()
for(String s:bedNames.keySet().stream().sorted()) {
	double bedYLim=bedY*bedIndex+bedIndex
	CSG bedRep=new Cube(bedX,bedY,1).toCSG().toZMax()
			.toXMin()
			.toYMin()
			.movey(bedYLim)
	bedRep.setColor(colors[bedIndex])
	BowlerStudioController.addCsg(bedRep)

	for(CSG c:bedNames.get(s)) {
		PrintBedObject obj =new PrintBedObject(c.getName(),c,bedX,0,bedYLim+bedY,bedYLim)
		pbo.put(s, obj)
	}
	bedIndex++;
}


