import java.lang.reflect.Type

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.reflect.TypeToken
import com.neuronrobotics.bowlerstudio.BowlerStudioController
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

import eu.mihosoft.vrl.v3d.CSG
import javafx.application.Platform
import javafx.scene.transform.Affine

double bedX=260
double bedY=260

class PrintBedObject{

	private String name;
	private CSG part;
	private File source;
	private TransformNR location=new TransformNR()
	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
	private String name2;
	private CSG part2;
	private double xMax;
	private double xMin;
	private double yMax;
	private double yMin;
	private Affine manipulator =new Affine()

	public PrintBedObject(String name, CSG part, double xMax, double xMin, double yMax, double yMin){
		this.part =  part;
		this.name =  name;
		this.xMax =  xMax;
		this.xMin =  xMin;
		this.yMax =  yMax;
		this.yMin =  yMin;
		source=new File(ScriptingEngine.getRepositoryCloneDirectory("https://github.com/OperationSmallKat/Marcos.git").getAbsolutePath()+"/print_bed_location_"+name+".json")
		if(source.exists()) {
			Type TT_mapStringString = new TypeToken<ArrayList<TransformNR>>() {
					}.getType();

			ArrayList<TransformNR> l = gson.fromJson(source.text, TT_mapStringString);
			if(l!=null&& l.size()>0)
				location=l.get(0)
		}else{
			save()
		}
		manipulator=part.getManipulator()
	}
	public void save() {
		update()
		source.text = gson.toJson([location])
	}
	public void update() {
		double minYTest = part.getMinY()-yMin+location.getY()
		double maxYTest = part.getMaxY()-yMax+location.getY()
		double minXTest = part.getMinX()-xMin+location.getX()
		double maxXTest = part.getMaxX()-xMax+location.getX()
		if(minYTest<0)
			location.translateY(-minYTest)
		if(minXTest<0)
			location.translateX(-minXTest)
		if(maxYTest>0)
			location.translateY(-maxYTest)
		if(maxXTest>0)
			location.translateX(-maxXTest)
		Platform.runLater({
			TransformFactory.nrToAffine(location,manipulator)
		})
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
		tmp.setManipulator(new Affine())
		BowlerStudioController.addCsg(tmp)
		bedNames.get(bedName).add(tmp)
	}
}
int bedIndex=0;
HashMap<String,PrintBedObject> pbo = new HashMap<>()
for(String s:bedNames.keySet()) {
	double bedYLim=bedY*bedIndex+bedIndex
	for(CSG c:bedNames.get(s)) {
		PrintBedObject obj =new PrintBedObject(c.getName(),c,bedX,0,bedYLim+bedY,bedYLim)
		pbo.put(s, obj)
	}
	bedIndex++;
}


