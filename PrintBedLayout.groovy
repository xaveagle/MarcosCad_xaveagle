import com.neuronrobotics.bowlerstudio.BowlerStudioController
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.creature.MobileBaseCadManager
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager

import eu.mihosoft.vrl.v3d.CSG

double bedX=260
double bedY=260

MobileBase marcos=DeviceManager.getSpecificDevice( "Marcos",{
	return ScriptingEngine.gitScriptRun(	"https://github.com/OperationSmallKat/Marcos.git","Marcos.xml",null)
})

IgenerateBed bedGen = (IgenerateBed)MobileBaseCadManager.get(marcos).getIgenerateBed()
ArrayList<CSG> cache = bedGen.cache
if(cache==null)
	throw new RuntimeException("This model has not bed generator!")
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
		BowlerStudioController.addCsg(tmp)
		bedNames.get(bedName).add(tmp)
	}
}

