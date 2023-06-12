import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.PrepForManufacturing
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.transform.Affine
import eu.mihosoft.vrl.v3d.ChamferedCylinder



File parametricsCSV = ScriptingEngine.fileFromGit("https://github.com/OperationSmallKat/Marcos.git", "parametrics.csv")
HashMap<String,Double> numbers;
BufferedReader reader;
String code="HashMap<String,Double> numbers = new HashMap<>()\n"
String vars=""
String equs=""
try {
	reader = new BufferedReader(new FileReader(parametricsCSV.getAbsolutePath()));
	String line = reader.readLine();
	while ((line = reader.readLine() )!= null) {
		if(line.length()>3) {
			//System.out.println(line);
			String[] parts = line.split(",");
			String value=(parts[2].replaceAll(parts[1], "")).trim()
			String reconstructed =parts[0]+"="+value;
			try {
				Double.parseDouble(value)
				vars+= reconstructed+"\n"
				vars+="numbers.put(\""+parts[0]+"\","+parts[0]+");\n"
			}catch(NumberFormatException ex) {
				equs+= reconstructed+"\n"
				equs+="numbers.put(\""+parts[0]+"\","+parts[0]+");\n"
			}
		}
	}
	reader.close();
} catch (IOException e) {
	e.printStackTrace();
}
code+=vars;
code+=equs;
code+="return numbers"
//println code
numbers=(HashMap<String,Double>) ScriptingEngine.inlineScriptStringRun(code, null, "Groovy");
for(String key :numbers.keySet()) {
	println key+" : "+numbers.get(key)
}


return new ChamferedCylinder(numbers.ServoHornDiameter,numbers.ServoHornHeight,0.5).toCSG()


return new ICadGenerator(){
			CSG moveDHValues(CSG incoming,DHParameterKinematics d, int linkIndex ){
				TransformNR step = new TransformNR(d.getChain().getLinks().get(linkIndex).DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			
			
			
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
				// read motor typ information out of the link configuration
				LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
				// load the vitamin for the servo
				CSG motor = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
				// Is this value actually something in the CSV?
				double distanceToMotorTop = motor.getMaxZ();
				
				// a list of CSG objects to be rendered
				ArrayList<CSG> back =[]
				// get the UI manipulator for the link
				Affine dGetLinkObjectManipulator = d.getLinkObjectManipulator(linkIndex)
				// UI manipulator for the root of the limb
				Affine root = d.getRootListener()
				if(linkIndex==0) {
					// the first link motor is located in the body
					motor.setManipulator(root)
					// pull the limb servos out the top
					motor.addAssemblyStep(1, new Transform().movex(-100))
				}else {
					// the rest of the motors are located in the preior link's kinematic frame 
					motor.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
					// pull the link motors out the thin side
					motor.addAssemblyStep(1, new Transform().movey(-100))
				}
				// do not export the motors to STL for manufacturing
				motor.setManufacturing({return null})
				
				if(linkIndex==2) {
					// this section is a place holder to visualize the tip of the limb
					CSG foot = new Sphere(10).toCSG()
					foot.setManipulator(dGetLinkObjectManipulator)
					back.add(foot)
				}
				back.add(motor)
				return back;
			}

			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]
				back.add(new Cube(1).toCSG())
				for(CSG c:back)
					c.setManipulator(arg0.getRootListener())
				for(DHParameterKinematics kin:arg0.getAllDHChains()) {
					CSG limbRoot =new Cube(1).toCSG()
					limbRoot.setManipulator(kin.getRootListener())
					back.add(limbRoot)
				}
				return back;
			}


		}
