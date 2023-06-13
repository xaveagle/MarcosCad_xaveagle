import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.creature.IgenerateBed
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.PrepForManufacturing
import eu.mihosoft.vrl.v3d.Sphere
import eu.mihosoft.vrl.v3d.Transform
import eu.mihosoft.vrl.v3d.parametrics.LengthParameter
import javafx.scene.paint.Color
import javafx.scene.transform.Affine
import eu.mihosoft.vrl.v3d.ChamferedCylinder

CSG ChamferedCylinder(double r, double h, double chamferHeight) {
	CSG cube1 = new Cylinder(r - chamferHeight,r - chamferHeight, h,40).toCSG();
	CSG cube2 = new Cylinder(r,r, h - chamferHeight * 2,40).toCSG().movez(chamferHeight);
	return cube1.union(cube2).hull()
}
double computeGearPitch(double diameterAtCrown,double numberOfTeeth){
	return ((diameterAtCrown/2)*((360.0)/numberOfTeeth)*Math.PI/180)
}


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
//for(String key :numbers.keySet()) {
//	println key+" : "+numbers.get(key)
//}

// Begin creating the resin print horn piece withn a chamfered cylendar
CSG core=  ChamferedCylinder(numbers.ServoHornDiameter/2.0,numbers.ServoHornHeight,numbers.Chamfer2)
//calculate the depth of the screw head based on the given measurments
double cutoutDepth = numbers.ServoHornHeight-numbers.ServoMountingScrewSpace - numbers.ServoHornSplineHeight
// the cutout for the head of the screw on the resin horn
CSG screwHeadCutOut = new Cylinder(numbers.ServoHornScrewHeadDiamter/2.0,numbers.ServoHornScrewHeadDiamter/2.0, cutoutDepth,30).toCSG()
		.toZMax()
		.movez(numbers.ServoHornHeight)
// cutout for the hole the shaft of the mount screw passes through
CSG screwHoleCutOut = new Cylinder(numbers.ServoHornScrewDiamter/2.0,numbers.ServoHornScrewDiamter/2.0, numbers.ServoMountingScrewSpace,30).toCSG()
		.toZMax()
		.movez(numbers.ServoHornHeight-cutoutDepth)
// Cut the holes from the core
CSG cutcore=core.difference([
	screwHeadCutOut,
	screwHoleCutOut
])
// use the gear maker to generate the spline
def gears = ScriptingEngine.gitScriptRun(
		"https://github.com/madhephaestus/GearGenerator.git", // git location of the library
		"bevelGear.groovy" , // file to load
		// Parameters passed to the funcetion
		[
			numbers.ServoHornNumberofTeeth,
			// Number of teeth gear a
			numbers.ServoHornNumberofTeeth,
			// Number of teeth gear b
			numbers.ServoHornSplineHeight,
			// thickness of gear A
			numbers.ServoHornToothBaseWidth,
			// gear pitch in arc length mm
			0,
			// shaft angle, can be from 0 to 100 degrees
			0// helical angle, only used for 0 degree bevels
		]
		)
// get just the pinion of the set
CSG spline = gears.get(0)
// cut the spline from the core
CSG resinPrintServoMount=cutcore.difference(spline)

class cadGenMarcos implements ICadGenerator,IgenerateBed{
	CSG resinPrintServoMount
	HashMap<String,Double> numbers
	LengthParameter tailLength		= new LengthParameter("Cable Cut Out Length",30,[500, 0.01])

	public cadGenMarcos(CSG res,HashMap<String,Double> n) {
		resinPrintServoMount=res
		numbers=n
	}
	ArrayList<CSG> cache = new ArrayList<CSG>()
	CSG moveDHValues(CSG incoming,DHParameterKinematics d, int linkIndex ){
		TransformNR step = new TransformNR(d.getChain().getLinks().get(linkIndex).DhStep(0)).inverse()
		Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}
	CSG reverseDHValues(CSG incoming,DHParameterKinematics d, int linkIndex ){
		TransformNR step = new TransformNR(d.getChain().getLinks().get(linkIndex).DhStep(0))
		Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}
	/**
	 * This function should generate the bed or beds or parts to be used in manufacturing If parts are
	 * to be ganged up to make print beds then this should happen here
	 *
	 * @param base the base to generate
	 * @return simulatable CAD objects
	 */
	public ArrayList<CSG> arrangeBed(MobileBase base){
		println "Generating Marcos Print Bed"
		ArrayList<CSG> resin = []
		ArrayList<CSG> one = []
		ArrayList<CSG> two = []
		ArrayList<CSG> three = []
		for(CSG bit :cache) {
			def bitGetStorageGetValue = bit.getStorage().getValue("bedType")
			if(bitGetStorageGetValue.present) {
				if(bitGetStorageGetValue.get().toString().contentEquals("resin")) {
					resin.add(bit)
				}
				if(bitGetStorageGetValue.get().toString().contentEquals("ff-One")) {
					one.add(bit)
				}
				if(bitGetStorageGetValue.get().toString().contentEquals("ff-Two")) {
					two.add(bit)
				}
				if(bitGetStorageGetValue.get().toString().contentEquals("ff-Three")) {
					three.add(bit)
				}
			}
		}

		CSG bedThree=null
		for(CSG p:one) {
			p=p.prepForManufacturing()
			if(bedThree==null)
				bedThree=p
			else {
				bedThree=bedThree.dumbUnion(p)
			}
		}
		bedThree.setName("FF-Bed-Three")
		CSG bedTwo=null
		for(CSG p:one) {
			p=p.prepForManufacturing()
			if(bedTwo==null)
				bedTwo=p
			else {
				bedTwo=bedTwo.dumbUnion(p)
			}
		}
		bedTwo.setName("FF-Bed-Two")		
		CSG bedOne=null
		for(CSG p:one) {
			p=p.prepForManufacturing()
			if(bedOne==null)
				bedOne=p
			else {
				bedOne=bedOne.dumbUnion(p)
			}
		}
		bedOne.setName("FF-Bed-One")
		CSG resinBed=null
		for(int i=0;i<4;i++) {
			for (int j=0;j<4;j++) {
				double x = i*(numbers.ServoHornDiameter)
				double y = j*(numbers.ServoHornDiameter+1.0)
				try {
					int index = i*4+(j)
					if(index<resin.size()) {
						println "Adding resin horn to resin bed "+index
						CSG part =resin[index].prepForManufacturing()
						CSG moved = part.movex(x).movey(y)
						if(resinBed==null)
							resinBed=moved
						else
							resinBed=resinBed.dumbUnion(moved)
					}
				}catch(Exception ex) {
					ex.printStackTrace()
				}
			}
		}

		resinBed.setName("Print-Bed-Resin-Printer")
		resinBed.setColor(Color.GREY)

		return [resinBed, bedOne,bedTwo,bedThree]

	}

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {

		// chaeck to see if this is the left side
		boolean left=false;
		boolean front=false;
		if(d.getRobotToFiducialTransform().getY()>0) {
			left=true;
		}
		if(d.getRobotToFiducialTransform().getX()>0) {
			front=true;
		}
		TransformNR dGetRobotToFiducialTransform = d.getRobotToFiducialTransform()
		dGetRobotToFiducialTransform.setY(numbers.BodyServoCenterWidth/2.0*(left?1.0:-1.0))
		dGetRobotToFiducialTransform.setX(numbers.BodyServoCenterLength/2.0*(front?1.0:-1.0))

		d.setRobotToFiducialTransform(dGetRobotToFiducialTransform)
		// read motor typ information out of the link configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// load the vitamin for the servo
		tailLength.setMM(0.1);
		CSG motor = Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
				.toZMax()
				.movez(numbers.JointSpacing/2.0+numbers.LooseTolerance)
		// Is this value actually something in the CSV?
		double distanceToMotorTop = motor.getMaxZ();
		println "Center to horn distance "+distanceToMotorTop
		// a list of CSG objects to be rendered
		ArrayList<CSG> back =[]
		// get the UI manipulator for the link
		Affine dGetLinkObjectManipulator = d.getLinkObjectManipulator(linkIndex)
		// UI manipulator for the root of the limb
		Affine root = d.getRootListener()


		if(linkIndex==0) {
			motor=motor.rotz(left?180:0)
			motor=motor.roty(front?180:0)
			// the first link motor is located in the body
			motor.setManipulator(root)
			// pull the limb servos out the top
			motor.addAssemblyStep(1, new Transform().movex(-100))
		}else {
			motor=motor.roty(left?180:0)
			motor=motor.rotz(90)
			// the rest of the motors are located in the preior link's kinematic frame
			motor.setManipulator(d.getLinkObjectManipulator(linkIndex-1))
			// pull the link motors out the thin side
			motor.addAssemblyStep(1, new Transform().movey(-100))
		}
		// do not export the motors to STL for manufacturing
		motor.setManufacturing({return null})
		motor.setColor(Color.BLUE)
		//Start the horn link
		// move the horn from tip of the link space, to the Motor of the last link space
		// note the hore is moved to the centerline distance value before the transform to link space
		CSG movedHorn = resinPrintServoMount.movez(distanceToMotorTop)
		if(linkIndex==0)
			movedHorn=movedHorn.roty(front?180:0)
		else
			movedHorn=movedHorn.roty(left?180:0)
		CSG myServoHorn = moveDHValues(movedHorn,d,linkIndex)
		if(linkIndex==0)
			myServoHorn.addAssemblyStep(2, new Transform().movey(-10))
		else
			myServoHorn.addAssemblyStep(2, new Transform().movez(10))
		//reorent the horn for resin printing
		myServoHorn.setManufacturing({incoming ->
			return reverseDHValues(incoming, d, linkIndex).toZMin()
					.roty(45)
					.toZMin()
					.movez(5)
		})
		myServoHorn.getStorage().set("bedType", "resin")
		myServoHorn.setName("Resin Horn "+linkIndex+" "+d.getScriptingName())
		// attach this links manipulator
		myServoHorn.setManipulator(dGetLinkObjectManipulator)
		back.add(myServoHorn)
		//end horn link
		if(linkIndex==2) {
			// this section is a place holder to visualize the tip of the limb
			CSG foot = new Sphere(10).toCSG()
			foot.setManipulator(dGetLinkObjectManipulator)
			back.add(foot)
		}
		back.add(motor)
		cache.addAll(back)
		return back;
	}

	@Override
	public ArrayList<CSG> generateBody(MobileBase arg0) {
		cache.clear()
		DHParameterKinematics d = arg0.getLegs().get(0);

		TransformNR dGetRobotToFiducialTransform = d.getRobotToFiducialTransform()
		double zCenterLine = dGetRobotToFiducialTransform.getZ()+numbers.ServoThickness/2.0

		CSG body  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"Body.stl")).movez(zCenterLine);
		CSG bodyCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyCover.stl")).movez(zCenterLine);
		CSG topCOver  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyServoCoverTop.stl")).movez(zCenterLine);
		CSG BottomCover  = Vitamins.get(ScriptingEngine.fileFromGit(
				"https://github.com/OperationSmallKat/Marcos.git",
				"BodyCoverBottom.stl")).movez(zCenterLine);
		
			
		
		bodyCOver.setName("BodyCover")
		body.setName("Body")
		body.setManufacturing({ incoming ->
			return incoming.rotx(180).toZMin().toXMin().toYMin()
		})
		bodyCOver.setManufacturing({ incoming ->
			return incoming.toZMin().toXMin().toYMin().movey(body.getTotalY()+1)
		})

		ArrayList<CSG> back =[body, bodyCOver,topCOver,BottomCover]
		for(CSG c:back) {
			c.setManipulator(arg0.getRootListener())
			c.getStorage().set("bedType", "ff-One")
		}
		for(DHParameterKinematics kin:arg0.getAllDHChains()) {
			CSG limbRoot =new Cube(1).toCSG()
			limbRoot.setManipulator(kin.getRootListener())
			back.add(limbRoot)
		}
		cache.addAll(back)
		return back;
	}


}
return new cadGenMarcos(resinPrintServoMount,numbers)


