import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.ILinkListener
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.pid.PIDLimitEvent

class GearWrist implements ILinkListener{
	String name
	MobileBase marcos
	DHParameterKinematics source;
	AbstractLink leftTarget;
	AbstractLink rightTarget;
	boolean front

	AbstractLink zero;
	AbstractLink one ;
	public GearWrist(String n){
		name=n
		marcos=DeviceManager.getSpecificDevice( "Marcos",{
			return ScriptingEngine.gitScriptRun(	"https://github.com/OperationSmallKat/Marcos.git","Marcos.xml",null)
		})
		front=name.contentEquals("Head")
		for(DHParameterKinematics d:marcos.getAppendages()) {
			String dName=d.getScriptingName();
			if(dName.contentEquals(name)) {
				source=d;
			}
			if(dName.contains("Dummy")) {
				boolean thisLimbFront =dName.contains("Front")
				if(thisLimbFront==front) {
					boolean left=dName.contains("Left")
					AbstractLink ctrl = d.getAbstractLink(0)
					if(left) {
						leftTarget=ctrl
					}else
						rightTarget=ctrl
				}
			}
		}
		if(source==null||leftTarget==null||rightTarget==null)
			throw new RuntimeException("Targets for gear wrist missing! s:"+source+" l:"+leftTarget+" r:"+rightTarget)
	}
	boolean connect() {
		println "Connecting"
		zero=source.getAbstractLink(0)
		one =source.getAbstractLink(1)
		zero.addLinkListener(this)
		one.addLinkListener(this)
		onLinkPositionUpdate(zero,zero.getCurrentEngineeringUnits())
		leftTarget.setUseLimits(false)
		rightTarget.setUseLimits(false)
	}
	void disconnect() {
		println "Disconnecting"
		zero.removeLinkListener(this)
		one.removeLinkListener(this)
	}
	@Override
	public void onLinkPositionUpdate(AbstractLink s, double engineeringUnitsValue) {
		double l1=zero.getCurrentEngineeringUnits();
		double l2=one.getCurrentEngineeringUnits();
		if(!front)
			l2=-l2
//		if(s==zero) {
//			double l2Max = leftTarget.getMaxEngineeringUnits()
//			double r2Max = rightTarget.getMaxEngineeringUnits()
//			double l2Min = leftTarget.getMinEngineeringUnits()
//			double r2Min = rightTarget.getMinEngineeringUnits()
//			if(!front) {
//				l2Max=-l2Max
//				r2Max=-r2Max
//				l2Min=-l2Min
//				r2Min=-r2Min
//			}
//			double newl1max =  l1-l2Max
//			double newl1min =  l1-l2Min
//			double newr1max =  l1-r2Max
//			double newr1min =  l1-r2Min
//
//			double highestMax = newl1max
//			if(newr1max<highestMax)
//				highestMax=newr1max
//			double lowestMin = newl1min
//			if(newr1min>lowestMin)
//				lowestMin=newr1min
//			//if(one.getMaxEngineeringUnits()>highestMax) {
//				println "Set Upper Limit "+highestMax
//				//one.setMaxEngineeringUnits(highestMax)
//			//}
//			//if(one.getMinEngineeringUnits()<lowestMin) {
//				println "Set Lower Limit "+lowestMin
//				//one.setMinEngineeringUnits(lowestMin)
//			//}
//		}
		//println "Wrist targets "+l1+":"+l2
		def l1l2 = l1-l2
		def l22 = -l1-l2
		if(leftTarget.getMaxEngineeringUnits()>l1l2&& leftTarget.getDeviceMinEngineeringUnits()<l1l2)
			leftTarget.setTargetEngineeringUnits(l1l2)
		if(rightTarget.getMaxEngineeringUnits()>l22&& rightTarget.getDeviceMinEngineeringUnits()<l22)
			rightTarget.setTargetEngineeringUnits(l22)
	}
	@Override
	public void onLinkLimit(AbstractLink source, PIDLimitEvent event) {
		// TODO Auto-generated method stub

	}
}

DeviceManager.getSpecificDevice("Head", {
	def w = new GearWrist("Head");
	w.connect()
	return w;
})

DeviceManager.getSpecificDevice("Tail", {
	def w = new GearWrist("Tail");
	w.connect()
	return w;
})
return null
