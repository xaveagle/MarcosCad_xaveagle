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
		AbstractLink zero=source.getAbstractLink(0)
		zero.addLinkListener(this)
		source.getAbstractLink(1).addLinkListener(this)
		onLinkPositionUpdate(zero,zero.getCurrentEngineeringUnits())
		leftTarget.setUseLimits(false)
		rightTarget.setUseLimits(false)
		
	}
	void disconnect() {
		println "Disconnecting"
		source.getAbstractLink(0).removeLinkListener(this)
		source.getAbstractLink(1).removeLinkListener(this)
	}
	@Override
	public void onLinkPositionUpdate(AbstractLink s, double engineeringUnitsValue) {
		double l1=source.getAbstractLink(0).getCurrentEngineeringUnits();
		double l2=source.getAbstractLink(1).getCurrentEngineeringUnits();
		if(!front)
			l2=-l2
		//println "Wrist targets "+l1+":"+l2
		leftTarget.setTargetEngineeringUnits(l1-l2)
		rightTarget.setTargetEngineeringUnits(-l1-l2)
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
