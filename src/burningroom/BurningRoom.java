package burningroom;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.auxiliary.StateEnumerator;
import burlap.behavior.singleagent.pomdp.BeliefPolicyAgent;
import burlap.behavior.singleagent.pomdp.wrappedmdpalgs.BeliefSparseSampling;
import burlap.behavior.valuefunction.QValue;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Attribute;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.ObjectClass;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.FullActionModel;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.common.NullAction;
import burlap.oomdp.singleagent.common.SimpleAction;
import burlap.oomdp.singleagent.environment.SimulatedEnvironment;
import burlap.oomdp.singleagent.explorer.TerminalExplorer;
import burlap.oomdp.singleagent.pomdp.BeliefMDPGenerator;
import burlap.oomdp.singleagent.pomdp.ObservationFunction;
import burlap.oomdp.singleagent.pomdp.PODomain;
import burlap.oomdp.singleagent.pomdp.SimulatedPOEnvironment;
import burlap.oomdp.singleagent.pomdp.beliefstate.tabular.HashableTabularBeliefStateFactory;
import burlap.oomdp.singleagent.pomdp.beliefstate.tabular.TabularBeliefState;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

/**
 * @author David Abel.
 */
public class BurningRoom implements DomainGenerator {

	public static final String ATTONFIRE = "isRoomOnFire";
	public static final String ATTROBOTDESTROYED = "isRobotDestroyed";
	public static final String ATTOBJECTDESTROYED = "isObjectDestroyed";
	public static final String ATTOBJECTRETRIEVED = "isObjectRetrieved";
	public static final String ATTISROBOTVALUABLE = "isRobotMoreValuable";
	public static final String ATTROBOTVALUABLEANSWER = "isRobotMoreValuableAnswer"; // Answer.
	
	
	public static final String CLASSSTATE = "mdp";
	public static final String CLASSOBSANSWER = "obsAnswer";

	public static final String ACTIONASK = "askIfRobotIsValuable";
	public static final String ACTIONLONGGRAB = "longGrab";
	public static final String ACTIONSHORTGRAB = "shortGrab";

//	private static double probOfDestroyingObjectAsk = 0.005;
//	private static double probOfDestroyingObjectShortGrab = .05;
//	private static double probOfDestroyingObjectLongGrab = .35;
//	private static double probOfRobotDyingInFire = .5;
	
	private static double probOfDestroyingObjectAsk = 0.005;
	private static double probOfDestroyingObjectShortGrab = .0;
	private static double probOfDestroyingObjectLongGrab = .05;
	private static double probOfRobotDyingInFire = .7;
	
	private static Random random = new Random();

	@Override
	public Domain generateDomain() {

		PODomain domain = new PODomain();

		// Attributes.
		Attribute attFire = new Attribute(domain, ATTONFIRE, Attribute.AttributeType.BOOLEAN);
		Attribute attRobDestr = new Attribute(domain, ATTROBOTDESTROYED, Attribute.AttributeType.BOOLEAN);
		Attribute attIsRobotValuable = new Attribute(domain, ATTISROBOTVALUABLE, Attribute.AttributeType.BOOLEAN);
		Attribute attIsObjectRetrieved = new Attribute(domain, ATTOBJECTRETRIEVED, Attribute.AttributeType.BOOLEAN);
		Attribute attIsObjectDestroyed = new Attribute(domain, ATTOBJECTDESTROYED, Attribute.AttributeType.BOOLEAN);
		Attribute attIsRobotValuableAnswer = new Attribute(domain, ATTROBOTVALUABLEANSWER, Attribute.AttributeType.BOOLEAN);
		
		// Object Classes.
		ObjectClass classState = new ObjectClass(domain, CLASSSTATE);
		classState.addAttribute(attFire);
		classState.addAttribute(attRobDestr);
		classState.addAttribute(attIsObjectRetrieved);
		classState.addAttribute(attIsObjectDestroyed);
		classState.addAttribute(attIsRobotValuable);
		
		// Observation class.
		ObjectClass classObAnser = new ObjectClass(domain, CLASSOBSANSWER);
		classObAnser.addAttribute(attIsRobotValuableAnswer);

		// Actions.
		new LongGrab(ACTIONLONGGRAB, domain);
		new ShortGrab(ACTIONSHORTGRAB, domain);
		new AskAction(ACTIONASK, domain);

		// Observation Function.
		new BurningRoomOF(domain);

		// Set the enumerator (used by planners).
		StateEnumerator senum = new StateEnumerator(domain, new SimpleHashableStateFactory());
		
		// Enumerate room not on fire cases.
		// first bool: isRoboValuable
		// second bool: isRoomOnFire
		senum.findReachableStatesAndEnumerate(this.getMDPState(domain, true, false));
		senum.findReachableStatesAndEnumerate(this.getMDPState(domain, false, false));
		
		// Enumerate room on fire cases (second boolean).
		senum.findReachableStatesAndEnumerate(this.getMDPState(domain, true, true));
		senum.findReachableStatesAndEnumerate(this.getMDPState(domain, false, true));
		domain.setStateEnumerator(senum);

		return domain;
	}

	public static State getMDPState(Domain domain, boolean isRoboValuable, boolean isOnFire){
		State s = new MutableState();
		ObjectInstance o = new MutableObjectInstance(domain.getObjectClass(CLASSSTATE), "state");
		
		o.setValue(ATTISROBOTVALUABLE, isRoboValuable); // Robo value determined by method call.
		o.setValue(ATTONFIRE, isOnFire); // Room on fire determined by method call.
		o.setValue(ATTROBOTDESTROYED, false); // Robot not yet destroyed.
		o.setValue(ATTOBJECTDESTROYED, false); // Object not yet destroyed.
		o.setValue(ATTOBJECTRETRIEVED, false); // Object not yet retrieved.
		
		s.addObject(o);
		
		System.out.println("unset: " + s.getCompleteStateDescriptionWithUnsetAttributesAsNull());
		
		return s;
	}

	public static State getAnswerObservation(Domain domain, boolean obsVal){
		State s = new MutableState(); // An observation.
		ObjectInstance o = new MutableObjectInstance(domain.getObjectClass(CLASSOBSANSWER), ATTROBOTVALUABLEANSWER );
		o.setValue(ATTROBOTVALUABLEANSWER, obsVal);
		s.addObject(o);
		return s;
	}
	
	
	// --- ACTIONS ---
	public static class ShortGrab extends SimpleAction implements FullActionModel {

		public ShortGrab(String name, Domain domain) {
			super(name, domain);
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			ObjectInstance o = s.getFirstObjectOfClass(CLASSSTATE);
			boolean isRoomOnFire = o.getBooleanValForAttribute(ATTONFIRE);
			
			if(isRoomOnFire) {
				// Some chance of robot dying.
				if(random.nextDouble() < probOfRobotDyingInFire) {
					o.setValue(ATTROBOTDESTROYED, true);
				}
				else {
					o.setValue(ATTROBOTDESTROYED, false);
				}
			}
			
			// Destroy the object with some low probability.
			if(random.nextDouble() < probOfDestroyingObjectShortGrab) {
				o.setValue(ATTOBJECTDESTROYED, true);
			}
			else {
				// Retrieved object.
				o.setValue(ATTOBJECTRETRIEVED, true);
			}

			return s;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {
			ObjectInstance o = s.getFirstObjectOfClass(CLASSSTATE);
			boolean isRoomOnFire = o.getBooleanValForAttribute(ATTONFIRE);
			
			// Make robot safe, obj destroy state.
			State roboSafeObjDestState = s.copy();
			ObjectInstance roboSafeObjDest = roboSafeObjDestState.getFirstObjectOfClass(CLASSSTATE);
			roboSafeObjDest.setValue(ATTROBOTDESTROYED, false);
			roboSafeObjDest.setValue(ATTOBJECTDESTROYED, true);
			roboSafeObjDest.setValue(ATTOBJECTRETRIEVED, false);
			
			
			// Make robot safe, obj retrieved state.
			State roboSafeObjSafeState = s.copy();
			ObjectInstance roboSafeObjSafe = roboSafeObjSafeState.getFirstObjectOfClass(CLASSSTATE);
			roboSafeObjSafe.setValue(ATTROBOTDESTROYED, false);
			roboSafeObjSafe.setValue(ATTOBJECTDESTROYED, false);
			roboSafeObjSafe.setValue(ATTOBJECTRETRIEVED, true);
			
			// List of transition probabilities to return.
			List<TransitionProbability> ltp = new ArrayList<TransitionProbability>();
			
			double probOfRoboSafeObjDest = probOfDestroyingObjectShortGrab;
			double probOfRobotSafeObjSafe = (1 - probOfDestroyingObjectShortGrab);
			
			// If the room is on fire, need to consider probability that the robot dies.
			if(isRoomOnFire) {
				// Make robot destroyed state, obj destroyed state.
				State roboDestObjDestState = s.copy();
				ObjectInstance roboDestObjDest = roboDestObjDestState.getFirstObjectOfClass(CLASSSTATE);
				roboDestObjDest.setValue(ATTROBOTDESTROYED, true);
				roboDestObjDest.setValue(ATTOBJECTDESTROYED, true);
				roboDestObjDest.setValue(ATTOBJECTRETRIEVED, false);
				
				// Make robot destroyed state, obj safe state.
				State roboDestObjSafeState = s.copy();
				ObjectInstance roboDestObjSafe = roboDestObjSafeState.getFirstObjectOfClass(CLASSSTATE);
				roboDestObjSafe.setValue(ATTROBOTDESTROYED, true);
				roboDestObjSafe.setValue(ATTOBJECTDESTROYED, false);
				roboDestObjSafe.setValue(ATTOBJECTRETRIEVED, true);

				TransitionProbability roboDestObjDestTP = new TransitionProbability(roboDestObjDestState, probOfDestroyingObjectShortGrab*probOfRobotDyingInFire);
				TransitionProbability roboDestObjSafeTP = new TransitionProbability(roboDestObjSafeState, (1 - probOfDestroyingObjectShortGrab)*probOfRobotDyingInFire);
				
				ltp.add(roboDestObjDestTP);
				ltp.add(roboDestObjSafeTP);
				
				// Adjust probabilities of transitioning to robot Safe if the room is on fire.
				probOfRoboSafeObjDest = probOfDestroyingObjectShortGrab*(1 - probOfRobotDyingInFire);
				probOfRobotSafeObjSafe = (1 - probOfDestroyingObjectShortGrab)*(1 - probOfRobotDyingInFire);
			}
			
			// Create TPs and add to list.
			TransitionProbability roboSafeObjDestTP = new TransitionProbability(roboSafeObjDestState, probOfRoboSafeObjDest);
			TransitionProbability robotSafeObjSafeTP = new TransitionProbability(roboSafeObjSafeState, probOfRobotSafeObjSafe);
			
			ltp.add(roboSafeObjDestTP);
			ltp.add(robotSafeObjSafeTP);
			
			return ltp;
		}
	}

	public static class AskAction extends SimpleAction implements FullActionModel {
	
		public AskAction(String name, Domain domain) {
			super(name, domain);
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {
			
			State destState = s.copy();
			ObjectInstance o = destState.getFirstObjectOfClass(CLASSSTATE);
			o.setValue(ATTOBJECTDESTROYED, true);
			
			List<TransitionProbability> ltp = new ArrayList<TransitionProbability>();
			TransitionProbability safeTP = new TransitionProbability(s, 1-probOfDestroyingObjectAsk);
			TransitionProbability destTP = new TransitionProbability(destState, probOfDestroyingObjectAsk);
			
			ltp.add(safeTP);
			ltp.add(destTP);

			return ltp;
			
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {
			
			ObjectInstance o = s.getFirstObjectOfClass(CLASSSTATE);
			
			// Destroy the object with some low probability.
			if(random.nextDouble() < probOfDestroyingObjectAsk) {
				o.setValue(ATTOBJECTDESTROYED, true);
			}
			
			return s;
		}
		
	}
	
	public static class LongGrab extends SimpleAction implements FullActionModel {

		public LongGrab(String name, Domain domain) {
			super(name, domain);
		}

		@Override
		protected State performActionHelper(State s, GroundedAction groundedAction) {

			ObjectInstance o = s.getFirstObjectOfClass(CLASSSTATE);
			
			// Destroy the object with some high probability.
			if(random.nextDouble() < probOfDestroyingObjectLongGrab) {
				o.setValue(ATTOBJECTDESTROYED, true);
			}
			else {
				o.setValue(ATTOBJECTRETRIEVED, true);
			}

			return s;
		}

		@Override
		public List<TransitionProbability> getTransitions(State s, GroundedAction groundedAction) {
			
			// Make destroy state.
			State destroyState = s.copy();
			ObjectInstance destroy = destroyState.getFirstObjectOfClass(CLASSSTATE);
			destroy.setValue(ATTOBJECTDESTROYED, true);
			destroy.setValue(ATTOBJECTRETRIEVED, false);
			TransitionProbability destroyTP = new TransitionProbability(destroyState, probOfDestroyingObjectLongGrab);
			
			// Make safe state.
			State safeState = s.copy();
			ObjectInstance safe = safeState.getFirstObjectOfClass(CLASSSTATE);
			safe.setValue(ATTOBJECTDESTROYED, false);
			safe.setValue(ATTOBJECTRETRIEVED, true);
			TransitionProbability safeTP = new TransitionProbability(safeState, 1 - probOfDestroyingObjectLongGrab);
			
			// Create TPs and add to list.
			List<TransitionProbability> ltp = new ArrayList<TransitionProbability>();
			ltp.add(destroyTP);
			ltp.add(safeTP);
			
			return ltp;
		}
	}

	// --- OBSERVATION FUNCTION ---
	public static class BurningRoomOF extends ObservationFunction{

		public BurningRoomOF(PODomain domain) {
			super(domain);
		}

		@Override
		public boolean canEnumerateObservations() {
			return true;
		}

		@Override
		public List<State> getAllPossibleObservations() {
			List<State> obs = Arrays.asList(new MutableState(), BurningRoom.getAnswerObservation(this.domain, true), BurningRoom.getAnswerObservation(this.domain, false));
			return obs;
		}

		@Override
		public double getObservationProbability(State observation, State state, GroundedAction action) {
			
			// If the agent asked a question.
			if(action.actionName().equals(ACTIONASK)) {
				
				if(observation.numTotalObjects() == 0){
					return 0.;
				}
				
				// Check if this observation agrees with the state.
				boolean answerOb = observation.getFirstObjectOfClass(CLASSOBSANSWER).getBooleanValForAttribute(ATTROBOTVALUABLEANSWER);
				boolean trueUtil = state.getFirstObjectOfClass(CLASSSTATE).getBooleanValForAttribute(ATTISROBOTVALUABLE);
				if(answerOb == trueUtil){
					return 1.;
				}
				return 0.;
			}
			// If we're taking another action and we get a real observation, it has Pr = 0.
			if(observation.numTotalObjects() > 0){
				return 0.;
			}
			// Null observation has Pr = 1 for all actions != ACTIONASK.
			return 1.;
			
		}

		@Override
		public List<ObservationProbability> getObservationProbabilities(State state, GroundedAction action) {

			if(action.actionName().equals(ACTIONLONGGRAB) || action.actionName().equals(ACTIONSHORTGRAB)
					|| state.getFirstObjectOfClass(CLASSSTATE).getBooleanValForAttribute(ATTROBOTDESTROYED)
					|| state.getFirstObjectOfClass(CLASSSTATE).getBooleanValForAttribute(ATTOBJECTRETRIEVED)){
				return Arrays.asList(new ObservationProbability(new MutableState(), 1.));
			}

			// Otherwise they asked a question.
			boolean obsVal = state.getFirstObjectOfClass(CLASSSTATE).getBooleanValForAttribute(ATTISROBOTVALUABLE);
			return Arrays.asList(new ObservationProbability(BurningRoom.getAnswerObservation(domain, obsVal), 1.));
		}

		@Override
		public State sampleObservation(State state, GroundedAction action) {
			return this.sampleObservationByEnumeration(state, action);
		}
	}

	public static class BurningRoomRF implements RewardFunction{

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			
			// Pseudo-terminal state (return 0 reward indefinitely).
			ObjectInstance prevStateO = s.getFirstObjectOfClass(CLASSSTATE);
			boolean prevObjRetrieved = prevStateO.getBooleanValForAttribute(ATTOBJECTRETRIEVED);
			boolean prevObjDestroyed = prevStateO.getBooleanValForAttribute(ATTOBJECTDESTROYED);
			boolean terminal = prevObjRetrieved || prevObjDestroyed;
			if (terminal) {
				return 0;
			}
			
			// Get state.
			ObjectInstance o = sprime.getFirstObjectOfClass(CLASSSTATE);
			
			boolean roboDestroyed = o.getBooleanValForAttribute(ATTROBOTDESTROYED);
			boolean objDestroyed = o.getBooleanValForAttribute(ATTOBJECTDESTROYED);
			boolean roboIsValuable = o.getBooleanValForAttribute(ATTISROBOTVALUABLE);
			boolean objSafe = o.getBooleanValForAttribute(ATTOBJECTRETRIEVED);
			
			// Compute reward.
			double total = 0.;
			if (objDestroyed) {
				total -= 10.;
			}
			
			if (roboDestroyed) {
				total -= 5.;
				if (roboIsValuable) {
					total -= 25.;
				}
			}
			
			if (objSafe && a.actionName().equals(ACTIONSHORTGRAB)) {
				total += 10;
			} 
			else if (objSafe && a.actionName().equals(ACTIONLONGGRAB)) {
				total += 6;
			}
			
			return total;
		}
	}

	public static class BurningRoomTF implements TerminalFunction{

		@Override
		public boolean isTerminal(State s) {
			ObjectInstance o = s.getFirstObjectOfClass(CLASSSTATE);
			boolean objRetrieved = o.getBooleanValForAttribute(ATTOBJECTRETRIEVED);
			boolean objDestroyed = o.getBooleanValForAttribute(ATTOBJECTDESTROYED);
			boolean terminal = objRetrieved || objDestroyed;
			
			if(terminal){
				return true;
			}
			return false;
		}
	}


	public static void main(String[] args) {

		BurningRoom cd = new BurningRoom();
		PODomain domain = (PODomain)cd.generateDomain();
		RewardFunction rf = new BurningRoomRF();
		TerminalFunction tf = new BurningRoomTF();
		
		for(int i = 0; i < domain.getStateEnumerator().numStatesEnumerated(); i++) {
			System.out.println("state " + i + "," + domain.getStateEnumerator().getStateForEnumerationId(i).getCompleteStateDescription());
		}

		// Room not on fire, robot's life matters.
		boolean isRoboValuable = false;
		boolean isRoomOnFire = true;
		State initialMDPState = BurningRoom.getMDPState(domain, isRoboValuable, isRoomOnFire);
		
		// Environment.
		SimulatedEnvironment env = new SimulatedEnvironment(domain, rf, tf, initialMDPState);
		SimulatedPOEnvironment penv = new SimulatedPOEnvironment((PODomain)domain, rf, tf, initialMDPState);

//		TerminalExplorer exp = new TerminalExplorer(domain, env);
//		exp.explore();
		
		// Robot believes it's life matters and doesn't matter with 0.5 probability.
		TabularBeliefState bs = new TabularBeliefState(domain);
		bs.setBelief(BurningRoom.getMDPState(domain, isRoboValuable, isRoomOnFire), 0.5);
		bs.setBelief(BurningRoom.getMDPState(domain, !isRoboValuable, isRoomOnFire), 0.5);
		
		System.out.println(Arrays.toString(bs.getBeliefVector()));

		// Belief MDP.
		BeliefMDPGenerator mdpGen = new BeliefMDPGenerator(domain);
		Domain bdomain = mdpGen.generateDomain();
		RewardFunction brf = new BeliefMDPGenerator.BeliefRF(domain, rf);
		SimulatedEnvironment benv = new SimulatedEnvironment(bdomain, brf, new NullTermination(), bs);

//		TerminalExplorer bexp = new TerminalExplorer(bdomain, benv);
//		bexp.explore();
		
		// Use SparseSampling to solve the POMDP.
		double gamma = 1.0;
		BeliefSparseSampling bss = new BeliefSparseSampling(domain, rf, gamma, new HashableTabularBeliefStateFactory(), 2, -1);
		bss.toggleDebugPrinting(false);
		bss.getSparseSamplingPlanner().toggleDebugPrinting(false);
		GreedyQPolicy p = new GreedyQPolicy(bss);
		
		// Agent.
		BeliefPolicyAgent agent = new BeliefPolicyAgent(domain, penv, p);
		agent.setBeliefState(bs);
		
		EpisodeAnalysis ea = agent.actUntilTerminal();
		System.out.println(ea.getActionSequenceString("\n"));

		List<QValue> qs = bss.getQs(bs);
		for(QValue q : qs){
			System.out.println(q.q + ": " + q.a.toString());
		}

	}

}
