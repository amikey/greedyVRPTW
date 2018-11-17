package algorithm;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import javax.swing.JFrame;

import viz.VrpPanel;
import util.VrpProblem;
import util.VrpReader;
import util.VrpSolution;

public class GreedyInitializer {
	private static final double TIME_DIFF_WEIGHT = .4;
	private static final double DISTANCE_WEIGHT = .4;
	private static final double URGENCY_WEIGHT = .2;

	private double timeDiffWeight = TIME_DIFF_WEIGHT;
	private double distanceWeight = DISTANCE_WEIGHT;
	private double urgencyWeight = URGENCY_WEIGHT;

	public GreedyInitializer(double timeDiffW, double distanceW, double urgencyW) {
		this.timeDiffWeight = timeDiffW;
		this.distanceWeight = distanceW;
		this.urgencyWeight = urgencyW;
	}

	/**
	 * ���������ǰ�ڵ��븽����������֮�У�����ʱ�䴰��Լ������õ� ͨ���Ѹ���Լ���ĺķ�ʱ��ͳ�ƣ���Ȩ��͵�ֵ��ѡȡ��С���Ǹ��㣬��ʹĿǰ���е�
	 * 
	 * @param curLastId
	 *            ��ǰ���ʵĽڵ㣬����ǲֿ��Ϊ-1
	 * @param curLastVistTime
	 *            ��ǰ��������������ʼʱ��
	 * @param remCapacity
	 *            ʣ������
	 * @param remainingNodes
	 *            ʣ�໹û���ʵĽڵ㼯��
	 * @param problem
	 *            ����
	 * @return ���س�����{��õ���һ�����ŵ��ID���Ǹ�����������ʱ��}
	 */
	private Number[] findClosest(int curLastId, double curLastVistTime, int remCapacity, Set<Integer> remainingNodes,
			VrpProblem problem) {

		int[] demands = problem.getDemands();
		int[] windowStartTimes = problem.getWindowStartTimes();
		int[] windowEndTimes = problem.getWindowEndTimes();
		double[][] distances = problem.getDistances();
		double[] distancesFromDepot = problem.getDistancesFromDepot();
		int[] serviceTimes = problem.getServiceTimes();
		int curLastServiceTime = (curLastId == -1) ? 0 : serviceTimes[curLastId];

		//������е�ʱ����ѭ�����Լ���Ѿ����ͣ��޷�����������µ��ˣ���ʱ���ʼ�����bestNodeId = -1��bestNodeVistitTime = -1���䣬Ϊ���ؽ��
		//ͨ���ƽ���ж��Ƿ�Ϊ-1���ж����·���Ƿ�������
		double bestVal = Integer.MAX_VALUE;
		int bestNodeId = -1;
		double bestNodeVistitTime = -1;

		Iterator<Integer> iter = remainingNodes.iterator();
		while (iter.hasNext()) {
			int nodeId = iter.next();
			//���������Ѿ������ˣ��¼�������������˳�����������continue���ж���һ�����Ƿ�����װ�복ʣ�������ĵ�
			//�����û�У�������bestNodeId��bestNodeVistitTime�����������û�б仯����ɷ���ֵ���ػ�ȥ
			if (demands[nodeId] > remCapacity) {
				continue;
			}
			// ��ǰ������һ����ľ���
			double distance = (curLastId == -1) ? distancesFromDepot[nodeId] : distances[curLastId][nodeId];
			// ����һ���㼴nodeId����������ʱ�䣬����
			// curLastVistTimezΪ��ǰ����������ʱ��
			// curLastVistTime+distance+curLastServiceTimeΪ�����ǰ���������֮�����Ϸ���ʱ���·;ʱ��󣬵����Ӧ��nodeId������Ϊ���
			// windowStartTimes[nodeId]Ϊ��һ����nodeId�����翪ʼʱ�䣬����ֻҪ����������Ŀ�ʼʱ�������翪ʼʱ��֮��������ʱ�俪ʼ�Ϳ���
			// ��ʼ�ˣ���������ĸ�ʱ����󣬼��ĸ�ʱ�俪ʼ��
			// ��Ϊ���windowStartTimes[nodeId]������һ������˵������nodeId��ʱ�򣬻�û�����翪ʼʱ�䣬�Ǿ͵õȵ�windowStartTimes[nodeId]��ʼ
			// �����curLastVistTime+distance+curLastServiceTime���������û��������Ŀ�ʼʱ�䣬��Ӧ�ð������ʱ���������ʱ�俪ʼ����
			double minVisitTime = Math.max(curLastVistTime + distance + curLastServiceTime, windowStartTimes[nodeId]);
			if (minVisitTime > windowEndTimes[nodeId]) {
				continue;
			}
			// �����м�����ڳ�������������Ŀ����ʱ��
			double timeDiff = minVisitTime - (curLastVistTime + curLastServiceTime);
			// ��������ʼ����ʱ���복�ӵ���ʱ�����м�ļ�϶���ж��Ƿ����
			double urgency = windowEndTimes[nodeId] - (curLastVistTime + curLastServiceTime + distance);
			// ͨ����Ȩ�ķ�ʽ������������ڵ��ѡ���Ƿ���á�
			// ������ʵֻ�����ж��Ƿ�������ڰ�����ʱ������ܣ�û�п��ࡣ
			double val = timeDiff * timeDiffWeight + distance * distanceWeight + urgency * urgencyWeight;
			if (val < bestVal) {
				bestVal = val;
				bestNodeId = nodeId;
				bestNodeVistitTime = minVisitTime;
			}
		}
		//ͨ������һ���������֣�ʹ�������������ͬ���󶼿���װ��һ�����󷵻س�ȥ����Ӵﵽ���ض����ͬ���󷵻�ֵ��Ч��
		return new Number[] { new Integer(bestNodeId), new Double(bestNodeVistitTime) };
	}

	/**
	 * ������ĵ�ǰ·�����в������������ӵ�·������
	 * 
	 * @param problem
	 * @param routes
	 * @return
	 */
	public VrpSolution nearestNeighborHeuristic(VrpProblem problem, List<List<Integer>> routes) {
		Set<Integer> remainingNodes = new HashSet<Integer>();
		for (int i = 0; i < problem.getNumCities(); i++) {
			remainingNodes.add(i);
		}
		for (List<Integer> route : routes) {
			for (Integer node : route) {
				remainingNodes.remove(node);
			}
		}
		List<Integer> curRoute = new ArrayList<Integer>();
		routes.add(curRoute);
		int curNodeId = -1;
		double curVisitTime = 0;
		int remCapacity = problem.getVehicleCapacity();
		while (remainingNodes.size() > 0) {
			Number[] ret = findClosest(curNodeId, curVisitTime, remCapacity, remainingNodes, problem);
			int nextNodeId = (Integer) ret[0];
			if (nextNodeId != -1) {
				remainingNodes.remove(nextNodeId);
				curRoute.add(nextNodeId);
				curNodeId = nextNodeId;
				curVisitTime = (Double) ret[1];
				remCapacity -= problem.getDemands()[nextNodeId];
			} else {
				//�гɵ�·���Ѿ�����Լ�����޷������ӽڵ�ֻ�ܷ����ˡ���ʱ������·��curRoute����routes��������
				curRoute = new ArrayList<Integer>();
				routes.add(curRoute);
				curVisitTime = 0;
				curNodeId = -1;
				remCapacity = problem.getVehicleCapacity();
			}
		}

		return new VrpSolution(routes, problem);
	}

	public VrpSolution nearestNeighborHeuristic(VrpProblem problem) {
		return nearestNeighborHeuristic(problem, new ArrayList<List<Integer>>());
	}

	public static void main(String[] args) throws IOException {

		File f = new File("problems/c1_2_1.txt");
	    VrpProblem problem = VrpReader.readSolomon(f, 100);
	    // seems like more for the first two and less for the last works
	    GreedyInitializer init = new GreedyInitializer(1.0, 1.0, 0);
	    VrpSolution sol = init.nearestNeighborHeuristic(problem);
	    System.out.println(sol.getNumVehicles());
	    System.out.println(sol.getToursCost());
	    System.out.println(sol.verify(problem));
	    
	    JFrame frame = new JFrame();
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    VrpPanel panel = new VrpPanel();
	    panel.setScale(problem);
	    panel.setSolution(sol);
	    frame.getContentPane().add(panel);
	    frame.pack();
	    frame.setVisible(true);

	}

}
