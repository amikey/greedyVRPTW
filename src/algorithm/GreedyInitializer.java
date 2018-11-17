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
	 * 用于求出当前节点与附近的其他点之中，满足时间窗等约束的最好点 通过把各个约束的耗费时差统计，加权求和得值，选取最小的那个点，即使目前最有的
	 * 
	 * @param curLastId
	 *            当前访问的节点，如果是仓库点为-1
	 * @param curLastVistTime
	 *            当前访问这个点的最晚开始时间
	 * @param remCapacity
	 *            剩余容量
	 * @param remainingNodes
	 *            剩余还没访问的节点集合
	 * @param problem
	 *            数据
	 * @return 返回出数组{求得的下一个最优点点ID，那个点的最晚访问时间}
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

		//如果运行的时候发现循环里的约束已经饱和，无法再满足加入新点了，这时候初始化点的bestNodeId = -1和bestNodeVistitTime = -1不变，为返回结果
		//通过推结果判断是否为-1来判断这个路线是否走完了
		double bestVal = Integer.MAX_VALUE;
		int bestNodeId = -1;
		double bestNodeVistitTime = -1;

		Iterator<Integer> iter = remainingNodes.iterator();
		while (iter.hasNext()) {
			int nodeId = iter.next();
			//车的容量已经快满了，新加入的容量超过了车的容量，则continue，判断下一个点是否有能装入车剩余容量的点
			//如果都没有，则最终bestNodeId、bestNodeVistitTime在这个过程中没有变化，组成返回值返回回去
			if (demands[nodeId] > remCapacity) {
				continue;
			}
			// 当前点与下一个点的距离
			double distance = (curLastId == -1) ? distancesFromDepot[nodeId] : distances[curLastId][nodeId];
			// 到下一个点即nodeId点的最晚访问时间，里面
			// curLastVistTimez为当前点的最晚访问时间
			// curLastVistTime+distance+curLastServiceTime为求出当前点最晚访问之后算上服务时间和路途时间后，到达对应的nodeId这个点后为多久
			// windowStartTimes[nodeId]为下一个点nodeId的最早开始时间，但是只要在能在最晚的开始时间与最早开始时间之间的任意个时间开始就可以
			// 开始了，所以求出哪个时间最大，即哪个时间开始。
			// 因为如果windowStartTimes[nodeId]大于另一个，则说明到达nodeId的时候，还没到最早开始时间，那就得等到windowStartTimes[nodeId]开始
			// 如果是curLastVistTime+distance+curLastServiceTime大，且这个点没超过最晚的开始时间，那应该按照这个时间来，这个时间开始服务
			double minVisitTime = Math.max(curLastVistTime + distance + curLastServiceTime, windowStartTimes[nodeId]);
			if (minVisitTime > windowEndTimes[nodeId]) {
				continue;
			}
			// 计算中间可用于车辆活动或者其他的空余的时间
			double timeDiff = minVisitTime - (curLastVistTime + curLastServiceTime);
			// 计算最晚开始服务时间与车子到达时间这中间的间隙，判断是否紧迫
			double urgency = windowEndTimes[nodeId] - (curLastVistTime + curLastServiceTime + distance);
			// 通过加权的方式，来评估这个节点的选择是否最好。
			// 但是其实只能是判断是否这个点在安排上时间最紧密，没有空余。
			double val = timeDiff * timeDiffWeight + distance * distanceWeight + urgency * urgencyWeight;
			if (val < bestVal) {
				bestVal = val;
				bestNodeId = nodeId;
				bestNodeVistitTime = minVisitTime;
			}
		}
		//通过定义一个父类数字，使得子类的两个不同对象都可以装成一个对象返回出去，间接达到返回多个不同对象返回值的效果
		return new Number[] { new Integer(bestNodeId), new Double(bestNodeVistitTime) };
	}

	/**
	 * 将输入的当前路径进行不断求最近邻添加到路径里面
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
				//行成的路径已经满足约束，无法再增加节点只能返回了。这时候把这个路径curRoute加入routes集合里面
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
