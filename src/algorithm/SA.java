package algorithm;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import util.VrpProblem;
import util.VrpReader;
import util.VrpSolution;
import viz.VrpPanel;

public class SA {
	
	private VrpProblem problem;
	private VrpSolution initSolution;
	
	public SA(VrpProblem problem, VrpSolution initSolution) {
		super();
		this.problem = problem;
		this.initSolution = initSolution;
	}

	/**
	 * 调用VrpSolution自身的计算函数求出里面存储的路径总 距离  耗费
	 * @param sol 求出的一个新的可行解，包含了所有点的多车辆各路径安排
	 * @return 返回出这个安排所需要的路途距离耗费
	 */
	public double toursCost(VrpSolution sol) {
		return sol.getToursCost();
	}

	/**
	 * 计算路径集合的距离花费
	 * @param routes 一辆车的路径为route，所有车的路径组成routes
	 * @param problem	读入的数据
	 * @return 返回这些路径的总花费
	 */
	public double calcToursCost(List<List<Integer>> routes, VrpProblem problem) {
	    double[][] distances = problem.getDistances();
	    double[] distancesFromDepot = problem.getDistancesFromDepot();
	    
	    double toursCost = 0;
	    for (List<Integer> route : routes) {
	      Iterator<Integer> iter = route.iterator();
	      if (!iter.hasNext()) {
	        continue;
	      }
	      int prev = iter.next();
	      toursCost += distancesFromDepot[prev];
	      while (iter.hasNext()) {
	        int cur = iter.next();
	        toursCost += distances[prev][cur];
	        prev = cur;
	      }
	      toursCost += distancesFromDepot[prev];
	    }
	    return toursCost;
	  }
	
	public List<List<Integer>> Sa_VRPTW(VrpSolution initSolution, double T0, double d, double Tk,int L) {
		// T0=1e5,d =1-7e-3, Tk=1e-3
		VrpSolution solution = initSolution;
		List<List<Integer>> bestpath, curentpath;
		double t = T0;
		//完整的初始解，包含多条车辆路径，每一条是一个List，多条用List<List<Integer>>来装
		bestpath = copyRoutes(initSolution.getRoutes());
		curentpath = copyRoutes(initSolution.getRoutes());
		Random random = new Random();
		Operator<Integer> operator = new Operator<>();
		while (t > Tk) {
			int it=0;
			System.out.println("****** 当前温度 T="+t+"*****");
			while (it<L) {
				System.out.println("迭代第 "+it+" 次");
				List<List<Integer>> update_path = operator.groupTwo_Exchange(curentpath);
				//如果拓展的路线无法满足约束条件，则从原始继续拓展
				while (!verify(update_path, problem)) {
//					System.out.println("-------不满足约束-------------");
					update_path = operator.groupTwo_Exchange(curentpath);
				}
				System.out.println("-------拓展新路线-------------");
				double delta = calcToursCost(update_path,problem) - calcToursCost(curentpath,problem);
				if (delta < 0) {//为负值，即结果成本降低了，则接受
					System.out.println("更新值 delta = "+delta);
					curentpath = update_path;
					bestpath = update_path;
				} else {
					double p = Math.exp(-delta/t);
					if (random.nextDouble() <= p) {
						curentpath = update_path;
					}
				}
				it++;
			}
			t *=d;
		}
		return bestpath;
	}
	
	public List<List<Integer>> copyRoutes(List<List<Integer>> routes){
		List<List<Integer>> copyRoutes = new ArrayList<List<Integer>>();
		Iterator<List<Integer>> iterator = routes.iterator();
		while (iterator.hasNext()) {
			List<Integer> eachRoute = iterator.next();
			Iterator<Integer> it = eachRoute.iterator();
			List<Integer> tmpRoute = new ArrayList<Integer>();
			while (it.hasNext()) {
				tmpRoute.add(it.next());
			}
			copyRoutes.add(tmpRoute);
		}
		return copyRoutes;
	}
	
	/**
	 * 用于判断所有车辆路径是否满足约束条件
	 * @param routes
	 * @param problem
	 * @return
	 */
	public Boolean isRouteAvailable(List<List<Integer>> routes, VrpProblem problem) {
		int[] demands = problem.getDemands();
		int[] windowStartTimes = problem.getWindowStartTimes();
		int[] windowEndTimes = problem.getWindowEndTimes();
		double[][] distances = problem.getDistances();
		double[] distancesFromDepot = problem.getDistancesFromDepot();
		int[] serviceTimes = problem.getServiceTimes();
		
		for (List<Integer> route : routes) {
			int curNodeId = -1;
			double curVisitTime = 0;
			int remCapacity = problem.getVehicleCapacity();
			int curLastServiceTime = (curNodeId == -1) ? 0 : serviceTimes[curNodeId];
			for (Integer nodeId : route) {
				//如果在下一个站开始前到达了，并且容量满足约束
				double distance = (curNodeId == -1) ? distancesFromDepot[nodeId] : distances[curNodeId][nodeId];
				//因为下一个点要到开始时间之后才会开始，所以与最早开始时间比较，选取大的一方
				double minVisitTime = Math.max(curVisitTime + distance + curLastServiceTime, windowStartTimes[nodeId]);
				//如果得到的开始时间晚于最晚开始时间，或者达到要求但是容量无法装下，则不满足约束，返回false
				if (minVisitTime > windowEndTimes[nodeId]
						|| remCapacity<demands[nodeId]) {
					return false;
				}else {
					curNodeId = nodeId;
					curVisitTime = minVisitTime;
					remCapacity-=demands[nodeId];
				}
			}
			return true;
		}
		
		return true;
	}

	/**
	   * Verify that the solution satisfied the constraints for the given problem,
	   * and that the reported values for the objective function are correct.
	   */
	  public boolean verify(List<List<Integer>> routes, VrpProblem problem) {
	    double[][] distances = problem.getDistances();
	    double[] distancesFromDepot = problem.getDistancesFromDepot();
	    
//	    int toursCost = calcToursCost(routes, problem);
	    
	    int[] windowStartTimes = problem.getWindowStartTimes();
	    int[] windowEndTimes = problem.getWindowEndTimes();
	    int[] serviceTimes = problem.getServiceTimes();
	    int[] demands = problem.getDemands();
	    boolean[] visited = new boolean[problem.getNumCities()];
	    
	    //follow the paths and make sure that the time constraints hold
	    for (List<Integer> route : routes) {
	      Iterator<Integer> iter = route.iterator();
	      if (route.isEmpty()) {
//	        System.out.println("EMPTY ROUTE!!!");
	        continue;
	      }
	      int prev = iter.next();
	      if (windowEndTimes[prev] < distancesFromDepot[prev]) {
//	        System.out.println("first node violated time constraint: endTime=" + 
//	            windowEndTimes[prev] + ", dist=" + distancesFromDepot[prev]);
	        return false;
	      }
	      visited[prev] = true;
	      int remCapacity = problem.getVehicleCapacity() - demands[prev];
	      double minVisitTime = Math.max(windowStartTimes[prev], distancesFromDepot[prev]);
	      
	      while (iter.hasNext()) {
	        int cur = iter.next();
	        visited[cur] = true;
	        double nextMinVisitTime = Math.max(minVisitTime + serviceTimes[prev] + distances[prev][cur], windowStartTimes[cur]);
	        if (nextMinVisitTime > windowEndTimes[cur]) {
//	          System.out.println(minVisitTime + "\t" + serviceTimes[prev] + "\t" + distances[prev][cur]);
//	          System.out.println("violated time constraint for " + prev + "->" + cur + 
//	              ": endTime=" + windowEndTimes[cur] + ", visitTime=" + nextMinVisitTime);
	          return false;
	        }
	        minVisitTime = nextMinVisitTime;
	        
	        remCapacity -= demands[cur];
	        if (remCapacity < 0) {
//	          System.out.println("violated capacity constraint");
	          return false;
	        }
	        prev = cur;
	      }
	    }  
	    
	    for (boolean b : visited) {
	      if (!b) {
	        System.out.println("one of the nodes not visited");
	        return false;
	      }
	    }
	    
//	    if (toursCost != this.toursCost) {
//	      System.out.println("tour costs do not match: " + toursCost + " != " + this.toursCost);
//	      return false;
//	    }
	    
	    return true;
	  }
	
	public static void main(String[] args) throws IOException {
		File f = new File("problems/c1_2_1.txt");
	    VrpProblem problem = VrpReader.readSolomon(f, 100);
	    GreedyInitializer init = new GreedyInitializer(1.0, 1.0, 0);
	    VrpSolution sol = init.nearestNeighborHeuristic(problem);
		SA sa = new SA(problem,sol);
		double T0=1e6;
		double d=0.97;
		double Tk=1e-5;
		int L = 20*problem.getNumCities();//内循环次数
		List<List<Integer>> bestRoutes = sa.Sa_VRPTW(sol, T0, d, Tk, L);
		
		VrpSolution bestSolution = new VrpSolution(bestRoutes, problem);
	    System.out.println(bestSolution.getNumVehicles());
	    System.out.println(bestSolution.getToursCost());
	    System.out.println(bestSolution.verify(problem));
	    
	    JFrame frame = new JFrame();
	    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	    VrpPanel panel = new VrpPanel();
	    panel.setScale(problem);
	    panel.setSolution(bestSolution);
	    frame.getContentPane().add(panel);
	    frame.pack();
	    frame.setVisible(true);

	}

}
